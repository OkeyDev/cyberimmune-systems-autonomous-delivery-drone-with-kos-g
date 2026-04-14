/**
 * \file
 * \~English
 * \brief Implementation of methods for simulated drone peripherals control.
 * \details The file contains implementation of methods, that simulate peripherals
 * control via socket communication with a compatible ArduPilot SITL firmware.
 *
 * \~Russian
 * \brief Реализация методов для работы с симулятором периферии дрона.
 * \details В файле реализованы методы, симулирующие управление периферией дрона
 * через взаимодействие с совместимой SITL-прошивкой ArduPilot через сокет.
 */

#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../../shared/include/ipc_messages_navigation_system.h"

#include <kos_net.h>

#include <stdio.h>
#include <math.h>

/** \cond */
#define SIM_PERIPHERY_MESSAGE_HEAD_SIZE 4
#define TAG_NUM 3

static const uint8_t SimPeripheryMessageHead[SIM_PERIPHERY_MESSAGE_HEAD_SIZE] = { 0x06, 0x66, 0xbe, 0xa7 };
/** \endcond */

/**
 * \~English Control messages types used to simulate
 * peripherals power supply mod change.
 * \~Russian Типы управляющих сообщений, используемых для симуляции
 * изменения режима подачи питания на периферийные устройства.
 */
enum SimPeripheryCommand : uint8_t {
    /**
     * \~English Auxiliary type. Signals an error.
     * \~Russian Вспомогательный тип. Сигнализирует о возникновении ошибки.
     */
    ERROR = 0x00,
    /**
     * \~English Drone motors are energized. On receive SITL firmware disables software restriction on arming.
     * \~Russian Двигатели запитаны. При получении SITL-прошивка снимает программный запрет на арминг.
     */
    MotorPermit,
    /**
     * \~English Drone motors are de-energized. On receive SITL firmware enables software restriction on arming.
     * \~Russian Двигатели обесточены. При получении SITL-прошивка включает программный запрет на арминг.
     */
    MotorForbid,
    /**
     * \~English Cargo drop motor is energized. On receive SITL firmware disables software restriction on cargo drop.
     * \~Russian Мотор сброса груза запитан. При получении SITL-прошивка снимает программный запрет на сброс груза.
     */
    CargoPermit,
    /**
     * \~English Cargo drop motor is de-energized. On receive SITL firmware enables software restriction on cargo drop.
     * \~Russian Мотор сброса груза обесточен. При получении SITL-прошивка включает программный запрет на сброс груза.
     */
    CargoForbid
};

/**
 * \~English A structure describing a message for SITL firmware
 * to simulate peripherals power supply mod change.
 * transmit current drone position.
 * \~Russian Структура, описывающая сообщение для SITL-прошивки
 * для симуляции изменения режима питания периферийных устройств.
 */
struct SimPeripheryMessage {
    /**
     * \~English Fixed header bytes that indicate the start of a new message.
     * \~Russian Фиксированные заголовочные байты, указывающие на начало нового сообщения.
     */
    uint8_t head[SIM_PERIPHERY_MESSAGE_HEAD_SIZE];
    /**
     * \~English Message type. For the types used, see \ref SimPeripheryCommand.
     * \~Russian Тип сообщения. Используемые типы см. в \ref SimPeripheryCommand.
     */
    SimPeripheryCommand command;
    /**
     * \~English Meaningless bytes. Needed to fill the size of the structure to a multiple of 4 bytes.
     * \~Russian Байты, не имеющие смысловой нагрузки. Нужны для заполнения размера структуры до
     * кратного 4 байтам.
     */
    uint8_t filler[3];

    /**
     * \~English Constructor that sets the correct header bytes and given type command.
     * \param[in] cmd Created message type.
     * \~Russian Конструктор, устанавливающий корректные заголовочные байты и команду поданного типа.
     * \param[in] cmd Тип создаваемого сообщения.
     */
    SimPeripheryMessage(SimPeripheryCommand cmd) {
        for (int i = 0; i < SIM_PERIPHERY_MESSAGE_HEAD_SIZE; i++)
            head[i] = SimPeripheryMessageHead[i];
        command = cmd;
    }
};

/** \cond */
int peripherySocket = NULL;
#ifdef IS_INSPECTOR
uint16_t peripheryPort = 5777;
#else
uint16_t peripheryPort = 5767;
#endif

float scanSquaredDistance = 0.25;
float latScale = 0.011131884502145f;
float lngScale = 0.011131884502145f;
int32_t tagLats[TAG_NUM] = { 600024368, 600024166, 600024428 };
int32_t tagLngs[TAG_NUM] = { 278576362, 278576914, 278577035 };
char tagPictures[TAG_NUM][9] = { "picture1", "picture2", "picture3" };

bool killSwitchEnabled;
/** \endcond */

int initPeripheryController() {
    if (!wait_for_iface(DEFAULT_INTERFACE, IWF_EXISTS, DEFAULT_TIMEOUT) || !configure_net_iface(DEFAULT_INTERFACE, DEFAULT_ADDR, DEFAULT_MASK, DEFAULT_GATEWAY, DEFAULT_MTU)) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initGpioPins() {
    peripherySocket = NULL;

    if ((peripherySocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logEntry("Failed to create socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(peripheryPort);

    if (connect(peripherySocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, peripheryPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int initCamera() {
#ifdef IS_INSPECTOR
    float avgLat = 0;
    for (int i = 0; i < TAG_NUM; i++)
        avgLat += 1.0f * tagLats[i] / TAG_NUM;
    float scale = cos(avgLat * 1.0e-7 * M_PI / 180.0f);
    if (scale < 0.01f)
        scale = 0.01f;
    lngScale *= scale;
#endif

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}

int setBuzzer(bool enable) {
    char logBuffer[256] = {0};
    snprintf(logBuffer, 256, "Buzzer is %s", enable ? "enabled" : "disabled");
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

    return 1;
}

int takePicture(std::string& picture) {
#ifdef IS_INSPECTOR
    int32_t lat, lng, alt;
    getCoords(lat, lng, alt);

    float latDif, lngDif;
    for (int i = 0; i < TAG_NUM; i++) {
        latDif = (tagLats[i] - lat) * latScale;
        lngDif = (tagLngs[i] - lng) * lngScale;

        if (latDif * latDif + lngDif * lngDif < scanSquaredDistance) {
            picture = std::string(tagPictures[i]);
            return 1;
        }
    }
    picture = std::string("picture0");
#endif

    return 1;
}

int setKillSwitch(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::MotorPermit : SimPeripheryCommand::MotorForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    killSwitchEnabled = enable;
    if (!publishMessage("api/events", enable ? "type=kill_switch&event=OFF" : "type=kill_switch&event=ON"))
        logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);

    return 1;
}

int setCargoLock(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::CargoPermit : SimPeripheryCommand::CargoForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    if (!publishMessage("api/events", enable ? "type=cargo_lock&event=OFF" : "type=kill_switch&event=ON"))
        logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);

    return 1;
}