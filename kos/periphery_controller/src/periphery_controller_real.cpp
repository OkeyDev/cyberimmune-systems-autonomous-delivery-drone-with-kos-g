/**
 * \file
 * \~English
 * \brief Implementation of methods for hardware peripherals control.
 * \details The file contains implementation of methods,
 * that control peripheral drone devices via GPIO interface.
 *
 * \~Russian
 * \brief Реализация методов для работы с аппаратной периферией.
 * \details В файле реализованы методы, управляющие периферийными устройствами
 * дрона через интерфейс GPIO.
 */

#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <gpio/gpio.h>
#include <uart/uart.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

/** \cond */
#define NAME_MAX_LENGTH 64

char gpio[] = "gpio0";
char gpioConfigSuffix[] = "default";
GpioHandle gpioHandler = NULL;

char bspUart[] = "uart4";
char rfidUart[] = "serial@7e201800";
char rfidConfigSuffix[] = "default";
UartHandle rfidUartHandler = NULL;

uint8_t pinBuzzer = 20;
uint8_t pinCargoLock = 21;
uint8_t pinKillSwitchFirst = 22;
uint8_t pinKillSwitchSecond = 27;

bool killSwitchEnabled;
/** \endcond */

/**
 * \~English Sets the mode of specified pin power supply.
 * \param[in] pin Pin to set mode.
 * \param[in] mode Mode. 1 is high, 0 is low.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Устанавливает режим подачи энергии на заданный пин.
 * \param[in] pin Пин, для которого устанавливается режим.
 * \param[in] mode Режим. 1 -- высокий, 0 -- низкий.
 * \return Возвращает 1, если режим был успешно установлен, иначе -- 0.
 */
int setPin(uint8_t pin, bool mode) {
    Retcode rc = GpioOut(gpioHandler, pin, mode);
    if (rcOk != rc) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to set GPIO pin %d to %d (" RETCODE_HR_FMT ")", pin, mode, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int initPeripheryController() {
    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        logEntry("Failed to get board name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char gpioConfig[NAME_MAX_LENGTH];
    if (snprintf(gpioConfig, NAME_MAX_LENGTH, "%s.%s", boardName, gpioConfigSuffix) < 0) {
        logEntry("Failed to generate GPIO config name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char rfidConfig[NAME_MAX_LENGTH] = {0};
    if (snprintf(rfidConfig, NAME_MAX_LENGTH, "%s.%s", boardName, rfidConfigSuffix) < 0) {
        logEntry("Failed to generate UART config name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char logBuffer[256] = {0};
    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize BSP (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspSetConfig(gpio, gpioConfig);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to set BSP config for GPIO %s (" RETCODE_HR_FMT ")", gpio, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = GpioInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize GPIO (" RETCODE_HR_FMT ")", RC_GET_CODE(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspEnableModule(bspUart);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to enable UART %s (" RETCODE_HR_FMT ")", rfidUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspSetConfig(bspUart, rfidConfig);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to set BSP config for UART %s (" RETCODE_HR_FMT ")", rfidUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize UART (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initGpioPins() {
    char logBuffer[256] = {0};
    Retcode rc = GpioOpenPort(gpio, &gpioHandler);
    if (rcOk != rc) {
        snprintf(logBuffer, 256, "Failed top open GPIO %s (" RETCODE_HR_FMT ")", gpio, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    uint8_t pins[] = { pinBuzzer, pinCargoLock, pinKillSwitchFirst, pinKillSwitchSecond };
    for (uint8_t pin : pins) {
        rc = GpioSetMode(gpioHandler, pin, GPIO_DIR_OUT);
        if (rcOk != rc) {
            snprintf(logBuffer, 256, "Failed to set GPIO pin %u mode (" RETCODE_HR_FMT ")", pin, RETCODE_HR_PARAMS(rc));
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
    }

    rc = UartOpenPort(rfidUart, &rfidUartHandler);
    if (rc != rcOk) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to open UART %s (" RETCODE_HR_FMT ")", rfidUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}

int setBuzzer(bool enable) {
    return setPin(pinBuzzer, enable);
}

int readRfid(char* tag) {
    char logBuffer[256] = {0};
    rtl_size_t writtenBytes, readBytes;
    uint8_t scanRequest[] = { 0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E };
    uint8_t rfidResponse[] = { 0x02, 0x22 };
    uint8_t noRfidResponse[] = { 0x01, 0xFF };
    uint8_t scanResponse[24] = {0};
    strcpy(tag, "");

    for (int i = 0; i < 50; i++) {
        UartWrite(rfidUartHandler, scanRequest, 7, NULL, &writtenBytes);

        scanResponse[0] = 0;
        while (scanResponse[0] != 0xBB)
            UartRead(rfidUartHandler, (rtl_uint8_t*)scanResponse, 1, NULL, &readBytes);
        UartRead(rfidUartHandler, (rtl_uint8_t*)(scanResponse + 1), 2, NULL, &readBytes);

        if (!memcmp(scanResponse + 1, rfidResponse, 2)) {
            UartRead(rfidUartHandler, (rtl_uint8_t*)(scanResponse + 3), 21, NULL, &readBytes);
            snprintf(tag, 36, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", scanResponse[8], scanResponse[9],
                scanResponse[10], scanResponse[11], scanResponse[12], scanResponse[13], scanResponse[14], scanResponse[15],
                scanResponse[16], scanResponse[17], scanResponse[18], scanResponse[19]);
            snprintf(logBuffer, 256, "Read RFID '%s'", tag);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            return 1;
        }
        else if (!memcmp(scanResponse + 1, noRfidResponse, 2))
            UartRead(rfidUartHandler, (rtl_uint8_t*)(scanResponse + 3), 5, NULL, &readBytes);
        else {
            snprintf(logBuffer, 256, "Unknown header %02x %02x %02x", scanResponse[0], scanResponse[1], scanResponse[2]);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            return 0;
        }
    }
    logEntry("No RFID was read", ENTITY_NAME, LogLevel::LOG_INFO);
    return 1;
}

int setKillSwitch(bool enable) {
    if (enable) {
        if (!publishMessage("api/events", "type=kill_switch&event=OFF"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
        if (!setPin(pinKillSwitchFirst, false) || !setPin(pinKillSwitchSecond, true))
            return 0;
        else {
            killSwitchEnabled = true;
            return 1;
        }
    }
    else {
        if (!publishMessage("api/events", "type=kill_switch&event=ON"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
        if (!setPin(pinKillSwitchFirst, false) || !setPin(pinKillSwitchSecond, false))
            return 0;
        else {
            killSwitchEnabled = false;
            return 1;
        }
    }
}

int setCargoLock(bool enable) {
    if (!publishMessage("api/events", enable ? "type=cargo_lock&event=OFF" : "type=kill_switch&event=ON"))
        logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
    return setPin(pinCargoLock, enable);
}
