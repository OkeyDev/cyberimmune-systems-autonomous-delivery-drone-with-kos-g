/**
 * \file
 * \~English \brief Implementation of the security module PeripheryController component main loop.
 * \~Russian \brief Реализация основного цикла компонента PeripheryController модуля безопасности.
 */

 #include <unistd.h>
#include <string>
#include <iostream>
#include <kosipc/make_application.h>
#include <kosipc/serve_static_channel.h>

#include "../include/periphery_controller_interface.h"

/**
 * \~English \brief PeripheryController component main program entry point.
 * \details First, waits for the Logger component to initialize. After that,
 * interfaces for peripherals management are prepared. The peripherals are then set to default mode.
 * Then the program enters a loop, where it receives IPC messages from other security module components,
 * performs the requested actions and sends IPC responses. In parallel with the main loop,
 * the other process is executed. If the motors are supplied with the power, than this process constantly
 * asks (2 time per second) the ATM server whether the power supply is allowed. Requests are signed and sent
 * through the CredentialManager and ServerConnector components respectively.
 * \~Russian \brief Точка входа в основную программу компонента PeripheryController.
 * \details Сначала производится ожидание инициализации компонента Logger. После этого подготавливаются интерфейсы
 * для управления периферийными устройствами. Затем периферийные устройства переводятся в режим по умолчанию.
 * Далее программа входит в цикл, в котором получает IPC-сообщения от других компонентов модуля безопасности,
 * исполняет запрашиваемые действия и отправляет IPC-ответы. Параллельно с основным циклом запускается процесс.
 * Этот процесс, в случае если подача питания на двигатели разрешена, регулярно (2 раза в секунду) опрашивает сервер
 * ОРВД, можно ли продолжать подавать питание на двигатели. Подпись и отправка запросов осуществляются через
 * компоненты CredentialManager и ServerConnector соответственно.
 */
int main(void) {
    if (!initPeripheryController())
        return EXIT_FAILURE;

    while (!initGpioPins()) {
        logEntry("Trying again to initialize GPIO pins in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!initCamera()) {
        logEntry("Trying again to initialize camera in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!setKillSwitch(false)) {
        logEntry("Trying again to turn off kill-switch in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!setBuzzer(false)) {
        logEntry("Trying again to turn off buzzer in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!setCargoLock(true)) {
        logEntry("Trying again to lock cargo in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

    kosipc::Application app     =  kosipc::MakeApplicationAutodetect();
    kosipc::components::Root       root;
    IPeripheryController           interface;
    root.interface              = &interface;
    kosipc::EventLoop loop      =  app.MakeEventLoop(ServeStaticChannel("periphery_controller_connection", root));
    loop.Run();

    return EXIT_SUCCESS;
}
