/**
 * \file
 * \~English \brief Implementation of the security module CredentialManager component main loop.
 * \~Russian \brief Реализация основного цикла компонента CredentialManager модуля безопасности.
 */

#include <string>
#include <iostream>
#include <kosipc/make_application.h>
#include <kosipc/serve_static_channel.h>

#include "../include/credential_manager_interface.h"

/**
 * \~English \brief CredentialManager component main program entry point.
 * \details First, waits for the Logger component to initialize. After that, the saved RSA key
 * of the security module is loaded (if there is no saved key, a new one is generated).
 * Then, through the ServerConnector component, the public parts of the key are exchanged with the ATM server.
 * The public part of the ATM server key is saved. Then the program enters a loop, where it receives
 * IPC messages from other security module components, performs the requested actions and sends IPC responses.
 * \~Russian \brief Точка входа в основную программу компонента CredentialManager.
 * \details Сначала производится ожидание инициализации компонента Logger. После этого загружается сохраненный
 * RSA-ключ модуля безопасности (при отсутствии сохраненного ключа генерируется новый). Затем через компонент
 * ServerConnector происходит обмен открытыми частями ключа с сервером ОРВД. Открытая часть ключа сервера ОРВД
 * сохраняется. Далее программа входит в цикл, в котором получает IPC-сообщения от других компонентов модуля
 * безопасности, исполняет запрашиваемые действия и отправляет IPC-ответы.
 */
int main(void) {
    if (!getRsaKey())
        return EXIT_FAILURE;

    if (!shareRsaKey())
        return EXIT_FAILURE;

    if (strcmp(PARTNER_ID, "NULL") && !getPartnerRsaKey())
        return EXIT_FAILURE;

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

    kosipc::Application app     =  kosipc::MakeApplicationAutodetect();
    kosipc::components::Root       root;
    ICredentialManager             interface;
    root.interface              = &interface;
    kosipc::EventLoop loop      =  app.MakeEventLoop(ServeStaticChannel("credential_manager_connection", root));
    loop.Run();

    return EXIT_SUCCESS;
}
