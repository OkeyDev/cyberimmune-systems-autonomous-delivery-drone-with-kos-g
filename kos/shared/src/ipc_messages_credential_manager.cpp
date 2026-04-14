/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to CredentialManager component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту CredentialManager.
 */
#include <iostream>
#include <string>
#include <kosipc/api.h>

#include "../include/ipc_messages_credential_manager.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/CredentialManagerInterface.idl.cpp.h>

using namespace kosipc::stdcpp::drone_controller;

int signMessage(char* message, char* signature, uint32_t signatureSize) {
    // TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    std::string sign;
    auto proxy              = app.MakeProxy<CredentialManagerInterface>(kosipc::ConnectStaticChannel("credential_manager_connection", "interface"));

    try {
        proxy->SignMessage(std::string(message), success, sign);
    }
    catch (...) {
        std::cerr << "Exception on proxy->SignMessage request: message=" << std::string(message) << " signing" << std::endl;
        return 0;
    }

    if (!success)
        return 0;
    sign.copy(signature, sign.size() + 1);
    return 1;
}

int checkSignature(char* message, MessageSource source, uint8_t &authenticity) {
    // TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<CredentialManagerInterface>(kosipc::ConnectStaticChannel("credential_manager_connection", "interface"));

    try {
        proxy->CheckSignature(std::string(message), (uint8_t)source, success, authenticity);
    }
    catch (...) {
        std::cerr << "Exception on proxy->CheckSignature request: message=" << std::string(message) << " signature checking" << std::endl;
        return 0;
    }

    return success;
}
