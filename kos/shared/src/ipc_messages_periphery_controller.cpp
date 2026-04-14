/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to PeripheryController component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту PeripheryController.
 */

#include <iostream>
#include <string>
#include <kosipc/api.h>

#include "../include/ipc_messages_periphery_controller.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryControllerInterface.idl.cpp.h>

using namespace kosipc::stdcpp::drone_controller;

int enableBuzzer() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<PeripheryControllerInterface>(kosipc::ConnectStaticChannel("periphery_controller_connection", "interface"));

    try {
        proxy->EnableBuzzer(success);
    }
    catch (...) {
        std::cerr << "Exception on proxy->EnableBuzzer request" << std::endl;
        return 0;
    }

    return success;
}

int setKillSwitch(uint8_t enable) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<PeripheryControllerInterface>(kosipc::ConnectStaticChannel("periphery_controller_connection", "interface"));

    try {
        proxy->SetKillSwitch(enable, success);
    }
    catch (...) {
        std::cerr << "Exception on proxy->SetKillSwitch request" << std::endl;
        return 0;
    }

    return success;
}

int setCargoLock(uint8_t enable) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<PeripheryControllerInterface>(kosipc::ConnectStaticChannel("periphery_controller_connection", "interface"));

    try {
        proxy->SetCargoLock(enable, success);
    }
    catch (...) {
        std::cerr << "Exception on proxy->SetCargoLock request" << std::endl;
        return 0;
    }

    return success;
}

int takePicture(std::string& picture) {
    //TODO: rewrite without PureClient
    //TODO: make parameters names be the same between interfaces
    //scanResult vs tagFound

    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<PeripheryControllerInterface>(kosipc::ConnectStaticChannel("periphery_controller_connection", "interface"));

    try {
        proxy->TakePicture(picture, success);
    }
    catch (...) {
        std::cerr << "Exception on proxy->TakePicture request" << std::endl;
        return 0;
    }

    return success;
}
