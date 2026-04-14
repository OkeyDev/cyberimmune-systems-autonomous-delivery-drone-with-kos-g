/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to NavigationSystem component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту NavigationSystem.
 */

#include <string>
#include <iostream>
#include <kosipc/api.h>
#include "../include/ipc_messages_navigation_system.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystemInterface.idl.cpp.h>

using namespace kosipc::stdcpp::drone_controller;

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<NavigationSystemInterface>(kosipc::ConnectStaticChannel("navigation_system_connection", "interface"));

    try {
        proxy->GetCoords(success, latitude, longitude, altitude);
    }
    catch (...) {
        std::cerr << "Exception on proxy->GetCoords request" << std::endl;
        return 0;
    }

    return success;
}

int getGpsInfo(float& dop, int32_t& sats) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<NavigationSystemInterface>(kosipc::ConnectStaticChannel("navigation_system_connection", "interface"));

    int32_t d;
    try {
        proxy->GetGpsInfo(success, d, sats);
    }
    catch (...) {
        std::cerr << "Exception on proxy->GetGpsInfo request" << std::endl;
        return 0;
    }

    if (!success)
        return 0;
    std::memcpy(&dop, &d, sizeof(float));
    return 1;
}

int getEstimatedSpeed(float& speed) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<NavigationSystemInterface>(kosipc::ConnectStaticChannel("navigation_system_connection", "interface"));

    int32_t s;
    try {
        proxy->GetSpeed(success, s);
    }
    catch (...) {
        std::cerr << "Exception on proxy->GetSpeed request" << std::endl;
        return 0;
    }

    if (!success)
        return 0;
    std::memcpy(&speed, &s, sizeof(float));
    return 1;
}
