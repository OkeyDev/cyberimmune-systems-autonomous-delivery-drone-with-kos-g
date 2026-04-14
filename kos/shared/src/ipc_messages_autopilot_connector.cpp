/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to AutopilotConnector component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту AutopilotConnector.
 */

#include <chrono>
#include <iterator>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <kosipc/api.h>

#include "../include/ipc_messages_autopilot_connector.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.cpp.h>

using namespace std::chrono_literals;
using namespace kosipc::stdcpp::drone_controller;

int waitForArmRequest() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    while (true) {
        try {
            proxy->WaitForArmRequest(success);
        }
        catch (...) {
            std::cerr << "Exception on waitForArmRequest" << std::endl;
            return 0;
        }

        if (success)
            return 1;

        std::this_thread::sleep_for(1000ms);
    }

    return 0;
}

int permitArm() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));
    try {
        proxy->PermitArm(success);
    }
    catch (...) {
        std::cerr << "Exception on permitArm" << std::endl;
        return 0;
    }

    return success;
}

int forbidArm() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->ForbidArm(success);
    }
    catch (...) {
        std::cerr << "Exception on forbidArm" << std::endl;
        return 0;
    }

    return success;
}

int pauseFlight() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->PauseFlight(success);
    }
    catch (...) {
        std::cerr << "Exception on pauseFlight" << std::endl;
        return 0;
    }

    return success;
}

int resumeFlight() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->ResumeFlight(success);
    }
    catch (...) {
        std::cerr << "Exception on resumeFlight" << std::endl;
        return 0;
    }

    return success;
}

int abortMission() {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->AbortMission(success);
    }
    catch (...) {
        std::cerr << "Exception on abortMission" << std::endl;
        return 0;
    }

    return success;
}

int changeSpeed(int32_t speed) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->ChangeSpeed(speed, success);
    }
    catch (...) {
        std::cerr << "Exception on changeSpeed: speed=" << speed << " command" << std::endl;
        return 0;
    }

    return success;
}

int changeAltitude(int32_t altitude) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->ChangeAltitude(altitude, success);
    }
    catch (...) {
        std::cerr << "Exception on changeAltitude: altitude=" << altitude << " command" << std::endl;
        return 0;
    }

    return success;
}

int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude) {
    //TODO: rewrite without PureClient
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->ChangeWaypoint(latitude, longitude, altitude, success);
    }
    catch (...) {
        std::cerr << "Exception on changeWaypoint: latitude=" << latitude << ", longitude=" << longitude << ", altitude=" << altitude << " command" << std::endl;
        return 0;
    }

    return success;
}

int setMission(uint8_t* mission, uint32_t missionSize) {
    //TODO: rewrite without PureClient and ugly sizeof
    uint8_t success;
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    std::vector<uint8_t> m(mission, mission + missionSize);
    auto proxy              = app.MakeProxy<AutopilotConnectorInterface>(kosipc::ConnectStaticChannel("autopilot_connector_connection", "interface"));

    try {
        proxy->SetMission(m, missionSize, success);
    }
    catch (...) {
        std::cerr << "Exception on setMission" << std::endl;
        return 0;
    }

    return success;
}
