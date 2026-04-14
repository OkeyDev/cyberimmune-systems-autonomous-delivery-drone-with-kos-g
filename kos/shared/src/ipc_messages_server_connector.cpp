/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to ServerConnector component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту ServerConnector.
 */

#include <string>
#include <iostream>
#include <kosipc/api.h>

#include "../include/ipc_messages_server_connector.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnectorInterface.idl.cpp.h>

using namespace kosipc::stdcpp::drone_controller;

int getBoardId(char* id) {
    uint8_t success;
    std::string boardId;
    //TODO: rewrite without PureClient
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<ServerConnectorInterface>(kosipc::ConnectStaticChannel("server_connector_connection", "interface"));

    try {
        proxy->GetBoardId(success, boardId);
    }
    catch (...) {
        std::cerr << "Exception on proxy->GetBoardId request: id=" << std::string(id) << std::endl;
        return 0;
    }

    if (!success)
        return 0;
    boardId.copy(id, boardId.size() + 1);
    return 1;
}

int sendRequest(char* query, char* response, uint32_t responseSize) {
    uint8_t success;
    std::string r;
    //TODO: rewrite this ugly thing
    std::memset(response, 0, responseSize);
    //TODO: rewrite without PureClient
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<ServerConnectorInterface>(kosipc::ConnectStaticChannel("server_connector_connection", "interface"));

    try {
        proxy->SendRequest(std::string(query), success, r);
    }
    catch (...) {
        std::cerr << "Exception on proxy->SendRequest: query=" << std::string(query) << std::endl;
        return 0;
    }

    if (!success)
        return 0;
    r.copy(response, r.size() + 1);
    return 1;
}

int publishMessage(char* topic, const char* publication) {
    uint8_t success;
    //TODO: rewrite without PureClient
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<ServerConnectorInterface>(kosipc::ConnectStaticChannel("server_connector_connection", "interface"));

    try {
        proxy->PublishMessage(std::string(topic), std::string(publication), success);
    }
    catch (...) {
        std::cerr << "Exception on proxy->PublishMessage: topic=" << std::string(topic) << " publication" << std::string(publication) << std::endl;
        return 0;
    }

    return success;
}

int receiveSubscription(char* topic, char* subscription, uint32_t subscriptionSize) {
    uint8_t success;
    std::string s;
    //TODO: rewrite this ugly thing
    std::memset(subscription, 0, subscriptionSize);
    //TODO: rewrite without PureClient
    kosipc::Application app = kosipc::MakeApplicationPureClient();
    auto proxy              = app.MakeProxy<ServerConnectorInterface>(kosipc::ConnectStaticChannel("server_connector_connection", "interface"));

    try {
        proxy->ReceiveSubscription(std::string(topic), s, success);
    }
    catch (...) {
        std::cerr << "Exception on proxy->ReceiveSubscription: topic=" << std::string(topic) << " receive" << std::endl;
        return 0;
    }

    if (!success)
        return 0;
    s.copy(subscription, s.size() + 1);
    return 1;
}
