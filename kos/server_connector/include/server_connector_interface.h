/**
 * \file
 * \~English
 * \brief Declaration of ServerConnectorInterface IDL-interface methods.
 * \details Interaction between ServerConnector component and other components
 * of the security module is done via IPC message sent through ServerConnectorInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the ServerConnectorInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса ServerConnectorInterface.
 * \details Взаимодействие с компонентом ServerConnector другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс ServerConnectorInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу ServerConnectorInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#include "../include/server_connector.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnector.edl.cpp.h>

using namespace kosipc::stdcpp::drone_controller;

class IServerConnector : public ServerConnectorInterface {
public:
/**
 * \~English IPC message handler. See \ref getBoardId.
 * \~Russian Обработчик IPC-сообщения. См. \ref getBoardId.
 */
    void GetBoardId(uint8_t& success, std::string& id) {
        //TODO: rewrite to cpp way
        id = std::string(getBoardName());
        success = (id.size() > 0);
    }

/**
 * \~English IPC message handler. See \ref sendRequest.
 * \~Russian Обработчик IPC-сообщения. См. \ref sendRequest.
 */
    void SendRequest(const std::string& query, uint8_t& success, std::string& response) {
        //TODO: rewrite to cpp way
        char q[MaxQueryLength + 1] = {0};
        query.copy(q, query.size() + 1);
        char r[MaxResponseLength + 1] = {0};
        success = requestServer(q, r, MaxResponseLength + 1);
        response = std::string(r);
    }

/**
 * \~English IPC message handler. See \ref sendRequest.
 * \~Russian Обработчик IPC-сообщения. См. \ref sendRequest.
 */
    void PublishMessage(const std::string& topic, const std::string& publication, uint8_t& success) {
        //TODO: rewrite to cpp way
        char t[MaxTopicLength + 1] = {0};
        topic.copy(t, topic.size() + 1);
        success = publish(t, publication);
    }

/**
 * \~English IPC message handler. See \ref sendRequest.
 * \~Russian Обработчик IPC-сообщения. См. \ref sendRequest.
 */
    void ReceiveSubscription(const std::string& topic, std::string& subscription, uint8_t& success) {
        //TODO: rewrite to cpp way
        char t[MaxTopicLength + 1] = {0};
        topic.copy(t, topic.size() + 1);
        char s[MaxSubscriptionLength + 1] = {0};
        success = getSubscription(t, s, MaxSubscriptionLength + 1);
        subscription = std::string(s);
    }
};