/**
 * \file
 * \~English
 * \brief Declaration of NavigationSystemInterface IDL-interface methods.
 * \details Interaction between NavigationSystem component and other components
 * of the security module is done via IPC message sent through NavigationSystemInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the NavigationSystemInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса NavigationSystemInterface.
 * \details Взаимодействие с компонентом NavigationSystem другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс NavigationSystemInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу NavigationSystemInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#include "../include/navigation_system.h"

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystem.edl.cpp.h>

using namespace kosipc::stdcpp::drone_controller;

class INavigationSystem : public NavigationSystemInterface {
public:
/**
 * \~English IPC message handler. See \ref getCoords.
 * \~Russian Обработчик IPC-сообщения. См. \ref getCoords.
 */
    void GetCoords(uint8_t& success, int32_t& lat, int32_t& lng, int32_t& alt) {
        success = getPosition(lat, lng, alt);
    }

/**
 * \~English IPC message handler. See \ref getGpsInfo.
 * \~Russian Обработчик IPC-сообщения. См. \ref getGpsInfo.
 */
    void GetGpsInfo(uint8_t& success, int32_t& dop, int32_t& sats) {
        float d;
        success = getInfo(d, sats);
        std::memcpy(&dop, &d, sizeof(float));
    }
};