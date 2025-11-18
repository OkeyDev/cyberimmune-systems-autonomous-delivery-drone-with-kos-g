/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to PeripheryController component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту PeripheryController.
 */

#include "../include/ipc_messages_periphery_controller.h"
#include "../include/initialization_interface.h"

#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryControllerInterface.idl.h>

int enableBuzzer() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_EnableBuzzer_req req;
    PeripheryControllerInterface_EnableBuzzer_res res;

    return ((PeripheryControllerInterface_EnableBuzzer(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setKillSwitch(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetKillSwitch_req req;
    PeripheryControllerInterface_SetKillSwitch_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetKillSwitch(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setCargoLock(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetCargoLock_req req;
    PeripheryControllerInterface_SetCargoLock_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetCargoLock(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int scanRfid(char* tag) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_ScanRfid_req req;
    PeripheryControllerInterface_ScanRfid_res res;
    char resBuffer[PeripheryControllerInterface_ScanRfid_res_arena_size];
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));
    nk_arena_reset(&resArena);

    if ((PeripheryControllerInterface_ScanRfid(&proxy.base, &req, NULL, &res, &resArena) != rcOk) || !res.success)
        return 0;

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, &resArena, &(res.tag), &len);
    if ((msg == NULL) || (len > PeripheryControllerInterface_ScanRfid_res_arena_size))
        return 0;
    strncpy(tag, msg, len);

    return 1;
}