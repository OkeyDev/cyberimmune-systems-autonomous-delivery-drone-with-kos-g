/**
 * \file
 * \~English
 * \brief Implementation of methods for hardware peripherals control.
 * \details The file contains implementation of methods,
 * that control peripheral drone devices via GPIO interface.
 *
 * \~Russian
 * \brief Реализация методов для работы с аппаратной периферией.
 * \details В файле реализованы методы, управляющие периферийными устройствами
 * дрона через интерфейс GPIO.
 */

#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <gpio/gpio.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

/** \cond */
#define NAME_MAX_LENGTH 64

char gpio[] = "gpio0";
GpioHandle gpioHandler = NULL;

uint8_t pinBuzzer = 20;
uint8_t pinCargoLock = 21;
uint8_t pinKillSwitchFirst = 22;
uint8_t pinKillSwitchSecond = 27;

bool killSwitchEnabled;
/** \endcond */

/**
 * \~English Sets the mode of specified pin power supply.
 * \param[in] pin Pin to set mode.
 * \param[in] mode Mode. 1 is high, 0 is low.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Устанавливает режим подачи энергии на заданный пин.
 * \param[in] pin Пин, для которого устанавливается режим.
 * \param[in] mode Режим. 1 -- высокий, 0 -- низкий.
 * \return Возвращает 1, если режим был успешно установлен, иначе -- 0.
 */
int setPin(uint8_t pin, bool mode) {
    Retcode rc = GpioOut(gpioHandler, pin, mode);
    if (rcOk != rc) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to set GPIO pin %d to %d (" RETCODE_HR_FMT ")", pin, mode, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int initPeripheryController() {
    char logBuffer[256] = {0};
    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize BSP (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspSetConfig(gpio, "rpi4_bcm2711.default");
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to set BSP config for GPIO %s (" RETCODE_HR_FMT ")", gpio, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = GpioInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize GPIO (" RETCODE_HR_FMT ")", RC_GET_CODE(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initGpioPins() {
    char logBuffer[256] = {0};
    Retcode rc = GpioOpenPort(gpio, &gpioHandler);
    if (rcOk != rc) {
        snprintf(logBuffer, 256, "Failed top open GPIO %s (" RETCODE_HR_FMT ")", gpio, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    uint8_t pins[] = { pinBuzzer, pinCargoLock, pinKillSwitchFirst, pinKillSwitchSecond };
    for (uint8_t pin : pins) {
        rc = GpioSetMode(gpioHandler, pin, GPIO_DIR_OUT);
        if (rcOk != rc) {
            snprintf(logBuffer, 256, "Failed to set GPIO pin %u mode (" RETCODE_HR_FMT ")", pin, RETCODE_HR_PARAMS(rc));
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
    }

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}

int setBuzzer(bool enable) {
    return setPin(pinBuzzer, enable);
}

int takePicture(char* picture) {
    picture[0] = '\0';
    return 0;
}

int setKillSwitch(bool enable) {
    if (enable) {
        if (!publishMessage("api/events", "type=kill_switch&event=OFF"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
        if (!setPin(pinKillSwitchFirst, false) || !setPin(pinKillSwitchSecond, true))
            return 0;
        else {
            killSwitchEnabled = true;
            return 1;
        }
    }
    else {
        if (!publishMessage("api/events", "type=kill_switch&event=ON"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
        if (!setPin(pinKillSwitchFirst, false) || !setPin(pinKillSwitchSecond, false))
            return 0;
        else {
            killSwitchEnabled = false;
            return 1;
        }
    }
}

int setCargoLock(bool enable) {
    if (!publishMessage("api/events", enable ? "type=cargo_lock&event=OFF" : "type=kill_switch&event=ON"))
        logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
    return setPin(pinCargoLock, enable);
}
