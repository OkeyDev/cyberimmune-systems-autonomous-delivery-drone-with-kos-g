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
#include <camera/camera.h>
#include <gpio/gpio.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <filesystem>
#include <fstream>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

/** \cond */
#define NAME_MAX_LENGTH 64
#define CAMERA_PICTURE_WIDTH 1280
#define CAMERA_PICTURE_HEIGHT 720

char gpio[] = "gpio0";
GpioHandle gpioHandler = NULL;

char cameraUsb[16] = {0};
CameraHandle cameraHandler = NULL;

uint8_t pinBuzzer = 20;
uint8_t pinCargoLock = 21;
uint8_t pinKillSwitchFirst = 22;
uint8_t pinKillSwitchSecond = 27;

bool killSwitchEnabled;
/** \endcond */

uint32_t calculateRealSize(uint8_t* buffer, uint32_t maxSize) {
    char logBuffer[256] = {0};

    int i = 0;
	while (true) {
        if ((buffer[i] != 0xFF) || ((i + 1) >= maxSize))
            return 0;

        switch (buffer[i + 1]) {
            case 0x01:
            case 0xD0:
            case 0xD1:
            case 0xD2:
            case 0xD3:
            case 0xD4:
            case 0xD5:
            case 0xD6:
            case 0xD7:
            case 0xD8:
                i += 2;
                break;
            case 0xD9:
                i += 2;
                return i;
            case 0xDA:
                i += 2;
                while (true) {
                    if (i >= maxSize)
                        return 0;

                    if ((buffer[i] == 0xFF) && ((i + 1) < maxSize) && (buffer[i + 1] != 0x00))
                        break;
                    else
                        i++;
                }
                break;
            default:
                i += 2;
                if ((i + 2) >= maxSize)
                    return 0;

				i += (((uint16_t)(buffer[i]) << 8) | buffer[i + 1]);
				break;
        }
    }
	return 0;
}

void base64Encode(uint8_t* src, uint32_t nbytes, char* out) {
	char k[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	uint32_t i = 0;
	uint32_t o = 0;
	for (; i + 3 <= nbytes; i += 3) {
		uint32_t v = (uint32_t(src[i]) << 16) | (uint32_t(src[i + 1]) << 8) | uint32_t(src[i + 2]);
		out[o++] = k[(v >> 18) & 0x3F];
		out[o++] = k[(v >> 12) & 0x3F];
		out[o++] = k[(v >> 6) & 0x3F];
		out[o++] = k[v & 0x3F];
	}

	uint32_t rem = nbytes - i;
	if (rem == 1) {
		uint32_t v = (uint32_t(src[i]) << 16);
		out[o++] = k[(v >> 18) & 0x3F];
		out[o++] = k[(v >> 12) & 0x3F];
		out[o++] = '=';
		out[o++] = '=';
	} else if (rem == 2) {
		uint32_t v = (uint32_t(src[i]) << 16) | (uint32_t(src[i + 1]) << 8);
		out[o++] = k[(v >> 18) & 0x3F];
		out[o++] = k[(v >> 12) & 0x3F];
		out[o++] = k[(v >> 6) & 0x3F];
		out[o++] = '=';
	}

	out[o] = '\0';
}

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

#ifdef IS_INSPECTOR
    rc = CameraInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize camera (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
#endif

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

int initCamera() {
#ifdef IS_INSPECTOR
    char logBuffer[256] = {0};
    while (CameraEnumPorts(0, 64, cameraUsb) == rcResourceNotFound) {
        logEntry("Failed to get camera port. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    Retcode rc = CameraOpenPort(cameraUsb, &cameraHandler);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to open camera port (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    int iter = 0;
    int profileIndex = -1;
    CameraInfo cameraInfo;
    while ((rc = CameraEnumProfiles(cameraHandler, iter, &cameraInfo)) == rcOk) {
        if ((cameraInfo.pFmt == CAMERA_PIX_FMT_MJPEG) && (cameraInfo.resX == CAMERA_PICTURE_WIDTH) && (cameraInfo.resY == CAMERA_PICTURE_HEIGHT)) {
            profileIndex = iter;
            break;
        }
        iter++;
    }
    if (profileIndex < 0) {
        logEntry("Failed to get appropriate camera profile", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    rc = CameraSelectProfile(cameraHandler, profileIndex);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to select camera profile (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    rc = CameraEnable(cameraHandler);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to enable camera (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    sleep(3);

    std::filesystem::create_directories("pictures");
#endif

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}

int setBuzzer(bool enable) {
    return setPin(pinBuzzer, enable);
}

int takePicture(std::string& picture) {
#ifdef IS_INSPECTOR
    char logBuffer[256] = {0};

    RtlTimeSpec timestamp;
    uint32_t maxSize = CAMERA_PICTURE_WIDTH * CAMERA_PICTURE_HEIGHT * 3;
    uint8_t* data = (uint8_t*)malloc(maxSize);
    Retcode rc = CameraReadFrame(cameraHandler, data, maxSize, &timestamp);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to read frame from camera (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    uint32_t dataSize = calculateRealSize(data, maxSize);
    if (!dataSize) {
        logEntry("Failed to calculate real image size", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    char filename[32] = {0};
    snprintf(filename, 32, "pictures/img_%ld_%d.jpg",
        timestamp.sec, timestamp.nsec / RTL_NSEC_PER_MSEC);
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        snprintf(logBuffer, 256, "Could not open file '%s'", filename);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    file.write((char*)data, dataSize);
    file.close();

    uint32_t base64Size = (int)(ceil(dataSize / 3.0f)) * 4 + 1;
    char* base64Data = (char*)malloc(base64Size);
    base64Encode(data, dataSize, base64Data);
    picture = std::string(base64Data);

    free(data);
    free(base64Data);
#endif

    return 1;
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
