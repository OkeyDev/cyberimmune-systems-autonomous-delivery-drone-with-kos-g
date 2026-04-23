/**
 * \file
 * \~English
 * \brief Implementation of methods for hardware autopilot communication.
 * \details The file contains implementation of methods,
 * that provide interaction between the security module and a
 * hardware flight controller with a compatible ArduPilot firmware.
 *
 * \~Russian
 * \brief Реализация методов для взаимодействия с аппаратным автопилотом.
 * \details В файле реализованы методы, обеспечивающие
 * взаимодействие между модулем безопасности и аппаратным полетным
 * контроллером с совместимой прошивкой ArduPilot.
 */

#include "../include/autopilot_connector.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <bsp/bsp.h>

#include <stdio.h>

/** \cond */
#define NAME_MAX_LENGTH 64

char autopilotUart[] = "uart2";
UartHandle autopilotUartHandler = NULL;
/** \endcond */

int initAutopilotConnector() {
    char logBuffer[256] = {0};
    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize BSP (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspEnableModule(autopilotUart);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to enable UART %s (" RETCODE_HR_FMT ")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize UART (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initConnection() {
    Retcode rc = UartOpenPort(autopilotUart, &autopilotUartHandler);
    if (rc != rcOk) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to open UART %s (" RETCODE_HR_FMT ")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

ssize_t readBytes(uint32_t byteNum, uint8_t* bytes) {
    rtl_size_t readBytes;
    Retcode rc = UartRead(autopilotUartHandler, bytes, byteNum, NULL, &readBytes);
    if (rc != rcOk) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to read from UART %s (" RETCODE_HR_FMT ")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else
        return (ssize_t)(readBytes);
}

int sendAutopilotBytes(uint8_t* bytes, ssize_t size) {
    rtl_size_t writtenBytes;
    Retcode rc = UartWrite(autopilotUartHandler, bytes, size, NULL, &writtenBytes);
    if (rc != rcOk) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to write to UART %s (" RETCODE_HR_FMT ")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (writtenBytes != size) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to write message to autopilot: %ld bytes were expected, %ld bytes were sent", size, writtenBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}