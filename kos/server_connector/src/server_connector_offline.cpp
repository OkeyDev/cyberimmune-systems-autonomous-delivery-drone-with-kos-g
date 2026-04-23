/**
 * \file
 * \~English
 * \brief Implementation of methods for ATM server communication simulation.
 * \details The file contains implementation of methods, that simulate
 * requests to an ATM server send and received responses process.
 *
 * \~Russian
 * \brief Реализация методов для имитации общения с сервером ОРВД.
 * \details В файле реализованы методы, имитирующие отправку запросов на сервер ОРВД
 * и обработку полученных ответов.
 */

#include "../include/server_connector.h"

#include <stdio.h>
#include <string.h>

int flightStatusSend, missionSend, areasSend, armSend, newMissionSend, scannedImage, sentTag;

int initServerConnector() {
    if (strlen(BOARD_ID))
        setBoardName(BOARD_ID);
    else
        setBoardName("00:00:00:00:00:00");

    flightStatusSend = true;
    missionSend = true;
    areasSend = true;
    armSend= false;
    newMissionSend = false;
    scannedImage = 0;
    sentTag = 0;

    return 1;
}

int requestServer(char* query, char* response, uint32_t responseSize) {
    if (strstr(query, "/api/auth?")) {
        if (responseSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Success#", 10);
    }
    else {
        if (responseSize < 3) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$#", 3);
    }

    return 1;
}

int publish(char* topic, const std::string& publication) {
    if (strstr(topic, "api/arm/request"))
        armSend = true;
    else if (strstr(topic, "api/nmission/request"))
        newMissionSend = true;
    else if (strstr(topic, "api/image/request")) {
        if (publication.find("image=picture1") != std::string::npos)
            scannedImage = 1;
        else if (publication.find("image=picture2") != std::string::npos)
            scannedImage = 2;
        else if (publication.find("image=picture3") != std::string::npos)
            scannedImage = 3;
        else
            scannedImage = 4;
    }
    else if (strstr(topic, "api/tag/request")) {
        if (publication.find("tag=A1") != std::string::npos)
            sentTag = 1;
        else if (publication.find("tag=A2") != std::string::npos)
            sentTag = 2;
        else if (publication.find("tag=A3") != std::string::npos)
            sentTag = 3;
        else
            sentTag = 4;
    }

    return 1;
}

int getSubscription(char* topic, char* message, uint32_t messageSize) {
    if (strstr(topic, "ping/")) {
        if (messageSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Delay 1#", 10);
    }
    else if (strstr(topic, "api/flight_status/") && flightStatusSend) {
        if (messageSize < 11) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Flight 0#", 11);
        flightStatusSend = false;
    }
    else if (strstr(topic, "api/fmission_kos/") && missionSend) {
#ifdef IS_INSPECTOR
        if (messageSize < 398) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H60.0024896_27.8573379_0.0&T1.0&W60.0024805_27.8574167_1.0&W60.0025112_27.8574308_1.0&W60.0025011_27.8575184_1.0&W60.0024398_27.8574901_1.0&W60.0024307_27.8575689_1.0&W60.0024368_27.8576362_1.0&D3.0&W60.0024166_27.8576914_1.0&D3.0&W60.0024428_27.8577035_1.0&D3.0&W60.002457_27.857581_1.0&L0.0_0.0_0.0&I60.0024368_27.8576362_0.0&I60.0024166_27.8576914_0.0&I60.0024428_27.8577035_0.0#", 398);
#else
        if (messageSize < 337) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H60.002459_27.8573238_0.0&T1.0&W60.0024896_27.8573379_1.0&W60.0024805_27.8574167_1.0&W60.0025112_27.8574308_1.0&W60.0025011_27.8575184_1.0&W60.0024398_27.8574901_1.0&W60.0024307_27.8575689_1.0&W60.0024327_27.8576712_1.0&D3.0&S5.0_1200.0&D1.0&S5.0_1800.0&W60.0024809_27.8576935_1.0&W60.0024778_27.8577197_1.0&L0.0_0.0_0.0#", 337);
#endif
        missionSend = false;
    }
    else if (strstr(topic, "api/forbidden_zones") && areasSend) {
        if (messageSize < 871) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$ForbiddenZones 2&outerOne&15&60.0025472_27.8573184&60.0024422_27.8572699&60.0023916_27.8577076&60.0024004_27.8577116&60.0024408_27.8573615&60.0024671_27.8573736&60.002457_27.8574611&60.002492_27.8574773&60.002494_27.8574598&60.0024677_27.8574477&60.0024778_27.8573601&60.0024428_27.857344&60.0024489_27.8572914&60.0025452_27.8573359&60.0025472_27.8573184&outerTwo&23&60.0025364_27.8573319&60.0025304_27.8573844&60.0024954_27.8573682&60.0024933_27.8573857&60.0025283_27.8574019&60.0025102_27.8575595&60.0024489_27.8575312&60.0024469_27.8575487&60.0024731_27.8575608&60.002461_27.8576658&60.0024698_27.8576699&60.0024819_27.8575648&60.0025081_27.857577&60.0024879_27.857752&60.0024617_27.8577399&60.0024657_27.8577049&60.002457_27.8577009&60.0024529_27.8577359&60.0023916_27.8577076&60.0023896_27.8577251&60.0024947_27.8577736&60.0025452_27.8573359&60.0025364_27.8573319#", 871);
        areasSend = false;
    }
    else if (strstr(topic, "api/arm/response/") && armSend) {
        if (messageSize < 16) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Arm 0$Delay 1#", 16);
    }
    else if (strstr(topic, "api/nmission/response/") && newMissionSend) {
        if (messageSize < 13) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Approve 0#", 13);
    }
    else if (strstr(topic, "api/image/response/") && scannedImage) {
        if (messageSize < 25) {
            scannedImage = 0;
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }

        if (scannedImage == 1)
            strncpy(message, "result=A1&rec_alt=NONE", 23);
        else if (scannedImage == 2)
            strncpy(message, "result=A2&rec_alt=NONE", 23);
        else if (scannedImage == 3)
            strncpy(message, "result=A3&rec_alt=NONE", 23);
        else
            strncpy(message, "result=NONE&rec_alt=2", 25);
        scannedImage = 0;
    }
    else if (strstr(topic, "api/tag/response/") && sentTag) {
        if (messageSize < 15) {
            sentTag = 0;
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }

        if (sentTag == 1)
            strncpy(message, "$FALSE A1#", 11);
        else if (sentTag == 2)
            strncpy(message, "$TRUE A2#", 10);
        else if (sentTag == 3)
            strncpy(message, "$FALSE A3#", 11);
        else
            strncpy(message, "$FALSE TAG#", 15);
        sentTag = 0;
    }
    else
        strcpy(message, "");

    return 1;
}