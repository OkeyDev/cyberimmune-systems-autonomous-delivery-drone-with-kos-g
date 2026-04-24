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
        else if (publication.find("image=picture4") != std::string::npos)
            scannedImage = 4;
        else if (publication.find("image=picture5") != std::string::npos)
            scannedImage = 5;
        else if (publication.find("image=picture6") != std::string::npos)
            scannedImage = 6;
        else if (publication.find("image=picture7") != std::string::npos)
            scannedImage = 7;
        else if (publication.find("image=picture8") != std::string::npos)
            scannedImage = 8;
        else
            scannedImage = 9;
    }
    else if (strstr(topic, "api/tag/request")) {
        if (publication.find("tag=A1") != std::string::npos)
            sentTag = 1;
        else if (publication.find("tag=A2") != std::string::npos)
            sentTag = 2;
        else if (publication.find("tag=A3") != std::string::npos)
            sentTag = 3;
        else if (publication.find("tag=E1") != std::string::npos)
            sentTag = 4;
        else if (publication.find("tag=E2") != std::string::npos)
            sentTag = 5;
        else if (publication.find("tag=E3") != std::string::npos)
            sentTag = 6;
        else if (publication.find("tag=E4") != std::string::npos)
            sentTag = 7;
        else if (publication.find("tag=E5") != std::string::npos)
            sentTag = 8;
        else
            sentTag = 9;
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
        if (messageSize < 670) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H55.6471502_37.5163809_0.0&T1.0&W55.6472150_37.5162284_1.0&W55.6472366_37.5162571_1.0&W55.6471960_37.5163524_1.0&W55.6471996_37.5163970_1.0&W55.6471915_37.5164161_1.0&W55.6472130_37.5164449_1.0&W55.6472508_37.5163559_1.0&W55.6472795_37.5163942_1.0&W55.6472849_37.5163815_1.0&D5.0&W55.6472867_37.5164038_1.0&D5.0&W55.6472741_37.5164069_1.0&D5.0&W55.6473028_37.5164452_1.0&W55.6472650_37.5165342_1.0&W55.6472506_37.5165150_1.0&L0.0_0.0_0.0&I55.6472849_37.5163815_0.0&I55.6472867_37.5164038_0.0&I55.6472741_37.5164069_0.0&I55.6473298_37.5163816_0.0&I55.6472419_37.5163240_0.0&I55.6472445_37.5163972_0.0&I55.6472291_37.5164863_0.0&I55.6471932_37.5164384_0.0#", 670);
#else
        if (messageSize < 446) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H55.6471645_37.5164001_0.0&T1.0&W55.6471502_37.5163809_1.0&W55.6472150_37.5162284_1.0&W55.6472366_37.5162571_1.0&W55.6471960_37.5163524_1.0&W55.6471996_37.5163970_1.0&W55.6471915_37.5164161_1.0&W55.6472130_37.5164449_1.0&W55.6472508_37.5163559_1.0&W55.6472723_37.5163846_1.0&W55.6472795_37.5163942_1.0&D3.0&S5.0_1200.0&D1.0&S5.0_1800.0&W55.6472813_37.5164165_1.0&W55.6473028_37.5164452_1.0&W55.6472650_37.5165342_1.0&L0.0_0.0_0.0#", 446);
#endif
        missionSend = false;
    }
    else if (strstr(topic, "api/forbidden_zones") && areasSend) {
        if (messageSize < 1332) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$ForbiddenZones 6&outerOne&7&55.6472685_37.5165788&55.6472739_37.5165661&55.6471376_37.5163841&55.6472133_37.5162061&55.6472061_37.5161965&55.647125_37.5163872&55.6472685_37.5165788&outerTwo&7&55.6472739_37.5165661&55.6472667_37.5165565&55.6473424_37.5163785&55.6472061_37.5161965&55.6472115_37.5161838&55.647355_37.5163754&55.6472739_37.5165661&innerOne&9&55.6471663_37.5164224&55.6471771_37.5163969&55.6471736_37.5163524&55.6472168_37.5162506&55.647224_37.5162602&55.6471861_37.5163492&55.6471897_37.5163938&55.6471735_37.516432&55.6471663_37.5164224&innerTwo&8&55.6472492_37.516254&55.6472059_37.5163557&55.6472094_37.5164003&55.6472203_37.5163748&55.6472185_37.5163525&55.6472509_37.5162763&55.6472635_37.5162731&55.6472492_37.516254&innerThree&11&55.6473316_37.5164039&55.6472814_37.5163369&55.6472706_37.5163623&55.6472777_37.5163719&55.6472832_37.5163592&55.6473047_37.5163879&55.6472939_37.5164133&55.647301_37.5164229&55.6473119_37.5163975&55.6473262_37.5164167&55.6473316_37.5164039&innerFour&15&55.647238_37.5165182&55.6472488_37.5164928&55.6472686_37.5164992&55.647274_37.5164865&55.6472453_37.5164482&55.6472561_37.5164227&55.6472848_37.5164611&55.6472902_37.5164483&55.6472615_37.51641&55.6472669_37.5163973&55.6472598_37.5163877&55.6472327_37.5164513&55.6472417_37.5164832&55.6472309_37.5165086&55.647238_37.5165182#", 1332);
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
        else if (scannedImage == 4)
            strncpy(message, "result=E1&rec_alt=NONE", 23);
        else if (scannedImage == 5)
            strncpy(message, "result=E2&rec_alt=NONE", 23);
        else if (scannedImage == 6)
            strncpy(message, "result=E3&rec_alt=NONE", 23);
        else if (scannedImage == 7)
            strncpy(message, "result=E4&rec_alt=NONE", 23);
        else if (scannedImage == 8)
            strncpy(message, "result=E5&rec_alt=NONE", 23);
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
        else if (sentTag == 4)
            strncpy(message, "$ACCEPTED E1#", 14);
        else if (sentTag == 5)
            strncpy(message, "$ACCEPTED E2#", 14);
        else if (sentTag == 6)
            strncpy(message, "$ACCEPTED E3#", 14);
        else if (sentTag == 7)
            strncpy(message, "$ACCEPTED E4#", 14);
        else if (sentTag == 8)
            strncpy(message, "$ACCEPTED E5#", 14);
        else
            strncpy(message, "$FALSE TAG#", 15);
        sentTag = 0;
    }
    else
        strcpy(message, "");

    return 1;
}