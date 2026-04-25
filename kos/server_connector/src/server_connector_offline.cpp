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
        if (messageSize < 679) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H55.6472042_37.5162538_0.24&T1.5&W55.6471502_37.5163809_1.5&W55.6471717_37.5164097_1.5&W55.6472122_37.5163143_1.5&W55.6472374_37.516308_1.5&W55.6472455_37.516289_1.5&W55.647267_37.5163177_1.5&W55.6472292_37.5164067_1.5&W55.6472579_37.516445_1.5&W55.6472525_37.5164577_1.5&D5.0&W55.6472651_37.5164546_1.5&D5.0&W55.6472633_37.5164323_1.5&D5.0&W55.647292_37.5164706_1.5&W55.6473298_37.5163816_1.5&W55.6473155_37.5163625_1.5&L55.6472042_37.5162538_0.24&I55.6472525_37.5164577_0.0&I55.6472651_37.5164546_0.0&I55.6472633_37.5164323_0.0&I55.647265_37.5165342_0.0&I55.6472094_37.5164003_0.0&I55.6472499_37.5163845_0.0&I55.647294_37.5163337_0.0&I55.6472581_37.5162858_0.0#", 679);
#else
        if (messageSize < 426) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H55.647215_37.5162284_0.0&T1.5&W55.6471502_37.5163809_1.5&W55.6471717_37.5164097_1.5&W55.6472122_37.5163143_1.5&W55.6472374_37.516308_1.5&W55.6472455_37.516289_1.5&W55.647267_37.5163177_1.5&W55.6472292_37.5164067_1.5&W55.6472507_37.5164355_1.5&W55.6472579_37.516445_1.5&D3.0&S5.0_1200.0&D1.0&S5.0_1800.0&W55.6472705_37.5164419_1.5&W55.647292_37.5164706_1.5&W55.6473298_37.5163816_1.5&L55.647215_37.5162284_0.0#", 426);
#endif
        missionSend = false;
    }
    else if (strstr(topic, "api/forbidden_zones") && areasSend) {
        if (messageSize < 1344) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$ForbiddenZones 6&outerOne&7&55.6472685_37.5165788&55.6472739_37.5165661&55.6471376_37.5163841&55.6472133_37.5162061&55.6472061_37.5161965&55.6471250_37.5163872&55.6472685_37.5165788&outerTwo&7&55.6472739_37.5165661&55.6472667_37.5165565&55.6473424_37.5163785&55.6472061_37.5161965&55.6472115_37.5161838&55.6473550_37.5163754&55.6472739_37.5165661&innerOne&9&55.6472420_37.5162444&55.6472312_37.5162698&55.6472060_37.5162761&55.6471628_37.5163778&55.6471699_37.5163874&55.6472078_37.5162984&55.6472329_37.5162921&55.6472492_37.5162540&55.6472420_37.5162444&innerTwo&8&55.6471735_37.5164320&55.6472167_37.5163302&55.6472419_37.5163240&55.6472311_37.5163494&55.6472185_37.5163525&55.6471860_37.5164288&55.6471878_37.5164511&55.6471735_37.5164320&innerThree&11&55.6472775_37.5165311&55.6472273_37.5164640&55.6472381_37.5164386&55.6472453_37.5164482&55.6472399_37.5164609&55.6472614_37.5164896&55.6472722_37.5164642&55.6472794_37.5164738&55.6472686_37.5164992&55.6472830_37.5165184&55.6472775_37.5165311&innerFour&15&55.6473137_37.5163402&55.6473029_37.5163656&55.6473119_37.5163975&55.6473064_37.5164102&55.6472777_37.5163719&55.6472669_37.5163973&55.6472956_37.5164356&55.6472902_37.5164483&55.6472615_37.5164100&55.6472561_37.5164227&55.6472489_37.5164132&55.6472760_37.5163496&55.6472957_37.5163560&55.6473065_37.5163306&55.6473137_37.5163402#", 1344);
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