/**
 * \file
 * \~English \brief Implementation of the security module FlightController
 * component main loop.
 * \~Russian \brief Реализация основного цикла компонента FlightController
 * модуля безопасности.
 */

#include "../include/drone_defender_system.h"

/** \cond */
#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000
// Радиус (в м) вокруг точки для запуска разблокировки грузчика
#define CARGOLOCK_ENABLE_DISTANCE 1

char boardId[32] = {0};
uint32_t sessionDelay;
std::thread sessionThread, updateThread;
/** \endcond */

#ifndef ENTITY_NAME
#define ENTITY_NAME "MAIN_DELIVERER"
#endif

/**
 * \~English Procedure that checks connection to the ATM server.
 * \~Russian Процедура, проверяющая наличие соединения с сервером ОРВД.
 */
void pingSession() {
  sleep(sessionDelay);

  char pingMessage[1024] = {0};
  while (true) {
    if (!receiveSubscription("ping/", pingMessage, 1024)) {
      logEntry("Failed to receive ping through Server Connector", ENTITY_NAME,
               LogLevel::LOG_WARNING);
      continue;
    }

    if (strcmp(pingMessage, "")) {
      uint8_t authenticity = 0;
      if (!checkSignature(pingMessage, MessageSource::SERVER_ORVD,
                          authenticity) ||
          !authenticity) {
        logEntry("Failed to check signature of ping received through Server "
                 "Connector",
                 ENTITY_NAME, LogLevel::LOG_WARNING);
        continue;
      }

      // Processing delay until next session
      sessionDelay = parseDelay(strstr(pingMessage, "$Delay "));
    } else {
      // No response from the server
      // If server does not respond for 3 more seconds, flight must be paused
      // until the response is received
    }

    sleep(sessionDelay);
  }
}

/**
 * \~English Procedure that tracks flight status and no flight areas changes.
 * \~Russian Процедура, отслеживающая изменение статуса полета и запретных зон.
 */
void serverUpdateCheck() {
  char message[4096] = {0};

  while (true) {
    if (receiveSubscription("api/flight_status/", message, 4096)) {
      if (strcmp(message, "")) {
        uint8_t authenticity = 0;
        if (checkSignature(message, MessageSource::SERVER_ORVD, authenticity) ||
            !authenticity) {
          if (strstr(message, "$Flight -1#")) {
            logEntry("Emergency stop request is received. Disabling motors",
                     ENTITY_NAME, LogLevel::LOG_INFO);
            if (!enableBuzzer())
              logEntry("Failed to enable buzzer", ENTITY_NAME,
                       LogLevel::LOG_WARNING);
            while (!setKillSwitch(false)) {
              logEntry("Failed to forbid motor usage. Trying again in 1s",
                       ENTITY_NAME, LogLevel::LOG_WARNING);
              sleep(1);
            }
          }
          // The message has two other possible options:
          //   "$Flight 1#" that requires to pause flight and remain landed
          //   "$Flight 0#" that requires to resume flight and keep flying
          // Implementation is required to be done
        } else
          logEntry("Failed to check signature of flight status received "
                   "through Server Connector",
                   ENTITY_NAME, LogLevel::LOG_WARNING);
      }
    } else
      logEntry("Failed to receive flight status through Server Connector",
               ENTITY_NAME, LogLevel::LOG_WARNING);

    if (receiveSubscription("api/forbidden_zones", message, 4096)) {
      if (strcmp(message, "")) {
        uint8_t authenticity = 0;
        if (checkSignature(message, MessageSource::SERVER_ORVD, authenticity) ||
            !authenticity) {
          deleteNoFlightAreas();
          loadNoFlightAreas(message);
          logEntry("New no-flight areas are received from the server",
                   ENTITY_NAME, LogLevel::LOG_INFO);
          printNoFlightAreas();
          // Path recalculation must be done if current path crosses new
          // no-flight areas
        } else
          logEntry("Failed to check signature of no-flight areas received "
                   "through Server Connector",
                   ENTITY_NAME, LogLevel::LOG_WARNING);
      }
    } else
      logEntry("Failed to receive no-flight areas through Server Connector",
               ENTITY_NAME, LogLevel::LOG_WARNING);

    sleep(1);
  }
}

bool cargoZoneReached = true;
void handleCargoLock(Coordinates *drone, MissionCommand *interestPoints,
                     int count) {

  MissionCommand *current = nullptr;
  for (int i = 0; i < count; i++) {
    if (isWaypointReached(drone, interestPoints[i].content.waypoint,
                          CARGOLOCK_ENABLE_DISTANCE)) {
      current = interestPoints + i;
      break;
    }
  }

  // Два переходных состояния
  // Если нет точки сброса рядом, но она была
  if (current == nullptr && cargoZoneReached) {
    logEntry("Cargo lock set to 0. You can't drop your cargo", ENTITY_NAME,
             LogLevel::LOG_WARNING);

    if (!setCargoLock(0)) {
      logEntry("Failed to set cargo Lock to 0", ENTITY_NAME,
               LogLevel::LOG_ERROR);
    } else {
      cargoZoneReached = false;
    }
    // Если точка сброса есть, но её не было
  } else if (current != nullptr && !cargoZoneReached) {
    logEntry("You are inside drop zone. Cargo lock set to 1", ENTITY_NAME,
             LogLevel::LOG_WARNING);

    if (!setCargoLock(1)) {
      logEntry("Failed to set cargo Lock to 1", ENTITY_NAME,
               LogLevel::LOG_ERROR);
    } else {
      cargoZoneReached = true;
    }
  }
}

/**
 * \~English Auxiliary procedure. Asks the ATM server to approve new mission and
 * parses its response.
 * \param[in] mission New mission in string format.
 * \param[out] result ATM server response: 1 if mission approved, 0 otherwise.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Вспомогательная процедура. Просит у сервера ОРВД одобрения новой
 * миссии и обрабатывает ответ.
 * \param[in] mission Новая миссия в виде строки.
 * \param[out] result Ответ сервера ОРВД: 1 при одобрении миссии, иначе -- 0.
 * \return Возвращает 1 при успешной отправке, иначе -- 0.
 */
int askForMissionApproval(char *mission, int &result) {
  int messageSize = 512 + strlen(mission);
  char *message = (char *)malloc(messageSize);
  char signature[257] = {0};

  // Creating a string that will be used to calculate the signature
  snprintf(message, messageSize, "/api/nmission?id=%s&mission=%s", boardId,
           mission);
  // Calculating a signature
  if (!signMessage(message, signature, 257)) {
    logEntry("Failed to sign new mission at Credential Manager", ENTITY_NAME,
             LogLevel::LOG_WARNING);
    free(message);
    return 0;
  }

  // Creating an actual message
  snprintf(message, messageSize, "mission=%s&sig=0x%s", mission, signature);
  // Publishing message to a certain topic
  if (!publishMessage("api/nmission/request", message)) {
    logEntry("Failed to publish new mission through Server Connector",
             ENTITY_NAME, LogLevel::LOG_WARNING);
    free(message);
    return 0;
  }

  // Waiting for a non-empty message from the specified topic
  while (!receiveSubscription("api/nmission/response/", message, 512) ||
         !strcmp(message, ""))
    sleep(1);

  uint8_t authenticity = 0;
  // Checking the signature of the received message
  if (!checkSignature(message, MessageSource::SERVER_ORVD, authenticity) ||
      !authenticity) {
    logEntry("Failed to check signature of new mission received through Server "
             "Connector",
             ENTITY_NAME, LogLevel::LOG_WARNING);
    free(message);
    return 0;
  }

  // Parsing the content of the received message
  if (strstr(message, "$Approve 0#") != NULL)
    result = 1;
  else if (strstr(message, "$Approve 1#") != NULL)
    result = 0;
  else {
    logEntry("Failed to parse server response on New Mission request",
             ENTITY_NAME, LogLevel::LOG_WARNING);
    free(message);
    return 0;
  }

  free(message);
  return 1;
}

int missionIndex = 0;
void writeWaypoint(MissionCommand* command, int32_t latitude, int32_t longitude, int32_t altitude) 
{
  (command + missionIndex)->type = CommandType::WAYPOINT;
  (command + missionIndex)->content.waypoint.latitude = latitude;
  (command + missionIndex)->content.waypoint.longitude = longitude;
  (command + missionIndex)->content.waypoint.altitude = altitude;

  missionIndex++;
}

void writeLand(MissionCommand* command) 
{
  (command + missionIndex)->type = CommandType::LAND;

  missionIndex++;
}

void writeDelay(MissionCommand* command, int32_t delay) 
{
  (command + missionIndex)->type = CommandType::DELAY;
  (command + missionIndex)->content.delay.delay = delay;

  missionIndex++;
} 

void writeTakeoff(MissionCommand* command, int32_t altitude) 
{
  (command + missionIndex)->type = CommandType::TAKEOFF;
  (command + missionIndex)->content.takeoff.altitude = altitude;

  missionIndex++;
}

void writeHome(MissionCommand* command, int32_t latitude, int32_t longitude, int32_t altitude) 
{
  (command + missionIndex)->type = CommandType::HOME;
  (command + missionIndex)->content.waypoint.latitude = latitude;
  (command + missionIndex)->content.waypoint.longitude = longitude;
  (command + missionIndex)->content.waypoint.altitude = altitude;

  missionIndex++;
}

void writeServo(MissionCommand* command, int32_t servoId, int32_t pwm) 
{
  (command + missionIndex)->type = CommandType::SET_SERVO;
  (command + missionIndex)->content.servo.number = servoId;
  (command + missionIndex)->content.servo.pwm = pwm;

  missionIndex++;
}

void createCustomMission() {

/*
  QGC WPL 110
0	1	0	16	0	0	0	0	55.6472150	37.5162284	0.000000	1
1	0	3	22	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	1.500000	1
2	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64715020	37.51638090	1.500000	1
3	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64717170	37.51640970	1.500000	1
4	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64721220	37.51631430	1.500000	1
5	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64723740	37.51630800	1.500000	1
6	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64724550	37.51628900	1.500000	1
7	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64726700	37.51631770	1.500000	1
8	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64722920	37.51640670	1.500000	1
9	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64725070	37.51643550	1.500000	1
10	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64725790	37.51644500	1.500000	1
11	0	3	93	3.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
12	0	3	183	5.00000000	1200.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
13	0	3	93	1.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
14	0	3	183	5.00000000	1800.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
15	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64727050	37.51644190	1.500000	1
16	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64729200	37.51647060	1.500000	1
17	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	55.64732980	37.51638160	1.500000	1
18	0	3	21	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
  */
  int32_t missionCommandCount = 19;
  MissionCommand *commands =
      (MissionCommand *)malloc(sizeof(MissionCommand) * missionCommandCount);
  writeHome(commands, 5564721500, 3751622840, 0);
  writeTakeoff(commands, 150);
  writeWaypoint(commands, 5564715020, 3751638090, 150);
  writeWaypoint(commands, 5564717170, 3751640970, 150);
  writeWaypoint(commands, 5564721220, 3751631430, 150);
  writeWaypoint(commands, 5564723740, 3751630800, 150);
  writeWaypoint(commands, 5564724550, 3751628900, 150);
  writeWaypoint(commands, 5564726700, 3751631770, 150);
  writeWaypoint(commands, 5564722920, 3751640670, 150);
  writeWaypoint(commands, 5564725070, 3751643550, 150);
  writeWaypoint(commands, 5564725790, 3751644500, 150);
  writeDelay(commands, 1);
  writeServo(commands, 5, 1200);
  writeDelay(commands, 1);
  writeServo(commands, 5, 1800);
  writeWaypoint(commands, 5564727050, 3751644190, 150);
  writeWaypoint(commands, 5564729200, 3751647060, 150);
  writeWaypoint(commands, 5564732980, 3751638160, 150);
  writeLand(commands);

  char missionBuffer[4096] = {0};
  missionToString(commands, missionCommandCount, missionBuffer, 4096);
  logEntry("Mission:", ENTITY_NAME, LogLevel::LOG_INFO);
  logEntry(missionBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
  int result = 0;
  askForMissionApproval(missionBuffer, result);

  while (!result) {
    logEntry("Failed to aprove custom mission from orvd", ENTITY_NAME,
             LogLevel::LOG_INFO);
    askForMissionApproval(missionBuffer, result);
  }
  logEntry("Successful for aproving mission from orvd", ENTITY_NAME,
           LogLevel::LOG_INFO);

  uint32_t missionSize = getMissionBytesSize(commands, missionCommandCount);
  uint8_t *bytes = (uint8_t *)malloc(missionSize);
  missionToBytes(commands, missionCommandCount, bytes);
  setMission(bytes, missionSize);
}

void receiveInspectorMessage() {
  char topicBuffer[256] = {0};
  char subscriptionBuffer[4096] = {0};
  char logBuffer[256] = {0};

  snprintf(topicBuffer, sizeof(topicBuffer), "api/dm/%s/%s", boardId,
           PARTNER_ID);
  while (!receiveSubscription(topicBuffer, subscriptionBuffer,
                              sizeof(subscriptionBuffer)) ||
         !strcmp(subscriptionBuffer, "")) {
    sleep(1);
  }
  logEntry(subscriptionBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);

  uint8_t authenticity = 0;
  while (!checkSignature(subscriptionBuffer, MessageSource::PARTNER_DRONE,
                         authenticity) ||
         !authenticity) {
    snprintf(
        logBuffer, 256,
        "Failed to check signature of message response. Trying again in %ds",
        RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(RETRY_DELAY_SEC);
  }
  logEntry("Successful receive message from the parther drone", ENTITY_NAME,
           LogLevel::LOG_INFO);
}

/**
 * \~English Security module main loop. Waits for all other components to
 * initialize. Authenticates on the ATM server and receives the mission from it.
 * After a mission and an arm request from the autopilot are received, requests
 * permission to take off from the ATM server. On receive supplies power to
 * motors. Then flight control must be performed.
 * \return Returns 1 on completion with no errors.
 * \~Russian Основной цикл модуля безопасности. Ожидает инициализации всех
 * остальных компонентов. Аутентифицируется на сервере ОРВД и получает от него
 * миссию. После получения миссии и запроса на арминг от автопилота, запрашивает
 * разрешение на взлет у сервера ОРВД. При его получении подает питание на
 * двигатели. Далее должен выполняться контроль полета.
 * \return Возвращает 1 при завершении без ошибок.
 */
int main(void) {
  char logBuffer[256] = {0};
  char signBuffer[257] = {0};
  char publicationBuffer[1024] = {0};
  char subscriptionBuffer[4096] = {0};

  // Get ID from ServerConnector
  while (!getBoardId(boardId)) {
    logEntry("Failed to get board ID from ServerConnector. Trying again in 1s",
             ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(1);
  }
  snprintf(logBuffer, 256, "Board '%s' is initialized", boardId);
  logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

  // Copter need to be registered at ORVD
  char authRequest[512] = {0};
  char authSignature[257] = {0};
  snprintf(authRequest, 512, "/api/auth?id=%s", boardId);
  while (!signMessage(authRequest, authSignature, 257)) {
    snprintf(logBuffer, 256,
             "Failed to sign auth message at Credential Manager. Trying again "
             "in %ds",
             RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(RETRY_DELAY_SEC);
  }

  char authResponse[1024] = {0};
  snprintf(authRequest, 512, "%s&sig=0x%s", authRequest, authSignature);
  while (!sendRequest(authRequest, authResponse, 1024) ||
         !strcmp(authResponse, "TIMEOUT")) {
    snprintf(logBuffer, 256,
             "Failed to send auth request through Server Connector. Trying "
             "again in %ds",
             RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(RETRY_DELAY_SEC);
  }

  uint8_t authenticity = 0;
  while (
      !checkSignature(authResponse, MessageSource::SERVER_ORVD, authenticity) ||
      !authenticity) {
    snprintf(logBuffer, 256,
             "Failed to check signature of auth response received through "
             "Server Connector. Trying again in %ds",
             RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(RETRY_DELAY_SEC);
  }
  logEntry("Successfully authenticated on the server", ENTITY_NAME,
           LogLevel::LOG_INFO);

  // Here, the message from the inspector should be received and parsed.
  // Message can be received with
  //     receiveSubscription("api/dm/{boardId}/{PARTNER_ID}",
  //     subscriptionBuffer, bufferSize).
  // If there was no message, the buffer will be empty.
  // Next, the authenticity of the message should be checked with
  //     checkSignature(subscriptionBuffer, MessageSource::PARTNER_DRONE,
  //     authenticity).
  // If authenticity is returned as 'True' than the message was indeed sent by
  // the inspector. Than the message must be parsed: content starts from the
  // first symbol and ends with "#". Mission will not be received from the
  // server. Instead it should be generated here and approved by the server with
  // 'askForMissionApproval' function.

  // Copter need to be registered at ORVD
  receiveInspectorMessage();
  // createCustomMission();

  // The drone is ready to arm
  logEntry("Ready to arm", ENTITY_NAME, LogLevel::LOG_INFO);
  while (true) {
    // Wait, until autopilot wants to arm (and fails so, as motors are disabled
    // by default)
    while (!waitForArmRequest()) {
      snprintf(logBuffer, 256,
               "Failed to receive an arm request from Autopilot Connector. "
               "Trying again in %ds",
               RETRY_DELAY_SEC);
      logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
      sleep(RETRY_DELAY_SEC);
    }
    logEntry("Received arm request. Notifying the server", ENTITY_NAME,
             LogLevel::LOG_INFO);

    // When autopilot asked for arm, we need to receive permission from ORVD
    snprintf(publicationBuffer, 1024, "/api/arm?id=%s", boardId);
    while (!signMessage(publicationBuffer, signBuffer, 257)) {
      snprintf(logBuffer, 256,
               "Failed to sign arm request at Credential Manager. Trying again "
               "in %ds",
               RETRY_DELAY_SEC);
      logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
      sleep(RETRY_DELAY_SEC);
    }

    snprintf(publicationBuffer, 1024, "sig=0x%s", signBuffer);
    while (!publishMessage("api/arm/request", publicationBuffer)) {
      snprintf(logBuffer, 256,
               "Failed to publish arm request through Server Connector. Trying "
               "again in %ds",
               RETRY_DELAY_SEC);
      logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
      sleep(RETRY_DELAY_SEC);
    }

    while (
        !receiveSubscription("api/arm/response/", subscriptionBuffer, 4096) ||
        !strcmp(subscriptionBuffer, ""))
      sleep(1);

    authenticity = 0;
    while (!checkSignature(subscriptionBuffer, MessageSource::SERVER_ORVD,
                           authenticity) ||
           !authenticity) {
      snprintf(logBuffer, 256,
               "Failed to check signature of arm response received through "
               "Server Connector. Trying again in %ds",
               RETRY_DELAY_SEC);
      logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
      sleep(RETRY_DELAY_SEC);
    }

    if (strstr(subscriptionBuffer, "$Arm 0$")) {
      // If arm was permitted, we enable motors
      logEntry("Arm is permitted", ENTITY_NAME, LogLevel::LOG_INFO);
      while (!setKillSwitch(true)) {
        snprintf(logBuffer, 256,
                 "Failed to permit motor usage at Periphery Controller. Trying "
                 "again in %ds",
                 RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
      }
      if (!permitArm())
        logEntry("Failed to permit arm through Autopilot Connector",
                 ENTITY_NAME, LogLevel::LOG_WARNING);
      // Get time until next session
      sessionDelay = parseDelay(strstr(subscriptionBuffer, "$Delay "));
      // Start ORVD threads
      sessionThread = std::thread(pingSession);
      updateThread = std::thread(serverUpdateCheck);
      break;
    } else if (strstr(subscriptionBuffer, "$Arm 1$")) {
      logEntry("Arm is forbidden", ENTITY_NAME, LogLevel::LOG_INFO);
      if (!forbidArm())
        logEntry("Failed to forbid arm through Autopilot Connector",
                 ENTITY_NAME, LogLevel::LOG_WARNING);
    } else
      logEntry("Failed to parse server response", ENTITY_NAME,
               LogLevel::LOG_WARNING);
    logEntry(
        "Arm was not allowed. Waiting for another arm request from autopilot",
        ENTITY_NAME, LogLevel::LOG_WARNING);
  };

  // disable cargo
  setCargoLock(false);
  initDefenderSystem(boardId, ENTITY_NAME, false);
  // If we get here, the drone is able to arm and start the mission
  // The flight is need to be controlled from now on

  while (true) {
    int32_t laltitude, longtitude, altitude;
    int result = getCoords(laltitude, longtitude, altitude);

    if (result) {
      Coordinates coords(laltitude, longtitude, altitude);
      updateDefenderSystem(&coords);

      char *buff;
      asprintf(&buff, "Latitude: %d; Logntitude; %d; Altitude: %d", laltitude,
               longtitude, altitude);
      logEntry(buff, ENTITY_NAME, LogLevel::LOG_INFO);
      free(buff);
    } else {
      logEntry("Error while reading geoCoords", ENTITY_NAME,
               LogLevel::LOG_WARNING);
    }

    sleep(1);
  }

  return EXIT_SUCCESS;
}
