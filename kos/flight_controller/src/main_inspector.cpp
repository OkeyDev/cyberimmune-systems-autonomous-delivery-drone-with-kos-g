/**
 * \file
 * \~English \brief Implementation of the security module FlightController
 * component main loop.
 * \~Russian \brief Реализация основного цикла компонента FlightController
 * модуля безопасности.
 */

#include "../include/drone_defender_system.h"
#include <cstdint>
#include <unistd.h>

/** \cond */
#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

char boardId[32] = {0};
uint32_t sessionDelay;
std::thread sessionThread, updateThread;
std::thread waypointThread, altiduteThread;
/** \endcond */

#ifndef ENTITY_NAME
#define ENTITY_NAME "MAIN_INSPECTOR"
#endif

/**
 * \~English Procedure that checks connection to the ATM server.
 * \~Russian Процедура, проверяющая наличие соединения с сервером ОРВД.
 */

void pingSession() {
  sleep(sessionDelay);

  char unsuccessful_connections_count = 0;
  char pingMessage[1024] = {0};
  bool is_reconnecting = false;
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

      unsuccessful_connections_count = 0;
      if (is_reconnecting) {
        resumeFlight();
        is_reconnecting = false;
      }

      // Processing delay until next session
      sessionDelay = parseDelay(strstr(pingMessage, "$Delay "));
    } else {
      logEntry("Failed to get ping from server. Retrying...", ENTITY_NAME,
               LogLevel::LOG_WARNING);
      // No response from the server
      // If server does not respond for 3 more seconds, flight must be paused
      // until the response is received

      if (is_reconnecting) {
        sleep(1);
        continue;
      }

      unsuccessful_connections_count++;
      if (unsuccessful_connections_count > 3) {
        logEntry("Can't access server. Pause fly now.", ENTITY_NAME,
                 LogLevel::LOG_ERROR);
        while (!pauseFlight()) {
          logEntry("Failed to pause flight after connection loss. ",
                   ENTITY_NAME, LogLevel::LOG_ERROR);
          sleep(0.1);
        };
      }
      sleep(1);
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

void forceSendMessage() {
  char messageBuffer[512];
  char messageSign[257];
  char topicBuffer[128];
  char logBuffer[256];
  char message[] = "HelloWorld";

  while (!signMessage(message, messageSign, sizeof(messageSign))) {
    snprintf(logBuffer, sizeof(logBuffer),
             "Failed to sign message for sending to partner; Retry in %ds",
             RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(RETRY_DELAY_SEC);
  }
  snprintf(messageBuffer, sizeof(messageBuffer), "%s#%s", message, messageSign);
  snprintf(topicBuffer, sizeof(topicBuffer), "api/dm/%s", PARTNER_ID);

  while (!publishMessage(topicBuffer, messageBuffer)) {
    snprintf(logBuffer, sizeof(logBuffer),
             "Failed publish message to %s. Retry in %ds", topicBuffer,
             RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
  }

  snprintf(logBuffer, sizeof(logBuffer), "Message sent to %s", topicBuffer);
  logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
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

  // Enable buzzer to indicate, that all modules has been initialized
  if (!enableBuzzer())
    logEntry("Failed to enable buzzer at Periphery Controller", ENTITY_NAME,
             LogLevel::LOG_WARNING);

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

  // Constantly ask server, if mission for the drone is available. Parse it and
  // ensure, that mission is correct
  while (!receiveSubscription("api/fmission_kos/", subscriptionBuffer, 4096) ||
         !strcmp(subscriptionBuffer, ""))
    sleep(1);

  authenticity = 0;
  while (!checkSignature(subscriptionBuffer, MessageSource::SERVER_ORVD,
                         authenticity) ||
         !authenticity) {
    snprintf(logBuffer, 256,
             "Failed to check signature of mission received through Server "
             "Connector. Trying again in %ds",
             RETRY_DELAY_SEC);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
    sleep(RETRY_DELAY_SEC);
  }

  if (loadMission(subscriptionBuffer)) {
    logEntry("Successfully received mission from the ser?ver", ENTITY_NAME,
             LogLevel::LOG_INFO);
    printMission();
  }

  forceSendMessage();

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

  // If we get here, the drone is able to arm and start the mission
  // The flight is need to be controlled from now on

  /* Tag Recognition */
  // Tag recognition is done via AI-driven recognition server.
  // First, the request should be done with 'requestRecognition' function.
  // It takes picture with a camera and sends it to AI-driven recognition
  // server. The response could be received with 'getRecognitionResponse'. Note,
  // that result may not be ready immediately. It returns recognized symbol or
  // 'NONE' with an altitude value to take a better picture.

  /* Delivery Point Determination */
  // If the symbol is recognized it should be sent to the ORVD server.
  // Calculate a signature for message "/api/tag/request?id={boardId}&tag={Tag}"
  // with 'signMessage'. Send message "tag={Tag}&sig=0x{signature}" to the topic
  // "api/tag/request" with 'publishMessage'. Receive the response from the
  // topic "api/tag/response/" with 'receiveSubscription'. Signature of the
  // received message can be checked with 'checkSignature'. If the response
  // contains "$TRUE {Tag}#" than the tag is cargo destination, if it contains
  // "$FALSE {Tag}#", than the tag is not the destination point (or wrong tag),
  // if it contains "$ACCEPTED {Tag}#", than the bonus tag was accepted.

  /* Send message to deliverer */
  // Calculate a signature for your message {Message} with 'signMessage'.
  // Content of {Message} is not specified and can be in any form.
  // Send message "{Message}#{Signature}" to the topic "api/dm/{PARTNER_ID}"
  // with 'publishMessage'.

  initDefenderSystem(boardId, ENTITY_NAME, true);

  while (true) {
    // :(
    // rip 22.04.2026
    // updateCurrentWaypointMQTT();

    int32_t latitude, longtitude, altitude;
    int result = getCoords(latitude, longtitude, altitude);

    if (result) {
      Coordinates coords(latitude, longtitude, altitude);
      updateDefenderSystem(&coords);
    } else {
      logEntry("Error while reading geoCoords", ENTITY_NAME,
               LogLevel::LOG_WARNING);
    }
    sleep(1);
  }

  return EXIT_SUCCESS;
}
