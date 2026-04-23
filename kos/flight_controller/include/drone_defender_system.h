#ifndef _DRONE_DEFENDER_SYSTEM_H
#define _DRONE_DEFENDER_SYSTEM_H
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_logger.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../include/flight_controller.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <unistd.h>

class Coordinates {
public:
  int32_t latitude;
  int32_t longtitude;
  int32_t altitude;

  Coordinates(int32_t lat, int32_t lon, int32_t alt)
      : latitude(lat), longtitude(lon), altitude(alt) {}
};

void initDefenderSystem(char *id, char *entryName, bool isInspectorState);

void setLogEntryName(char *logEntryName);

void updateDefenderSystem(Coordinates *drone);

#endif
