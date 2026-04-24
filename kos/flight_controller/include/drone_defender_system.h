#ifndef _DRONE_DEFENDER_SYSTEM_H
#define _DRONE_DEFENDER_SYSTEM_H
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_logger.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../include/flight_controller.h"

#include <cstdint>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <unistd.h>

// Погресть в cm при определении высоты
#define ALTITUDE_EPSILON 10
// Ожидание между обновлениями (в мс)
#define UPDATE_DELAY 500
// Расстояние (в м) при котором считается что дрон достиг необходимой точки
#define REACH_DISTANCE 0.5

#define MAX_ALTIDUTE 150
#define MIN_ALTIDUTE 55

class Coordinates {
public:
  int32_t latitude;
  int32_t longtitude;
  int32_t altitude;

  Coordinates(int32_t lat, int32_t lon, int32_t alt)
      : latitude(lat), longtitude(lon), altitude(alt) {}
};

void initDefenderSystem(char *id, char *entryName, bool isInspectorState);
bool isWaypointReached(Coordinates *drone, CommandWaypoint waypoint,
                       double distance);

void setLogEntryName(char *logEntryName);

void updateDefenderSystem(Coordinates *drone);
void updateDefenderSystemLoop();

void setTargetAltitude(int32_t altitude);
void forceSetTargetWaypoint(MissionCommand *command);
void retrunTargetWaypointBack();

#endif
