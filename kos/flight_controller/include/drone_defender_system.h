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

void setLogEntryName(char* logEntryName);
void updateCurrentWaypoint();
void handleAltiduteChange();
void handleSpeedChange();

#endif