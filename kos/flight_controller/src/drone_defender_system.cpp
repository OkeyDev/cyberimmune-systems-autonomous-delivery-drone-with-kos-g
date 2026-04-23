#include "../include/drone_defender_system.h"

#define ALTITUDE_EPSILON 10
#define DISTANCE_EPSILON 1.2f
#define UPDATE_DELAY 1
#define MAX_SPEED 1

// -1 = if we doesn't know, what point we are currently on,
// might be that drone isn't even on the flight
int32_t targetWaypointIndex = -1;
MissionCommand *targetWaypoint;
MissionCommand *lastWaypoint;
bool ended = false;
int32_t targetAltidute = 0;

MissionCommand* targetInterestPoint;

int32_t lastCoordX = 0;
int32_t lastCoordY = 0;

//
char *globalEntityName = "DEFAULT_LOG_NAME";

void setLogEntryName(char *logEntryName) { globalEntityName = logEntryName; }

static double degToRad(double degree) { return degree * (M_PI / 180.0); }

/*
 *
a = math.sin(delta_lat / 2)**2 + \
math.cos(lat1_rad) * math.cos(lat2_rad) * \
math.sin(delta_lon / 2)**2
c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

distance = R * c
    */

static double calcDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0f;

  auto lat1_rad = degToRad(lat1);
  auto lat2_rad = degToRad(lat2);
  auto d_lat = degToRad(lat2 - lat1);
  auto d_lon = degToRad(lon2 - lon1);

  auto a = std::pow(std::sin(d_lat / 2), 2) +
           std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::pow(std::sin(d_lon / 2), 2);

  if (a > 1.0)
    a = 1.0;

  auto c = 2 * std::asin(std::sqrt(std::min(a, 1.0)));

  return R * c;
}


static float dotProduct(float x1, float y1, float x2, float y2) {
  return x1 * x2 + y1 * y2;
}

bool isDroneInRestrictedZone() {
  // TODO: Implement
  
  int count = 0;
  auto noFlightAreas = getNoFlightAreas(count);

  for (int i = 0; i < count; i++)
  {
    auto area = noFlightAreas + i;
    for (int j = 0; j < area->pointNum; j++)
    {
      auto point = area->points + j;
    }
  }
  

  return true;
}

MissionCommand *getCurrentWaypoint() {
  // updateCurrentWaypoint();
  return targetWaypoint;
}

// TODO: MUST BE TESTED
void handleIncorrectMovement() {
  MissionCommand *waypoint = getCurrentWaypoint();

  // mission isn't loaded
  if (waypoint == nullptr)
    return;

  int32_t laltitude, longtitude, altitude;
  int result = getCoords(laltitude, longtitude, altitude);

  if (!result)
    return;

  if (lastCoordX == -1 && lastCoordY == -1) {
    // remembers current position
    lastCoordX = laltitude;
    lastCoordY = longtitude;
  }

  float waypointDirX =
      1.0f / float(waypoint->content.waypoint.latitude - laltitude);
  float waypointDirY =
      1.0f / float(waypoint->content.waypoint.longitude - longtitude);

  float currentDirX = 1.0f / float(laltitude - lastCoordX);
  float currentDirY = 1.0f / float(longtitude - lastCoordY);

  if (std::abs(1 - dotProduct(waypointDirX, waypointDirY, currentDirX,
                              currentDirY)) < 0.05f) {
    char message[128];

    const CommandWaypoint content = waypoint->content.waypoint;

    snprintf(message, sizeof(message),
             "Handle incorrect flight behaviour. Changing waypoint: %d, %d, %d",
             content.latitude, content.longitude, content.altitude);

    logEntry(message, globalEntityName, LogLevel::LOG_WARNING);

    changeWaypoint(content.latitude, content.longitude, content.altitude);

    lastCoordX = laltitude;
    lastCoordY = longtitude;
  }
}

void setTargetAltitude(int32_t altitude) {
  targetAltidute = altitude;
}

void setNextWaypoint(MissionCommand *commands, int count, int start = 0) {
  auto prevWaypoint = targetWaypoint;
  auto prevWaypointIndex = targetWaypointIndex;

  for (int i = start; i < count; i++) {
    MissionCommand *command = commands + i;

    if (command->type == CommandType::WAYPOINT) {
      targetWaypointIndex = i;
      lastWaypoint = targetWaypoint;
      targetWaypoint = command;
      setTargetAltitude(command->content.waypoint.altitude);
      return;
    }
  }

  // Mission ended
  if (targetWaypoint == prevWaypoint) {
    targetWaypointIndex = -1;
    lastWaypoint = nullptr;
    targetWaypoint = nullptr;
    setTargetAltitude(0);
  } else {
    char logBuffer[256];
    snprintf(logBuffer, sizeof(logBuffer), "Waypoint changed from %d to %d", prevWaypointIndex, targetWaypointIndex);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
  }
}

void updateCurrentWaypoint() {
  int count = 0;
  auto commands = getMissionCommands(count);

  if (commands == nullptr || count == 0) {
    return;
  }

  // First start
  if (targetWaypoint == nullptr) {
    setNextWaypoint(commands, count, 0);
  }

  if (targetWaypoint == nullptr) {
    return;
  }

  auto x1 = targetWaypoint->content.waypoint.latitude;
  auto y1 = targetWaypoint->content.waypoint.longitude;

  int32_t x2, y2, altidute;
  auto result = getCoords(x2, y2, altidute);

  if (!result) {
    logEntry("Failed to receive coordinates", globalEntityName,
             LogLevel::LOG_WARNING);
  }

  auto a = (x1 - x2);
  auto b = (y1 - y2);
  double lat1 = (double)x1 / 10000000.0f;
  double lon1 = (double)y1 / 10000000.0f;
  double lat2 = (double)x2 / 10000000.0f;
  double lon2 = (double)y2 / 10000000.0f;
  auto dist = calcDistance(lat1, lon1, lat2, lon2);

  char logBuffer[256];
  snprintf(logBuffer, sizeof(logBuffer),
           "lat1: %f; lon1: %f; lat2: %f; lon2: %f; DIST: %f; Waypoint: %d",
           lat1, lon1, lat2, lon2, dist, targetWaypointIndex);
  logEntry(logBuffer, globalEntityName, LogLevel::LOG_INFO);

  if (dist < DISTANCE_EPSILON) {
    setNextWaypoint(commands, count, targetWaypointIndex + 1);
  }
}


void handleAltiduteChange() {
  // somehow need to get destination waypoint altidute to make normal compare?
  int32_t laltitude, longtitude, altitude;
  int result = getCoords(laltitude, longtitude, altitude);

  if (!result) {
    logEntry("Failed to get coordinates", globalEntityName, LogLevel::LOG_ERROR);
    return;
  }

  // what magic number is supposed to indicate that is going something wrong?
  // altitude <= 400? (if i remember correctly that altitude is in cm)
  auto mission = getCurrentWaypoint();
  if (mission == nullptr) {
    return;
  }

  // int32_t requiredAltitude = mission->content.waypoint.altitude;

  char logBuffer[256];
  snprintf(logBuffer, 256, "req: %d; current: %d;", targetAltidute, altitude);
  if (std::abs(targetAltidute - altitude) > ALTITUDE_EPSILON) {
    snprintf(logBuffer, 256, "Detecsted altitude change. Altitude change detected. Target: %d", targetAltidute);
    logEntry(logBuffer, globalEntityName, LogLevel::LOG_WARNING);
    changeAltitude(targetAltidute);
  }
}

void handleSpeedChange()
{
  int32_t laltitude, longtitude, altitude;
  int result = getCoords(laltitude, longtitude, altitude);

  if (!result) {
    logEntry("Failed to get coordinates", globalEntityName, LogLevel::LOG_ERROR);
    return;
  }

  if (targetWaypoint == nullptr || lastWaypoint == nullptr) {
    return;
  }

  double lat1 = (double)targetWaypoint->content.waypoint.latitude / 10000000.0f;
  double lon1 = (double)targetWaypoint->content.waypoint.longitude / 10000000.0f;
  double lat2 = (double)lastWaypoint->content.waypoint.latitude / 10000000.0f;
  double lon2 = (double)lastWaypoint->content.waypoint.longitude / 10000000.0f;
  auto dist = calcDistance(lat1, lon1, lat2, lon2);

  char logBuffer[256];
  if (dist / UPDATE_DELAY > MAX_SPEED) {
    snprintf(logBuffer, 256, "Detecsted speed change. Speed change detected. Target: %d", MAX_SPEED / 100);
    logEntry(logBuffer, globalEntityName, LogLevel::LOG_WARNING);
    changeSpeed(MAX_SPEED / 100);
  }
}

void handleRecognitionResponse()
{

}

void updateDroneDefendSystem()
{
  updateCurrentWaypoint();
  handleAltiduteChange();
  handleSpeedChange();
}
