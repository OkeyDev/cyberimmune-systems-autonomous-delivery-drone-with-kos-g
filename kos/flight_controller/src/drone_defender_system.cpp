#include "../include/drone_defender_system.h"

#include <vector>

#define ALTITUDE_EPSILON 10
#define DISTANCE_EPSILON 1.2f
// Расстояние (в м) при котором считается что дрон достиг необходимой точки
#define REACH_DISTANCE 0.75
#define UPDATE_DELAY 1
#define MAX_SPEED 1
#define MAX_ALTIDUTE 150
#define MIN_ALTIDUTE 55

// in m (or not?)
#define CARGOLOCK_ENABLE_DISTANCE 1

// in seconds
#define WAIT_FOR_RECOGNITION_DONE 1

// -1 = if we doesn't know, what point we are currently on,
// might be that drone isn't even on the flight
int32_t targetWaypointIndex = -1;
MissionCommand *targetWaypoint;
MissionCommand *lastWaypoint;
bool ended = false;
int32_t targetAltidute = 0;
bool isDroneInspector = false;
bool disableWaypointUpdate = false;

std::vector<MissionCommand*> targetInterestWaypoints = {};

int32_t lastCoordX = 0;
int32_t lastCoordY = 0;

//
char *globalEntryName = "DEFAULT_LOG_NAME";
char* boardId = nullptr;

static double degToRad(int32_t degree)
static double degToRad(double degree) { return degree * (M_PI / 180.0); }

MissionCommand* getInterestWaypointNearBy(Coordinates *drone, double distance) {

  for (auto point : targetInterestWaypoints)
  {
    double dist = calcDistance(point->content.waypoint.latitude, point->content.waypoint.longitude, 
      drone->latitude, drone->longtitude);
  
    if (dist < distance) {
      return point;
    }    
  }

  return nullptr;
}


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

static double calcDistance(int32_t lat1, int32_t lon1, int32_t lat2,
                           int32_t lon2) {

    double lat1_d = (double)lat1 / 10000000.0f;
    double lon1_d = (double)lon1 / 10000000.0f;
    double lat2_d = (double)lat2 / 10000000.0f;
    double lon2_d = (double)lon2 / 10000000.0f;

    calcDistance(lat1_d, lon1_d, lat2_d, lon2_d);
  }

static double calcDistance(CommandWaypoint a, CommandWaypoint b) {
  calcDistance(a.latitude, a.longitude, b.altitude, b.longitude);
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

double normalizeAngle(double angle_deg) {
    angle_deg = std::fmod(angle_deg, 360.0);
    if (angle_deg > 180.0) angle_deg -= 360.0;
    if (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

// Вычисление азимута (bearing) от точки A к точке B
// Возвращает угол в градусах от севера по часовой стрелке
double computeBearing(Coordinates* from, Coordinates* to) {
    double lat1 = degToRad(from->latitude); 
    double lat2 = degToRad(to->latitude);
    double lon1 = degToRad(from->longtitude);
    double lon2 = degToRad(to->longtitude);
    
    double delta_lon = lon2 - lon1;
    
    double x = std::sin(delta_lon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) - 
                std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    
    double bearing_rad = std::atan2(x, y);
    double bearing_deg = bearing_rad * RAD_TO_DEG;
    
    return normalizeAngle(bearing_deg);
}

// TODO: MUST BE TESTED
void handleIncorrectMovement(Coordinates *drone) {
  MissionCommand *waypoint = getCurrentWaypoint();

  // mission isn't loaded
  if (waypoint == nullptr)
    return;


  if (lastCoordX == -1 && lastCoordY == -1) {
    // remembers current position
    lastCoordX = drone->latitude;
    lastCoordY = drone->longtitude;
    return;
  }

  float waypointDirX =
      1.0f / float(waypoint->content.waypoint.latitude - drone->latitude);
  float waypointDirY =
      1.0f / float(waypoint->content.waypoint.longitude - drone->longtitude);

  float currentDirX = 1.0f / float(drone->latitude - lastCoordX);
  float currentDirY = 1.0f / float(drone->longtitude - lastCoordY);

  if (std::abs(1 - dotProduct(waypointDirX, waypointDirY, currentDirX,
                              currentDirY)) < 0.05f) {
    char message[128];

    const CommandWaypoint content = waypoint->content.waypoint;

    snprintf(message, sizeof(message),
             "Handle incorrect flight behaviour. Changing waypoint: %d, %d, %d",
             content.latitude, content.longitude, content.altitude);

    logEntry(message, globalEntryName, LogLevel::LOG_WARNING);

    changeWaypoint(content.latitude, content.longitude, content.altitude);
  }

  // do not forget to update last coordinates
  lastCoordX = drone->latitude;
  lastCoordY = drone->longtitude;
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
    logEntry(logBuffer, globalEntryName, LogLevel::LOG_INFO);
  }
}

bool isWaypointReached(Coordinates *drone, CommandWaypoint waypoint,
                       double distance) {
  double dist = calcDistance(waypoint.latitude, waypoint.longitude,
                             drone->latitude, drone->longtitude);

  if (dist < distance) {
    return 1;
  }

  return 0;
}


void updateCurrentWaypoint(Coordinates *drone) {
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

  int32_t x2, y2, altidute;
  auto result = getCoords(x2, y2, altidute);

  if (!result) {
    logEntry("Failed to receive coordinates", globalEntryName,
             LogLevel::LOG_WARNING);
  }

  if (isWaypointReached(drone, targetWaypoint->content.waypoint, REACH_DISTANCE)) {
    setNextWaypoint(commands, count, targetWaypointIndex + 1);
  }
}

void handleAltiduteChange() {
  // somehow need to get destination waypoint altidute to make normal compare?
  int32_t laltitude, longtitude, altitude;
  int result = getCoords(laltitude, longtitude, altitude);

  if (!result) {
    logEntry("Failed to get coordinates", globalEntryName, LogLevel::LOG_ERROR);
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
    logEntry(logBuffer, globalEntryName, LogLevel::LOG_WARNING);
    changeAltitude(targetAltidute);
  }
}

void handleSpeedChange()
{
  int32_t laltitude, longtitude, altitude;
  int result = getCoords(laltitude, longtitude, altitude);

  if (!result) {
    logEntry("Failed to get coordinates", globalEntryName, LogLevel::LOG_ERROR);
    return;
  }

  // what magic number is supposed to indicate that is going something wrong?
  // altitude <= 400? (if i remember correctly that altitude is in cm)
  auto mission = getCurrentWaypoint();
  if (mission == nullptr || lastWaypoint == nullptr) {
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
    logEntry(logBuffer, globalEntryName, LogLevel::LOG_WARNING);
    changeSpeed(MAX_SPEED / 100);
  }
}


// isWaypointReached(command->content.waypoint);
// cargoLock(): if isWaypointReached() {setCargoLock(1)} // Освободить
// updateCurrentWaypoint(): if (isWaypointReached(CURRENT_WAYPOINT)) setNextWaypoint()
// recognitionSystem(): if (isWaypointReached(ROI_WAYPIOINT)) {сделать снимок}

void handleCargoLock(Coordinates *drone) {
  int32_t latitude, longtitude, altitude;
  getCoords(latitude, longtitude, altitude);

  int32_t count = 0;
  auto commands = getMissionCommands(count);

  for (int i = 0; i < count; i++) {
    MissionCommand *command = commands + i;

    // Меньше трех метров
    if (command->type == CommandType::INTEREST &&
        calcDistance(command->content.waypoint.latitude,
                     command->content.waypoint.longitude, latitude,
                     longtitude) < CARGOLOCK_ENABLE_DISTANCE) {
      setCargoLock(1);
      return;
    }
  }
}

void receiveInterestPoints()
{
    char messageTopic[256] = {0};
    char subscriptionBuffer[4096] = {0};
    char logBuffer[256] = {0};

    while (!receiveSubscription("api/tag/response", subscriptionBuffer, sizeof(subscriptionBuffer)) 
        || !strcmp(subscriptionBuffer, ""))
    {
        logEntry("Failed to receive subscription (Interest Point)", globalEntryName, LogLevel::LOG_ERROR);
        sleep(1);
    }
    logEntry(subscriptionBuffer, globalEntryName, LogLevel::LOG_WARNING);

    uint8_t authenticity = 0;
    while (!checkSignature(subscriptionBuffer, MessageSource::SERVER_ORVD, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature (InSterest Point). Trying again in %ds", 1);
        logEntry(logBuffer, globalEntryName, LogLevel::LOG_WARNING);
        sleep(1);
    }

    logEntry("Successful receive message (Interest Point)", globalEntryName, LogLevel::LOG_INFO);
    
    
}

void sendInterestPoint(char* tag) {
  char messageBuffer[2048] = {0};
  char signature[256] = {0};
  char signatureBuffer[2048] = {0};
  char messageTopic[] = "api/tag/request";

  snprintf(signatureBuffer, sizeof(signatureBuffer), "api/tag/request?id=%s&tag=%s", boardId, tag);
  if (!signMessage(signatureBuffer, signature, sizeof(signature))) {
    logEntry("Failed to sign message (Interest Point)", globalEntryName, LogLevel::LOG_ERROR);
    return;  
  }

  snprintf(messageBuffer, sizeof(messageBuffer), "tag=%s&sig=0x%s", tag, signature);
  while (!publishMessage("api/tag/request", messageBuffer))
  {
    logEntry("Failed to publish message (Interest Point). Retry in 1 second...", globalEntryName, LogLevel::LOG_ERROR);
    sleep(1);
  }

  logEntry("Message has been published (Interest Point)", globalEntryName, LogLevel::LOG_INFO);
}

void handleRecognitionResponse(Coordinates* drone)
{
  if (targetWaypoint == nullptr) {
    return; 
  }

  // getInterestWaypointNearBy - goes with isWaypointReached
  // so if true, then interest is reached = swaga
  MissionCommand* interest = getInterestWaypointNearBy(drone, REACH_DISTANCE);
  if (interest == nullptr)  {
    return;
  }

  const CommandWaypoint point = interest->content.waypoint;

  int requestResult = requestRecognition(); 
  while (requestResult)
  {
    char tagResult[4096] = {0};
    char logBuffer[256] = {0};
    int32_t altidute = 0;
    int responseResult = getRecognitionResponse(tagResult, altidute);
    if (!strcmp(tagResult, "") && !responseResult) {
      snprintf(logBuffer, sizeof(logBuffer), "Failed to receive response from AI. Retry in: %d", WAIT_FOR_RECOGNITION_DONE);
      logEntry(logBuffer, globalEntryName, LogLevel::LOG_ERROR);
      changeWaypoint(point.latitude, point.longitude, targetAltidute);
    }
    else if (!strcmp(tagResult, "NONE"))
    {
      snprintf(logBuffer, sizeof(logBuffer), "AI requested to change altidute: %d", altidute);
      logEntry(logBuffer, globalEntryName, LogLevel::LOG_ERROR);

      if (altidute >= MAX_ALTIDUTE) {
        logEntry("AI trying to naebat nas", globalEntryName, LogLevel::LOG_WARNING);
        changeWaypoint(point.latitude, point.longitude, MAX_ALTIDUTE);
      }
      else if (altidute <= MIN_ALTIDUTE) {
        logEntry("AI trying to naebat nas", globalEntryName, LogLevel::LOG_WARNING);
        changeWaypoint(point.latitude, point.longitude, MIN_ALTIDUTE); 
      } 
      else {
        changeWaypoint(point.latitude, point.longitude, altidute);
      }
    }
    else if (strcmp(tagResult, "") && responseResult) {
      sendInterestPoint(tagResult);
      logEntry("Successfull AI response to our beatiful picture", globalEntryName, LogLevel::LOG_INFO);
      break;
    }
  }
}

void initDefenderSystem(char* id, char* entryName, bool isInspectorState)
{
  int count = 0;
  auto commands = getMissionCommands(count);

  boardId = id;
  globalEntryName = entryName;
  isDroneInspector = isInspectorState;

  for (int i = 0; i < count; i++)
  {
    MissionCommand* command = commands + i;

    if (command->type == CommandType::INTEREST) {
      targetInterestWaypoints.push_back(command);
    }
  }
}

void updateDefenderSystem(Coordinates *drone)
{
  if (!disableWaypointUpdate) {
    updateCurrentWaypoint(drone);
  }
  
  handleAltiduteChange();
  handleSpeedChange();

  if (isDroneInspector) {
    handleRecognitionResponse(drone);
  } 
  else {
    handleCargoLock(drone);
  }
}
