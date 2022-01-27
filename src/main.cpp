#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Roomba.h>
#include <ArduinoJson.h>
#include "config.h"
#include "debug.h"
#include "mqtt.h"
extern "C"
{
#include "user_interface.h"
}
#include <Roomba.h>

// Roomba setup
int RX_PIN = 5;
int TX_PIN = 4;
SoftwareSerial serial(RX_PIN, TX_PIN);
Roomba roomba(&serial, Roomba::Baud115200);

// unique ID for ESP device i.e. roomba_98cdac3050de
char *entityId;

// Roomba state
typedef struct
{
  // Sensor values
  int16_t distance;
  uint8_t chargingState;
  uint16_t voltage;
  int16_t current;
  // Supposedly unsigned according to the OI docs, but I've seen it
  // underflow to ~65000mAh, so I think signed will work better.
  int16_t charge;
  uint16_t capacity;

  // Derived state
  bool cleaning;
  bool docked;

  int timestamp;
  bool sent;
} RoombaState;

RoombaState roombaState = {};

// Roomba sensor packet
uint8_t roombaPacket[100];
uint8_t sensors[] = {
    Roomba::SensorDistance,       // PID 19, 2 bytes, mm, signed
    Roomba::SensorChargingState,  // PID 21, 1 byte
    Roomba::SensorVoltage,        // PID 22, 2 bytes, mV, unsigned
    Roomba::SensorCurrent,        // PID 23, 2 bytes, mA, signed
    Roomba::SensorBatteryCharge,  // PID 25, 2 bytes, mAh, unsigned
    Roomba::SensorBatteryCapacity // PID 26, 2 bytes, mAh, unsigned
};

// Network setup
bool OTAStarted;

int getBatteryLevel(RoombaState state)
{
  int batteryLevel = 0;
  if (state.capacity != 0)
  {
    batteryLevel = (state.charge * 100) / state.capacity;
  }
  return batteryLevel;
}

void wakeup()
{
  DLOG("Wakeup Roomba\n");
  pinMode(BRC_PIN, OUTPUT);
  digitalWrite(BRC_PIN, LOW);
  delay(200);
  pinMode(BRC_PIN, INPUT);
  delay(200);
  serial.write(128); // Start
}

void wakeOnDock()
{
  DLOG("Wakeup Roomba on dock\n");
  wakeup();
}

void wakeOffDock()
{
  DLOG("Wakeup Roomba off Dock\n");
  roomba.safeMode();
  delay(300);
  roomba.passiveMode();
}

void roomba_song(int *bytes, int len)
{
  for (int now = 0; now < len; now++)
  {
    serial.write((byte)(bytes[now]));
    delay(100);
  }
}

bool performCommand(const char *cmdchar)
{
  wakeup();

  // Char* string comparisons dont always work
  String cmd(cmdchar);

  // MQTT protocol commands
  if (cmd == "turn_on")
  {
    DLOG("Turning on\n");
    roomba.cover();
    roombaState.cleaning = true;
  }
  else if (cmd == "turn_off")
  {
    DLOG("Turning off\n");
    roomba.power();
    roombaState.cleaning = false;
  }
  else if (cmd == "start" || cmd == "pause")
  {
    DLOG("Toggling\n");
    roomba.cover();
  }
  else if (cmd == "stop")
  {
    if (roombaState.cleaning)
    {
      DLOG("Stopping\n");
      roomba.cover();
    }
    else
    {
      DLOG("Not cleaning, can't stop\n");
    }
  }
  else if (cmd == "clean_spot" || cmd == "spot")
  {
    DLOG("Cleaning Spot\n");
    roombaState.cleaning = true;
    roomba.spot();
  }
  else if (cmd == "locate")
  {
    DLOG("Locating...\n");
    // Each single letter array is a bar of music.
    // Each double letter array is play command for the corresponding music.
    int a[] = {140, 1, 9, 55, 32, 55, 32, 55, 32, 51, 24, 58, 8, 55, 32, 51, 24, 58, 8, 55, 64};
    // int b[] = {140, 2, 9, 62, 32, 62, 32, 62, 32, 63, 24, 58, 8, 54, 32, 51, 24, 58, 8, 55, 64};
    // int c[] = {140, 3, 12, 67, 32, 55, 24, 55, 8, 67, 32, 66, 24, 65, 8, 64, 8, 63, 8, 64, 16, 30, 16, 56, 16, 61, 32};
    // int d[] = {140, 4, 14, 60, 24, 59, 8, 58, 8, 57, 8, 58, 16, 10, 16, 52, 16, 54, 32, 51, 24, 58, 8, 55, 32, 51, 24, 58, 8, 55, 64};

    delay(100);
    serial.write(130);
    delay(100);
    // serial.write(132);
    // delay(100);

    // Load the songs to the Roomba
    roomba_song(a, (sizeof(a) / sizeof(int)));
    // roomba_song(b, (sizeof(b) / sizeof(int)));
    // roomba_song(c, (sizeof(c) / sizeof(int)));
    // roomba_song(d, (sizeof(d) / sizeof(int)));

    DLOG("Songs loaded, playing...\n");
    // Play the songs. Pauses must be included while the music plays.
    roomba.playSong(1);
    delay(5000);
    // roomba.playSong(2);
    // delay(4000);
    // roomba.playSong(3);
    // delay(3500);
    // roomba.playSong(4);
    // delay(2000);
    roomba.safeMode();
    DLOG("Locating done\n");
  }
  else if (cmd == "return_to_base" || cmd == "dock")
  {
    DLOG("Returning to Base\n");
    roombaState.cleaning = true;
    roomba.dock();
  }
  else
  {
    return false;
  }
  return true;
}

void createEntityID()
{
  byte MAC[6];
  WiFi.macAddress(MAC);
  char MACc[30];
  sprintf(MACc, "%02X%02X%02X%02X%02X%02X", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
  char uniqId[50];
  sprintf(uniqId, "roomba_%s", MACc);

  int len = strlen(uniqId);
  entityId = (char *)malloc(len + 1);
  memcpy(entityId, strlwr(uniqId), len);
  entityId[len] = 0;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  DLOG("Received mqtt callback for topic %s\n", topic);
  if (strcmp(getMqttCommandTopic(), topic) == 0)
  {
    char *cmd = (char *)malloc(length + 1);
    memcpy(cmd, payload, length);
    cmd[length] = 0;

    if (!performCommand(cmd))
    {
      DLOG("Unknown command %s\n", cmd);
    }
    free(cmd);
  }
}

void debugCallback()
{
  String cmd = getRemoteDebugLastCommand();

  // Debugging commands via telnet
  if (performCommand(cmd.c_str()))
  {
  }
  else if (cmd == "quit")
  {
    DLOG("Stopping Roomba\n");
    serial.write(173);
  }
  else if (cmd == "rreset")
  {
    DLOG("Resetting Roomba\n");
    roomba.reset();
  }
  else if (cmd == "version")
  {
    const char compile_date[] = __DATE__ " " __TIME__;
    DLOG("Compiled on: %s\n", compile_date);
  }
  else if (cmd == "sleep5s")
  {
    DLOG("Going to sleep for 5 seconds\n");
    delay(100);
    ESP.deepSleep(5e6);
  }
  else if (cmd == "sleep10m")
  {
    DLOG("Going to sleep for 10 minutes\n");
    delay(100);
    ESP.deepSleep(600e6);
  }
  else if (cmd == "wake")
  {
    DLOG("Toggle BRC pin\n");
    wakeup();
  }
  else if (cmd == "streamresume")
  {
    DLOG("Resume streaming\n");
    roomba.streamCommand(Roomba::StreamCommandResume);
  }
  else if (cmd == "streampause")
  {
    DLOG("Pause streaming\n");
    roomba.streamCommand(Roomba::StreamCommandPause);
  }
  else if (cmd == "stream")
  {
    DLOG("Requesting stream\n");
    roomba.stream(sensors, sizeof(sensors));
  }
  else if (cmd == "streamreset")
  {
    DLOG("Resetting stream\n");
    roomba.stream({}, 0);
  }
  else
  {
    DLOG("Unknown command %s\n", cmd.c_str());
  }
}

// Check the battery, if it's too low, sleep the ESP (so we don't murder the battery)
void sleepIfNecessary()
{
  bool shouldSleep = false;

  int batteryLevel = getBatteryLevel(roombaState);

  if (batteryLevel > 0 && batteryLevel < 10)
  {
    DLOG("Battery level is low (%d%%). Sleeping for 10 minutes\n", batteryLevel);
    shouldSleep = true;
  }

  float mV = roombaState.voltage;
  // check if mV is > 0 because it's initial value is 0 and we don't wan't to panic
  // before any real measure is done
  if (mV > 0 && mV < 10800)
  {
    DLOG("Battery voltage is low (%.1fV). Sleeping for 10 minutes\n", mV / 1000);
    shouldSleep = true;
  }

  if (shouldSleep)
  {
    // Fire off a quick message with our most recent state, if MQTT is connected
    delay(100);
    if (mqttConnected())
    {
      StaticJsonDocument<200> root;
      root["battery_level"] = batteryLevel;
      root["cleaning"] = false;
      root["docked"] = false;
      root["charging"] = false;
      root["voltage"] = mV / 1000;
      root["charge"] = 0;
      String jsonStr;
      serializeJson(root, jsonStr);
      mqttPublishState(jsonStr.c_str());
    }
    delay(200);

    // Sleep for 10 minutes
    ESP.deepSleep(600e6);
  }

  // this is probably some error, maybe with the wiring
  if (mV == 0 || batteryLevel == 0)
  {
    DLOG("Battery readings seem to be wrong! Voltage: %.1fV, level: %d%%\n", mV, batteryLevel);
  }
}

bool parseRoombaStateFromStreamPacket(uint8_t *packet, int length, RoombaState *state)
{
  state->timestamp = millis();
  int i = 0;
  while (i < length)
  {
    switch (packet[i])
    {
    case Roomba::Sensors7to26: // 0
      i += 27;
      break;
    case Roomba::Sensors7to16: // 1
      i += 11;
      break;
    case Roomba::SensorVirtualWall: // 13
      i += 2;
      break;
    case Roomba::SensorDistance: // 19
      state->distance = packet[i + 1] * 256 + packet[i + 2];
      i += 3;
      break;
    case Roomba::SensorChargingState: // 21
      state->chargingState = packet[i + 1];
      i += 2;
      break;
    case Roomba::SensorVoltage: // 22
      state->voltage = packet[i + 1] * 256 + packet[i + 2];
      i += 3;
      break;
    case Roomba::SensorCurrent: // 23
      state->current = packet[i + 1] * 256 + packet[i + 2];
      i += 3;
      break;
    case Roomba::SensorBatteryCharge: // 25
      state->charge = packet[i + 1] * 256 + packet[i + 2];
      i += 3;
      break;
    case Roomba::SensorBatteryCapacity: //26
      state->capacity = packet[i + 1] * 256 + packet[i + 2];
      i += 3;
      break;
    case Roomba::SensorBumpsAndWheelDrops: // 7
      i += 2;
      break;
    case 128: // Unknown
      DLOG("Unknown Packet ID %d, ignoring.\n", packet[i]);
      i += 2;
      break;
    default:
      DLOG("Unhandled Packet ID %d\n", packet[i]);
      return false;
      break;
    }
  }
  return true;
}

void verboseLogPacket(uint8_t *packet, uint8_t length)
{
  VLOG("Packets[%d]: ", length);
  for (int i = 0; i < length; i++)
  {
    VLOG("%d ", packet[i]);
  }
  VLOG("\n");
}

void readSensorPacket()
{
  uint8_t packetLength;
  bool received = roomba.pollSensors(roombaPacket, sizeof(roombaPacket), &packetLength);
  if (received)
  {
    RoombaState rs = {};
    bool parsed = parseRoombaStateFromStreamPacket(roombaPacket, packetLength, &rs);
    verboseLogPacket(roombaPacket, packetLength);
    if (parsed)
    {
      roombaState = rs;
      VLOG("Got Packet of len=%d! Distance:%dmm ChargingState:%d Voltage:%dmV Current:%dmA Charge:%dmAh Capacity:%dmAh\n",
           packetLength, roombaState.distance, roombaState.chargingState, roombaState.voltage, roombaState.current, roombaState.charge, roombaState.capacity);
      roombaState.cleaning = false;
      roombaState.docked = false;
      if (roombaState.current < -400)
      {
        roombaState.cleaning = true;
      }
      else if (roombaState.current > -50)
      {
        roombaState.docked = true;
      }
    }
    else
    {
      DLOG("Failed to parse packet\n");
    }
  }
}

void onOTAStart()
{
  DLOG("Starting OTA session\n");
  DLOG("Pause streaming\n");
  roomba.streamCommand(Roomba::StreamCommandPause);
  OTAStarted = true;
}

void reconnect()
{
  mqttConnect(entityId, MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD, callback);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // High-impedence on the BRC_PIN
  pinMode(BRC_PIN, INPUT);

  createEntityID();

  String hostname(entityId);
  WiFi.hostname(hostname);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  ArduinoOTA.setHostname(entityId);
  ArduinoOTA.begin();
  ArduinoOTA.onStart(onOTAStart);

  reconnect();

  initDebug(entityId, debugCallback);

  sleepIfNecessary();

  roomba.start();
  delay(100);

  // Reset stream sensor values
  roomba.stream({}, 0);
  delay(100);

  // Request sensor stream
  roomba.stream(sensors, sizeof(sensors));
}

void sendConfig()
{
  if (!mqttConnected())
  {
    DLOG("MQTT Disconnected, not sending config\n");
    return;
  }
  StaticJsonDocument<500> root;
  root["name"] = entityId;
  root["unique_id"] = entityId;
  root["device"]["identifiers"][0] = entityId;
  root["device"]["manufacturer"] = "iRobot";
  root["device"]["model"] = "Roomba";
  root["schema"] = "state";
  root["stat_t"] = getMqttStateTopic();
  root["json_attr_t"] = getMqttStateTopic();
  root["cmd_t"] = getMqttCommandTopic();
  root["send_cmd_t"] = getMqttCommandTopic();
  root["sup_feat"][0] = "start";
  root["sup_feat"][1] = "stop";
  root["sup_feat"][2] = "pause";
  root["sup_feat"][3] = "return_home";
  root["sup_feat"][4] = "locate";
  root["sup_feat"][5] = "clean_spot";
  root["sup_feat"][6] = "battery";
  String jsonStr;
  serializeJson(root, jsonStr);
  DLOG("Reporting config: %s\n", jsonStr.c_str());
  mqttPublishConfig(jsonStr.c_str());
}

void sendStatus()
{
  if (!mqttConnected())
  {
    DLOG("MQTT Disconnected, not sending status\n");
    return;
  }

  int batteryLevel = getBatteryLevel(roombaState);

  DLOG("Reporting packet: Distance:%dmm ChargingState:%d (%s) Voltage:%dmV Current:%dmA Charge:%dmAh Capacity:%dmAh BatteryLevel:%d%%\n",
       roombaState.distance, roombaState.chargingState, Roomba::chargingStateToString(roombaState.chargingState), roombaState.voltage, roombaState.current, roombaState.charge, roombaState.capacity, batteryLevel);
  StaticJsonDocument<200> root;
  root["battery_level"] = batteryLevel;
  root["cleaning"] = roombaState.cleaning;
  root["docked"] = roombaState.docked;
  root["charging"] = roombaState.chargingState == Roomba::ChargeStateReconditioningCharging || roombaState.chargingState == Roomba::ChargeStateFullCharging || roombaState.chargingState == Roomba::ChargeStateTrickleCharging;
  root["voltage"] = roombaState.voltage;
  root["current"] = roombaState.current;
  root["charge"] = roombaState.charge;
  root["battery_capacity"] = roombaState.capacity;
  root["charging_state"] = Roomba::chargingStateToString(roombaState.chargingState);
  String curState = "idle";
  if (roombaState.docked)
  {
    curState = "docked";
  }
  else
  {
    if (roombaState.cleaning)
    {
      curState = "cleaning";
    }
  }
  root["state"] = curState;
  String jsonStr;
  serializeJson(root, jsonStr);
  DLOG("Reporting status: %s\n", jsonStr.c_str());
  mqttPublishState(jsonStr.c_str());
}

int lastStateMsgTime = 0;
int lastWakeupTime = 0;
int lastConnectTime = 0;
int configLoop = 0;

void loop()
{
  ArduinoOTA.handle();
  yield();
  if (OTAStarted)
  {
    return;
  }

  handleDebug();

  long now = millis();
  if ((now - lastConnectTime) > 5000)
  {
    lastConnectTime = now;
    if (!mqttConnected())
    {
      DLOG("MQTT not connected, connecting...\n");
      reconnect();
      sendConfig();
    }
    else
    {
      if (configLoop == 19)
      {
        sendConfig();
        configLoop = 0;
      }
      else
      {
        configLoop++;
      }
    }
  }

  // Wakeup the roomba at fixed intervals
  if (now - lastWakeupTime > 50000)
  {
    lastWakeupTime = now;
    if (!roombaState.cleaning)
    {
      if (roombaState.docked)
      {
        wakeOnDock();
      }
      else
      {
        wakeup();
      }
    }
    else
    {
      wakeup();
    }
  }
  // Report the status over mqtt at fixed intervals
  if (now - lastStateMsgTime > 10000)
  {
    lastStateMsgTime = now;
    float stateAge = (now - roombaState.timestamp) / 1000.0;
    if (stateAge > 30 || roombaState.sent)
    {
      DLOG("Roomba state already sent (%.1fs old) (sent=%d)\n", stateAge, roombaState.sent);
      DLOG("Request stream\n");
      roomba.stream(sensors, sizeof(sensors));
    }
    else
    {
      sendStatus();
      roombaState.sent = true;
    }
    sleepIfNecessary();
  }

  mqttLoop();
  readSensorPacket();
}
