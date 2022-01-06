#include "secrets.h"

#define HOSTNAME "roomba" // e.g. roomba.local
//#define BRC_PIN 14 // GPIO 14 -> D5
#define BRC_PIN 0 // GPIO 0 -> D3

#define ROOMBA_650_SLEEP_FIX 1

#define LOGGING 1

#define ADC_VOLTAGE_DIVIDER 44.551316985
//#define ENABLE_ADC_SLEEP

#define MQTT_SERVER "192.168.0.44"
#define MQTT_USER ""
#define MQTT_COMMAND_TOPIC "vacuum/command"
#define MQTT_STATE_TOPIC "vacuum/state"
