#include <PubSubClient.h>

const char *getMqttCommandTopic();
const char *getMqttStateTopic();

bool mqttConnect(const char *clientId, const char *hostname, int port, const char *user, const char *password, MQTT_CALLBACK_SIGNATURE);
bool mqttConnected();
void mqttPublishState(const char *payload);
void mqttPublishConfig(const char *payload);
void mqttLoop();