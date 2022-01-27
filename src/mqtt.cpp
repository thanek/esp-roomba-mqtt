#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define MQTT_TOPIC_PREFIX "homeassistant/vacuum"

WiFiClient wifiClient;
PubSubClient client(wifiClient);

char *configTopic;
char *stateTopic;
char *commandTopic;

char *generateMQTTTopic(const char *clientId, const char *topic)
{
    // build mqtt target topic
    char mqttTopic[256];
    sprintf(mqttTopic, "%s/%s/%s", MQTT_TOPIC_PREFIX, clientId, topic);

    int len = strlen(mqttTopic);
    char *tpc = (char *)malloc(len + 1);
    memcpy(tpc, mqttTopic, len);
    tpc[len] = 0;

    return tpc;
}

bool mqttConnect(
    const char *clientId, const char *host, int port, const char *user, const char *password, MQTT_CALLBACK_SIGNATURE)
{
    if (!configTopic)
        configTopic = generateMQTTTopic(clientId, "config");
    if (!stateTopic)
        stateTopic = generateMQTTTopic(clientId, "state");
    if (!commandTopic)
        commandTopic = generateMQTTTopic(clientId, "command");

    if (!client.connected())
    {
        client.setServer(host, port);
        client.setCallback(callback);

        DLOG("Attempting MQTT connection (%s@%s:%d)...\n", clientId, host, port);
        if (client.connect(clientId, user, password))
        {
            DLOG("MQTT connected\n");
            client.subscribe(commandTopic);
            return false;
        }
        else
        {
            DLOG("MQTT failed rc=%d try again in 5 seconds\n", client.state());
            return false;
        }
    }
    return true;
}

bool mqttConnected()
{
    return client.connected();
}

const char *getMqttCommandTopic()
{
    return commandTopic;
}
const char *getMqttStateTopic()
{
    return stateTopic;
}

void mqttPublishState(const char *payload)
{
    client.publish(stateTopic, payload, true);
}

void mqttPublishConfig(const char *payload)
{
    client.publish(configTopic, payload);
}

void mqttLoop()
{
    client.loop();
}