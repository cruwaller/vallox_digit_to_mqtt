#include "mqtt.h"
#include "debug_print.h"
#include "wifi.h"
#include "helpers.h"

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif
#include <PubSubClient.h>
#include <EspMQTTClient.h>

#ifndef DEVICE_MODEL
#define DEVICE_MODEL "MQTT Client"
#endif
#ifndef DEVICE_MANUFACTURER
#define DEVICE_MANUFACTURER "OULWare"
#endif

// TODO: Move MQTT config to Web UI and store credentials to database
#ifndef MQTT_SERVER
#define MQTT_SERVER "192.168.1.2" // Just to make compilation happy
#define MQTT_CONFIG_OK 0
#else
#define MQTT_CONFIG_OK 1
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER NULL
#endif
#ifndef MQTT_PASSWD
#define MQTT_PASSWD NULL
#endif
#ifndef MQTT_CLIENT_PREFIX
#define MQTT_CLIENT_PREFIX "home/EspMQTT"
#endif
#ifndef MQTT_HA_DISCOVERY_PREFIX
#define MQTT_HA_DISCOVERY_PREFIX "homeassistant"
#endif

#ifndef MQTT_STATUS_DELAY_S
#define MQTT_STATUS_DELAY_S 60U // default to 1min
#endif
#define MQTT_STATUS_DELAY_MS (MQTT_STATUS_DELAY_S * 1000)


static const String discovery_prefix = MQTT_HA_DISCOVERY_PREFIX;
static String mac_address;
static String mqtt_device_info;
static String mqtt_client_name;
static String mqtt_will_topic;
static uint32_t mqtt_last_publish_ms;

static char const * const mqtt_will_sub_topic = "/state";

static EspMQTTClient mqtt_client(
    NULL, NULL,
    MQTT_SERVER, MQTT_USER, MQTT_PASSWD,
    DEFAULT_MQTT_CLIENT_NAME, MQTT_PORT);

static HAMqttDevice dev_ip("IP", HAMqttDevice::SENSOR, "", "mdi:ip-network");
static HAMqttDevice dev_rssi("RSSI", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_SIGNAL_STRENGTH, "dB", "mdi:wifi");
static HAMqttDevice dev_reboot("Reboot", HAMqttDevice::BUTTON, "", "mdi:restart");

/* Store different configs to vector */
static std::vector<mqtt_topic_config_t const *> _topic_configs;


static String get_ip_addr(void)
{
    return WiFi.localIP().toString();
}


static String get_rssi(void)
{
    return String(WiFi.RSSI());
}

static mqtt_topic_config_t devices_rssi = {
    .dev = &dev_rssi, .get = get_rssi, .set = NULL
};


/*static String get_reboot_state(void)
{
    return String("OFF");
}*/


static void mqtt_handle_reboot_set(const String &payload)
{
    if (payload.equals("REBOOT"))
        ESP.restart();
}


static mqtt_topic_config_t devices_reboot = {
    .dev=&dev_reboot, .get=NULL, .set=mqtt_handle_reboot_set
};


static void mqtt_get_unique_info(void)
{
    char macStr[(6 * 2) + 1]; // Used for unique id
    uint8_t mac[6];

    // Get the MAC address
    WiFi.macAddress(mac);
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    mac_address = macStr;

    // Create a client ID based on MAC address
    mqtt_client_name = MQTT_CLIENT_PREFIX;
    mqtt_client_name += "_";
    mqtt_client_name += macStr;

    mqtt_will_topic = mqtt_client_name;
    mqtt_will_topic += mqtt_will_sub_topic;

    // Disable all Serial debug logs
    mqtt_client.enableDebuggingMessages(false);
    mqtt_client.setMqttReconnectionAttemptDelay(5000); // 5sec
    mqtt_client.setMaxPacketSize(2048);
    mqtt_client.setMqttClientName(mqtt_client_name.c_str());
    mqtt_client.enableLastWillMessage(mqtt_will_topic.c_str(), "offline", true);

    // Build a device info for auto discovery
    /*
        "device": {
            "name": "<DEVICE NAME>",
            "model": "MQTT Client",
            "manufacturer": "OULWare",
            "sw_version": "<git_sha>",
            "identifiers": ["<MAC address>"],
            "connections": [["mac","<MAC address>"]]:
        }

        Abbreviations:
            mdl = model
            mf = manufacturer
            sw = sw_version
            ids = identifiers
            cns = connections
    */
    mqtt_device_info = "{\"name\":\"";
    mqtt_device_info += wifi_get_name();
    mqtt_device_info += "\",\"mdl\":\"";
    mqtt_device_info += DEVICE_MODEL;
    mqtt_device_info += "\",\"mf\":\"";
    mqtt_device_info += DEVICE_MANUFACTURER;
    mqtt_device_info += "\",\"sw\":\"";
    mqtt_device_info += LATEST_COMMIT_STR;
    mqtt_device_info += "\",\"ids\":[\"";
    mqtt_device_info += macStr;
    mqtt_device_info += "\"],\"cns\":[[\"mac\",\"";
    mqtt_device_info += macStr;
    mqtt_device_info += "\"]]}";
}


static void mqtt_handle_configuration(void)
{
    // Send IP address configuration to HA
    String topic = dev_ip.getConfigTopic();
    String payload = dev_ip.getConfigPayload();
    mqtt_client.publish(topic, payload);

    // Send other configurations to HA
    for (auto config : _topic_configs) {
        if (config && config->dev) {
            topic = config->dev->getConfigTopic();
            payload = config->dev->getConfigPayload();
            mqtt_client.publish(topic, payload);
        }
    }
}


static void IRAM_ATTR mqtt_publish_status(void)
{
    for (auto config : _topic_configs) {
        if (config && config->dev && config->get)
            mqtt_client.publish(config->dev->getStateTopic(), config->get());
    }
}


static void mqtt_handle_publish_all(void)
{
    // delay update by 5 second
    mqtt_client.executeDelayed(5 * 1000, []() {
        // Send the current states
        mqtt_client.publish(mqtt_will_topic.c_str(), "online");
        mqtt_client.publish(dev_ip.getStateTopic(), get_ip_addr());
        mqtt_publish_status();
    });
    mqtt_last_publish_ms = millis();
}


// This function is called once MQTT is connected
void onConnectionEstablished()
{
    // Subscribe
    for (auto config : _topic_configs) {
        if (config && config->dev && config->set)
            mqtt_client.subscribe(config->dev->getCommandTopic(), config->set);
    }

    // Publish configs
    mqtt_handle_configuration();
    // .. and values
    mqtt_handle_publish_all();

    // Subscribe to Home Assitant connection status events.
    mqtt_client.subscribe(MQTT_HA_DISCOVERY_PREFIX "/status", [](const String &payload) {
        // Home assistant get online => publish the device config
        if (payload.equals("online")) {
            mqtt_handle_configuration();
            mqtt_handle_publish_all();
        }
    });
}


//------------------------------------------------------------
// MQTT

void mqtt_init(void)
{
    mqtt_get_unique_info();

    /* Update local mqtt configs */
    mqtt_fill_device_basic_info(dev_ip);
    dev_ip.addConfigVar("ent_cat", "diagnostic");  // 'ent_cat' = 'entity_category
    mqtt_fill_device_basic_info(dev_rssi);
    dev_rssi.addConfigVar("ent_cat", "diagnostic");
    mqtt_fill_device_basic_info(dev_reboot);
    dev_reboot.addConfigVar("ent_cat", "config");
    dev_reboot.addConfigVar("payload_press", "REBOOT");

    /* Add default infos on the list */
    mqtt_register_topic(&devices_rssi);
    mqtt_register_topic(&devices_reboot);
}


void IRAM_ATTR mqtt_handle(uint32_t const now)
{
#ifndef WIFI_MODE_AP
#define WIFI_MODE_AP WIFI_AP    // ESP8266 compatibility
#endif

    // Ignore MQTT service if WiFi is in AP mode or MQTT config is not defined
    if (!MQTT_CONFIG_OK || WiFi.getMode() == WIFI_MODE_AP)
        return;

    if (mqtt_client.isMqttConnected() &&
        (int32_t)MQTT_STATUS_DELAY_MS <= (int32_t)(now - mqtt_last_publish_ms))
    {
        mqtt_last_publish_ms = now;
        mqtt_publish_status();
    }
    mqtt_client.loop();
}


void mqtt_fill_device_basic_info(HAMqttDevice & topic)
{
    mqtt_fill_device_basic_info(&topic);
}


void mqtt_fill_device_basic_info(HAMqttDevice * const topic)
{
    topic->setHomeAssistantMqttPrefix(discovery_prefix);
    topic->setTopicId(mac_address);
    topic->addConfigVar("avty_t", mqtt_will_topic);    // availability topic
    topic->addConfigVar("device", mqtt_device_info);
}


void mqtt_register_topic(mqtt_topic_config_t const * const config)
{
    if (!config)
        return;
    // Check whether the config is already on the list
    if (std::find(_topic_configs.begin(), _topic_configs.end(), config) != _topic_configs.end())
        return; // found, skip double
    // Add to list
    _topic_configs.push_back(config);
}
