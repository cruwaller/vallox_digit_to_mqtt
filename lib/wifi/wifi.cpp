#include "wifi.h"
#include "debug_print.h"
#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266mDNS.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <ESPmDNS.h>
#include <WebServer.h>
#include <HTTPUpdate.h>
#include <esp_wifi.h>
#endif

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "esp mqtt"
#endif
#ifndef WIFI_AP_PSK
#define WIFI_AP_PSK "espmqtt"
#endif
#ifndef HTTP_SERVER_PORT
#define HTTP_SERVER_PORT     80
#endif
#define CONFIG_VERSION  "ESP MQTT client"


static DNSServer dnsServer;
static WebServer server(HTTP_SERVER_PORT);
static IotWebConf iotWebConf(WIFI_AP_SSID, &dnsServer, &server, WIFI_AP_PSK, CONFIG_VERSION);


static const char PROGMEM HTML_GO_BACK[] = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
    </head>
    <body>
        <script>
            javascript:history.back();
        </script>
    </body>
</html>
)rawliteral";

static const char PROGMEM HTML_INDEX[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
    <title>ESP_MQTT</title>
    <style>
        body {
            background-color: #1E1E1E;
            font-family: Arial, Helvetica, Sans-Serif;
            Color: #69cbf7;
        }
    </style>
</head>
<body onload="javascript:start();">
  <center>
    <h2>Hello!</h2>
  </center>
</body>
</html>
)rawliteral";


void sendReturn()
{
    server.send_P(200, "text/html", HTML_GO_BACK);
}


void handleRoot()
{
    if (iotWebConf.handleCaptivePortal()) {
        // -- Captive portal request were already served.
        return;
    }
    server.send_P(200, "text/html", HTML_INDEX);
}


void handleMacAddress()
{
    uint8_t primaryChan;
#if defined(ARDUINO_ARCH_ESP32)
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);
    (void)secondChan;
#elif defined(ARDUINO_ARCH_ESP8266)
    primaryChan = wifi_get_channel();
#endif

    String message = "WiFi STA MAC: ";
    message += WiFi.macAddress();
    message += "\n  - channel in use: ";
    message += primaryChan;
    message += "\n  - mode: ";
    message += (uint8_t)WiFi.getMode();
    message += "\n\nWiFi SoftAP MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: ";
    message += WiFi.softAPIP().toString();
    message += "\n";
    server.send(200, "text/plain", message);
}


void handleNotFound()
{
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
}


void web_services_start(void)
{
    char * host = iotWebConf.getThingName();
    String instance = String(host) + "_" + WiFi.macAddress();
    instance.replace(":", "");
    MDNS.end();
    if (MDNS.begin(host)) {
        MDNS.setInstanceName(instance);
        /* http server */
        MDNS.addService("http", "tcp", HTTP_SERVER_PORT);
        MDNS.addServiceTxt("http", "tcp", "vendor", "OULWare");
        MDNS.addServiceTxt("http", "tcp", "type",   "MQTT Client");
        MDNS.addServiceTxt("http", "tcp", "version", LATEST_COMMIT_STR);
        PRINT("MDNS started. host: ");
        PRINTLN(host);
    } else {
        PRINTLN("MDNS begin failed!");
    }
}


void web_services_stop(void)
{
    MDNS.end();
    PRINTLN("MDNS stopped");
}


void wifiConnected()
{
    PRINTLN("WiFi was connected.");
    web_services_start();
}


void configSaved()
{
    PRINTLN("Configuration was updated.");
    web_services_stop();
    web_services_start();
}


#if defined(ARDUINO_ARCH_ESP32)
void WiFiEvent(WiFiEvent_t event)
{
    //PRINTF("[WiFi-event] event: %d\r\n", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        PRINT("WiFi connected. IP address: ");
        PRINTLN(WiFi.localIP());
        web_services_start();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        PRINTLN("WiFi lost connection");
        web_services_stop();
        WiFi.reconnect(); // Force reconnect
        break;

    case SYSTEM_EVENT_AP_START:
        PRINT("WiFi AP started. IP address: ");
        PRINTLN(WiFi.softAPIP());
        web_services_start();
        break;
    case SYSTEM_EVENT_AP_STOP:
        PRINTLN("WiFi AP shutdown");
        web_services_stop();
        break;
    default:
        break;
    }


#if 0
src/wifi.cpp: In function 'void WiFiEvent(system_event_id_t)':
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_WIFI_READY' not handled in switch [-Wswitch]
     switch (event)
            ^
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_SCAN_DONE' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_START' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_STOP' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_CONNECTED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_AUTHMODE_CHANGE' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_LOST_IP' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_WPS_ER_SUCCESS' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_WPS_ER_FAILED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_WPS_ER_TIMEOUT' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_WPS_ER_PIN' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_STA_WPS_ER_PBC_OVERLAP' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_AP_START' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_AP_STOP' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_AP_STACONNECTED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_AP_STADISCONNECTED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_AP_STAIPASSIGNED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_AP_PROBEREQRECVED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_GOT_IP6' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_ETH_START' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_ETH_STOP' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_ETH_CONNECTED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_ETH_DISCONNECTED' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_ETH_GOT_IP' not handled in switch [-Wswitch]
src/wifi.cpp:258:12: warning: enumeration value 'SYSTEM_EVENT_MAX' not handled in switch [-Wswitch]
#endif
}

#elif defined(ARDUINO_ARCH_ESP8266)
static WiFiEventHandler stationConnectedHandler;
static WiFiEventHandler stationDisconnectedHandler;
static WiFiEventHandler stationGotIpAddress;
static WiFiEventHandler stationDhcpTimeout;
static WiFiEventHandler AP_ConnectedHandler;
static WiFiEventHandler AP_DisconnectedHandler;


void onStationConnected(const WiFiEventStationModeConnected& evt) {
}

void onStationDisconnected(const WiFiEventStationModeDisconnected& evt) {
    web_services_stop();
    WiFi.reconnect(); // Force reconnect
}

void onStationGotIP(const WiFiEventStationModeGotIP& evt) {
    web_services_start();
}

void onStationDhcpTimeout(void)
{
}

void onSoftAPModeStationConnected(const WiFiEventSoftAPModeStationConnected& evt) {
    web_services_start();
}

void onSoftAPModeStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
    web_services_stop();
}
#endif


void wifi_setup(void)
{
#if defined(ARDUINO_ARCH_ESP32)
    WiFi.setTxPower(WIFI_POWER_13dBm);
#endif
    //WiFi.persistent(false);
    //WiFi.disconnect(true);
    //WiFi.mode(WIFI_OFF);

#if defined(ARDUINO_ARCH_ESP32)
    WiFi.onEvent(WiFiEvent);
#elif defined(ARDUINO_ARCH_ESP8266)
    /* STA mode callbacks */
    stationConnectedHandler = WiFi.onStationModeConnected(&onStationConnected);
    stationDisconnectedHandler = WiFi.onStationModeDisconnected(&onStationDisconnected);
    stationGotIpAddress = WiFi.onStationModeGotIP(&onStationGotIP);
    stationDhcpTimeout = WiFi.onStationModeDHCPTimeout(&onStationDhcpTimeout);
    /* AP mode callbacks */
    AP_ConnectedHandler = WiFi.onSoftAPModeStationConnected(&onSoftAPModeStationConnected);
    AP_DisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&onSoftAPModeStationDisconnected);
#endif

    iotWebConf.setWifiConnectionTimeoutMs(30 * 1000); // 30sec timeout
    //iotWebConf.setStatusPin(STATUS_PIN);
    //iotWebConf.setConfigPin(CONFIG_PIN);
    //iotWebConf.addSystemParameter(&stringParam);
    iotWebConf.setConfigSavedCallback(&configSaved);
    //iotWebConf.setFormValidator(&formValidator);
    //iotWebConf.setWifiConnectionCallback(&wifiConnected);
    iotWebConf.skipApStartup();

    // -- Initializing the configuration.
    bool validConfig = iotWebConf.init();
    if (!validConfig) {
        //stringParamValue[0] = '\0';
    }
    iotWebConf.doLoop();

    server.on("/", handleRoot);
    server.on("/return", sendReturn);
    server.on("/mac", handleMacAddress);
    server.on("/config", []{ iotWebConf.handleConfig(); });
    server.onNotFound([](){ iotWebConf.handleNotFound(); });
    /* handling uploading firmware file (OTA update) */
#if defined(ARDUINO_ARCH_ESP32)
    server.on(
        "/update", HTTP_POST, []() {
            server.sendHeader("Connection", "close");
            //server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
            if (Update.hasError()) {
                //server.sendHeader("Connection", "close");
                server.send(200, "text/plain", "OTA flash failed!");
            } else {
                server.send(200, "text/html", "<HEAD><meta http-equiv=\"refresh\" content=\"0;url=/\"></HEAD>");
            }
            delay(100);
            ESP.restart();
        },
        []() {
            HTTPUpload &upload = server.upload();
            if (upload.status == UPLOAD_FILE_START) {
                PRINTF("Update: %s\r\n", upload.filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN))
                { //start with max available size
                    Update.printError(Serial);
                    PRINTLN("OTA update start failed...");
                } else {
                    PRINTLN("OTA update starts...");
                }
            }
            else if (upload.status == UPLOAD_FILE_WRITE)
            {
                /* flashing firmware to ESP*/
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                }
            }
            else if (upload.status == UPLOAD_FILE_END)
            {
                if (Update.end(true)) {
                    //true to set the size to the current progress
                    PRINTF("Update Success: %u\nRebooting...\r\n", upload.totalSize);
                } else {
                    Update.printError(Serial);
                    PRINTLN("Update failure...");
                }
            }
        });
#endif /* ARDUINO_ARCH_ESP32 */
}

void wifi_loop(void)
{
    iotWebConf.doLoop();
}

char const * const wifi_get_name(void)
{
    return iotWebConf.getThingName();
}
