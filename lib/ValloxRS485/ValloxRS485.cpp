#include "ValloxRS485.h"
#include "mqtt.h"
#include "helpers.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DigitModBus.h"
#include "debug_print.h"

#ifndef VALLOX_PIN_CTRL
#define VALLOX_PIN_CTRL -1
#endif


#if !defined(VALLOX_PIN_RX) || !defined(VALLOX_PIN_TX)
#error "Mandatory pins are not defined!"
#endif


#define ADDRESS_CLIENT       2
#define ADDRESS_MASTER       1
#define COMMUNICATION_SPEED  9600


static DigitModBus vallox_modbus(VALLOX_PIN_RX, VALLOX_PIN_TX, VALLOX_PIN_CTRL);


static HAMqttDevice dev_temp_fresh("Temperature, fresh in", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_TEMPERATURE, "°C");
static HAMqttDevice dev_temp_push("Temperature, fresh push", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_TEMPERATURE, "°C");
static HAMqttDevice dev_temp_pull("Temperature, waste pull", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_TEMPERATURE, "°C");
static HAMqttDevice dev_temp_waste("Temperature, waste out", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_TEMPERATURE, "°C");

static HAMqttDevice dev_fan_speed("Fan Speed", HAMqttDevice::SELECT, "", "mdi:fan");
static HAMqttDevice dev_fan_speed_base("Fan Speed Basic", HAMqttDevice::SELECT, "", "mdi:fan");

static HAMqttDevice dev_humidity("Humidity", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_HUMIDITY, "%");  // mdi:water-percent
static HAMqttDevice dev_carbon_dioxide("CO2", HAMqttDevice::SENSOR, HAMqttDevice::CLASS_CO2, "PPM");  // mdi:molecule-co2

static HAMqttDevice dev_power("Power", HAMqttDevice::SWITCH, "", "mdi:power");
static HAMqttDevice dev_carbon_dioxide_en("CO2 Control", HAMqttDevice::SWITCH, "", "mdi:toggle-switch-outline");
static HAMqttDevice dev_humidity_en("Humidity Control", HAMqttDevice::SWITCH, "", "mdi:toggle-switch-outline");
static HAMqttDevice dev_post_heating_en("Post Heating Control", HAMqttDevice::SWITCH, "", "mdi:toggle-switch-outline");
static HAMqttDevice dev_filter("Change Filter", HAMqttDevice::BINARY_SENSOR, "", "mdi:air-filter");
static HAMqttDevice dev_post_heating("Heating", HAMqttDevice::BINARY_SENSOR, "", "mdi:heating-coil");
static HAMqttDevice dev_error("Error", HAMqttDevice::BINARY_SENSOR, "", "mdi:fan-alert");
static HAMqttDevice dev_maintenance("Maintenance", HAMqttDevice::BINARY_SENSOR, "", "mdi:wrench-clock");

static HAMqttDevice dev_fireplace("Fireplace Boost", HAMqttDevice::SWITCH, "", "mdi:toggle-switch-outline");

static HAMqttDevice dev_rh_base("Humidity Base", HAMqttDevice::NUMBER, "%", "mdi:water-percent");


static inline String respInt8(int8_t const val) {
    if (val != INT8_UNKNOWN)
        return String(val);
    return "";
}
static inline String respUInt8(uint8_t const val) {
    if (val != UINT8_UNKNOWN)
        return String(val);
    return "";
}

static String temperature_fresh(void) {
    return respInt8(vallox_modbus.getTemperatureOutside());
}
static String temperature_push(void) {
    return respInt8(vallox_modbus.getTemperatureIncoming());
}
static String temperature_pull(void) {
    return respInt8(vallox_modbus.getTemperatureInside());
}
static String temperature_waste(void) {
    return respInt8(vallox_modbus.getTemperatureExhaust());
}

static String fanSpeedGet(void) {
    return respUInt8(vallox_modbus.getFanSpeed());
}
static void fanSpeedSet(const String &payload) {
    uint8_t const speed = payload.toInt();
    vallox_modbus.setFanSpeed(speed);
}

static String fanSpeedBaseGet(void) {
    return respUInt8(vallox_modbus.getFanSpeedBase());
}
static void fanSpeedBaseSet(const String &payload) {
    uint8_t const speed = payload.toInt();
    vallox_modbus.setFanSpeedBase(speed);
}

static String humidityGet(void) {
    float const resp = vallox_modbus.getHumidityCurrentHighest();
    if (resp != FLOAT_UNKNOWN)
        return String(resp);
    return "0";
}
static String carbonDioxideGet(void) {
    uint16_t const resp = vallox_modbus.getCurrentLevelCO2();
    if (resp != UINT16_UNKNOWN)
        return String(resp);
    return "0";
}

static String switch_power_get(void) {
    return String(vallox_modbus.getPilotLightPower() ? "ON" : "OFF");
}
static void switch_power_set(const String &payload) {
    PRINTF("Power set: %s !! ignored !!\r\n", payload.c_str());
    //vallox_modbus.setPilotLightPower(payload.equals("ON"));
}

static String switch_co2_en_get(void) {
    return String(vallox_modbus.getPilotLightCo2En() ? "ON" : "OFF");
}
static void switch_co2_en_set(const String &payload) {
    PRINTF("CO2 en set: %s\r\n", payload.c_str());
    vallox_modbus.setPilotLightCo2En(payload.equals("ON"));
}

static String switch_rh_en_get(void) {
    return String(vallox_modbus.getPilotLightHumidityEn() ? "ON" : "OFF");
}
static void switch_rh_en_set(const String &payload) {
    PRINTF("RH en set: %s\r\n", payload.c_str());
    vallox_modbus.setPilotLightHumidityEn(payload.equals("ON"));
}

static String switch_heat_en_get(void) {
    return String(vallox_modbus.getPilotLightPostHeatingEn() ? "ON" : "OFF");
}
static void switch_heat_en_set(const String &payload) {
    PRINTF("Post Heating set: %s\r\n", payload.c_str());
    vallox_modbus.setPilotLightPostHeatingEn(payload.equals("ON"));
}

static String binary_filter_get(void) {
    return String(vallox_modbus.getPilotLightFilter() ? "ON" : "OFF");
}
static String binary_heating_get(void) {
    return String(vallox_modbus.getPilotLightPostHeating() ? "ON" : "OFF");
}
static String binary_error_get(void) {
    return String(vallox_modbus.getPilotLightError() ? "ON" : "OFF");
}
static String binary_maintenance_get(void) {
    return String(vallox_modbus.getPilotLightMaintenance() ? "ON" : "OFF");
}

static String switch_fireplace_boost_get(void) {
    return String(vallox_modbus.fireplaceBoostGet() ? "ON" : "OFF");
}
static void switch_fireplace_boost_set(const String &payload) {
    PRINTF("Fireplace boost set: %s\r\n", payload.c_str());
    vallox_modbus.fireplaceBoostSet(payload.equals("ON"));
}

static String number_rh_base_get(void) {
    return respUInt8(vallox_modbus.getHumidityBasicLevel());
}
static void number_rh_base_set(const String &payload) {
    PRINTF("RH base level set: '%s'\r\n", payload.c_str());
    vallox_modbus.setHumidityBasicLevel(payload.toInt());
}


static mqtt_topic_config_t devs_all[] = {
    {.dev=&dev_temp_fresh, .get=temperature_fresh, .set=NULL},
    {.dev=&dev_temp_push, .get=temperature_push, .set=NULL},
    {.dev=&dev_temp_pull, .get=temperature_pull, .set=NULL},
    {.dev=&dev_temp_waste, .get=temperature_waste, .set=NULL},
    {.dev=&dev_fan_speed, .get=fanSpeedGet, .set=fanSpeedSet},
    {.dev=&dev_fan_speed_base, .get=fanSpeedBaseGet, .set=fanSpeedBaseSet},
    {.dev=&dev_humidity, .get=humidityGet, .set=NULL},
    {.dev=&dev_carbon_dioxide, .get=carbonDioxideGet, .set=NULL},
    // Switches
    {.dev=&dev_power, .get=switch_power_get, .set=switch_power_set},
    {.dev=&dev_carbon_dioxide_en, .get=switch_co2_en_get, .set=switch_co2_en_set},
    {.dev=&dev_humidity_en, .get=switch_rh_en_get, .set=switch_rh_en_set},
    {.dev=&dev_post_heating_en, .get=switch_heat_en_get, .set=switch_heat_en_set},
    {.dev=&dev_filter, .get=binary_filter_get, .set=NULL},
    {.dev=&dev_post_heating, .get=binary_heating_get, .set=NULL},
    {.dev=&dev_error, .get=binary_error_get, .set=NULL},
    {.dev=&dev_maintenance, .get=binary_maintenance_get, .set=NULL},

    {.dev=&dev_fireplace, .get=switch_fireplace_boost_get, .set=switch_fireplace_boost_set},
    {.dev=&dev_rh_base, .get=number_rh_base_get, .set=number_rh_base_set},
};


ValloxRS485::ValloxRS485()
{
    // Possible FAN speeds
    dev_fan_speed.addConfigVar("options", "[\"1\",\"2\",\"3\",\"4\",\"5\",\"6\",\"7\",\"8\"]");
    dev_fan_speed_base.addConfigVar("options", "[\"1\",\"2\",\"3\",\"4\",\"5\",\"6\",\"7\",\"8\"]");
    dev_fan_speed_base.addDeviceCategory(HAMqttDevice::CATEGORY_CONFIG);

    dev_power.addDeviceCategory(HAMqttDevice::CATEGORY_CONFIG);
    dev_carbon_dioxide_en.addDeviceCategory(HAMqttDevice::CATEGORY_CONFIG);
    dev_humidity_en.addDeviceCategory(HAMqttDevice::CATEGORY_CONFIG);
    dev_post_heating_en.addDeviceCategory(HAMqttDevice::CATEGORY_CONFIG);
    dev_rh_base.addDeviceCategory(HAMqttDevice::CATEGORY_CONFIG);

    dev_filter.addDeviceCategory(HAMqttDevice::CATEGORY_DIAGNOSTIC);
    dev_post_heating.addDeviceCategory(HAMqttDevice::CATEGORY_DIAGNOSTIC);
    dev_error.addDeviceCategory(HAMqttDevice::CATEGORY_DIAGNOSTIC);
    dev_maintenance.addDeviceCategory(HAMqttDevice::CATEGORY_DIAGNOSTIC);
}


ValloxRS485::~ValloxRS485()
{
    vallox_modbus.end();
}


void ValloxRS485::begin(void)
{
    PRINTLN("** VALLOX ** ");
    PRINTF("  My address %u, remote %u, speed %u",
           ADDRESS_CLIENT, ADDRESS_MASTER, COMMUNICATION_SPEED);
    PRINTLN();

    for (uint8_t iter = 0; iter < ARRAY_SIZE(devs_all); iter++) {
        mqtt_fill_device_basic_info(devs_all[iter].dev);
        mqtt_register_topic(&devs_all[iter]);
    }

    vallox_modbus.setMyAddress(ADDRESS_CLIENT);
    vallox_modbus.setRemoteAddress(ADDRESS_MASTER);
    vallox_modbus.begin(COMMUNICATION_SPEED);
}


void ValloxRS485::loop(uint32_t const now_ms)
{
    vallox_modbus.serialInputHandle(now_ms);  // empty serial rx queue
}
