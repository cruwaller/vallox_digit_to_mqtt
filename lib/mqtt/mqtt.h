#pragma once

#include <HAMqttDevice.h>
#include <Arduino.h>


void mqtt_init(void);
void mqtt_log(char const * str);
void mqtt_handle(uint32_t const now);

void mqtt_fill_device_basic_info(HAMqttDevice & topic);
void mqtt_fill_device_basic_info(HAMqttDevice * const topic);

/* MQTT client helper code */
typedef std::function<String(void)> topic_data_get_callback;
typedef std::function<void(const String &message)> topic_data_set_callback;

typedef struct device {
    HAMqttDevice* const dev;
    topic_data_get_callback get;
    topic_data_set_callback set;
} mqtt_topic_config_t;

void mqtt_register_topic(mqtt_topic_config_t const * const config);
