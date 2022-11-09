#ifndef HA_MQTT_DEVICE_H
#define HA_MQTT_DEVICE_H

#include "Arduino.h"
#include <vector>


class HAMqttDevice
{
public:
    enum DeviceType { ALARM_CONTROL_PANEL, BINARY_SENSOR, CAMERA,
                      COVER, FAN, LIGHT, LOCK, SENSOR, SWITCH,
                      CLIMATE, VACUUM, NUMBER, SELECT, BUTTON };

    enum DeviceClass {CLASS_NONE, CLASS_VA, CLASS_AQI, CLASS_BATTERY,
                      CLASS_CO2, CLASS_CO, CLASS_AMPS, CLASS_DATE, CLASS_DISTANCE,
                      CLASS_DURATION, CLASS_ENERGY, CLASS_FREQ, CLASS_GAS,
                      CLASS_HUMIDITY, CLASS_ILLUMINANECE, CLASS_MOISTURE,
                      CLASS_MONETARY, CLASS_POWER_FACTOR, CLASS_POWER,
                      CLASS_PRESSURE, CLASS_REACTIVE_POWER, CLASS_SIGNAL_STRENGTH,
                      CLASS_SPEED, CLASS_TEMPERATURE, CLASS_TIMESTAMP, CLASS_VOLTAGE,
                      CLASS_VOLUME, CLASS_WEIGHT
                      /* Add rest when needed...
                            https://www.home-assistant.io/integrations/sensor/#device-class
                       */
                      };

    enum DeviceCategory {CATEGORY_STATE, CATEGORY_CONFIG, CATEGORY_DIAGNOSTIC};

private:
    // Device proprieties
    const String _name;
    const DeviceType _type;
    String _haMqttPrefix;

    String _topic;

    // Config variables handling
    struct ConfigVar {
        String key;
        String value;
    };
    std::vector<ConfigVar> _configVars;

    // Device attributes handling
    struct Attribute {
        String key;
        String value;
    };
    std::vector<Attribute> _attributes;

public:
    HAMqttDevice(
        const String& name,
        const DeviceType type,
        const String& unit = "",
        const String& icon = "",
        const String& identifier = "",
        const String& haMQTTPrefix = "homeassistant"
    );
    HAMqttDevice(const String& name,
                 const DeviceType type,
                 const DeviceClass dev_cla,
                 const String& unit = "",
                 const String& icon = "",
                 const String& identifier = "",
                 const String& haMQTTPrefix = "homeassistant"):
            HAMqttDevice(name, type, unit, icon, identifier, haMQTTPrefix)
    {
        addDeviceClass(dev_cla);
        switch (dev_cla) {
            case CLASS_TEMPERATURE:
            case CLASS_HUMIDITY:
            case CLASS_CO2:
                setTopicValueFloat();
                break;
            default:
                break;
        }
    }
    ~HAMqttDevice();

    HAMqttDevice& setHomeAssistantMqttPrefix(const String& haPrefix);

    HAMqttDevice& setTopicId(const String& id);
    HAMqttDevice& addIcon(const String& icon);
    HAMqttDevice& addMeasurementUnit(const String& unit);
    HAMqttDevice& addDeviceClass(const DeviceClass dev_cla);
    HAMqttDevice& addDeviceCategory(const DeviceCategory cat);

    virtual HAMqttDevice& enableCommandTopic();
    virtual HAMqttDevice& enableStateTopic();
    HAMqttDevice& enableAttributesTopic();

    HAMqttDevice& addConfigVar(const String& key, const String& value);
    HAMqttDevice& addAttribute(const String& key, const String& value);
    HAMqttDevice& clearAttributes();

    const String getConfigPayload() const;
    const String getAttributesPayload() const;

    HAMqttDevice& setTopicValueInt(void) {
        addConfigVar("val_tpl", "\"{{ value | int }}\"");  // value_template
        return *this;
    }
    HAMqttDevice& setTopicValueFloat(void) {
        addConfigVar("val_tpl", "\"{{ value | float }}\"");
        return *this;
    }

    inline const String getTopic() const { return _topic; }
    inline const String getStateTopic() const { return _topic + "/state"; }
    inline const String getConfigTopic() const { return _topic + "/config"; }
    inline const String getAttributesTopic() const { return _topic + "/attr"; }
    inline const String getCommandTopic() const { return _topic + "/cmd"; }

private:
    static String deviceTypeToStr(DeviceType type);
    static String deviceClassToStr(DeviceClass type);
    static String deviceCategoryToStr(DeviceCategory cat);
};



class HAMqttDeviceJson: public HAMqttDevice
{
public:
    HAMqttDeviceJson();
    ~HAMqttDeviceJson();

    HAMqttDevice& enableCommandTopic() {
        addConfigVar("cmd_t", "~");
        return *this;
    }
    HAMqttDevice& enableStateTopic() {
        addConfigVar("stat_t", "~");
        return *this;
    }
};

#endif
