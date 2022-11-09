#include "HAMqttDevice.h"
#include "debug_print.h"


HAMqttDevice::HAMqttDevice(
    const String& name,
    const DeviceType type,
    const String& unit,
    const String& icon,
    const String& identifier,
    const String& haMQTTPrefix) :
    _name(name),
    _type(type),
    _haMqttPrefix(haMQTTPrefix)
{
    setTopicId(identifier);

    // Preconfigure mandatory config vars that we already know
    addConfigVar("name", _name);

    // When the command topic is mandatory, enable it.
    switch(type)
    {
        case DeviceType::FAN:
            addConfigVar("pct_cmd_t", "~/pct_cmd");  // 'percentage_command_topic'
        case DeviceType::ALARM_CONTROL_PANEL:
        case DeviceType::LIGHT:
        case DeviceType::LOCK:
        case DeviceType::SWITCH:
        case DeviceType::NUMBER:
        case DeviceType::SELECT:
        case DeviceType::BUTTON:
            enableCommandTopic();
        default:
            break;
    }

    // When the state topic is mandatory, enable it.
    switch(type)
    {
        case DeviceType::FAN:
            addConfigVar("pct_stat_t", "~/pct_stat");  // 'percentage_state_topic'
        case DeviceType::ALARM_CONTROL_PANEL:
        case DeviceType::BINARY_SENSOR:
        case DeviceType::LIGHT:
        case DeviceType::LOCK:
        case DeviceType::SENSOR:
        case DeviceType::SWITCH:
        case DeviceType::NUMBER:
        case DeviceType::SELECT:
            enableStateTopic();
        default:
            break;
    }
    addMeasurementUnit(unit);
    addIcon(icon);
}

HAMqttDevice::~HAMqttDevice(){}

HAMqttDevice& HAMqttDevice::setHomeAssistantMqttPrefix(const String& prefix)
{
    _haMqttPrefix = prefix;
    return *this;
}

HAMqttDevice& HAMqttDevice::setTopicId(const String& id)
{
    // Id = id+_+name to lower case replacing spaces by underscore
    // (ex: name="Kitchen Light" -> id="kitchen_light" or id="001122334455_kitchen_light")
    String _identifier = id ? id + "_" + _name : _name;
    _identifier.replace(",", "");
    _identifier.replace(":", "");
    _identifier.replace(' ', '_');
    _identifier.toLowerCase();

    // Define the MQTT topic of the device
    _topic = _haMqttPrefix + '/' + deviceTypeToStr(_type) + '/' + _identifier;
    addConfigVar("~", _topic);
    addConfigVar("uniq_id", _identifier);
    return *this;
}

HAMqttDevice& HAMqttDevice::addIcon(const String& icon)
{
    if (!icon.isEmpty())
    {
        addConfigVar("ic", icon);
    }
    return *this;
}

HAMqttDevice& HAMqttDevice::addMeasurementUnit(const String& unit)
{
    if (!unit.isEmpty())
    {
        addConfigVar("unit_of_meas", unit);
    }
    return *this;
}

HAMqttDevice& HAMqttDevice::addDeviceClass(const DeviceClass dev_cla)
{
    String const _class = deviceClassToStr(dev_cla);
    if (_class)
    {
        addConfigVar("dev_cla", _class);  // 'dev_cla': 'device_class',
    }
    return *this;
}

HAMqttDevice& HAMqttDevice::addDeviceCategory(const DeviceCategory cat)
{
    if (cat != CATEGORY_STATE)
        addConfigVar("ent_cat", deviceCategoryToStr(cat));
    return *this;
}

HAMqttDevice& HAMqttDevice::enableCommandTopic()
{
    addConfigVar("cmd_t", "~/cmd");
    return *this;
}

HAMqttDevice& HAMqttDevice::enableStateTopic()
{
    addConfigVar("stat_t", "~/state");
    return *this;
}

HAMqttDevice& HAMqttDevice::enableAttributesTopic()
{
    addConfigVar("json_attr_t", "~/attr");
    return *this;
}

HAMqttDevice& HAMqttDevice::addConfigVar(const String& name, const String& value)
{
    bool found = false;
    // replace if key exists
    std::for_each(_configVars.begin(), _configVars.end(),
                  [&](ConfigVar &i) {
                    if (i.key == name) {
                        i.value = value;
                        PRINTF("Var '%s' updated to '%s'\r\n", i.key.c_str(), i.value.c_str());
                        found = true;
                    }
                });
    if (!found)
        // add if not found
        _configVars.push_back({ name, value });
    return *this;
}

HAMqttDevice& HAMqttDevice::addAttribute(const String& name, const String& value)
{
    bool found = false;
    // replace if key exists
    std::for_each(_attributes.begin(), _attributes.end(),
                  [&](Attribute &i) {
                    if (i.key == name) {
                        i.value = value;
                        found = true;
                    }
                });
    if (!found)
        // add if not found
        _attributes.push_back({ name, value });
    return *this;
}

HAMqttDevice& HAMqttDevice::clearAttributes()
{
    _attributes.clear();
    return *this;
}

const String HAMqttDevice::getConfigPayload() const
{
    String configPayload = "{";
    bool valueIsDictionnary, valueIsString, valueIsList;

    for (uint8_t i = 0 ; i < _configVars.size() ; i++)
    {
        configPayload.concat('"');
        configPayload.concat(_configVars[i].key);
        configPayload.concat("\":");

        valueIsDictionnary = _configVars[i].value[0] == '{';
        valueIsList = _configVars[i].value[0] == '[';
        valueIsString = _configVars[i].value[0] == '"';

        if (!valueIsDictionnary && !valueIsString && !valueIsList)
            configPayload.concat('"');

        configPayload.concat(_configVars[i].value);

        if (!valueIsDictionnary && !valueIsString && !valueIsList)
            configPayload.concat('"');

        configPayload.concat(',');
    }
    configPayload.setCharAt(configPayload.length()-1, '}');

    return configPayload;
}

const String HAMqttDevice::getAttributesPayload() const
{
    String attrPayload = "{";

    for (uint8_t i = 0 ; i < _attributes.size() ; i++)
    {
        attrPayload.concat('"');
        attrPayload.concat(_attributes[i].key);
        attrPayload.concat("\":\"");
        attrPayload.concat(_attributes[i].value);
        attrPayload.concat("\",");
    }
    attrPayload.setCharAt(attrPayload.length()-1, '}');

    return attrPayload;
}

String HAMqttDevice::deviceTypeToStr(DeviceType const type)
{
    switch (type)
    {
        case DeviceType::ALARM_CONTROL_PANEL: return "alarm_control_panel";
        case DeviceType::BINARY_SENSOR: return "binary_sensor";
        case DeviceType::CAMERA: return "camera";
        case DeviceType::COVER: return "cover";
        case DeviceType::FAN: return "fan";
        case DeviceType::LIGHT: return "light";
        case DeviceType::LOCK: return "lock";
        case DeviceType::SENSOR: return "sensor";
        case DeviceType::SWITCH: return "switch";
        case DeviceType::NUMBER: return "number";
        case DeviceType::CLIMATE: return "climate";
        case DeviceType::VACUUM: return "vacuum";
        case DeviceType::SELECT: return "select";
        case DeviceType::BUTTON: return "button";
        default: return "[Unknown DeviceType]";
    }
}

String HAMqttDevice::deviceClassToStr(DeviceClass const type)
{
    switch (type) {
        case CLASS_VA: return "apparent_power";
        case CLASS_AQI: return "aqi";
        case CLASS_BATTERY: return "battery";
        case CLASS_CO2: return "carbon_dioxide";
        case CLASS_CO: return "carbon_monoxide";
        case CLASS_AMPS: return "current";
        case CLASS_DATE: return "date";
        case CLASS_DISTANCE: return "distance";
        case CLASS_DURATION: return "duration";
        case CLASS_ENERGY: return "energy";
        case CLASS_FREQ: return "frequency";
        case CLASS_GAS: return "gas";
        case CLASS_HUMIDITY: return "humidity";
        case CLASS_ILLUMINANECE: return "illuminance";
        case CLASS_MOISTURE: return "moisture";
        case CLASS_MONETARY: return "monetary";
        case CLASS_POWER_FACTOR: return "power_factor";
        case CLASS_POWER: return "power";
        case CLASS_PRESSURE: return "pressure";
        case CLASS_REACTIVE_POWER: return "reactive_power";
        case CLASS_SIGNAL_STRENGTH: return "signal_strength";
        case CLASS_SPEED: return "speed";
        case CLASS_TEMPERATURE: return "temperature";
        case CLASS_TIMESTAMP: return "timestamp";
        case CLASS_VOLTAGE: return "voltage";
        case CLASS_VOLUME: return "volume";
        case CLASS_WEIGHT: return "weight";
        case CLASS_NONE:
        default: return "";
    }
}

String HAMqttDevice::deviceCategoryToStr(DeviceCategory const cat)
{
    switch (cat) {
        case CATEGORY_CONFIG: return "config";
        case CATEGORY_DIAGNOSTIC: return "diagnostic";
        case CATEGORY_STATE:
        default:
            return "";
    }
}
