#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>
#else
#include <user_interface.h>
#endif
#include "wifi.h"
#include "mqtt.h"
#include "BaseClass.h"
#include "debug_print.h"

#if CONTROLLER_VALLOX
#include "ValloxRS485.h"
ValloxRS485 contoller;
#else
#warning "Dummy implementation only!"
BaseClass contoller;
#endif


#if defined(ARDUINO_ARCH_ESP32)
#if 0
void initVariant(void)
{
    /* Disable watchdogs to precent unwanted resets */
    disableCore0WDT();
    disableCore1WDT();

    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
}

void feedTheDog(void)
{
    // feed dog 0
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
    TIMERG0.wdt_feed = 1;                       // feed dog
    TIMERG0.wdt_wprotect = 0;                   // write protect
    // feed dog 1
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
    TIMERG1.wdt_feed = 1;                       // feed dog
    TIMERG1.wdt_wprotect = 0;                   // write protect
}
#endif
#define feedTheDog yield
#elif defined(ARDUINO_ARCH_ESP8266)
#define feedTheDog yield
#endif

void setup()
{
    String boot_log;
#if defined(ARDUINO_ARCH_ESP32)
    esp_reset_reason_t const rst_reason = esp_reset_reason();
    switch (rst_reason) {
        case ESP_RST_UNKNOWN:
            boot_log += "Reset: Unkown";
            break;
        case ESP_RST_EXT:
            boot_log += "Reset: external reset";
            break;
        case ESP_RST_SW:
            boot_log += "Reset: software restart";
            break;
        case ESP_RST_PANIC:
            boot_log += "Reset: exception (panic)";
            break;
        case ESP_RST_INT_WDT:
            boot_log += "Reset: interrupt watchdog";
            break;
        case ESP_RST_TASK_WDT:
            boot_log += "Reset: task watchdog";
            break;
        case ESP_RST_WDT:
            boot_log += "Reset: other watchdog";
            break;
        case ESP_RST_BROWNOUT:
            boot_log += "Reset: brownout reset";
            break;
        case ESP_RST_SDIO:
            boot_log += "Reset: SDIO reset";
            break;
        case ESP_RST_DEEPSLEEP:
        case ESP_RST_POWERON:
        default:
            /* 0 = normal startup by power on */
            break;
    }
#elif defined(ARDUINO_ARCH_ESP8266)
    rst_info const * const resetInfo = ESP.getResetInfoPtr();
    //ESP.getResetInfo();  // -> String()
    switch (resetInfo->reason) {
        case REASON_WDT_RST:
            /* 1 = hardware watch dog reset */
            boot_log += "Reset: HW WD";
            break;
        case REASON_EXCEPTION_RST:
            /* 2 = exception reset, GPIO status won’t change */
            boot_log += "Reset: Exception";
            break;
        case REASON_SOFT_WDT_RST:
            /* 3 = software watch dog reset, GPIO status won’t change */
            boot_log += "Reset: SW WD";
            break;
        case REASON_SOFT_RESTART:
            /* 4 = software restart ,system_restart , GPIO status won’t change */
            boot_log += "Reset: SW restart";
            break;
        case REASON_DEEP_SLEEP_AWAKE:
            /* 5 = wake up from deep-sleep */
            boot_log += "Reset: deep sleep wakeup";
            break;
        case REASON_EXT_SYS_RST:
            /* 6 = external system reset */
            boot_log += "Reset: External";
            break;
        case REASON_DEFAULT_RST:
        default:
            /* 0 = normal startup by power on */
            break;
    }
#endif

#if DEBUG_ENABLED
    // Serial.setRxBufferSize(512);
    Serial.begin(MONITOR_SPEED);
#endif

    PRINTLN(" ==== ESP MQTT Client ====");
    if (boot_log)
        PRINTLN(boot_log);

    // Setup web services
    wifi_setup();

#if !MQTT_DISABLED
    // Init MQTT services
    mqtt_init();
#endif

    contoller.begin();
    PRINTLN("...Setup done!");
}


void loop()
{
    uint32_t const now = millis();
    contoller.loop(now);
    feedTheDog();
    wifi_loop();
#if !MQTT_DISABLED
    feedTheDog();
    mqtt_handle(now);
#endif
}
