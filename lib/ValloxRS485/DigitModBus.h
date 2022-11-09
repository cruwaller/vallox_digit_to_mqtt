#ifndef DIGIT_MOD_BUS_H_
#define DIGIT_MOD_BUS_H_

/*

  HW Serials:
  ============
  Serial  :  0 (RX) and  1 (TX)
  Serial1 : 19 (RX) and 18 (TX)
  Serial2 : 17 (RX) and 16 (TX)
  Serial3 : 15 (RX) and 14 (TX)


  SW Serial:
  ============
  Note:
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

  Not all pins on the Leonardo and Micro support change interrupts,
  so only the following can be used for RX:
  8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).


 */


#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "debug_print.h"
#include "helpers.h"


#define UINT8_UNKNOWN    (UINT8_MAX)
#define INT8_UNKNOWN     (INT8_MAX)
#define FLOAT_UNKNOWN    (-255.0f)
#define UINT16_UNKNOWN   (UINT16_MAX)

#define ADDRESS_MAX_VAL         0xF
#define ADDRESS_ALL             0x0
#define ADDRESS_MAIN_BOARD(_x)  (0x10 + (_x & ADDRESS_MAX_VAL))
#define ADDRESS_REMOTE(_x)      (0x20 + (_x & ADDRESS_MAX_VAL))


class DigitModBus {

    typedef struct DigitReq_s {
        uint8_t    system;
        uint8_t    sender;
        uint8_t    receiver;
        uint8_t    request;
        uint8_t    variable;
        uint8_t    crc;
    } DigitReq_s;

    typedef struct DigitResp_s {
        uint8_t    system;
        uint8_t    sender;
        uint8_t    receiver;
        uint8_t    variable;
        uint8_t    data;
        uint8_t    crc;
    } DigitResp_s;

    typedef DigitResp_s DigitSet_s;  // Set is similar to poll resp

    typedef struct DigitAck_s {
        uint8_t    crc;
    } DigitAck_s;

    typedef enum ValloxDigitIoGenInfo_e {
        io_gen_inf_TalviKesa            = 1 << 0, // bit 1
        io_gen_inf_vikatietorele        = 1 << 1, // bit 2
        io_gen_inf_tulopuhallin         = 1 << 2, // bit 3
        io_gen_inf_etulammitys          = 1 << 3, // bit 4
        io_gen_inf_poistopuhallin       = 1 << 4, // bit 5
        io_gen_inf_takkaKytkin          = 1 << 5  // bit 6
    } ValloxDigitIoGenInfo_e;

    typedef enum ValloxDigitErrorCodes_e {
        error_tuloilman_anturivika      = 0x5,
        error_hiilidioksidi_alarm       = 0x6,
        error_ulkoilma_anturivika       = 0x7,
        error_poistoilma_anturivika     = 0x8,
        error_vesipatteri_jaatymisvaara = 0x9,
        error_jateilma_anturivika       = 0xA
    } ValloxDigitErrorCodes_e;

    // 0 = off, 1 = on
    typedef enum ValloxDigitSelectLights_e {
        light_power                     = 1 << 0,
        light_CO2                       = 1 << 1,
        light_RH                        = 1 << 2,     // kosteus anturi
        light_jalkiLammNappain          = 1 << 3,
        light_suodatinVahti             = 1 << 4,     // RO
        light_jalkiLammValo             = 1 << 5,     // RO
        light_error                     = 1 << 6,     // RO
        light_huoltomuistutin           = 1 << 7      // RO
    } ValloxDigitSelectLights_e;

    enum {
        POLL_REGISTER                               = 0x0,
        // 0x01 - 0x05
        IO_PORT1                                    = 0x06, // RO, bitmap of fan speed - Use 0x29 instead
        IO_PORT2                                    = 0x07, // RO, bitmap
        IO_PORT3_gen_info                           = 0x08, // RO, bitmap
        // 0x09 - 0x28
        FanSpeed                                    = 0x29,
        SuurinTamanHetkKosteusPros                  = 0x2A, // (x - 51) / 2.04 = %
        SuurinTamanHetkCO2_MSB                      = 0x2B, // 16-bit Pitoisuus PPM
        SuurinTamanHetkCO2_LSB                      = 0x2C,
        CO2_anturit                                 = 0x2D, // bit 1 = anturi 1 etc ... bit 5 , 0 = ei asennettu
        volt_mgs_mA_input                           = 0x2E, // scale: 00 ... 0xff
        mitattuRhPitoisuusAnt1                      = 0x2F, // (x - 51) / 2.04 = % , 0x33 = 0% .. 0xff = 100%
        mitattuRhPitoisuusAnt2                      = 0x30, // (x - 51) / 2.04 = % , 0x33 = 0% .. 0xff = 100%
        // 0x31
        ulkoLampotila                               = 0x32, // NTC asteikko
        JateilmanLampotila                          = 0x33, // NTC asteikko
        PoistoilmanLampotila                        = 0x34, // NTC asteikko
        TuloilmanLampotila                          = 0x35, // NTC asteikko
        vikatilanVirheNro                           = 0x36, // ValloxDigitErrorCodes_e
        // 0x37 - 0x54
        jalkilamm_on_secs                           = 0x55, // in sec, % = X / 2.5
        jalkilamm_off_secs                          = 0x56, // in sec, % = X / 2.5
        jalkilamm_kohdearvo                         = 0x57, // NTC asteikko
        // 0x58 - 0x6C
        FLAGS2                                      = 0x6D,
        // 0x6E
        FLAGS4                                      = 0x6F,
        FLAGS5                                      = 0x70,
        FLAGS6                                      = 0x71,
        // 0x72 - 0x78
        Tehostus_Laskuri                            = 0x79, // Aika min jaljella
        // 0x7A - 0x8E
        RS485_send_allowed                          = 0x8F, // DATA = always 0 - write only!
        // 0x90
        RS485_send_declined                         = 0x91, // DATA = always 0 - write only!
        // 0x92 - 0xA2
        Select_SignalLights                         = 0xA3, // ValloxDigitSelectLights_e
        jalkiLamm_target                            = 0xA4, // NTC value
        FanSpeed_MAX                                = 0xA5,
        huoltomuistutuksenAikaVali_kk               = 0xA6,
        etulammKytkentaLampotila                    = 0xA7, // NTC value
        tuloPuhallinPysaytysLampotila               = 0xA8, // NTC value
        FanSpeedBase                                = 0xA9,
        PROGRAM                                     = 0xAA,
        huoltomuistutuksenLaskuri_kk                = 0xAB, // Counting down
        // 0xAC , 0xAD
        perusKosteusTaso                            = 0xAE, // 0x33 = 0, 0xff = 100 , kaava: (x - 51) / 2.04
        kennonohitusToimintaLampotila               = 0xAF, // NTC value
        DC_Fan_In_procent                           = 0xB0,
        DC_Fan_Out_procent                          = 0xB1,
        KennonJaatymisenestonLampotilojenHystereesi = 0xB2, // 0x03 ~ 1C
        CO2_basic_MSB                               = 0xB3, // 16-bit PPM value
        CO2_basic_LSB                               = 0xB4, // 16-bit PPM value
        PROGRAM2                                    = 0xB5
    };

 public:
    DigitModBus(const uint8_t  rxPin,
                const uint8_t  txPin,
                const uint8_t  duplexPin = -1,
                const uint8_t  reTxCnt = 9,  // init + 9 retx
                const uint8_t  verbose = 0);
    ~DigitModBus();

    // Set addresses before calling `begin()`
    int8_t setMyAddress(uint8_t const addr) {
        if (addr && addr <= ADDRESS_MAX_VAL) {
            m_myAddr = ADDRESS_REMOTE(addr);
            return 0;
        }
        return -1;
    }
    int8_t setRemoteAddress(uint8_t const addr) {
        if (addr && addr <= ADDRESS_MAX_VAL) {
            m_remoteAddr = ADDRESS_MAIN_BOARD(addr);
            return 0;
        }
        return -1;
    }

    void begin(uint16_t const baudRate);
    void end();
    void serialInputHandle(uint32_t now_ms);


    uint8_t getFanSpeed(void) const {
        return m_variables.fanspeed.updated ? m_variables.fanspeed.value : UINT8_UNKNOWN;
    }
    void setFanSpeed(uint8_t speed) const {
        if (0 == speed || 8 < speed) return;
        speed = (1 << speed) - 1;
        sendSetValue(FanSpeed, speed);
    }

    uint8_t getFanSpeedBase(void) const {
        return m_variables.fanspeedBase.updated ? m_variables.fanspeedBase.value : UINT8_UNKNOWN;
    }
    void setFanSpeedBase(uint8_t speed) const {
        if (0 == speed || 8 < speed) return;
        speed = (1 << speed) - 1;
        sendSetValue(FanSpeedBase, speed);
    }

    uint16_t getCurrentLevelCO2(void) const {
        return m_variables.co2_level.updated ? m_variables.co2_level.value : UINT16_UNKNOWN;
    };

    float getHumidityCurrentHighest(void) const {
        if (!m_variables.humidity.updated) return FLOAT_UNKNOWN;
        return convertHumidity(m_variables.humidity.value);
    }
    float getHumiditySensor(uint8_t const sensor) const {
        if (1 == sensor) {
            if (!m_variables.humidity_rh1.updated) return FLOAT_UNKNOWN;
            return convertHumidity(m_variables.humidity_rh1.value);
        } else {
            if (!m_variables.humidity_rh2.updated) return FLOAT_UNKNOWN;
            return convertHumidity(m_variables.humidity_rh2.value);
        }
    }
    uint8_t getHumidityBasicLevel(void) const {
        if (!m_variables.humidity_basic_level.updated)
            return UINT8_UNKNOWN;
        return (uint8_t)convertHumidity(m_variables.humidity_basic_level.value);
    }
    void setHumidityBasicLevel(uint8_t const level) {
        if (level <= 100)  // range: 0...100
            sendSetValue(FanSpeed, convertHumidity2hex(level));
    }

    int8_t getTemperatureOutside(void) const {
        if (!m_variables.temperature_outdoor.updated) return INT8_UNKNOWN;
        return convertTemperature(m_variables.temperature_outdoor.value);
    }
    int8_t getTemperatureExhaust(void) const {
        if (!m_variables.temperature_exhaust.updated) return INT8_UNKNOWN;
        return convertTemperature(m_variables.temperature_exhaust.value);
    }
    int8_t getTemperatureInside(void) const {
        if (!m_variables.temperature_pull.updated) return INT8_UNKNOWN;
        return convertTemperature(m_variables.temperature_pull.value);
    }
    int8_t getTemperatureIncoming(void) const {
        if (!m_variables.temperature_push.updated) return INT8_UNKNOWN;
        return convertTemperature(m_variables.temperature_push.value);
    }

    bool getPilotLightPower(void) const {
        return m_variables.select_lights.power_btn;
    }
    void setPilotLightPower(bool const set) {
        if (!m_variables.select_lights.updated) return;
        m_variables.select_lights.power_btn = set;
        sendSetValue(Select_SignalLights, m_variables.select_lights.value);
    }
    bool getPilotLightCo2En(void) const {
        return m_variables.select_lights.co2_btn;
    }
    void setPilotLightCo2En(bool const set) {
        if (!m_variables.select_lights.updated) return;
        m_variables.select_lights.co2_btn = set;
        sendSetValue(Select_SignalLights, m_variables.select_lights.value);
    }
    bool getPilotLightHumidityEn(void) const {
        return m_variables.select_lights.rh_btn;
    }
    void setPilotLightHumidityEn(bool const set) {
        if (!m_variables.select_lights.updated) return;
        m_variables.select_lights.rh_btn = set;
        sendSetValue(Select_SignalLights, m_variables.select_lights.value);
    }
    bool getPilotLightPostHeatingEn(void) const {
        return m_variables.select_lights.post_heating_btn;
    }
    void setPilotLightPostHeatingEn(bool const set) {
        if (!m_variables.select_lights.updated) return;
        m_variables.select_lights.post_heating_btn = set;
        sendSetValue(Select_SignalLights, m_variables.select_lights.value);
    }
    bool getPilotLightFilter(void) const {
        return m_variables.select_lights.filter_light;
    }
    bool getPilotLightPostHeating(void) const {
        return m_variables.select_lights.post_heating_light;
    }
    bool getPilotLightError(void) const {
        return m_variables.select_lights.error_light;
    }
    bool getPilotLightMaintenance(void) const {
        return m_variables.select_lights.maintenance_light;
    }

    bool fireplaceBoostGet(void) const {
        return m_variables.flags6.fireplace_boost;
    }
    void fireplaceBoostSet(bool const enable) {
        if (!enable || !m_variables.flags6.updated) return;
        m_variables.flags6.fireplace_boost_trigger = 1;
        sendSetValue(FLAGS6, m_variables.flags6.value);
    }




 private:

    void message_handle(uint32_t now_ms);
    void value_request(uint32_t now_ms);

    String decodeErrorNumber(uint8_t error);

    inline float convertHumidity(uint8_t const value) const {
        if (value == UINT8_UNKNOWN || value < 51)
            return FLOAT_UNKNOWN;
        return (float)(value - 51) / 2.04f;
    }
    inline uint8_t convertHumidity2hex(float const value) const {
        return (uint8_t)(value * 2.04) + 51;
    }

    inline int8_t convertTemperature(uint8_t const value) const {
        if (value == UINT8_UNKNOWN)
            return INT8_UNKNOWN;
        return (int8_t)m_ntc_to_temp[value];
    }
    inline uint8_t convertTemperature2Ntc(int8_t const value) const {
        for (uint8_t iter = 0; iter < ARRAY_SIZE(m_ntc_to_temp); iter++) {
            if (m_ntc_to_temp[iter] == value)
                return iter;
        }
        return 0xAE;  // 25C by default - should not happen!
    }

    inline uint8_t crcCalculate(uint8_t const * _ptr, uint8_t size) const {
        uint16_t crc = 0;
        while (size--) {
            crc += *_ptr++;
        }
        return (crc & 0xff);
    }

    inline bool crcValidate(uint8_t const * const _ptr, uint8_t size) const {
        if (size) {
            size -= 1;
            return (_ptr[size] == crcCalculate(_ptr, size));
        }
        return false;
    }

    inline void cleanRxBuffer(void) const {
        if (!m_serialPtr)
            return;
        while (m_serialPtr->available())
            (void)m_serialPtr->read();
    }

    inline bool validateCommandRead(uint8_t const command) const {
        return (
            (POLL_REGISTER == command) ||
            (IO_PORT1 <= command && command <= IO_PORT3_gen_info) ||
            (FanSpeed <= command && command <= vikatilanVirheNro && 0x31 != command) ||
            (jalkilamm_on_secs <= command && command <= jalkilamm_kohdearvo) ||
            (FLAGS2 == command) ||
            (FLAGS4 <= command && command <= FLAGS6) ||
            (Tehostus_Laskuri == command) ||
            ((Select_SignalLights <= command) && (command <= huoltomuistutuksenLaskuri_kk)) ||
            ((perusKosteusTaso <= command) && (command <= PROGRAM2))
        );
    }

    inline bool validateCommandWrite(uint8_t const command) const {
        return (
            (FanSpeed == command) ||
            (jalkilamm_on_secs == command) ||
            (jalkilamm_off_secs == command) ||
            (FLAGS5 == command) ||
            (FLAGS6 == command) ||
            ((Select_SignalLights <= command) && (command <= huoltomuistutuksenLaskuri_kk)) ||
            ((perusKosteusTaso <= command) && (command <= PROGRAM2))
        );
    }

    inline bool validateCommand(uint8_t const command) const {
        return validateCommandRead(command) || validateCommandWrite(command);
    }

    inline int numOfAvailable(void) const {
        return m_serialPtr ? m_serialPtr->available() : 0;
    }

    inline int readBytes(uint8_t * const ptr, uint8_t const size) const {
        return m_serialPtr ? m_serialPtr->readBytes(ptr, size) : 0;
    }

    uint8_t sendUpdateRequest(uint8_t const command) const;
    int8_t sendSetValue(uint8_t const command, uint8_t const value,
                        uint8_t const waitAck = 1) const;
    inline void sendReq(uint8_t * data, size_t size) const;
    inline int8_t waitResp(uint8_t * data, size_t size) const;

    HardwareSerial *  m_hwSerial;
    SoftwareSerial *  m_swSerial;
    Stream          * m_serialPtr;
    uint8_t           m_ctrlPin;
    uint8_t           m_myAddr;
    uint8_t           m_remoteAddr;
    uint8_t           m_reTxCnt;
    uint8_t           m_verbose;
    union {
        DigitResp_s response;
        DigitAck_s ack;
        uint8_t data[sizeof(DigitResp_s)];
    } serial_rx_buffer;


    struct uint8_value {
        uint32_t updated;
        uint8_t value;
    };

    struct {

        struct {
            uint32_t updated;
            union {
                uint8_t value;
                struct {
                    uint8_t dummy1: 5;
                    uint8_t post_heating: 1; // 0 = off, 1 = on
                    uint8_t dummy2: 2;
                };
            };
        } io_port2;  // reg: IO_PORT2

        struct {
            uint32_t updated;
            union {
                uint8_t value;
                struct {
                    uint8_t winter_summer_plate: 1;     // 0 = winter   1 = summer
                    uint8_t error_relay: 1;             // 0 = open     1 = closed
                    uint8_t fan_input: 1;               // 0 = on       1 = off
                    uint8_t pre_heat: 1;                // 0 = off      1 = on
                    uint8_t fan_output: 1;              // 0 = on       1 = off
                    uint8_t fireplace_boost_switch: 1;  // 0 = open     1 = closed
                    uint8_t dummy: 2;
                };
            };
        } io_port3;  // reg: IO_PORT3_gen_info

        uint8_value fanspeed;  // value is converted to 1...8
        uint8_value fanspeedBase; // FanSpeedBase;

        uint8_value humidity;
        uint8_value humidity_rh1;
        uint8_value humidity_rh2;
        uint8_value humidity_basic_level;

        struct {
            uint32_t updated;
            union {
                uint16_t value;
                struct {
                    uint8_t value_lsb;
                    uint8_t value_msb;
                };
            };
        } co2_level;

        uint8_value temperature_outdoor;
        uint8_value temperature_exhaust;
        uint8_value temperature_pull;  // from indoor
        uint8_value temperature_push;  // to indoor

        uint8_value error_info;  // reg: vikatilanVirheNro

        uint8_value post_heating_target;

        struct {
            uint32_t updated;
            union {
                uint8_t value;
                struct {
                    uint8_t dummy: 7;
                    uint8_t pre_heating_state: 1;
                };
            };
        } flags5;
        struct {
            uint32_t updated;
            union {
                uint8_t value;
                struct {
                    uint8_t ignore: 4;
                    uint8_t remote_ctrl: 1;
                    uint8_t fireplace_boost_trigger: 1;
                    uint8_t fireplace_boost: 1;
                    uint8_t dummy: 1;
                };
            };
        } flags6;
        uint8_value fireplace_boost_time_min;

        struct {
            uint32_t updated;
            union {
                uint8_t value;
                struct {
                    uint8_t power_btn: 1;
                    uint8_t co2_btn: 1;
                    uint8_t rh_btn: 1;
                    uint8_t post_heating_btn: 1;
                    uint8_t filter_light: 1;
                    uint8_t post_heating_light: 1;
                    uint8_t error_light: 1;
                    uint8_t maintenance_light: 1;
                };
            };
        } select_lights;

    } m_variables;

    static const int8_t  m_ntc_to_temp[256];
};



#endif /* DIGIT_MOD_BUS_H_ */
