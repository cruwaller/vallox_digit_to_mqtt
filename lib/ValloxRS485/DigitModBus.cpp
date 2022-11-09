#include "DigitModBus.h"

#if defined(ARDUINO_ARCH_ESP8266)
#define GPIO_IS_VALID_GPIO(_pin) ((0 <= (_pin)) && ((_pin) < NUM_DIGITAL_PINS))
#endif

#define RS485Transmit       HIGH
#define RS485Receive        LOW

#define VALLOX_DOMAIN       (1)  // Domain

#define VALUES_VALID_MS     (10000)


DigitModBus::DigitModBus(const uint8_t  rxPin,
                         const uint8_t  txPin,
                         const uint8_t  duplexPin,
                         const uint8_t  reTxCnt,
                         const uint8_t  verbose)
{
    m_hwSerial = NULL;
    m_swSerial = NULL;
    m_serialPtr = NULL;
    m_ctrlPin = duplexPin;
    m_reTxCnt = reTxCnt;
    m_verbose = verbose;

    setMyAddress(2);        /* Expect original control panel to be the nbr 1 */
    setRemoteAddress(1);    /* One main controller is always nbr 1 */

#if defined(ARDUINO_ARCH_ESP8266)
    if (rxPin == 3 && txPin == 1) {
        m_serialPtr = m_hwSerial = &Serial1;
    } else
#elif defined(ARDUINO_ARCH_ESP32)
    if (rxPin == 3 && txPin == 1) {
        m_serialPtr = m_hwSerial = &Serial;
    } else if (rxPin == 9 && txPin == 10) {
        m_serialPtr = m_hwSerial = &Serial1;
    } else if (rxPin == 16 && txPin == 17) {
        m_serialPtr = m_hwSerial = &Serial2;
    } else
    /* ESP32 hwserial with matrix? */
#endif
    {
        // Use software serial...
        m_serialPtr = m_swSerial = new SoftwareSerial(rxPin, txPin);
    }
}


DigitModBus::~DigitModBus() {
    ;
}

void DigitModBus::begin(uint16_t const baudRate) {
    if (GPIO_IS_VALID_GPIO(m_ctrlPin)) {
        pinMode(m_ctrlPin, OUTPUT);
        digitalWrite(m_ctrlPin, RS485Receive);  // Init Transceiver
    }
    if (m_hwSerial) {
        m_hwSerial->begin(baudRate);
    } else {
        // Use software serial...
        m_swSerial->begin(baudRate);
    }
}

void DigitModBus::end() {
    if (m_hwSerial) {
        m_hwSerial->end();
    } else {
        m_swSerial->end();
    }
}


enum {
    STATE_SYSTEM = 0,
    STATE_SNDR,
    STATE_RCVR,
    STATE_CMD,
    STATE_VAL,
    STATE_CRC,
};

static uint8_t state, in_idx, in_len;

void DigitModBus::serialInputHandle(uint32_t const now_ms) {
    if (!m_serialPtr)
        return;

    uint8_t state_next = STATE_SYSTEM;
    int available = m_serialPtr->available();

    //if (16 < available) available = 16;
    //else if (available < 0) available = 0;

    while (available--) {
        uint8_t const data = m_serialPtr->read();
        serial_rx_buffer.data[in_idx++] = data;
        state_next = STATE_SYSTEM;
        switch (state) {
            case STATE_SYSTEM:
                if (data == VALLOX_DOMAIN)
                    state_next = STATE_SNDR;
                break;
            case STATE_SNDR:
                // Handle all messages for now
                //if (data == m_remoteAddr || data == ADDRESS_MAIN_BOARD(ADDRESS_ALL))
                    state_next = STATE_RCVR;
                break;
            case STATE_RCVR:
                // Handle all messages for now
                //if (data == m_myAddr || data == ADDRESS_REMOTE(ADDRESS_ALL))
                    state_next = STATE_CMD;
                break;
            case STATE_CMD:
                if (validateCommand(data)) {
                    state_next = STATE_VAL;
                    in_len = in_idx + sizeof(serial_rx_buffer.response.variable);
                } else {
                    PRINTF("Invalid command '%02X'\r\n", data);
                }
                break;
            case STATE_VAL:
                state_next = (in_idx < in_len) ? STATE_VAL : STATE_CRC;
                break;
            case STATE_CRC: {
                PRINTF("Sender 0x%02x, receiver 0x%02x - ",
                       serial_rx_buffer.response.sender, serial_rx_buffer.response.receiver);

                if (crcValidate((uint8_t*)serial_rx_buffer.data, sizeof(serial_rx_buffer.data))) {
                    message_handle(now_ms);
                } else {
                    PRINT("!! CRC FAIL !!");
                }
                PRINTLN();
                break;
            }
            default:
                break;
        }

        if (state_next == STATE_SYSTEM) {
            in_idx = 0;
        }

        state = state_next;
    }

    // TODO: reguest parameters if not updated lately
    value_request(now_ms);
}



/******* PRIVATE FUNCTIONS **********/

void DigitModBus::message_handle(uint32_t const now_ms)
{
    switch (serial_rx_buffer.response.variable) {
        case POLL_REGISTER:
            PRINTF("POLL << 0x%02X >>", serial_rx_buffer.response.data);
            break;

        case IO_PORT2:
            m_variables.io_port2.value = serial_rx_buffer.response.data;
            m_variables.io_port2.updated = now_ms;
            PRINTF("IO_PORT2, Post heating %u\r\n", m_variables.io_port2.post_heating);
            break;
        case IO_PORT3_gen_info:
            m_variables.io_port3.value = serial_rx_buffer.response.data;
            m_variables.io_port3.updated = now_ms;
            PRINTF("Generic IO");
            break;
        case FanSpeed:
            m_variables.fanspeed.value = __builtin_popcount(serial_rx_buffer.response.data);
            m_variables.fanspeed.updated = now_ms;
            PRINTF("Fan speed: %u", m_variables.fanspeed.value);
            break;
        case SuurinTamanHetkKosteusPros:
            m_variables.humidity.value = serial_rx_buffer.response.data;
            m_variables.humidity.updated = now_ms;
            PRINTF("RH.high: %.2f", convertHumidity(serial_rx_buffer.response.data));
            break;
        case SuurinTamanHetkCO2_MSB:
            m_variables.co2_level.value_msb = serial_rx_buffer.response.data;
            m_variables.co2_level.updated = 0; // mark to invalid until LSB is also received
            //PRINTF("CO2,MSB: %u", serial_rx_buffer.response.data);
            PRINTF("CO2,MSB");
            break;
        case SuurinTamanHetkCO2_LSB:
            m_variables.co2_level.value_lsb = serial_rx_buffer.response.data;
            m_variables.co2_level.updated = (m_variables.co2_level.updated ? 0 : now_ms);
            PRINTF("CO2, PPM: %d", (m_variables.co2_level.updated ? m_variables.co2_level.value : -1));
            break;
        //case CO2_anturit:
        //    break;
        //case volt_mgs_mA_input:
        //    break;
        case mitattuRhPitoisuusAnt1:
            m_variables.humidity_rh1.value = serial_rx_buffer.response.data;
            m_variables.humidity_rh1.updated = now_ms;
            PRINTF("RH.1: %.2f", convertHumidity(serial_rx_buffer.response.data));
            break;
        case mitattuRhPitoisuusAnt2:
            m_variables.humidity_rh2.value = serial_rx_buffer.response.data;
            m_variables.humidity_rh2.updated = now_ms;
            PRINTF("RH.2: %.2f", convertHumidity(serial_rx_buffer.response.data));
            break;
        case ulkoLampotila:
            m_variables.temperature_outdoor.value = serial_rx_buffer.response.data;
            m_variables.temperature_outdoor.updated = now_ms;
            PRINTF("C, ulko: %d", convertTemperature(serial_rx_buffer.response.data));
            break;
        case JateilmanLampotila:
            m_variables.temperature_exhaust.value = serial_rx_buffer.response.data;
            m_variables.temperature_exhaust.updated = now_ms;
            PRINTF("C, jate: %d", convertTemperature(serial_rx_buffer.response.data));
            break;
        case PoistoilmanLampotila:
            m_variables.temperature_pull.value = serial_rx_buffer.response.data;
            m_variables.temperature_pull.updated = now_ms;
            PRINTF("C, poisto: %d", convertTemperature(serial_rx_buffer.response.data));
            break;
        case TuloilmanLampotila :
            m_variables.temperature_push.value = serial_rx_buffer.response.data;
            m_variables.temperature_push.updated = now_ms;
            PRINTF("C, tulo: %d", convertTemperature(serial_rx_buffer.response.data));
            break;
        case vikatilanVirheNro:
            m_variables.error_info.value = serial_rx_buffer.response.data;
            m_variables.error_info.updated = now_ms;
            PRINT("ERROR: ");
            PRINT(decodeErrorNumber(serial_rx_buffer.response.data));
            break;
        //case jalkilamm_on_secs:
        //    break;
        //case jalkilamm_off_secs:
        //    break;
        //case jalkilamm_kohdearvo:
        //    break;
        //case FLAGS2:
        //    break;
        //case FLAGS4:
        //    break;
        case FLAGS5:
            m_variables.flags5.value = serial_rx_buffer.response.data;
            m_variables.flags5.updated = now_ms;
            PRINTF("FLAGS5");
            break;
        case FLAGS6:
            m_variables.flags6.value = serial_rx_buffer.response.data;
            m_variables.flags6.updated = now_ms;
            PRINTF("FLAGS6");
            break;
        case Tehostus_Laskuri:
            m_variables.fireplace_boost_time_min.value = serial_rx_buffer.response.data;
            m_variables.fireplace_boost_time_min.updated = now_ms;
            PRINTF("Boost counter, min: %u", serial_rx_buffer.response.data);
            break;
        case Select_SignalLights: {
            m_variables.select_lights.value = serial_rx_buffer.response.data;
            m_variables.select_lights.updated = now_ms;
#if DEBUG_ENABLED
            PRINT("Select ::: Buttons:");
            if (m_variables.select_lights.post_heating_btn) PRINT(" PWR");
            if (m_variables.select_lights.co2_btn) PRINT(" CO2");
            if (m_variables.select_lights.rh_btn) PRINT(" RH");
            if (m_variables.select_lights.post_heating_btn) PRINT(" PostHeat");
            PRINT(" ::: Lights:");
            if (m_variables.select_lights.filter_light) PRINT(" Filter");
            if (m_variables.select_lights.post_heating_light) PRINT(" PostHeat");
            if (m_variables.select_lights.error_light) PRINT(" Error");
            if (m_variables.select_lights.maintenance_light) PRINT(" Maintenance");
#endif
            break;
        }
        case jalkiLamm_target:
            m_variables.post_heating_target.value = serial_rx_buffer.response.data;
            m_variables.post_heating_target.updated = now_ms;
            PRINTF("Jalkilammitys, target: %d", convertTemperature(serial_rx_buffer.response.data));
            break;
        // case FanSpeed_MAX:
        //     break;
        // case huoltomuistutuksenAikaVali_kk:
        //     break;
        // case etulammKytkentaLampotila:
        //     break;
        // case tuloPuhallinPysaytysLampotila:
        //     break;
        case FanSpeedBase:
            m_variables.fanspeedBase.value = __builtin_popcount(serial_rx_buffer.response.data);
            m_variables.fanspeedBase.updated = now_ms;
            PRINTF("Base fan speed: %u", m_variables.fanspeedBase.value);
            break;
        // case PROGRAM:
        //     break;
        // case huoltomuistutuksenLaskuri_kk:
        //     break;
        case perusKosteusTaso:
            m_variables.humidity_basic_level.value = serial_rx_buffer.response.data;
            m_variables.humidity_basic_level.updated = now_ms;
            PRINTF("RH.base: %.2f", convertHumidity(serial_rx_buffer.response.data));
            break;
        // case kennonohitusToimintaLampotila:
        //     break;
        // case DC_Fan_In_procent:
        //     break;
        // case DC_Fan_Out_procent:
        //     break;
        // case KennonJaatymisenestonLampotilojenHystereesi:
        //     break;
        // case CO2_basic_MSB:
        //     break;
        // case CO2_basic_LSB:
        //     break;
        // case PROGRAM2:
        //     break;
        default:
            PRINTF("Register: 0x%02X = 0x%02X", serial_rx_buffer.response.variable, serial_rx_buffer.response.data);
            break;
    }
}

void DigitModBus::value_request(uint32_t const now_ms)
{
    static uint32_t last_call_ms;
    // Limit send cycle a bit to give some time for
    //      the main board to respond
    if ((now_ms - last_call_ms) < 20)
        return;
    last_call_ms = now_ms;

    /* Temperatures, CO2 and RH,highest are all reported automatically
        Regs:
            SuurinTamanHetkKosteusPros
            SuurinTamanHetkCO2_MSB
            SuurinTamanHetkCO2_LSB
            ulkoLampotila
            JateilmanLampotila
            PoistoilmanLampotila
            TuloilmanLampotila
     */

    #define UPDATE_INTERVAL_ms (60 * 1000)  // 1min

    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.io_port2.updated))
        sendUpdateRequest(IO_PORT2);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.io_port3.updated))
        sendUpdateRequest(IO_PORT3_gen_info);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.fanspeed.updated))
        sendUpdateRequest(FanSpeed);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.post_heating_target.updated))
        sendUpdateRequest(jalkiLamm_target);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.flags5.updated))
        sendUpdateRequest(FLAGS5);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.flags6.updated))
        sendUpdateRequest(FLAGS6);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.fireplace_boost_time_min.updated))
        sendUpdateRequest(Tehostus_Laskuri);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.select_lights.updated))
        sendUpdateRequest(Select_SignalLights);
    if (UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.humidity_basic_level.updated))
        sendUpdateRequest(perusKosteusTaso);

    // Fetch the error info
    if (m_variables.select_lights.error_light &&
            UPDATE_INTERVAL_ms < (int32_t)(now_ms - m_variables.error_info.updated))
        sendUpdateRequest(vikatilanVirheNro);
}

String DigitModBus::decodeErrorNumber(uint8_t const error)
{
    switch (error) {
        case 0x05: return "Input air sensor fault";
        case 0x06: return "CO2 alarm";
        case 0x07: return "Outdoor air sensor fault";
        case 0x08: return "Output air sensor fault";
        case 0x09: return "Water heater freezing alarm";
        case 0x0A: return "Exhaust air sensor fault";
        default: return "";
    };
}

uint8_t DigitModBus::sendUpdateRequest(uint8_t const command) const {
    // Fetch latest value
    if (validateCommandRead(command)) {
        DigitReq_s        request;
        //DigitResp_s       response = {0};

        request.system   = VALLOX_DOMAIN;    // static
        request.sender   = m_myAddr;
        request.receiver = m_remoteAddr;
        request.request  = POLL_REGISTER;   // static
        request.variable = command;         // Read value from serial to be used! Be sure it is valid command!!
        request.crc      = crcCalculate((uint8_t*)&request, (sizeof(request) - 1));

        PRINTF("Read variable: 0x%02X\r\n", command);

#if 0
        for (uint8_t reqCnt = 0; reqCnt <= m_reTxCnt; reqCnt++) {
            PRINTF("  tx %u -- ", (reqCnt+1));
            cleanRxBuffer();
            sendReq((uint8_t*)&request, sizeof(request));
            int8_t const respRet = waitResp((uint8_t*)&response, sizeof(response));
            if (respRet < 0) {
                PRINTLN("no resp!");
                delay(10);
                continue;
            }

            if (response.system    == VALLOX_DOMAIN &&
                    response.receiver  == request.sender &&
                    response.receiver  == request.sender &&
                    response.variable  == request.variable) {
                PRINTF("value = 0x%02X\r\n", response.data);
                return response.data;
            } else {
                PRINTLN("invalid resp!");
            }
        }
#else
        sendReq((uint8_t*)&request, sizeof(request));
        delay(20);
#endif
    }
    return UINT8_UNKNOWN;
}

int8_t DigitModBus::sendSetValue(uint8_t const command, uint8_t const value,
                                 uint8_t const waitAck) const {

    if (validateCommandWrite(command)) {
        DigitSet_s request;
        //DigitAck_s response = {0};

        request.system   = VALLOX_DOMAIN;    // static
        request.sender   = m_myAddr;
        request.receiver = m_remoteAddr;
        request.variable = command;
        request.data     = value;
        request.crc      = crcCalculate((uint8_t*)&request, (sizeof(request) - 1));

        PRINTF("Sending 'set' 0x%02X = 0x%02X\r\n", command, value);

#if 0
        for (uint8_t reqCnt = 0; reqCnt <= m_reTxCnt; reqCnt++) {
            PRINTF("  tx %u -- ", (reqCnt+1));
            cleanRxBuffer();
            sendReq((uint8_t*)&request, sizeof(request));

            if (!waitAck) {
                PRINTLN("unacknowledged!");
                return 0;
            }

            int8_t const respRet = waitResp((uint8_t*)&response, sizeof(response));
            if (respRet < 0) {
                PRINTLN("no ack!");
                delay(10);
                continue;
            }

            if (response.crc == request.crc) {
                PRINTLN("ok");
                return 0;
            } else {
                PRINTLN("invalid ack!");
            }
        }
#else
        sendReq((uint8_t*)&request, sizeof(request));

        // Send to other panels
        request.sender   = m_remoteAddr;
        request.receiver = ADDRESS_REMOTE(ADDRESS_ALL);
        request.crc      = crcCalculate((uint8_t*)&request, (sizeof(request) - 1));
        sendReq((uint8_t*)&request, sizeof(request));
#endif
    } else {
        PRINTF("No writable register '0x%02X'!!\r\n", command);
    }
    return -1;  // failed
}

inline void DigitModBus::sendReq(uint8_t * data, size_t const size) const {
    if (!m_serialPtr)
        return;
    if (GPIO_IS_VALID_GPIO(m_ctrlPin))
        digitalWrite(m_ctrlPin, RS485Transmit); // Enable RS485 Transmit
    m_serialPtr->write(data, size);
    if (GPIO_IS_VALID_GPIO(m_ctrlPin)) {
        m_serialPtr->flush();
        digitalWrite(m_ctrlPin, RS485Receive);  // Disable RS485 Transmit
    }
}

inline int8_t DigitModBus::waitResp(uint8_t * respPtr, size_t const size) const {
    unsigned long const readTime = millis();
    while (numOfAvailable() < (int)size) {
        if (10 < (millis() - readTime)) {
            if (m_verbose)
                PRINTLN("Resp timeout!");
            return -1;
        }
    }
    if (readBytes(respPtr, size) != (int)size) {
        if (m_verbose)
            PRINTLN("readBytes - Failed!");
        return -2;
    }

    if (m_verbose) {
        PRINTF("Resp received in [ms] : %u - ", (uint32_t)(millis() - readTime));
        for (uint8_t iter = 0; iter < size; iter++) {
            PRINTF("0x%02X ", respPtr[iter]);
        }
    }

    if (!crcValidate(respPtr, size)) {
        if (m_verbose)
            PRINTLN(" -> CRC Failed!");
        return -3;
    }

    if (m_verbose)
        PRINTLN();

    return 0;
}



const int8_t DigitModBus::m_ntc_to_temp[256] = {
    //HEX ,     °C
    /*0x00 */ -74,
    /*0x01 */ -70,
    /*0x02 */ -66,
    /*0x03 */ -62,
    /*0x04 */ -59,
    /*0x05 */ -56,
    /*0x06 */ -54,
    /*0x07 */ -52,
    /*0x08 */ -50,
    /*0x09 */ -48,
    /*0x0A */ -47,
    /*0x0B */ -46,
    /*0x0C */ -44,
    /*0x0D */ -43,
    /*0x0E */ -42,
    /*0x0F */ -41,
    /*0x10 */ -40,
    /*0x11 */ -39,
    /*0x12 */ -38,
    /*0x13 */ -37,
    /*0x14 */ -36,
    /*0x15 */ -35,
    /*0x16 */ -34,
    /*0x17 */ -33,
    /*0x18 */ -33,
    /*0x19 */ -32,
    /*0x1A */ -31,
    /*0x1B */ -30,
    /*0x1C */ -30,
    /*0x1D */ -29,
    /*0x1E */ -28,
    /*0x1F */ -28,
    /*0x20 */ -27,
    /*0x21 */ -27,
    /*0x22 */ -26,
    /*0x23 */ -25,
    /*0x24 */ -25,
    /*0x25 */ -24,
    /*0x26 */ -24,
    /*0x27 */ -23,
    /*0x28 */ -23,
    /*0x29 */ -22,
    /*0x2A */ -22,
    /*0x2B */ -21,
    /*0x2C */ -21,
    /*0x2D */ -20,
    /*0x2E */ -20,
    /*0x2F */ -19,
    /*0x30 */ -19,
    /*0x31 */ -19,
    /*0x32 */ -18,
    /*0x33 */ -18,
    /*0x34 */ -17,
    /*0x35 */ -17,
    /*0x36 */ -16,
    /*0x37 */ -16,
    /*0x38 */ -16,
    /*0x39 */ -15,
    /*0x3A */ -15,
    /*0x3B */ -14,
    /*0x3C */ -14,
    /*0x3D */ -14,
    /*0x3E */ -13,
    /*0x3F */ -13,
    /*0x40 */ -12,
    /*0x41 */ -12,
    /*0x42 */ -12,
    /*0x43 */ -11,
    /*0x44 */ -11,
    /*0x45 */ -11,
    /*0x46 */ -10,
    /*0x47 */ -10,
    /*0x48 */  -9,
    /*0x49 */  -9,
    /*0x4A */  -9,
    /*0x4B */  -8,
    /*0x4C */  -8,
    /*0x4D */  -8,
    /*0x4E */  -7,
    /*0x4F */  -7,
    /*0x50 */  -7,
    /*0x51 */  -6,
    /*0x52 */  -6,
    /*0x53 */  -6,
    /*0x54 */  -5,
    /*0x55 */  -5,
    /*0x56 */  -5,
    /*0x57 */  -4,
    /*0x58 */  -4,
    /*0x59 */  -4,
    /*0x5A */  -3,
    /*0x5B */  -3,
    /*0x5C */  -3,
    /*0x5D */  -2,
    /*0x5E */  -2,
    /*0x5F */  -2,
    /*0x60 */  -1,
    /*0x61 */  -1,
    /*0x62 */  -1,
    /*0x63 */  -1,
    /*0x64 */   0,
    /*0x65 */   0,
    /*0x66 */   0,
    /*0x67 */   1,
    /*0x68 */   1,
    /*0x69 */   1,
    /*0x6A */   2,
    /*0x6B */   2,
    /*0x6C */   2,
    /*0x6D */   3,
    /*0x6E */   3,
    /*0x6F */   3,
    /*0x70 */   4,
    /*0x71 */   4,
    /*0x72 */   4,
    /*0x73 */   5,
    /*0x74 */   5,
    /*0x75 */   5,
    /*0x76 */   5,
    /*0x77 */   6,
    /*0x78 */   6,
    /*0x79 */   6,
    /*0x7A */   7,
    /*0x7B */   7,
    /*0x7C */   7,
    /*0x7D */   8,
    /*0x7E */   8,
    /*0x7F */   8,
    /*0x80 */   9,
    /*0x81 */   9,
    /*0x82 */   9,
    /*0x83 */  10,
    /*0x84 */  10,
    /*0x85 */  10,
    /*0x86 */  11,
    /*0x87 */  11,
    /*0x88 */  11,
    /*0x89 */  12,
    /*0x8A */  12,
    /*0x8B */  12,
    /*0x8C */  13,
    /*0x8D */  13,
    /*0x8E */  13,
    /*0x8F */  14,
    /*0x90 */  14,
    /*0x91 */  14,
    /*0x92 */  15,
    /*0x93 */  15,
    /*0x94 */  15,
    /*0x95 */  16,
    /*0x96 */  16,
    /*0x97 */  16,
    /*0x98 */  17,
    /*0x99 */  17,
    /*0x9A */  18,
    /*0x9B */  18,
    /*0x9C */  18,
    /*0x9D */  19,
    /*0x9E */  19,
    /*0x9F */  19,
    /*0xA0 */  20,
    /*0xA1 */  20,
    /*0xA2 */  21,
    /*0xA3 */  21,
    /*0xA4 */  21,
    /*0xA5 */  22,
    /*0xA6 */  22,
    /*0xA7 */  22,
    /*0xA8 */  23,
    /*0xA9 */  23,
    /*0xAA */  24,
    /*0xAB */  24,
    /*0xAC */  24,
    /*0xAD */  25,
    /*0xAE */  25,
    /*0xAF */  26,
    /*0xB0 */  26,
    /*0xB1 */  27,
    /*0xB2 */  27,
    /*0xB3 */  27,
    /*0xB4 */  28,
    /*0xB5 */  28,
    /*0xB6 */  29,
    /*0xB7 */  29,
    /*0xB8 */  30,
    /*0xB9 */  30,
    /*0xBA */  31,
    /*0xBB */  31,
    /*0xBC */  32,
    /*0xBD */  32,
    /*0xBE */  33,
    /*0xBF */  33,
    /*0xC0 */  34,
    /*0xC1 */  34,
    /*0xC2 */  35,
    /*0xC3 */  35,
    /*0xC4 */  36,
    /*0xC5 */  36,
    /*0xC6 */  37,
    /*0xC7 */  37,
    /*0xC8 */  38,
    /*0xC9 */  38,
    /*0xCA */  39,
    /*0xCB */  40,
    /*0xCC */  40,
    /*0xCD */  41,
    /*0xCE */  41,
    /*0xCF */  42,
    /*0xD0 */  43,
    /*0xD1 */  43,
    /*0xD2 */  44,
    /*0xD3 */  45,
    /*0xD4 */  45,
    /*0xD5 */  46,
    /*0xD6 */  47,
    /*0xD7 */  48,
    /*0xD8 */  48,
    /*0xD9 */  49,
    /*0xDA */  50,
    /*0xDB */  51,
    /*0xDC */  52,
    /*0xDD */  53,
    /*0xDE */  53,
    /*0xDF */  54,
    /*0xE0 */  55,
    /*0xE1 */  56,
    /*0xE2 */  57,
    /*0xE3 */  59,
    /*0xE4 */  60,
    /*0xE5 */  61,
    /*0xE6 */  62,
    /*0xE7 */  63,
    /*0xE8 */  65,
    /*0xE9 */  66,
    /*0xEA */  68,
    /*0xEB */  69,
    /*0xEC */  71,
    /*0xED */  73,
    /*0xEE */  75,
    /*0xEF */  77,
    /*0xF0 */  79,
    /*0xF1 */  81,
    /*0xF2 */  82,
    /*0xF3 */  86,
    /*0xF4 */  90,
    /*0xF5 */  93,
    /*0xF6 */  97,
    /*0xF7 */ 100,
    /*0xF8 */ 100,
    /*0xF9 */ 100,
    /*0xFA */ 100,
    /*0xFB */ 100,
    /*0xFC */ 100,
    /*0xFD */ 100,
    /*0xFE */ 100,
    /*0xFF */ 100
};
