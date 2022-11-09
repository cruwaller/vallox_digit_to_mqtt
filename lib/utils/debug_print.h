#pragma once

#if DEBUG_ENABLED
#include "HardwareSerial.h"
#define PRINT(...) Serial.print(__VA_ARGS__)
#define PRINTLN(...) Serial.println(__VA_ARGS__)
#define PRINTF(...) Serial.printf(__VA_ARGS__)
#define PRINTFLN(...) do {Serial.printf(__VA_ARGS__); Serial.println();} while(0)
#else
#define PRINT(...)
#define PRINTF(...)
#define PRINTLN(...)
#define PRINTFLN(...)
#endif
