//
// Created by gladf on 12.11.2025.
//

#ifndef VJOY_FEEDER_FEEDER_H
#define VJOY_FEEDER_FEEDER_H

#include <iostream>
#include <windows.h>
#include <optional>
#include <ostream>
#include <string>
#include "public.h"
#include "ffb.h"

#define CLUTCH_AXIS HID_USAGE_SL0
#define BRAKE_AXIS HID_USAGE_RZ
#define ACCELERATION_AXIS HID_USAGE_Y
#define WHEEL_AXIS HID_USAGE_X

#define PADDLE_LEFT_BTN 9
#define PADDLE_RIGHT_BTN 10

#define L1_BTN 1
#define R1_BTN 2
#define L2_BTN 3
#define R2_BTN 4
#define L3_BTN 5
#define R3_BTN 6
#define L4_BTN 7
#define R4_BTN 8

#define GEAR_1 11
#define GEAR_2 12
#define GEAR_3 13
#define GEAR_4 14
#define GEAR_5 15
#define GEAR_6 16
#define GEAR_R 17

#define AXIS_MIN 0
#define AXIS_MAX 32767

#define PEDAL_MIN 20
#define PEDAL_MAX 900

#define PADDLES_MIN 0
#define PADDLES_MAX 3

#define GEAR_MIN 0
#define GEAR_MAX 7

#define INPUTS_COUNT 6


struct Pedal {
    UINT16 min = 20;
    UINT16 max = 900;
    bool inverted = false;
    double expo = 0.12;
};

struct UARTFrame {
    UINT16 wheel = AXIS_MAX / 2;
    UINT16 clutch = PEDAL_MAX;
    UINT16 brake = PEDAL_MIN;
    UINT16 acceleration = PEDAL_MIN;
    UINT8 gear = GEAR_MIN;
    UINT16 buttons = 0;
};

bool readLineUntilEnd( HANDLE h, std::string& pending, std::string& out );
std::optional<UARTFrame> parseFrame(const std::string& line);
BOOL sendUART(HANDLE, INT8);
HANDLE openSerial(const wchar_t* com_name, DWORD baud = CBR_9600);
void feedGears(UINT, UINT8 gear, UINT8*);
void feed(UINT, const UARTFrame&, UINT8*);
void cleanup(UINT rID);
LONG scalePedal(UINT16 raw, const Pedal &p, double *ema_state = nullptr, double ema_alpha = 0.0);

inline BOOL isSet(const UINT16 buttons, const UINT16 pos) {
    return (buttons >> pos) & 1;
}

// Exponential Moving Average for smoothing
inline double ema_step(const double x, const double prev, const double alpha=0.2) {
    return alpha * x + (1.0 - alpha) * prev;
}

inline bool parse_u16_str(const std::string& s, UINT16& v) {
    const unsigned long val = std::stoul(s);
    if (val > 65535UL)
        return false;
    v = static_cast<UINT16>(val);
    return true;
}

inline bool parse_u8_str(const std::string& s, UINT8& v) {
    const unsigned long val = std::stoul(s);
    if (val > 255UL) return false;
    v = static_cast<UINT8>(val); return true;
}

#endif //VJOY_FEEDER_FEEDER_H