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

#define AXIS_MIN 0
#define AXIS_MAX 32767

#define PEDAL_MIN 20
#define PEDAL_MAX 900


struct Pedal {
    uint16_t min = 20;
    uint16_t max = 900;
    bool inverted = false;
    double expo = 0.12;
};

struct UARTFrame {
    UINT16 wheel;
    UINT16 clutch;
    UINT16 brake;
    UINT16 acceleration;
    UINT8 paddles;
};

bool readLineUntilEnd( HANDLE h, std::string& pending, std::string& out );
std::optional<UARTFrame> parseFrame(const std::string& line);
HANDLE openSerial(const wchar_t* com_name, DWORD baud = CBR_9600);
void feed(
    UINT rID,
    LONG wheel,
    LONG clutch,
    LONG brake,
    LONG acceleration,
    UINT8 paddles
);
void cleanup(UINT rID);
LONG scalePedal(UINT16 raw, const Pedal &p, double *ema_state = nullptr, double ema_alpha = 0.0);

// Exponential Moving Average for smoothing
inline double ema_step(const double x, const double prev, const double alpha=0.2) {
    return alpha * x + (1.0 - alpha) * prev;
}

inline bool parse_u16_str(const std::string& s, uint16_t& v) {
    const unsigned long val = std::stoul(s);
    if (val > 65535UL)
        return false;
    v = static_cast<uint16_t>(val);
    return true;
}

inline bool parse_u8_str(const std::string& s, uint8_t& v) {
    const unsigned long val = std::stoul(s);
    if (val > 255UL) return false;
    v = static_cast<uint8_t>(val); return true;
}

#endif //VJOY_FEEDER_FEEDER_H