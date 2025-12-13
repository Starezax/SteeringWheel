//
// Created by gladf on 13.11.2025.
//

#include <windows.h>
#include <cmath>
#include "feeder.h"
#include "public.h"
#include "vJoyInterface.h"

LONG scalePedal(
    const UINT16 raw,
    const Pedal &p,
    double *ema_state,
    const double ema_alpha
) {

    // normalize to [0, 1]
    if (p.max <= p.min) return AXIS_MIN;
    double norm = static_cast<double>(raw - p.min) / static_cast<double>(p.max - p.min);
    if (norm < 0) norm = 0;
    if (norm > 1) norm = 1;

    // deadzones
    // if (norm <= p.dead_low)
    // if (t <= c.dead_low) t = 0.0;
    // else if (t >= 1.0 - c.dead_high) t = 1.0;
    // else {
    //     const double span = 1.0 - c.dead_low - c.dead_high;
    //     t = (t - c.dead_low) / span;
    // }

    if (p.inverted) norm = 1.0 - norm;
    if (p.expo > 0.0) norm = (1.0 - p.expo) * norm + p.expo * (norm * norm * norm);

    // EMA smoothing
    if (ema_state && ema_alpha > 0.0) {
        *ema_state = ema_step(norm, *ema_state, ema_alpha);
        norm = *ema_state;
    }

    const double out_double = static_cast<double>(AXIS_MIN) + norm * static_cast<double>(AXIS_MAX - AXIS_MIN);
    LONG out = static_cast<LONG>(std::llround(out_double));
    if (out < AXIS_MIN) out = AXIS_MIN;
    if (out > AXIS_MAX) out = AXIS_MAX;
    return out;
}

void feedGears(const UINT rID, const UINT8 gear, UINT8 *lastGear) {
    if (gear == 0) {
        // neutral gear
        if (*lastGear != 0) {
            // only feed if not neutral already
            SetBtn(FALSE, rID, *lastGear);
            *lastGear = 0;
        }
        return;
    }

    const UINT8 gearBtn = gear + 10;    // gear + 10 offset = btn number
    if (gearBtn != *lastGear) {
        SetBtn(FALSE, rID, *lastGear);  // release current gear
        SetBtn(TRUE, rID, gearBtn);     // switch to the next gear
        *lastGear = gearBtn;
    }
}

void feed(const UINT rID, const UARTFrame &f, UINT8 *lastGear) {
    SetAxis(f.wheel, rID, WHEEL_AXIS);
    SetAxis(f.clutch, rID, CLUTCH_AXIS);
    SetAxis(f.brake, rID, BRAKE_AXIS);
    SetAxis(f.acceleration, rID, ACCELERATION_AXIS);

    // Paddles & buttons
    SetBtn(isSet(f.buttons, 0), rID, PADDLE_LEFT_BTN);
    SetBtn(isSet(f.buttons, 1), rID, PADDLE_RIGHT_BTN);
    SetBtn(isSet(f.buttons, 2), rID, L1_BTN);
    SetBtn(isSet(f.buttons, 3), rID, R1_BTN);
    SetBtn(isSet(f.buttons, 4), rID, L2_BTN);
    SetBtn(isSet(f.buttons, 5), rID, R2_BTN);
    SetBtn(isSet(f.buttons, 6), rID, L3_BTN);
    SetBtn(isSet(f.buttons, 7), rID, R3_BTN);
    SetBtn(isSet(f.buttons, 8), rID, L4_BTN);
    SetBtn(isSet(f.buttons, 9), rID, R4_BTN);
    SetBtn(isSet(f.buttons, 10), rID, HANDBRAKE_BTN);

    // gears
    feedGears(rID, f.gear, lastGear);
}

void cleanup(const UINT rID) {
    SetAxis(0, rID, WHEEL_AXIS);
    SetAxis(0, rID, CLUTCH_AXIS);
    SetAxis(0, rID, BRAKE_AXIS);
    SetAxis(0, rID, ACCELERATION_AXIS);
    SetBtn(FALSE, rID, PADDLE_LEFT_BTN);
    SetBtn(FALSE, rID, PADDLE_RIGHT_BTN);

    RelinquishVJD(rID);
}
