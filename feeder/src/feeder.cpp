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

void feed(
    const UINT rID,
    const LONG wheel,
    const LONG clutch,
    const LONG brake,
    const LONG acceleration,
    const UINT8 paddles
) {
    SetAxis(wheel, rID, WHEEL_AXIS);
    SetAxis(clutch, rID, CLUTCH_AXIS);
    SetAxis(brake, rID, BRAKE_AXIS);
    SetAxis(acceleration, rID, ACCELERATION_AXIS);

    // Paddles → buttons
    const bool left  = (paddles & 0x01) != 0;
    const bool right = (paddles & 0x02) != 0;
    SetBtn(left  ? TRUE : FALSE, rID, PADDLE_LEFT_BTN);
    SetBtn(right ? TRUE : FALSE, rID, PADDLE_RIGHT_BTN);
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