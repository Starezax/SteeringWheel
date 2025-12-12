//
// Created by gladf on 04.12.2025.
//

#ifndef VJOY_FEEDER_FFB_H
#define VJOY_FEEDER_FFB_H

#define MAX_EFFECTS 16


#include <array>
#include <atomic>
#include <mutex>
#include <windows.h>
#include "public.h"
#include "vJoyInterface.h"

class FFBState
{
public:
    UINT devID;

    struct Effect
    {
        BOOL active = FALSE;
        UINT8 gain = 255;
        INT16 constMag = 0;
        BOOL haveConst = FALSE;
    };

    struct Snapshot
    {
        std::array<Effect, MAX_EFFECTS> effects;
        UINT8 deviceGain = 255;
    };

    void handlePacket(const FFB_DATA *packet);

    Snapshot snapshot() const
    {
        std::lock_guard lock(mut);
        const Snapshot s{effects, deviceGain};
        return s;
    }

    static FLOAT computeTorque(const Snapshot &s)
    {
        FLOAT total = 0.0f;
        const FLOAT devGain = s.deviceGain / 255.0f;

        for (const auto &e: s.effects)
        {
            if (!e.active || !e.haveConst) continue;

            const FLOAT effGain = e.gain / 255.0f;
            const FLOAT magNorm = e.constMag / 10000.0f;
            total += magNorm * effGain * devGain;
        }

        if (total > 1.0f) total = 1.0f;
        if (total < -1.0f) total = -1.0f;
        return total;
    }

private:
    static INT16 clamp(INT v, INT lo, INT hi)
    {
        return static_cast<INT16>(max(lo, min(hi, v)));
    }

    static INT16 complement2(const LONG v)
    {
        return static_cast<INT16>(static_cast<UINT16>(v & 0xFFFF));
    }

    static INT normEBI(const UINT8 ebi)
    {
        if (ebi == 0) return 0;
        const INT idx = static_cast<INT>(ebi) - 1;
        return (idx < 0) ? 0 : idx;
    }

    mutable std::mutex mut;
    std::array<Effect, MAX_EFFECTS> effects;
    UINT8 deviceGain = 255;
};;

void CALLBACK ffbCallback(PVOID ffbPacket, PVOID userdata);


#endif //VJOY_FEEDER_FFB_H