//
// Created by gladf on 04.12.2025.
//

#ifndef VJOY_FEEDER_FFB_H
#define VJOY_FEEDER_FFB_H

#define MAX_EFFECTS 16


#include <algorithm>
#include <array>
#include <atomic>
#include <iostream>
#include <mutex>
#include <ostream>
#include <windows.h>
#include "public.h"
#include "vJoyInterface.h"

class FFBState
{
public:
    UINT devID = 1;

    struct ConditionAxis
    {
        INT16   centerPointOffset = 0;
        UINT16  deadBand          = 0;
        INT16   posCoef           = 0;
        INT16   negCoef           = 0;
        UINT16  posSat            = 0;
        UINT16  negSat            = 0;
    };

    struct Effect
    {
        FFBEType kind = ET_NONE;
        BOOL active = FALSE;
        UINT8 gain = 255;
        INT16 constMag = 0;

        ConditionAxis condX{};

        BOOL haveConst = FALSE;
        BOOL haveCondX = FALSE;
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

    static FLOAT computeTorque(const Snapshot &s, const FLOAT wheelPos)
    {
        FLOAT total = 0.0f;
        const FLOAT devGain = s.deviceGain / 255.0f;

        for (const auto &e: s.effects)
        {
            if (!e.active) continue;

            const FLOAT gain = (e.gain / 255.0f) * devGain;

            // constant
            if (e.haveConst)
            {
                total += (e.constMag / 10000.0f) * gain;
            }

            // conditional
            if (e.haveCondX)
            {
                total += computeCondForce(wheelPos, e.condX) * gain;
            }
        }

        return std::clamp(total, -1.0f, 1.0f);
    }

private:
    static FLOAT computeCondForce(const FLOAT wheelPos, const ConditionAxis &x)
    {
        const FLOAT off     = x.centerPointOffset   / 10000.0f;
        const FLOAT db      = x.deadBand            / 10000.0f;
        const FLOAT posC    = x.posCoef             / 10000.0f;
        const FLOAT negC    = x.negCoef             / 10000.0f;
        const FLOAT posSat  = x.posSat              / 10000.0f;
        const FLOAT negSat  = x.negSat              / 10000.0f;

        FLOAT force = 0.0f;

        if (wheelPos > (off + db))
        {
            force = (wheelPos - (off + db)) * posC;
            force = min(force, posSat);
        } else if (wheelPos < (off - db))
        {
            force = (wheelPos - (off - db)) * negC;
            force = min(force, negSat);
        }

        force = -force;
        return std::clamp(force, -1.0f, 1.0f);
    }

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
        return idx;
    }

    mutable std::mutex mut;
    std::array<Effect, MAX_EFFECTS> effects;
    UINT8 deviceGain = 255;
};;

void CALLBACK ffbCallback(PVOID ffbPacket, PVOID userdata);


#endif //VJOY_FEEDER_FFB_H