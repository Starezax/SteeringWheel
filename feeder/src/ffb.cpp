//
// Created by gladf on 07.12.2025.
//

#include "ffb.h"

#include <iostream>

void FFBState::handlePacket(const FFB_DATA* packet)
{
    if (!packet) return;

    FFBPType ptype;
    if (Ffb_h_Type(packet, &ptype) != ERROR_SUCCESS) return;

    switch (ptype)
    {
    case PT_NEWEFREP:
        {
            INT ebi = 0;
            FFBEType effType{};
            if (Ffb_h_EffNew(packet, &effType) != ERROR_SUCCESS) return;
            if (Ffb_h_EBI(packet, &ebi) != ERROR_SUCCESS) return;

            const INT idx = normEBI(ebi);
            if (idx >= MAX_EFFECTS) return;

            {
                std::lock_guard lock(mut);
                effects[idx] = Effect{};
                effects[idx].kind = effType;
            }

            break;
        }
        case PT_CONSTREP:
            {
                INT ebi = 0;
                FFB_EFF_CONSTANT effect{};
                if (Ffb_h_Eff_Constant(packet, &effect) != ERROR_SUCCESS) return;
                if (Ffb_h_EBI(packet, &ebi) != ERROR_SUCCESS) return;

                const INT idx = normEBI(ebi);
                if (idx >= MAX_EFFECTS) return;
                {
                    auto const mag = complement2(effect.Magnitude);
                    auto const magNorm = clamp(mag, -10000, 10000);
                    std::cout << "FFB_EFF_REPORT: " << std::endl;
                    std::lock_guard lock(mut);
                    effects[idx].active = TRUE;
                    effects[idx].constMag = magNorm;
                    effects[idx].haveConst = TRUE;\
                    std::cout << "Magnitude:\t" << static_cast<int>(mag) << std::endl;

                }
                break;
            }
    case PT_CONDREP:
            {
                INT ebi = 0;
                FFB_EFF_COND cond{};
                if (Ffb_h_Eff_Cond(packet, &cond) != ERROR_SUCCESS) return;
                if (Ffb_h_EBI(packet, &ebi) != ERROR_SUCCESS) return;

                const INT idx = normEBI(ebi);
                if (idx >= MAX_EFFECTS) return;
                if (cond.isY) return;

                ConditionAxis cx;
                cx.centerPointOffset    = clamp(complement2(cond.CenterPointOffset), -10000, 10000);
                cx.deadBand             = static_cast<UINT16>(min(static_cast<int>(cond.DeadBand), 10000));
                cx.posCoef              = clamp(complement2(cond.PosCoeff), -10000, 10000);
                cx.negCoef              = clamp(complement2(cond.NegCoeff), -10000, 10000);
                cx.posSat               = static_cast<UINT16>(min(static_cast<int>(cond.PosSatur), 10000));
                cx.negSat               = static_cast<UINT16>(min(static_cast<int>(cond.NegSatur), 10000));

                {
                    std::lock_guard lock(mut);
                    effects[idx].condX = cx;
                    effects[idx].haveCondX = TRUE;
                }

                std::cout << "EBI:\t" << static_cast<INT>(cond.EffectBlockIndex) << std::endl;
                break;
            }
    case PT_EFOPREP:
            {
                INT ebi = 0;
                FFB_EFF_OP op{};
                if (Ffb_h_EffOp(packet, &op) != ERROR_SUCCESS) return;
                if (Ffb_h_EBI(packet, &ebi) != ERROR_SUCCESS) return;

                const INT idx = normEBI(ebi);
                if (idx >= MAX_EFFECTS) return;

                {
                    std::lock_guard lock(mut);
                    if (op.EffectOp == EFF_STOP)
                    {
                        effects[idx].constMag = 0;
                        effects[idx].active = FALSE;
                    } else if (op.EffectOp == EFF_START || op.EffectOp == EFF_SOLO)
                    {
                        effects[idx].active = TRUE;
                    }
                }

                break;
            }
    case PT_GAINREP:
            {
                BYTE gain = 0;
                if (Ffb_h_DevGain(packet, &gain) != ERROR_SUCCESS) return;

                {
                    std::lock_guard lock(mut);
                    deviceGain = gain;
                }
                break;
            }
    case PT_EFFREP:
            {
                FFB_EFF_CONST report{};
                if (Ffb_h_Eff_Const(packet, &report) != ERROR_SUCCESS) return;

                const INT idx = normEBI(report.EffectBlockIndex);
                if (idx >= MAX_EFFECTS) return;

                {
                    std::lock_guard lock(mut);
                    effects[idx].kind = report.EffectType;
                    effects[idx].gain = report.Gain;
                }
                std::cout << "PT_EFFREP: " << report.EffectType << std::endl;

                break;
            }
        default: break;
    }
}



void ffbCallback(PVOID ffbPacket, PVOID userdata) {
    auto* state = static_cast<FFBState*>(userdata);
    const auto *packet = static_cast<FFB_DATA*>(ffbPacket);

    if (!state || !packet) return;

    FFBPType ptype;
    if (Ffb_h_Type(packet, &ptype) == ERROR_SUCCESS) {
        std::cout << "FFB type caught: " << ptype << "\n";
    }
    state->handlePacket(packet);
}
