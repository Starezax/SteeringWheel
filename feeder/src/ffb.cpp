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
                    std::cout << "Magnitude:\t" << static_cast<int>(mag) << std::endl;
                    std::lock_guard lock(mut);
                    effects[idx].constMag = magNorm;
                    effects[idx].haveConst = TRUE;
                }
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
                    std::lock_guard<std::mutex> lock(mut);
                    deviceGain = gain;
                }
                break;
            }
    case PT_EFFREP:
            {
                INT ebi = 0;
                FFB_EFF_REPORT report{};
                if (Ffb_h_Eff_Report(packet, &report) != ERROR_SUCCESS) return;
                if (Ffb_h_EBI(packet, &ebi) != ERROR_SUCCESS) return;

                const INT idx = normEBI(ebi);
                if (idx >= MAX_EFFECTS) return;

                {
                    std::lock_guard lock(mut);
                    effects[idx].gain = report.Gain;
                }
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
