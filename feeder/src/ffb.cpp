//
// Created by gladf on 07.12.2025.
//

#include "ffb.h"

#include <iostream>

void ffbCallback(PVOID ffbPacket, PVOID userdata) {
    auto* ctx = static_cast<FFBCtx*>(userdata);
    const auto *packet = static_cast<FFB_DATA*>(ffbPacket);

    if (!packet) return;

    FFBPType ptype;
    if (Ffb_h_Type(packet, &ptype) != ERROR_SUCCESS) return;

    std::cout << "FFB type caught: " << ptype << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    switch (ptype) {
        case PT_CONSTREP: {
            FFB_EFF_CONSTANT effect{};
            if (Ffb_h_Eff_Constant(packet, &effect) != ERROR_SUCCESS) return;

            FLOAT magNorm = static_cast<FLOAT>(effect.Magnitude) / 10000.0f;
            if (magNorm > 1.0f) magNorm = 1.0f;
            if (magNorm < -1.0f) magNorm = -1.0f;

            ctx->torque.store(magNorm, std::memory_order_relaxed);

            std::cout << "FFB_EFF_REPORT: " << std::endl;
            std::cout << "Magnitude:\t" << effect.Magnitude << std::endl;
            std::cout << "------------------------------------------" << std::endl;

            break;
        }
        case PT_EFFREP: {
            FFB_EFF_REPORT effect{};
            if (Ffb_h_Eff_Report(packet, &effect) != ERROR_SUCCESS) return;
            std::cout << "FFB_EFF_REPORT: " << std::endl;
            std::cout << "Duration:\t" << effect.Duration << std::endl;
            std::cout << "TrigerRpt:\t" << effect.TrigerRpt << std::endl;
            std::cout << "SamplePrd:\t" << effect.SamplePrd << std::endl;
            std::cout << "Gain:\t" << effect.Gain << std::endl;
            std::cout << "TrigerBtn:\t" << effect.TrigerBtn << std::endl;
            std::cout << "Polar:\t" << effect.Polar << std::endl;
            std::cout << "Direction:\t" << static_cast<int>(effect.Direction) << std::endl;
            std::cout << "DirY:\t" << effect.DirY << std::endl;
            std::cout << "DirX:\t" << static_cast<int>(effect.DirX) << std::endl;
            std::cout << "------------------------------------------" << std::endl;

            break;
        }
        case PT_EFOPREP:{
            FFB_EFF_OP op{};
            if (Ffb_h_EffOp(packet, &op) != ERROR_SUCCESS) return;

            if (op.EffectOp == EFF_STOP) {
                ctx->torque.store(0.0f, std::memory_order_relaxed);
            }
            std::cout << "FFB_EFF_OP: " << std::endl;
            std::cout << "EffectOp:\t" << op.EffectOp << std::endl;
            std::cout << "LoopCount:\t" << op.LoopCount << std::endl;
            std::cout << "------------------------------------------" << std::endl;


            break;
        }
        case PT_GAINREP: {
            BYTE gain = 0;
            if (Ffb_h_DevGain(packet, &gain) != ERROR_SUCCESS) return;

            std::cout << "PT_GAINREP: " << std::endl;
            std::cout << "Gain:\t" << gain << std::endl;
            std::cout << "------------------------------------------" << std::endl;
            break;

        }
        default: {
            std::cout << "Unhandled FFB type caught: " << ptype << std::endl;
            std::cout << "------------------------------------------" << std::endl;
            break;
        }
    }

}
