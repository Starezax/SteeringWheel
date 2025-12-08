//
// Created by gladf on 04.12.2025.
//

#ifndef VJOY_FEEDER_FFB_H
#define VJOY_FEEDER_FFB_H
#include <atomic>
#include <windows.h>
#include "public.h"
#include "vJoyInterface.h"

struct FFBCtx {
    std::atomic<float> torque = 0.0f;
    std::atomic<BYTE> gain = 0;

    UINT devID = 1;
};



void CALLBACK ffbCallback(PVOID ffbPacket, PVOID userdata);


#endif //VJOY_FEEDER_FFB_H