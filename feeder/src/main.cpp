//
// Created by gladf on 12.11.2025.
//


#include <windows.h>
#include <iostream>
#include <ostream>
#include <random>
#include <thread>

#include "feeder.h"
#include "ffb.h"
#include "public.h"
#include "vJoyInterface.h"


Pedal accPedal { 1400, 3700 };
Pedal brakePedal { 280, 1600};
Pedal clutchPedal {160, 2300};

double emaAcc = 0.0;
double emaBrake = 0.0;
double emaClutch = 0.0;

static std::atomic_bool running{true};

HANDLE h = INVALID_HANDLE_VALUE;
FFBState ffbState{};


static BOOL WINAPI CtrlHandler(const DWORD type) {
    switch (type) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_LOGOFF_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            running = false;
            return TRUE;
        default:
            return FALSE;
    }
}

static void FFBDeliveryWorker() {
    constexpr auto period = std::chrono::milliseconds(5);
    INT8 last = 127;

    while (running.load(std::memory_order_relaxed)) {
        auto snap = ffbState.snapshot();
        const INT8 val = ffbState.computeTorque(snap) * 100;
        if (val != last) {
            sendUART(h, val);
            last = val;
        }

        // 200 Hz
        std::this_thread::sleep_for(period);
    }

    sendUART(h, 0); // stop
}


int main() {

    if (!vJoyEnabled()) {
        std::cout << "Failed getting vJoy driver" << std::endl;
        return -2;
    }

    constexpr UINT rID = 1;
    const auto comName = L"\\\\.\\COM10";

    std::cout << "Initialized vJoy device ID: " << rID << std::endl;
    std::cout << "Vendor: " << TEXT(GetvJoyManufacturerString()) <<
        "\nProduct: " << TEXT(GetvJoyProductString()) <<
        "\nVersion Number: " << TEXT(GetvJoySerialNumberString()) << std::endl;

    const VjdStat status = GetVJDStatus(rID);
    if (status == VJD_STAT_BUSY || status == VJD_STAT_MISS) {
        std::cout << "vJoy device is busy or missing with status" << status << std::endl;
        return 2;
    }
    if (status == VJD_STAT_FREE && !AcquireVJD(rID)) {
        std::cout << "vJoy device cannot be acquired" << std::endl;
        return 3;
    }

    ResetVJD(rID);

    ffbState.devID = rID;
    FfbRegisterGenCB(ffbCallback, &ffbState);

    h = openSerial(comName, CBR_9600);
    if (h == INVALID_HANDLE_VALUE) return 1;

    std::thread ffbThread(FFBDeliveryWorker);

    UINT8 lastGear = GEAR_1;
    std::string line;
    std::string pending;
    while (running.load(std::memory_order_relaxed)) {
        if (!readLineUntilEnd(h, pending, line)) {
            continue;
        }

        std::optional<UARTFrame> f_or_none = parseFrame(line);
        if (!f_or_none || !f_or_none.has_value()) continue;
        UARTFrame f = f_or_none.value();

        f.clutch = scalePedal(f.clutch, clutchPedal, &emaClutch);
        f.brake = scalePedal(f.brake, brakePedal, &emaBrake);
        f.acceleration = scalePedal(f.acceleration, accPedal, &emaAcc);

        feed(rID, f, &lastGear);
    }

    CloseHandle(h);
    cleanup(rID);
    return 0;
}