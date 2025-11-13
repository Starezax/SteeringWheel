//
// Created by gladf on 12.11.2025.
//


#include <windows.h>
#include <iostream>
#include <ostream>

#include "feeder.h"
#include "public.h"
#include "vJoyInterface.h"


Pedal accPedal { };
Pedal brakePedal {};
Pedal clutchPedal {};

double emaAcc = 0.0;
double emaBrake = 0.0;
double emaClutch = 0.0;

static std::atomic_bool running{true};

static BOOL WINAPI CtrlHandler(DWORD type) {
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



int main() {

    if (!vJoyEnabled()) {
        std::cout << "Failed getting vJoy driver" << std::endl;
        return -2;
    }

    constexpr UINT rID = 1;
    const auto comName = L"\\\\.\\COM3";

    std::cout << "Initialized vJoy device ID: " << rID << std::endl;
    std::cout << "Vendor: " << TEXT(GetvJoyManufacturerString()) <<
        "\nProduct: " << TEXT(GetvJoyProductString()) <<
        "\nVersion Number:" << TEXT(GetvJoySerialNumberString()) << std::endl;

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

    HANDLE h = openSerial(comName, CBR_9600);
    if (h == INVALID_HANDLE_VALUE) return 1;

    std::string pending;
    std::string line;
    while (running.load(std::memory_order_relaxed)) {
        std::cout << "Running cycle" << std::endl;
        if (!readLineUntilEnd(h, pending, line)) {
            std::cout << "Skipping line " << line << std::endl;
            continue;
        }

        const std::optional<UARTFrame> f = parseFrame(line);
        if (!f) continue;
        std::cout << "Continue: " << f->clutch << std::endl;
        const LONG clutch = scalePedal(f->clutch, clutchPedal, &emaClutch);
        const LONG brake = scalePedal(f->brake, brakePedal, &emaBrake);
        const LONG acc = scalePedal(f->acceleration, accPedal, &emaAcc);

        feed(rID, f->wheel, clutch, brake, acc, f->paddles);
    }

    CloseHandle(h);
    cleanup(rID);
    return 0;
}