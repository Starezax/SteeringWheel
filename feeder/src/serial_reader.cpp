//
// Created by gladf on 12.11.2025.
//

#include <windows.h>
#include <string>
#include <vector>
#include <cstdio>
#include <cstdint>
#include <optional>
#include <charconv>
#include <array>
#include <algorithm>
#include <iostream>

#include "feeder.h"

HANDLE openSerial(const wchar_t* com_name, const DWORD baud) {
    HANDLE h = CreateFileW(
        com_name, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);

    if (h == INVALID_HANDLE_VALUE) {
        std::fprintf(stderr, "CreateFileW failed, err=%lu\n", GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb{0};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h, &dcb)) {
        std::fprintf(stderr, "GetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(h, &dcb)) {
        std::fprintf(stderr, "SetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = 200;
    to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 200;
    to.WriteTotalTimeoutMultiplier = 0;
    if (!SetCommTimeouts(h, &to)) {
        std::fprintf(stderr, "SetCommTimeouts failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

inline void trimCRLF(std::string& s) {
    while (!s.empty() && (s.back()=='\r' || s.back()=='\n')) s.pop_back();
}


void normalizeInput(UARTFrame& f) {
    if (f.gear < GEAR_MIN) f.gear = GEAR_MIN;
    if (f.gear > GEAR_MAX) f.gear = GEAR_MAX;

}


std::optional<UARTFrame> parseFrame(const std::string& line) {
    std::string clean = line;

    trimCRLF(clean);
    if (clean.empty())
        return std::nullopt;

    std::string vals[INPUTS_COUNT];
    size_t idx = 0;
    size_t start = 0;
    for (; idx < INPUTS_COUNT; ++idx) {
        const size_t comma = clean.find('|', start);
        std::string token = (comma == std::string::npos)
                                ? clean.substr(start)
                                : clean.substr(start, comma - start);

        if (token.empty()) return std::nullopt;
        vals[idx] = token;
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    if (idx != INPUTS_COUNT - 1) return std::nullopt;

    UARTFrame f{};
    if (!parse_u16_str(vals[0], f.wheel))  return std::nullopt;
    if (!parse_u16_str(vals[1], f.clutch)) return std::nullopt;
    if (!parse_u16_str(vals[2], f.brake)) return std::nullopt;
    if (!parse_u16_str(vals[3], f.acceleration)) return std::nullopt;
    if (!parse_u8_str(vals[4], f.gear)) return std::nullopt;
    if (!parse_u16_str(vals[5], f.buttons)) f.buttons = 0;

    normalizeInput(f);

    return f;
}


bool readLineUntilEnd(
    HANDLE h,
    std::string& pending,
    std::string& out
) {
    char buf[128];
    const DWORD start = GetTickCount();

    for (;;) {

        if (const auto pos = pending.find('\n');
            pos != std::string::npos) {

            out.assign(pending.begin(), pending.begin() + pos + 1);
            pending.erase(0, pos + 1);
            trimCRLF(out);
            return true;
        }

        DWORD elapsed = GetTickCount() - start;
        if (elapsed >= 100) return false;

        DWORD got = 0;
        if (!ReadFile(h, buf, sizeof(buf), &got, nullptr)) {
            std::cerr << "ReadFile error=%lu\n" << GetLastError() << std::endl;
            Sleep(1);
            return false;
        }
        if (got > 0) {
            pending.append(buf, buf + got);
        } else {
            Sleep(1);
        }

        if (pending.size() > 512) {
            auto p = pending.rfind('\n');
            if (p != std::string::npos) pending.erase(0, p + 1);
            else pending.clear();
        }
    }
}


BOOL sendUART(
    HANDLE h,
    const INT8 torque
) {
    CHAR buf[16];
    const SIZE_T size = std::snprintf(buf, sizeof(buf), "%d\n", torque);

    if (h == INVALID_HANDLE_VALUE) {
        return FALSE;
    }

    DWORD written = 0;
    return WriteFile(
        h,
        buf,
        static_cast<DWORD>(size),
        &written,
        nullptr
    ) && written == static_cast<DWORD>(size);
}
