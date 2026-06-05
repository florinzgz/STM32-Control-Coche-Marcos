/**
 * @file  Wire.h  (test stub)
 * @brief Minimal stub replacing the real Wire.h for host-side unit tests.
 *
 * Provides a controllable TwoWire class that simulates I2C success/failure.
 * Tests control behavior via g_wire_end_result and g_wire_read_value.
 */

#pragma once
#include <cstdint>
#include <cstddef>

/* ---- Controllable test state ------------------------------------------- */

/// Return value for endTransmission() — 0 = success, non-zero = error
inline uint8_t g_wire_end_result      = 0;
/// Value returned by read() after a successful requestFrom()
inline uint8_t g_wire_read_value      = 0xFF;
/// Number of bytes returned by requestFrom() — 0 means failure
inline uint8_t g_wire_request_result  = 1;

/* ---- Minimal TwoWire stub ---------------------------------------------- */
struct TwoWire {
    void begin(int, int)                          {}
    void end()                                    {}
    void setClock(uint32_t)                       {}
    void setTimeOut(uint16_t)                     {}
    void beginTransmission(uint8_t)               {}
    size_t write(uint8_t)                         { return 1; }
    uint8_t endTransmission(bool = true)          { return g_wire_end_result; }
    uint8_t requestFrom(uint8_t, uint8_t count)   { return (g_wire_end_result == 0) ? g_wire_request_result : 0; }
    int available()                               { return (g_wire_request_result > 0 && g_wire_end_result == 0) ? 1 : 0; }
    int read()                                    { return g_wire_read_value; }
};

inline TwoWire Wire;
