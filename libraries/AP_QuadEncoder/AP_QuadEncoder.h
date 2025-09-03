#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_Quadrature {
public:
    AP_Quadrature(uint8_t pinA, uint8_t pinB, int32_t cpr);
    void update_pin(uint8_t &pin, uint8_t new_pin, uint8_t &pin_value);
    void irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp);
    void update();
    void init();
    int32_t get_position() const;
    float get_angle() const;

private:
    struct IrqState {
        uint8_t  phase;             // current phase of encoder (from 0 to cpr)
        int32_t  angle;             // angle measured by cumulative steps forward or backwards since last update
        uint32_t last_reading_ms;   // system time of last update from encoder
    } irq_state;

    uint8_t _pinA, _pinB;
    uint8_t A_value, B_value;
    int32_t _cpr;
    
    static const int8_t _lookupTable[16];

};