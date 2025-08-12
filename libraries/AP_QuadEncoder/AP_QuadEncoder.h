#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_QuadEncoder {
public:
    enum Side { LEFT, RIGHT };

    AP_QuadEncoder(uint8_t pinA, uint8_t pinB, int32_t cpr, Side side);
    void init();
    int32_t get_position() const;
    float get_angle() const;

private:
    uint8_t _pinA, _pinB;
    int32_t _cpr;
    volatile int32_t _position;
    volatile uint8_t _prevAB;
    static const int8_t _lookupTable[16];

    Side _side;
    static AP_QuadEncoder* _left_instance;
    static AP_QuadEncoder* _right_instance;

    static void isr_left();
    static void isr_right();

    void isr();
};