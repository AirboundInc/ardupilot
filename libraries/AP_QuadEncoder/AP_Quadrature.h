#pragma once

#include <AP_QuadEncoder/AP_RotaryEncoder.h>
#include <AP_QuadEncoder/AP_RotaryEncoder_Backend.h>
#include <Filter/Filter.h>
#include <AP_Math/AP_Math.h>

class AP_Quadrature : public AP_RotaryEncoder_Backend
{   public:
    // constructor
    using AP_RotaryEncoder_Backend::AP_RotaryEncoder_Backend;

    // update state
    void update(void) override;

private:
    
    // check if pin has changed and initialise gpio event callback
    void update_pin(uint8_t &pin, uint8_t new_pin, uint8_t &pin_value);

    // gpio interrupt handlers
    void irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp); // combined irq handler

    // convert pin a and b status to phase
    static uint8_t pin_ab_to_phase(bool pin_a, bool pin_b);

    // update phase, rotation_count and error count using pin a and b's latest state
    void update_phase_and_error_count();

    struct IrqState {
        uint8_t  phase;             // current phase of encoder (from 0 to cpr)
        int32_t  angle;             // angle measured by cumulative steps forward or backwards since last update
        uint32_t last_reading_ms;   // system time of last update from encoder
    } irq_state;

    uint8_t _pinA = -1;
    uint8_t _pinB = -1;
    uint8_t A_value, B_value;
    int32_t _cpr;
    
    static const int8_t _lookupTable[16];

};