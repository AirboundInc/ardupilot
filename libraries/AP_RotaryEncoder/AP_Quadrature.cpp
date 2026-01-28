#include <AP_HAL/AP_HAL.h>

#include "AP_Quadrature.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const int8_t AP_Quadrature::_lookupTable[16] = {
    0,  -1,   1,   0,
    1,   0,   0,  -1,
   -1,   0,   0,   1,
    0,   1,  -1,   0
};

// Constructor
AP_Quadrature::AP_Quadrature(AP_RotaryEncoder &frontend, uint8_t instance, AP_RotaryEncoder::RotaryEncoder_State &state) :
    AP_RotaryEncoder_Backend(frontend, instance, state)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Quadrature backend created for instance %d", instance);
}

void AP_Quadrature::update_pin(uint8_t &pin, uint8_t new_pin, uint8_t &pin_value)
{

    if (new_pin == pin) {
        // no change
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "QEnc update_pin: instance %d, old_pin %d, new_pin %d", _state.instance, pin, new_pin);

    // remove old gpio event callback if present
    if (pin != UINT8_MAX &&
        !hal.gpio->detach_interrupt(pin)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "QEnc: Failed to detach from pin %u", pin);
        // ignore this failure or the user may be stuck
    }

    pin = new_pin;

    // install interrupt handler on rising or falling edge of gpio for pin a
    if (new_pin != UINT8_MAX) {
        hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
        if (!hal.gpio->attach_interrupt(
                pin,
                FUNCTOR_BIND_MEMBER(&AP_Quadrature::irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "QEnc: Failed to attach to pin %u", pin);
        }
        pin_value = hal.gpio->read(pin);
    }
}

void AP_Quadrature::irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp)
{
    // Add interrupt debug
    static uint32_t irq_count = 0;
    irq_count++;
    

    uint8_t index = 0;
    if(pin == _pinA) {
        index = A_value << 3 | B_value << 2 | pin_value << 1 | B_value;
        A_value = pin_value;
    }
    else if(pin == _pinB) {
        index = A_value << 3 | B_value << 2 | A_value << 1 | pin_value;
        B_value = pin_value;
    }
    else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IRQ on unknown pin %d (instance %d)", pin, _state.instance);
        return;
    }

    int8_t direction = _lookupTable[index];

    // Accumulate phase changes first
    irq_state.phase += direction;
    
    // Calculate velocity using a sliding window approach
    uint32_t current_time_us = timestamp;
    
    // Get CPR from frontend and calculate angle
    uint16_t cpr = _frontend.get_counts_per_revolution(_state.instance);
    if (cpr > 0) {
        irq_state.angle = irq_state.phase * (360.0f / cpr);
    } else {
        irq_state.angle = 0.0f;
    }

    irq_state.time = current_time_us;
}

void AP_Quadrature::update(void)
{
    update_pin(_pinA, get_pin_a(), A_value);
    update_pin(_pinB, get_pin_b(), B_value);

    // disable interrupts to prevent race with irq_handler
    void *irqstate = hal.scheduler->disable_interrupts_save();

    // copy count, angle, and last timestamp so it is accessible to front end
    copy_state_to_frontend(irq_state.phase,
                           irq_state.angle,
                           irq_state.time);

    // restore interrupts
    hal.scheduler->restore_interrupts(irqstate);
}