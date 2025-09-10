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
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: Failed to attach to pin %u", pin);
        }
        pin_value = hal.gpio->read(pin);
    }
}

void AP_Quadrature::irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp)
{
    // Add interrupt debug
    static uint32_t irq_count = 0;
    irq_count++;
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IRQ %lu: pin%d=%d, A=%d, B=%d, ts=%lu", 
                  (unsigned long)irq_count, pin, pin_value, A_value, B_value, (unsigned long)timestamp);

    uint8_t index = 0;
    if(pin == _pinA) {
        // FIXED: Build index with old_A, old_B, new_A, old_B
        index = A_value << 3 | B_value << 2 | pin_value << 1 | B_value;
        A_value = pin_value;
    }
    else if(pin == _pinB) {
        // FIXED: Build index with old_A, old_B, old_A, new_B  
        index = A_value << 3 | B_value << 2 | A_value << 1 | pin_value;
        B_value = pin_value;
    }
    else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IRQ on unknown pin %d", pin);
        return;
    }

    int8_t direction = _lookupTable[index];

    // Accumulate phase changes
    irq_state.phase += direction;
    irq_state.angle = irq_state.phase * (360.0f / _cpr);
    irq_state.last_reading_ms = timestamp / 1000;  // Convert microseconds to milliseconds
}

void AP_Quadrature::update(void)
{
    static bool first_run = true;
    if (first_run) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Quadrature::update() called, pins A=%d B=%d", 
                      get_pin_a(), get_pin_b());
        first_run = false;
    }

    update_pin(_pinA, get_pin_a(), A_value);
    update_pin(_pinB, get_pin_b(), B_value);

    // disable interrupts to prevent race with irq_handler
    void *irqstate = hal.scheduler->disable_interrupts_save();

    // copy count, angle and last timestamp so it is accessible to front end
    copy_state_to_frontend(irq_state.phase,
                           irq_state.angle,
                           irq_state.last_reading_ms);

    // restore interrupts
    hal.scheduler->restore_interrupts(irqstate);

}