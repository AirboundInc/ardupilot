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

void AP_Quadrature::update_pin(uint8_t &pin, uint8_t new_pin, uint8_t &pin_value)
{
    if (new_pin == pin) {
        // no change
        return;
    }

    // remove old gpio event callback if present
    if (pin != (uint8_t)-1 &&
        !hal.gpio->detach_interrupt(pin)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "QEnc: Failed to detach from pin %u", pin);
        // ignore this failure or the user may be stuck
    }

    pin = new_pin;

    // install interrupt handler on rising or falling edge of gpio for pin a
    if (new_pin != (uint8_t)-1) {
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
    uint8_t index = 0;
    if(pin == _pinA) {
        index = pin_value << 3 | B_value << 2 | A_value << 1 | B_value;
        A_value = pin_value;
    }

    else if(pin == _pinB) {
        index = A_value << 3 | pin_value << 2 | B_value << 1 | A_value;
        B_value = pin_value;
    }

    else {
        return;
    }

    irq_state.phase = _lookupTable[index];
    irq_state.angle = irq_state.phase * (360.0f / _cpr);
    irq_state.last_reading_ms = timestamp * 1e-3f;

}

void AP_Quadrature::update(void)
{
    update_pin(_pinA, get_pin_a(), A_value);
    update_pin(_pinB, get_pin_b(), B_value);

    // disable interrupts to prevent race with irq_handler
    void *irqstate = hal.scheduler->disable_interrupts_save();

    // copy phase, angle and last timestamp so it is accessible to front end
    copy_state_to_frontend(irq_state.phase,
                           irq_state.angle,
                           irq_state.last_reading_ms);

    // restore interrupts
    hal.scheduler->restore_interrupts(irqstate);
}