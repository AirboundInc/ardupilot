#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include "AP_QuadEncoder.h"

const int8_t AP_QuadEncoder::_lookupTable[16] = {
    0,  -1,   1,   0,
    1,   0,   0,  -1,
   -1,   0,   0,   1,
    0,   1,  -1,   0
};

AP_QuadEncoder* AP_QuadEncoder::_left_instance = nullptr;
AP_QuadEncoder* AP_QuadEncoder::_right_instance = nullptr;

AP_QuadEncoder::AP_QuadEncoder(uint8_t pinA, uint8_t pinB, int32_t cpr, Side side)
    : _pinA(pinA), _pinB(pinB), _cpr(cpr), _position(0), _prevAB(0), _side(side)
{
    if (_side == LEFT) {
        _left_instance = this;
    } else {
        _right_instance = this;
    }
}

void AP_QuadEncoder::init() {
    auto& gpio = hal.gpio;
    gpio->pinMode(_pinA, HAL_GPIO_INPUT);
    gpio->pinMode(_pinB, HAL_GPIO_INPUT);

    _prevAB = (gpio->read(_pinA) << 1) | gpio->read(_pinB);

    if (_side == LEFT) {
        gpio->attach_interrupt(_pinA, isr_left, AP_HAL::GPIO::INTERRUPT_BOTH);
        gpio->attach_interrupt(_pinB, isr_left, AP_HAL::GPIO::INTERRUPT_BOTH);
    } else {
        gpio->attach_interrupt(_pinA, isr_right, AP_HAL::GPIO::INTERRUPT_BOTH);
        gpio->attach_interrupt(_pinB, isr_right, AP_HAL::GPIO::INTERRUPT_BOTH);
    }
}

void AP_QuadEncoder::isr_left() {
    if (_left_instance) _left_instance->isr();
}
void AP_QuadEncoder::isr_right() {
    if (_right_instance) _right_instance->isr();
}

void AP_QuadEncoder::isr() {
    auto& gpio = hal.gpio;
    uint8_t a = gpio->read(_pinA);
    uint8_t b = gpio->read(_pinB);
    uint8_t currAB = (a << 1) | b;
    uint8_t index = (_prevAB << 2) | currAB;
    _position += _lookupTable[index];
    _prevAB = currAB;
}

int32_t AP_QuadEncoder::get_position() const {
    return _position;
}

float AP_QuadEncoder::get_angle() const {
    return (float(_position) / float(_cpr)) * 360.0f;
}