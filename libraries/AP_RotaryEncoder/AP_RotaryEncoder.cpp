/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RotaryEncoder.h"
#include "AP_Quadrature.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_RotaryEncoder::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: RotaryEncoder type
    // @Description: What type of RotaryEncoder is connected
    // @Values: 0:None,1:Quadrature,10:SITL Quadrature
    // @User: Standard
    AP_GROUPINFO_FLAGS("L_TYPE", 0, AP_RotaryEncoder, _type[0], 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _CPR
    // @DisplayName: RotaryEncoder counts per revolution
    // @Description: RotaryEncoder counts per full revolution of the rotary encoder
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("L_CPR",     1, AP_RotaryEncoder, _counts_per_revolution[0], ROTARY_ENCODER_CPR_DEFAULT),

    // @Param: _POS_0
    // @DisplayName: RotaryEncoder position offset
    // @Description: Add a positional offset for the rotary encoder
    // @Units: radians
    // @Range: -3.14 3.14
    // @Increment: 0.01
    // @User: Standard

    AP_GROUPINFO("L_POS0",     3, AP_RotaryEncoder, pos_offset_zero[0], 0.0f),

    // @Param: _PINA
    // @DisplayName: Input Pin A
    // @Description: Input Pin A
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("L_PINA",    4, AP_RotaryEncoder, _pina[0], -1),

    // @Param: _PINB
    // @DisplayName: Input Pin B
    // @Description: Input Pin B
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("L_PINB",    5, AP_RotaryEncoder, _pinb[0], -1),

#if ROTARY_ENCODER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second RotaryEncoder type
    // @Description: What type of RotaryEncoder sensor is connected
    // @Values: 0:None,1:Quadrature,10:SITL Quadrature
    // @User: Standard
    AP_GROUPINFO("R_TYPE",   6, AP_RotaryEncoder, _type[1], 0),

    // @Param: 2_CPR
    // @DisplayName: RotaryEncoder 2 counts per revolution
    // @Description: RotaryEncoder 2 counts per full revolution of the rotary encoder
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("R_CPR",     7, AP_RotaryEncoder, _counts_per_revolution[1], ROTARY_ENCODER_CPR_DEFAULT),

    // @Param: 2_POS_X
    // @DisplayName: RotaryEncoder 2's X position offset
    // @Description: X position of the center of the second rotary encoder in body frame. Positive X is forward of the origin.
    // @Units: radians
    // @Range: -3.14 3.14
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("R_POS0",    9, AP_RotaryEncoder, pos_offset_zero[1], 0.0f),

    // @Param: 2_PINA
    // @DisplayName: Second Encoder Input Pin A
    // @Description: Second Encoder Input Pin A
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("R_PINA",   10, AP_RotaryEncoder, _pina[1], 53),

    // @Param: 2_PINB
    // @DisplayName: Second Encoder Input Pin B
    // @Description: Second Encoder Input Pin B
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("R_PINB",   11, AP_RotaryEncoder, _pinb[1], 52),
#endif

    AP_GROUPEND
};

AP_RotaryEncoder::AP_RotaryEncoder(void)
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_RotaryEncoder class.
void AP_RotaryEncoder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<ROTARY_ENCODER_MAX_INSTANCES; i++) {
        switch ((RotaryEncoder_Type)_type[i].get()) {

        case RotaryEncoder_TYPE_QUADRATURE:
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
            drivers[i] = new AP_Quadrature(*this, i, state[i]);
#endif
            break;
        case RotaryEncoder_TYPE_NONE:
            break;
        }


        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;  // num_instances is a high-water-mark
        }
    }
}

// update RotaryEncoder state for all instances. This should be called by main loop
void AP_RotaryEncoder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr && _type[i] != RotaryEncoder_TYPE_NONE) {
            drivers[i]->update();
        }
    }
}

#if HAL_LOGGING_ENABLED
// log rotary encoder information
void AP_RotaryEncoder::Log_Write() const
{
    // return immediately if no rotary encoders are enabled
    if (!enabled(0) && !enabled(1)) {
        return;
    }

    struct log_RotaryEncoder pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ROTARYENCODER_MSG),
        time_us     : AP_HAL::micros64(),
        angular_position_0  : get_angular_position(0),
        angular_velocity_0  : get_rate(0),
        angular_position_1  : get_angular_position(1),
        angular_velocity_1  : get_rate(1),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif

// check if an instance is healthy
bool AP_RotaryEncoder::healthy(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    return true;
}

// check if an instance is activated
bool AP_RotaryEncoder::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    if (_type[instance] == RotaryEncoder_TYPE_NONE) {
        return false;
    }
    return true;
}

// get the counts per revolution of the encoder
uint16_t AP_RotaryEncoder::get_counts_per_revolution(uint8_t instance) const
{
    // for invalid instances return zero vector
    if (instance >= ROTARY_ENCODER_MAX_INSTANCES) {
        return 0;
    }
    return (uint16_t)_counts_per_revolution[instance];
}

// get the angular position in degrees
float AP_RotaryEncoder::get_angular_position(uint8_t instance) const
{
    // for invalid instances return zero
    return M_2_PI * state[instance].count / _counts_per_revolution[instance];
}

// get the instantaneous rate in radians/second
float AP_RotaryEncoder::get_rate(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= ROTARY_ENCODER_MAX_INSTANCES) {
        return 0.0f;
    }

    // protect against divide by zero
    if ((state[instance].dt_ms == 0) || _counts_per_revolution[instance] == 0) {
        return 0;
    }

    // calculate delta_angle (in radians) per second
    return M_2PI * (state[instance].angular_position_count_change / ((float)_counts_per_revolution[instance])) / (state[instance].dt_ms * 1e-3f);
}

// get the system time (in milliseconds) of the last update
uint32_t AP_RotaryEncoder::get_last_reading_ms(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= ROTARY_ENCODER_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].last_reading_ms;
}

// singleton instance
AP_RotaryEncoder *AP_RotaryEncoder::_singleton;

namespace AP {

AP_RotaryEncoder *rotaryencoder()
{
    return AP_RotaryEncoder::get_singleton();
}

}