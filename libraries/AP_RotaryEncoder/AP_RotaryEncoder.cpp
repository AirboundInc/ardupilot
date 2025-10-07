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
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const struct AP_Param::GroupInfo AP_RotaryEncoder::var_info[] = {
    // @Param: LT
    // @DisplayName: RotaryEncoder type for left encoder
    // @Description: What type of RotaryEncoder is connected
    // @Values: 0:None,1:Quadrature
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("LT", 0, AP_RotaryEncoder, _type[0], 0, AP_PARAM_FLAG_ENABLE),

    // @Param: LC
    // @DisplayName: RotaryEncoder counts per revolution for left encoder
    // @Description: RotaryEncoder counts per full revolution of the rotary encoder
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("LC",     1, AP_RotaryEncoder, _counts_per_revolution[0], ROTARY_ENCODER_CPR_DEFAULT),

    // @Param: L0
    // @DisplayName: RotaryEncoder position offset for left encoder
    // @Description: Add a positional offset for the rotary encoder
    // @Units: deg
    // @Range: -180 180
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("L0",     2, AP_RotaryEncoder, pos_offset_zero[0], 0.0f),

    // @Param: LA
    // @DisplayName: Input Pin A for left encoder
    // @Description: Input Pin A
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("LA",   3, AP_RotaryEncoder, _pina[0], -1),

    // @Param: LB
    // @DisplayName: Input Pin B for left encoder
    // @Description: Input Pin B
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("LB",   4, AP_RotaryEncoder, _pinb[0], -1),

#if ROTARY_ENCODER_MAX_INSTANCES > 1
    // @Param: RT
    // @DisplayName: RotaryEncoder type for right encoder
    // @Description: What type of RotaryEncoder sensor is connected
    // @Values: 0:None,1:Quadrature
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RT",   5, AP_RotaryEncoder, _type[1], 0),

    // @Param: RC
    // @DisplayName: RotaryEncoder counts per revolution for right encoder
    // @Description: RotaryEncoder counts per full revolution of the rotary encoder
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RC",   6, AP_RotaryEncoder, _counts_per_revolution[1], ROTARY_ENCODER_CPR_DEFAULT),

    // @Param: R0
    // @DisplayName: RotaryEncoder's X position offset for right encoder
    // @Description: X position of the center of the second rotary encoder in body frame. Positive X is forward of the origin.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("R0",  7, AP_RotaryEncoder, pos_offset_zero[1], 0.0f),

    // @Param: RA
    // @DisplayName: Input Pin A for right encoder 
    // @Description: Input Pin A
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RA",   8, AP_RotaryEncoder, _pina[1], -1),

    // @Param: RB
    // @DisplayName: Input Pin B for right encoder
    // @Description: Input Pin B
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RB",   9, AP_RotaryEncoder, _pinb[1], -1),
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
            drivers[i] = new AP_Quadrature(*this, i, state[i]);
            break;
        case RotaryEncoder_TYPE_NONE:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "No encoder configured for instance %d", i);
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
    // Early return if no encoders are configured to save CPU cycles
    if (!any_enabled()) {
        return;
    }
    
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
        pos_left    : get_angular_position(0, true),
        pos_right   : get_angular_position(1, true),
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

// check if any encoder instance is configured and enabled
bool AP_RotaryEncoder::any_enabled(void) const
{
    for (uint8_t i = 0; i < ROTARY_ENCODER_MAX_INSTANCES; i++) {
        if (_type[i] != RotaryEncoder_TYPE_NONE && 
            _pina[i] != -1 && 
            _pinb[i] != -1) {
            return true;
        }
    }
    return false;
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
float AP_RotaryEncoder::get_angular_position(uint8_t instance, bool degrees) const
{
    // for invalid instances return zero
    if (instance >= ROTARY_ENCODER_MAX_INSTANCES) {
        return 0.0f;
    }

    float position = M_2_PI * state[instance].count / _counts_per_revolution[instance];
    return degrees ? position * (180.0f / M_PI) - pos_offset_zero[instance] : position - pos_offset_zero[instance] * M_2_PI / _counts_per_revolution[instance];
}

// get the system time (in milliseconds) of the last update
uint32_t AP_RotaryEncoder::get_last_reading_ms(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= ROTARY_ENCODER_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].time;
}

void AP_RotaryEncoder::set_position_offset(uint8_t instance, float position) {
    pos_offset_zero[instance].set(position);
}

// singleton instance
AP_RotaryEncoder *AP_RotaryEncoder::_singleton;

namespace AP {

AP_RotaryEncoder *rotaryencoder()
{
    return AP_RotaryEncoder::get_singleton();
}

}