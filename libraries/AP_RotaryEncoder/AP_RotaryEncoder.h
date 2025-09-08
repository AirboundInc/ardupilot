#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_RotaryEncoder_config.h"

// Maximum number of RotaryEncoder measurement instances available on this platform
#define ROTARY_ENCODER_MAX_INSTANCES      2
#define ROTARY_ENCODER_CPR_DEFAULT        4096    // default encoder counts per full revolution of the rotary encoder


class AP_RotaryEncoder_Backend; 
 
class AP_RotaryEncoder
{
public:
    friend class AP_RotaryEncoder_Backend;
    friend class AP_RotaryEncoder_Quadrature;

    AP_RotaryEncoder(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RotaryEncoder );

    // get singleton instance
    static AP_RotaryEncoder *get_singleton() {
        return _singleton;
    }

    // RotaryEncoder driver types
    enum RotaryEncoder_Type : uint8_t {
        RotaryEncoder_TYPE_NONE             =   0,
        RotaryEncoder_TYPE_QUADRATURE       =   1,
    };

    // The RotaryEncoder_State structure is filled in by the backend driver
    struct RotaryEncoder_State {
        uint8_t                instance;                        // the instance number of this RotaryEncoder
        int32_t                count;                           // encoder position in counts 0 to CPR
        float                  angular_position;                // total angle measured in radians
        uint32_t               last_reading_ms;                 // time of last reading
        int32_t                angular_position_count_change;   // angle position count change during the last update (used to calculating rate)
        uint32_t               dt_ms;                           // time change (in milliseconds) for the previous period (used to calculating rate)
    };

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all sensors. Should be called from main loop
    void update(void);

    // log data to logger
    void Log_Write() const;

    // return the number of rotary encoder sensor instances
    uint8_t num_sensors(void) const { return num_instances; }

    // return true if healthy
    bool healthy(uint8_t instance) const;

    // return true if the instance is enabled
    bool enabled(uint8_t instance) const;

    // return true if any encoder instance is configured and enabled
    bool any_enabled(void) const;

    // get the counts per revolution of the encoder
    uint16_t get_counts_per_revolution(uint8_t instance) const;

    // get total delta angle (in radians) measured by the rotary encoder
    float get_delta_angle(uint8_t instance) const;

    // get the total angle position (in radians) measured by the rotary encoder
    float get_angular_position(uint8_t instance, bool degrees) const;

    // get the instantaneous rate in radians/second
    float get_rate(uint8_t instance, bool degrees) const;

    // get the system time (in milliseconds) of the last update
    uint32_t get_last_reading_ms(uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // parameters for each instance
    AP_Int8  _type[ROTARY_ENCODER_MAX_INSTANCES];
    AP_Int16 _counts_per_revolution[ROTARY_ENCODER_MAX_INSTANCES];
    AP_Int8  _pina[ROTARY_ENCODER_MAX_INSTANCES];
    AP_Int8  _pinb[ROTARY_ENCODER_MAX_INSTANCES];
    AP_Float pos_offset_zero[ROTARY_ENCODER_MAX_INSTANCES];

    RotaryEncoder_State state[ROTARY_ENCODER_MAX_INSTANCES];
    AP_RotaryEncoder_Backend *drivers[ROTARY_ENCODER_MAX_INSTANCES];
    uint8_t num_instances;

private:

    static AP_RotaryEncoder *_singleton;
};

namespace AP {
    AP_RotaryEncoder *rotaryencoder();
}
