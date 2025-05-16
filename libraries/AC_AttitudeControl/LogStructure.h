#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_ATT_CONTROL \
    LOG_ATT_CONTROL_TS_MSG

// @LoggerMessage: ACTS
// @Description: Attitude Control Tailsitter messages
// @Field: TimeUS: Time since system startup
// @Field: FiltAccZ: Filtered acceleration along Z axis
// @Field: FnetP: Net perpendicular force
// @Field: FwindP: Wind perpendicular force
// @Field: PhiLeft: Thrust vectoring angle left 
// @Field: PhiRight: Thrust vectoring angle right
// @Field: ThrustRight: Thrust from right motor
// @Field: ThrustRightH: Horizontal component of right thrust
// @Field: ThrustRightV: Vertical component of right thrust
// @Field: ThrustRightP: Perpendicular component of right thrust
// @Field: ThrustLeft: Thrust from left motor
// @Field: ThrustLeftH: Horizontal component of left thrust
// @Field: ThrustLeftV: Vertical component of left thrust
// @Field: ThrustLeftP: Perpendicular component of left thrust
// @Field: PitchPIDBoostWind: Calculated Pitch PID boost wind

// attitude control tailsitter logging
// struct PACKED log_AttControlTS {
//     LOG_PACKET_HEADER;
//     uint64_t time_us;
//     float filt_acc_z;
//     float f_net_p;
//     float f_wind_p;
//     float phi_left;
//     float phi_right;
//     float thrust_right;
//     float thrust_right_h;
//     float thrust_right_v;
//     float thrust_right_p;
//     float thrust_left;
//     float thrust_left_h;
//     float thrust_left_v;
//     float thrust_left_p;
//     float pitch_pid_boost_wind;
// };

struct PACKED log_AttControlTS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float filt_acc_z;
};

#define LOG_STRUCTURE_FROM_ATTC                         \
    { LOG_ATT_CONTROL_TS_MSG, sizeof(log_AttControlTS), \
        "ACTS", "Qf", "TimeUS,FiltAccZ", "so", "F-", true },
