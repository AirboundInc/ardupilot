#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_ATT_CONTROL \
    LOG_ACTS0_MSG,               \
        LOG_ACTS1_MSG,           \
        LOG_ACTS2_MSG

// @LoggerMessage: ATS0
// @Description: Attitude control TailSitter messages 0
// @Field: TimeUS: Time since system startup
// @Field: BodyAccZ: Filtered acceleration along Z axis in body frame
// @Field: FnetP: Net perpendicular force
// @Field: FwindP: Wind perpendicular force
// @Field: PhiLeft: Thrust vectoring angle left
// @Field: PhiRight: Thrust vectoring angle right
// @Field: PIDBoost: Calculated Pitch PID boost wind
struct PACKED log_ACTS0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float body_acc_z;
    float f_net_p;
    float f_wind_p;
    float phi_left;
    float phi_right;
    float rpm_left;
    float rpm_right;
    float pitch_pid_boost_wind;
};

// @LoggerMessage: ATS1
// @Description: Attitude control TailSitter messages 1
// @Field: TimeUS: Time since system startup
// @Field: ThstL: Thrust from left motor
// @Field: ThstH: Horizontal component of left thrust
// @Field: ThstLV: Vertical component of left thrust
// @Field: ThstLP: Perpendicular component of left thrust
// @Field: ThstR: Thst from right motor
// @Field: ThstRH: Horizontal component of right thrust
// @Field: ThstRV: Vertical component of right thrust
// @Field: ThstRP: Perpendicular component of right thrust
struct PACKED log_ACTS1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;

    float thrust_left;
    float thrust_left_h;
    float thrust_left_v;
    float thrust_left_p;

    float thrust_right;
    float thrust_right_h;
    float thrust_right_v;
    float thrust_right_p;
};

// @LoggerMessage: ATS2
// @Description: Attitude control TailSitter messages 2
// @Field: TimeUS: Time since system startup
// @Field: BodyAccX: Body acceleration along X axis
// @Field: BodyAccY: Body acceleration along Y axis
// @Field: BodyAccZ: Body acceleration along Z axis
// @Field: AccZGC: Body acceleration along Z axis - GRAVITY_MSS * sin(pitch)
struct PACKED log_ACTS2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float body_acc_x;
    float body_acc_y;
    float body_acc_z;
    float accel_g_z_comp;
};

#define LOG_STRUCTURE_FROM_ATTC                                                                                                    \
    { LOG_ACTS0_MSG, sizeof(log_ACTS0),                                                                                            \
        "ATS0", "Qffffffff", "TimeUS,BodyAccZ,FnetP,FwindP,PhiLeft,PhiRight,rpmL,rpmR,PIDBst", "soNNdd---", "F--------", true },   \
        { LOG_ACTS1_MSG, sizeof(log_ACTS1),                                                                                        \
            "ATS1", "Qffffffff", "TimeUS,ThstL,ThstLH,ThstLV,ThstLP,ThstR,ThstRH,ThstRV,ThstRP", "sNNNNNNNN", "F--------", true }, \
        { LOG_ACTS2_MSG, sizeof(log_ACTS2),                                                                                        \
            "ATS2", "Qffff", "TimeUS,BodyAccX,BodyAccY,BodyAccZ,AccZGC", "soooo", "F----", true },
