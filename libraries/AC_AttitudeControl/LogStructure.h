#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_ATT_CONTROL \
    LOG_ACTS0_MSG,               \
        LOG_ACTS1_MSG,           \
        LOG_ACTS2_MSG

// @LoggerMessage: ATS0
// @Description: Attitude control TailSitter messages 0
// @Field: TimeUS: Time since system startup
// @Field: FnetZ: Net horizontal force
// @Field: FnetX: Net vertical force
// @Field: FwindP: Wind perpendicular force
// @Field: PhiLeft: Thrust vectoring angle left
// @Field: PhiRight: Thrust vectoring angle right
struct PACKED log_ACTS0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float force_net_z;
    float force_net_x;
    float force_wind_p;
    float phi_left;
    float phi_right;
    float rpm_left;
    float rpm_right;
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
// @Field: BAccX: Body acceleration along X axis
// @Field: BAccY: Body acceleration along Y axis
// @Field: BAccZ: Body acceleration along Z axis
// @Field: ELFAccX: Earth local frame (ZX) acceleration along X axis
// @Field: ELFAccZ: Earth local frame (ZX) acceleration along Z axis
// @Field: PIDBst: Calculated Pitch PID boost wind
// @Field: PIDOutBst: Calculated Pitch PID output post boosting
struct PACKED log_ACTS2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float body_acc_x;
    float body_acc_y;
    float body_acc_z;
    float elf_acc_x;
    float elf_acc_z;
    float pitch_pid_boost_wind;
    float pid_out_boosted;
};

#define LOG_STRUCTURE_FROM_ATTC                                                                                                    \
    { LOG_ACTS0_MSG, sizeof(log_ACTS0),                                                                                            \
        "ATS0", "Qfffffff", "TimeUS,FnetZ,FnetX,FwindP,PhiLeft,PhiRight,rpmL,rpmR", "sNNNdd--", "F-------", true },                \
        { LOG_ACTS1_MSG, sizeof(log_ACTS1),                                                                                        \
            "ATS1", "Qffffffff", "TimeUS,ThstL,ThstLH,ThstLV,ThstLP,ThstR,ThstRH,ThstRV,ThstRP", "sNNNNNNNN", "F--------", true }, \
        { LOG_ACTS2_MSG, sizeof(log_ACTS2),                                                                                        \
            "ATS2", "Qfffffff", "TimeUS,BAccX,BAccY,BAccZ,ELFAccX,ELFAccZ,PIDBst,PIDOutBst", "sooooo--", "F-------", true },
