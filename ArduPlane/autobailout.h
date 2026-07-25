#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

// Autobailout parameters
// Mirror the AUTOB_ parameters from the autobailout.lua script. 

class Autobailout {
public:
    Autobailout() { AP_Param::setup_object_defaults(this, var_info); }
    CLASS_NO_COPY(Autobailout);

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float pit_lim;     // Zone 4 entry: AHRS pitch below which bailout triggers (deg)
    AP_Int16 pit_tout;    // Zone 4 entry: sustain time before bailout (ms)
    AP_Int32 btrn_dly;    // settle delay after back-transition before bailout arms (ms)
    AP_Float para_ang;    // Zone 5 entry: AHRS pitch below which parachute triggers (deg)
    AP_Int16 para_tout;   // Zone 5 entry: sustain time before parachute (ms)
    AP_Int8  para_en;     // enable the Zone 5 parachute deploy action
    AP_Float win_tim;     // Zone 4 exit: rolling recovery window (s)
    AP_Float avg_lim;     // Zone 4 exit: mean pitch-error recovery limit (deg)
    AP_Float peak_lim;    // Zone 4 exit: peak pitch recovery limit (deg)
};
