#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include "AP_AutoTune.h"

class AP_YawController
{
public:
    AP_YawController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_YawController);

    // return true if rate control or damping is enabled
    bool enabled() const { return rate_control_enabled() || (_K_D > 0.0); } 

    // return true if rate control is enabled
    bool rate_control_enabled(void) const { return _rate_enable != 0; }

    // heading-hold outer loop: returns a demanded yaw rate (deg/s) to close
    // heading error while wings-level with no rudder input. allow_lock should
    // be false whenever banked/turning or the pilot has rudder input, in
    // which case the lock releases and 0 is returned.
    float get_heading_hold_rate(bool allow_lock, float max_rate);

    // true if the heading-hold outer loop currently has a locked target heading
    bool heading_locked() const { return _heading_locked; }

    // the locked target heading in centidegrees, only meaningful when heading_locked() is true
    int32_t locked_heading_cd() const { return _locked_heading_cd; }


    // get actuator output for sideslip and yaw damping control
    int32_t get_servo_out(float scaler, bool disable_integrator);

    // get actuator output for direct rate control
    // desired_rate is in deg/sec. scaler is the surface speed scaler
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator);

    void reset_I();

    void reset_rate_PID();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    // start/stop auto tuner
    void autotune_start(void);
    void autotune_restore(void);
    

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_FixedWing &aparm;
    AP_Float _K_A;
    AP_Float _K_I;
    AP_Float _K_D;
    AP_Float _K_FF;
    AP_Int16 _imax;
    AP_Int8  _rate_enable;
    AP_Float _K_HDG;
    bool _heading_locked;
    uint32_t _heading_lock_timer_ms;
    int32_t _locked_heading_cd;
    AC_PID rate_pid{0.04, 0.15, 0, 0.15, 0.666, 3, 0, 12, 150, 1};

    uint32_t _last_t;
    float _last_out;
    float _last_rate_hp_out;
    float _last_rate_hp_in;
    float _K_D_last;

    float _integrator;

    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    
    AP_PIDInfo _pid_info;
};
