#pragma once

/*
 Generic PD controller
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AC_PD {
public:
    // Constructor
    AC_PD(float initial_p, float initial_d);

    CLASS_NO_COPY(AC_PD);

    // update controller
    float update(float error, float dt);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    float get_P() const {
        return output_P;
    }
    float get_D() const {
        return output_D;
    }

    // Load gain properties
    void        load_gains();

    // Save gain properties
    void        save_gains();

    // Accessors
    AP_Float    &kP() { return _kp; }
    const AP_Float &kP() const { return _kp; }
    void        kP(const float v) { _kp.set(v); }

    AP_Float    &kD() { return _kd; }
    const AP_Float &kD() const { return _kd; }
    void        kD(const float v) { _kd.set(v); }

protected:
    AP_Float        _kp;
    AP_Float        _kd;
    float           output_P;
    float           output_D;
    float           derivative;

private:
    const float default_kp;
    const float default_kd;
};
