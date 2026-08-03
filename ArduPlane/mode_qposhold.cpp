#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQPosHold::_enter()
{
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // manual throttle, so there's nothing to "wait" for like QLOITER's
    // arm-then-wait-for-stick-up gate -- mirrors QSTABILIZE's _enter()
    quadplane.throttle_wait = false;

    quadplane.last_loiter_ms = AP_HAL::millis();
    last_target_loc_set_ms = 0;

    return true;
}

void ModeQPosHold::update()
{
    plane.mode_qstabilize.update();
}

void ModeQPosHold::run()
{
    if (quadplane.assist.check_VTOL_recovery()) {
        plane.mode_qhover.run();
        return;
    }

    const uint32_t now = AP_HAL::millis();

#if AC_PRECLAND_ENABLED
    const uint32_t precland_timeout_ms = 250;
    const uint32_t last_pos_set_ms = last_target_loc_set_ms;
    const uint32_t last_vel_set_ms = quadplane.poscontrol.last_velocity_match_ms;

    if (last_pos_set_ms != 0 && now - last_pos_set_ms < precland_timeout_ms) {
        Vector2f rel_origin;
        if (plane.next_WP_loc.get_vector_xy_from_origin_NE(rel_origin)) {
            quadplane.pos_control->set_pos_desired_xy_cm(rel_origin);
            last_target_loc_set_ms = 0;
        }
    }
    if (last_vel_set_ms != 0 && now - last_vel_set_ms < precland_timeout_ms) {
        Vector2f target_accel;
        Vector2f target_speed_xy_cms{quadplane.poscontrol.velocity_match.x*100, quadplane.poscontrol.velocity_match.y*100};
        quadplane.pos_control->input_vel_accel_xy(target_speed_xy_cms, target_accel);
        quadplane.poscontrol.last_velocity_match_ms = 0;
    }
#endif

    if (quadplane.tailsitter.in_vtol_transition(now)) {
        Mode::run();
        return;
    }

    // ---- throttle: raw pilot stick, same source QSTABILIZE uses ----
    const float pilot_throttle_scaled = quadplane.get_pilot_throttle();

    if (quadplane.should_relax()) {
        loiter_nav->soften_for_landing();
    }

    if (now - quadplane.last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    quadplane.last_loiter_ms = now;

    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // ---- XY: identical to QLOITER ----
    float target_roll_cd, target_pitch_cd;
    quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd,
                                             loiter_nav->get_angle_max_cd(),
                                             attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);

    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    loiter_nav->update();

    plane.nav_roll_cd = loiter_nav->get_roll();
    plane.nav_pitch_cd = loiter_nav->get_pitch();

    plane.quadplane.assign_tilt_to_fwd_thr();

    if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_xy();
    }

    quadplane.set_pilot_yaw_rate_time_constant();

    Vector3f target { plane.nav_roll_cd*0.01f, plane.nav_pitch_cd*0.01f, quadplane.get_desired_yaw_rate_cds() * 0.01f };

#if AP_PLANE_SYSTEMID_ENABLED
    auto &systemid = plane.g2.systemid;
    systemid.update();
    target += systemid.get_attitude_offset_deg();
#endif

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target.x*100, target.y*100, target.z*100);

    // ---- Z: raw pilot throttle, bypassing pos_control's z-controller entirely ----
    if ((pilot_throttle_scaled <= 0) && !quadplane.air_mode_active()) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
    } else {
        float throttle_out = pilot_throttle_scaled;
#if AP_PLANE_SYSTEMID_ENABLED
        throttle_out += plane.g2.systemid.get_throttle_offset();
#endif
        attitude_control->set_throttle_out(throttle_out, true, 0);
    }


    plane.stabilize_roll();
    plane.stabilize_pitch();

    output_rudder_and_steering(0.0);
}

#endif
