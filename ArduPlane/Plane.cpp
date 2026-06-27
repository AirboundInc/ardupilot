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
#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Get target altitude relative to home - for lua bindings
bool Plane::get_target_alt_rel_home_m(float &alt_m) const
{
    if (!control_mode->does_auto_throttle()) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif
    alt_m = TECS_controller.get_hgt_dem();
    return true;
}

// Get current altitude relative to home - for lua bindings
bool Plane::get_current_alt_rel_home_m(float &alt_m) const
{
    if (!control_mode->does_auto_throttle()) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif
    alt_m = TECS_controller.get_height();
    return true;
}

// Get target roll - for lua bindings
bool Plane::get_target_roll_deg(float &roll_deg) const
{
    if (!control_mode->does_auto_throttle()) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif
    roll_deg = nav_roll_cd * 0.01f;
    return true;
}

// Get target pitch - for lua bindings
bool Plane::get_target_pitch_deg(float &pitch_deg) const
{
    if (!control_mode->does_auto_throttle()) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif
    pitch_deg = nav_pitch_cd * 0.01f;
    return true;
}

// Check for fixed wing flight - for lua bindings
bool Plane::is_fixed_wing_flight(void) const
{
    if (!control_mode->does_auto_throttle()) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif
    return true;
}

// Crosstrack error for fixed wing flight - for lua bindings
bool Plane::get_crosstrack_error_m(float &xte_m) const
{
    if (!control_mode->does_auto_throttle()) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif
    if (nav_controller == nullptr) {
        return false;
    }
    xte_m = nav_controller->crosstrack_error();
    return true;
}

/*
  constructor for main Plane class
 */
Plane::Plane(void)
#if HAL_LOGGING_ENABLED
    : logger(g.log_bitmask)
#endif
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

Plane plane;
AP_Vehicle& vehicle = plane;
