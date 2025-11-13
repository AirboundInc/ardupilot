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
#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_RotaryEncoder.h"

class AP_RotaryEncoder_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_RotaryEncoder_Backend(AP_RotaryEncoder &frontend, uint8_t instance, AP_RotaryEncoder::RotaryEncoder_State &state);

    // we declare a virtual destructor so that RotaryEncoder drivers can
    // override with a custom destructor if need be
    virtual ~AP_RotaryEncoder_Backend(void) {}

    // update the state structure. All backends must implement this.
    virtual void update() = 0;

protected:

    // return pin number.  returns -1 if pin is not defined for this instance
    int8_t get_pin_a() const;
    int8_t get_pin_b() const;

    // copy state to front end helper function
    void copy_state_to_frontend(int32_t count, uint32_t angle, uint32_t time);

    AP_RotaryEncoder &_frontend;
    AP_RotaryEncoder::RotaryEncoder_State &_state;
};
