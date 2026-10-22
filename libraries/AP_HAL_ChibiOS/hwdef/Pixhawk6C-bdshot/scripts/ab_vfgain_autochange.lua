-- Sets Q_TAILSIT_VFGAIN based on whether the aircraft is actually in vertical (VTOL) flight,
-- not on the flight mode. Q-modes such as QRTL/QLAND fly a fixed wing approach first, and
-- AUTO can fly VTOL legs, so the mode number alone does not tell us how we are flying.
--   Vertical flight          -> VFGAIN_VTOL
--   Fixed wing / transitions -> VFGAIN_FW

local VFGAIN_VTOL       = 0.2
local VFGAIN_FW         = 0
local LOOP_ARMED_MS     = 200  -- fast enough to follow transitions
local LOOP_DISARMED_MS  = 1000

-- True in a VTOL mode (or VTOL leg of AUTO) once past the fixed wing approach. This includes
-- the back transition pull-up, where vectoring helps the elevons rotate the aircraft to vertical
local function in_vertical_flight()
    return quadplane:in_vtol_mode()
end

function update()
    if not arming:is_armed() then
        return update, LOOP_DISARMED_MS  -- do nothing if disarmed
    end

    local vertical = in_vertical_flight()
    local target = vertical and VFGAIN_VTOL or VFGAIN_FW

    -- compare against the live value so writes by other scripts are corrected
    local current = param:get('Q_TAILSIT_VFGAIN')
    if current == nil or math.abs(current - target) > 0.001 then
        param:set('Q_TAILSIT_VFGAIN', target)
        if vertical then
            gcs:send_text(6, string.format("Set Q_TAILSIT_VFGAIN to %.1f (vertical flight)", target))
        else
            gcs:send_text(6, string.format("Set Q_TAILSIT_VFGAIN to %.1f (fixed wing flight)", target))
        end
    end

    return update, LOOP_ARMED_MS
end

gcs:send_text(6, "Airbound Lua: VF gain script loaded.")
return update()
