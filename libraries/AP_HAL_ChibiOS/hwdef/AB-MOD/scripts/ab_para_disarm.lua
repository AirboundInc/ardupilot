-- Description: This script monitors a specific RC channel and triggers a parachute deployment when the channel is pressed.
-- It disarms the vehicle and waits a configurable delay before deploying the parachute using a servo.
-- Uses a Lua-defined parameter (PARA_DELAY_MS) that can be edited in Mission Planner for delayed parachute deployment after disarm.
-- Uses a Lua-defined parameter (PARA_TRIG_CH) that can be edited in Mission Planner to configure the RC channel that can be monitored for Parachute deployment.
--
-- SAFETY INTERLOCK: while the safety switch is engaged (hardware button, or "toggle safety switch"
-- from the GCS) the PWM output is blocked by the flight controller, so the parachute physically
-- cannot deploy. This script therefore does not pretend otherwise: the trigger is ignored, no
-- "deploying"/"DEPLOYED" message is sent, any pending deployment is aborted, and the servo is held
-- at PARA_PWM_HOLD. After the safety switch is released the trigger must be released and pressed
-- again -- a switch left held high while safety was engaged will never fire on its own.
-- The deploy messages are also suppressed if no SERVOx_FUNCTION is set to SERVO_FUNCTION_NUM,
-- the other case where the servo command goes nowhere.

-- CONFIGURABLE CONSTANTS
local SERVO_FUNCTION_NUM = 94              -- Servo function (e.g., Scripting1)
local RC_TRIGGER_THRESHOLD = 1800          -- RC input value threshold
local LOOP_INTERVAL_MS = 100               -- Script loop interval
local GCS_SEVERITY_INFO = 6                -- MAVLink info severity level
local GCS_SEVERITY_WARNING = 4             -- MAVLink warning severity (yellow HUD)
local GCS_SEVERITY_CRITICAL = 2            -- MAVLink critical severity (red HUD)

-- PARAMETER SETUP
local PARAM_TABLE_KEY = 72
assert(param:add_table(PARAM_TABLE_KEY, "PARA_", 4), "Failed to create param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DELAY_MS", 100), "Failed to create Param1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "TRIG_CH", 11), "Failed to create Param2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "PWM_HOLD", 1900), "Failed to create Param3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "PWM_RELEASE", 1100), "Failed to create Param4")

local PARA_DELAY_MS = Parameter()
PARA_DELAY_MS:init("PARA_DELAY_MS")
local PARA_TRIG_CH = Parameter()
PARA_TRIG_CH:init("PARA_TRIG_CH")
local PARA_PWM_HOLD = Parameter()
PARA_PWM_HOLD:init("PARA_PWM_HOLD")
local PARA_PWM_RELEASE = Parameter()
PARA_PWM_RELEASE:init("PARA_PWM_RELEASE")

-- STATE VARIABLES
local last_pressed = false
local disarm_time = 0
local waiting_disarm_delay = false
local parachute_triggered = false
local last_inhibited = nil                 -- previous state of the safety interlock

-- MAIN LOOP
function update()
    local Channel_num = PARA_TRIG_CH:get()
    local rc_val = rc:get_pwm(Channel_num)
    local PWM_OFF = PARA_PWM_HOLD:get()
    local PWM_ON = PARA_PWM_RELEASE:get()

    -- SAFETY INTERLOCK: no deployment is possible while the safety switch is engaged.
    -- get_safety_state() is true when safety is on (hardware button or GCS toggle).
    local inhibited = SRV_Channels:get_safety_state()

    if inhibited ~= last_inhibited then
        last_inhibited = inhibited
        if inhibited then
            -- abort anything in flight and report the abort if a deploy was pending
            if parachute_triggered or waiting_disarm_delay then
                gcs:send_text(GCS_SEVERITY_CRITICAL, "Safety ON: pending parachute deploy ABORTED.")
            else
                gcs:send_text(GCS_SEVERITY_WARNING, "Safety ON: parachute deployment inhibited.")
            end
            waiting_disarm_delay = false
            parachute_triggered = false
        else
            gcs:send_text(GCS_SEVERITY_INFO, "Safety OFF: parachute trigger live.")
        end
    end

    if inhibited then
        -- hold the release servo parked and swallow any trigger edge, so that clearing the
        -- safety switch with the trigger already held high cannot deploy the parachute
        SRV_Channels:set_output_pwm(SERVO_FUNCTION_NUM, PWM_OFF)
        local pressed_now = (rc_val ~= nil) and (rc_val > RC_TRIGGER_THRESHOLD)
        -- warn once on each press of the trigger, however many times it is pressed
        if pressed_now and not last_pressed then
            gcs:send_text(GCS_SEVERITY_WARNING, "Parachute trigger ignored: safety switch engaged.")
        end
        -- require a fresh release->press transition once safety is cleared
        last_pressed = pressed_now
        return update, LOOP_INTERVAL_MS
    end

    if rc_val == nil then
        gcs:send_text(GCS_SEVERITY_INFO, "RC input not available.")
        return update, LOOP_INTERVAL_MS
    end

    local is_pressed = rc_val > RC_TRIGGER_THRESHOLD
    local delay_ms = PARA_DELAY_MS:get()

    if is_pressed ~= last_pressed then
        last_pressed = is_pressed

        if is_pressed then
            arming:disarm()
            disarm_time = millis()
            waiting_disarm_delay = true
            parachute_triggered = true
            gcs:send_text(GCS_SEVERITY_INFO, "Parachute triggered. Disarming. Waiting " .. delay_ms .. " ms.")
        else
            waiting_disarm_delay = false
            parachute_triggered = false
            SRV_Channels:set_output_pwm(SERVO_FUNCTION_NUM, PWM_OFF)
            gcs:send_text(GCS_SEVERITY_INFO, "RC released. Servo set to HOLD.")
        end
    end

    if waiting_disarm_delay and parachute_triggered and
       (millis() - disarm_time >= delay_ms) then

        -- only claim a deployment if the release output can actually reach a pin
        if SRV_Channels:find_channel(SERVO_FUNCTION_NUM) == nil then
            gcs:send_text(GCS_SEVERITY_CRITICAL, "PARACHUTE NOT DEPLOYED: no SERVO output set to function " .. SERVO_FUNCTION_NUM .. ".")
        else
            SRV_Channels:set_output_pwm(SERVO_FUNCTION_NUM, PWM_ON)
            gcs:send_text(GCS_SEVERITY_INFO, delay_ms .. " ms passed. Deploying Parachute.")
            gcs:send_text(GCS_SEVERITY_CRITICAL, "EMERGENCY: PARACHUTE DEPLOYED!")
        end

        waiting_disarm_delay = false
        parachute_triggered = false
    end
    -- Maintain servo state
    if parachute_triggered and waiting_disarm_delay then
        -- Keep servo in deploy position during wait
        SRV_Channels:set_output_pwm(SERVO_FUNCTION_NUM, PWM_ON)
    elseif not is_pressed then
        -- Ensure servo stays in idle position when not pressed
        SRV_Channels:set_output_pwm(SERVO_FUNCTION_NUM, PWM_OFF)
    end

    return update, LOOP_INTERVAL_MS
end

gcs:send_text(GCS_SEVERITY_INFO, "Airbound Lua: Parachute script loaded.")
if SRV_Channels:find_channel(SERVO_FUNCTION_NUM) == nil then
    gcs:send_text(GCS_SEVERITY_CRITICAL, "Parachute: no SERVO output set to function " .. SERVO_FUNCTION_NUM .. " - release DISABLED.")
end
return update()
