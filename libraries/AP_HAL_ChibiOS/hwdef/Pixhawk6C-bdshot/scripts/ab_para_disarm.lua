-- Description: This script monitors a specific RC channel and triggers a parachute deployment when the channel is pressed.
-- It disarms the vehicle and waits a configurable delay before deploying the parachute using a servo.
-- Uses a Lua-defined parameter (PARA_DELAY_MS) that can be edited in Mission Planner for delayed parachute deployment after disarm.
-- Uses a Lua-defined parameter (PARA_TRIG_CH) that can be edited in Mission Planner to configure the RC channel that can be monitored for Parachute deployment.

-- CONFIGURABLE CONSTANTS
local SERVO_FUNCTION_NUM = 94              -- Servo function (e.g., Scripting1)
local RC_TRIGGER_THRESHOLD = 1800          -- RC input value threshold
local LOOP_INTERVAL_MS = 100               -- Script loop interval
local GCS_SEVERITY_INFO = 6                -- MAVLink info severity level
local GCS_SEVERITY_CRITICAL = 2            -- MAVLink critical severity (red HUD)
local PARA_DEPLOY_MODES = { [17]=true, [18]=true, [19]=true, [20]=true, [21]=true }             -- Modes in which the auto-deploy failsafe is active

-- PARAMETER SETUP
local PARAM_TABLE_KEY = 72
assert(param:add_table(PARAM_TABLE_KEY, "PARA_", 8), "Failed to create param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DELAY_MS", 100), "Failed to create Param1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "TRIG_CH", 11), "Failed to create Param2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "PWM_HOLD", 1900), "Failed to create Param3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "PWM_RELEASE", 1100), "Failed to create Param4")
assert(param:add_param(PARAM_TABLE_KEY, 5, "TILT_MAX", 80), "Failed to create Param5")
assert(param:add_param(PARAM_TABLE_KEY, 6, "ALT_MIN", 20), "Failed to create Param6")
assert(param:add_param(PARAM_TABLE_KEY, 7, "HOLD_MS", 1500), "Failed to create Param7")
assert(param:add_param(PARAM_TABLE_KEY, 8, "SINK_MIN", 1.0), "Failed to create Param8")

local PARA_DELAY_MS = Parameter()
PARA_DELAY_MS:init("PARA_DELAY_MS")
local PARA_TRIG_CH = Parameter()
PARA_TRIG_CH:init("PARA_TRIG_CH")
local PARA_PWM_HOLD = Parameter()
PARA_PWM_HOLD:init("PARA_PWM_HOLD")
local PARA_PWM_RELEASE = Parameter()
PARA_PWM_RELEASE:init("PARA_PWM_RELEASE")
local PARA_TILTMAX = Parameter()
PARA_TILTMAX:init("PARA_TILT_MAX")
local PARA_ALTMIN = Parameter()
PARA_ALTMIN:init("PARA_ALT_MIN")
local PARA_HOLD_MS = Parameter()
PARA_HOLD_MS:init("PARA_HOLD_MS")
local PARA_SINK_MIN = Parameter()
PARA_SINK_MIN:init("PARA_SINK_MIN")

-- STATE VARIABLES
local last_pressed = false
local disarm_time = 0
local waiting_disarm_delay = false
local parachute_triggered = false

-- To control the auto deployment
local last_run = 0
local tilt_exceed_start = 0

local debug = false

-- MAIN LOOP
function update()
    local Channel_num = PARA_TRIG_CH:get()
    local rc_val = rc:get_pwm(Channel_num)
    if rc_val == nil then
        gcs:send_text(GCS_SEVERITY_INFO, "RC input not available.")
        return update, LOOP_INTERVAL_MS
    end

    local is_pressed = rc_val > RC_TRIGGER_THRESHOLD
    local delay_ms = PARA_DELAY_MS:get()
    local PWM_OFF = PARA_PWM_HOLD:get()


    -- Parachute auto-deployment logic
    -- Running every 0.4 seconds

    local current_time = millis()
    if (current_time - last_run > 400) then
        last_run = current_time

        -- Get the AHRS quaternion for tilt calculation
        local quat = ahrs:get_quaternion()
        if quat == nil then
            return update, LOOP_INTERVAL_MS
        end

        -- Get the altitude
        local altitude = ahrs:get_relative_position_D_home()
        if debug then gcs:send_text(GCS_SEVERITY_INFO, ">> Altitude: " .. altitude) end

        -- Get the vehicle mode
        local mode = vehicle:get_mode()

        -- Check if it is in a quadplane mode and the altitude is above minimum
        if (PARA_DEPLOY_MODES[mode] and (altitude < -PARA_ALTMIN:get())) then
            -- Get descent rate
            local vel = ahrs:get_velocity_NED()
            local sink_rate = vel and vel:z() or 0

            -- Calculate tilt from quaternion
            local q1 = quat:q1()
            local q2 = quat:q2()
            local q3 = quat:q3()
            local q4 = quat:q4()
            local cos_tilt = math.max(-1, math.min(1, 2*(q1*q3 - q2*q4)))
            local tilt = math.acos(cos_tilt) * 180/math.pi
            if debug then gcs:send_text(GCS_SEVERITY_INFO, ">> Tilt: " .. tilt) end

            if (tilt > PARA_TILTMAX:get()) and (sink_rate > PARA_SINK_MIN:get()) then
                -- Start timing if this is the first exceedance
                if tilt_exceed_start == 0 then
                    tilt_exceed_start = millis()
                elseif (millis() - tilt_exceed_start >= PARA_HOLD_MS:get()) then
                    arming:disarm()
                    disarm_time = millis()
                    waiting_disarm_delay = true
                    parachute_triggered = true
                    tilt_exceed_start = 0
                end
            else
                tilt_exceed_start = 0
            end
        end
    end

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
    local PWM_ON = PARA_PWM_RELEASE:get()
    if waiting_disarm_delay and parachute_triggered and
       (millis() - disarm_time >= delay_ms) then

        SRV_Channels:set_output_pwm(SERVO_FUNCTION_NUM, PWM_ON)
        gcs:send_text(GCS_SEVERITY_INFO, delay_ms .. " ms passed. Deploying Parachute.")
        gcs:send_text(GCS_SEVERITY_CRITICAL, "EMERGENCY: PARACHUTE DEPLOYED!")

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
return update()