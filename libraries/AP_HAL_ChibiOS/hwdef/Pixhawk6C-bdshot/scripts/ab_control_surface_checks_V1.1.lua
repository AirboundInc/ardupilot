-- Lua script for control surface + tilt motor check using RC overrides
-- Triggered via RC switch (e.g., RC7_OPTION=300)
-- ⚠️ Changes flight mode to MANUAL for the test, then returns to QLOITER.
-- 🟢 Repeatable on each RC switch LOW->HIGH transition (a switch already high at boot is ignored)
-- Starting a rerun clears any previous pass; arming during the sequence aborts it
-- RC_CHAN_MON is driven Low then High; arming stays blocked unless it reads <= MON_LOW_US and >= MON_HIGH_US
-- CTSF_REQ selects when the check is required:
--   0 = after every reboot (hard or soft)
--   1 = after every reboot and before every take-off (re-required on disarm after a flight)

-- USER CONFIGURATION
local RC_CHAN_ROLL           = 1    -- Your aircraft's Roll input channel
local RC_CHAN_PITCH          = 2    -- Your aircraft's Pitch input channel
local RC_CHAN_FLAP           = 9    -- Your aircraft's Flap input channel
local RC_CHAN_MON            = 11   -- Channel driven low/high and range-checked during the sequence

local RC_NEUTRAL_US          = 1500 -- Neutral PWM for RC channels
local RC_DEFLECTION          = 500  -- PWM deflection from neutral to simulate full stick
local STEP_DELAY_MS          = 3000 -- Delay between each movement in milliseconds
local MODE_CHANGE_TIMEOUT_MS = 5000 -- Max time to wait for a mode change
local REMINDER_INTERVAL_MS   = 15000 -- 15 seconds for the nag message
local MON_LOW_US             = 1100 -- Monitored channel must read at or below this to pass
local MON_HIGH_US            = 1900 -- Monitored channel must read at or above this to pass
local RC_VALID_MIN_US        = 900  -- Trigger readings below this are treated as no RC input
local PARA_LOW_TIMEOUT_MS    = 10000 -- Max time to wait for the real Para switch to read LOW before leaving MANUAL

-- CTSF_REQ values
local REQ_EVERY_REBOOT       = 0 -- Check required once per boot
local REQ_EVERY_TAKEOFF      = 1 -- Check also required again after each flight

-- ArduPilot flight modes
local MODE_MANUAL            = 0 -- Manual mode (QuadPlane)
local MODE_QLOITER           = 19 -- QLoiter mode

-- PARAMETER SETUP
local PARAM_TABLE_KEY        = 73
assert(param:add_table(PARAM_TABLE_KEY, "CTSF_", 4), "Failed to create param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "TRIG_CH", 10), "Failed to create Param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "REQ", REQ_EVERY_REBOOT), "Failed to create Param")

local CTSF_TRIG_CH = Parameter()
CTSF_TRIG_CH:init("CTSF_TRIG_CH")
local CTSF_REQ = Parameter()
CTSF_REQ:init("CTSF_REQ")

-- Script state
local running                = false -- True if the test sequence is active
local step                   = 0 -- Current step in the sequence
local last_step_time         = 0 -- Timestamp of the last action
local prev_switch_state      = true -- Previous state of the trigger switch; assume high until seen low so a switch left high never triggers
local checks_completed       = false -- Tracks if the sequence has finished successfully
local last_reminder_time     = millis() -- Timestamp for the reminder timer
local was_armed              = false -- Armed state on the previous loop
local flew_this_arm          = false -- True if the vehicle flew at any point while armed
local mon_active             = false -- True while the monitored channel is being sampled
local mon_min                = nil -- Lowest PWM seen on the monitored channel
local mon_max                = nil -- Highest PWM seen on the monitored channel
local mon_failed             = false -- True if the last sequence failed the monitored channel check
local last_para_prompt       = 0 -- Timestamp of the last "move Para switch LOW" prompt

-- Request an authorization ID for custom pre-arm checks
local auth_id = arming:get_aux_auth_id()
if not auth_id then
    gcs:send_text(4, "ERROR: Could not get arming aux auth ID")
end

-- Get RC channel objects for Pitch, Roll, and Flaps
local rc_pitch_chan = rc:get_channel(RC_CHAN_PITCH)
local rc_roll_chan = rc:get_channel(RC_CHAN_ROLL)
local rc_flap_chan = rc:get_channel(RC_CHAN_FLAP)
local rc_mon_chan = rc:get_channel(RC_CHAN_MON)

if not rc_pitch_chan or not rc_roll_chan or not rc_flap_chan or not rc_mon_chan then
    gcs:send_text(4, "ERROR: RC Roll/Pitch/flap/Para channels not found.")
    return -- Stop the script from running
end

-- Helper to change flight mode
local function change_mode(mode_id, mode_name)
    if vehicle:set_mode(mode_id) then
        gcs:send_text(6, "Attempting to switch to " .. mode_name .. " mode...")
        return true
    else
        gcs:send_text(4, "ERROR: Command to switch to " .. mode_name .. " failed.")
        return false
    end
end

-- Helper to release RC overrides back to transmitter control
local function release_overrides()
    rc_pitch_chan:set_override(0)
    rc_roll_chan:set_override(0)
    rc_flap_chan:set_override(0)
    rc_mon_chan:set_override(0)
end

-- Helper to stop the sequence and restore everything it changed
-- keep_mode = true stays in the current mode instead of switching MANUAL -> QLOITER
local function stop_sequence(keep_mode)
    -- Release first so CH11 is back at the TX value before leaving MANUAL (para deploy is held only in MANUAL)
    release_overrides()
    if step >= 1 then -- MANUAL/VFGAIN were only changed once step 0 succeeded
        param:set('Q_TAILSIT_VFGAIN', 0.2)
        gcs:send_text(6, "Set Q_TAILSIT_VFGAIN to 0.2 (VTOL mode)")
        if not keep_mode and vehicle:get_mode() == MODE_MANUAL then
            change_mode(MODE_QLOITER, "QLOITER")
        end
    end
    mon_active = false
    running = false
end

-- Helper to record the min/max PWM of the monitored channel
local function sample_monitor()
    local pwm = rc:get_pwm(RC_CHAN_MON)
    if pwm then
        if not mon_min or pwm < mon_min then mon_min = pwm end
        if not mon_max or pwm > mon_max then mon_max = pwm end
    end
end

-- Helper to evaluate the monitored channel range; returns true on pass
local function monitor_passed()
    if not mon_min then
        gcs:send_text(2, "Para check FAILED: no RC input seen")
        return false
    end
    local ok = mon_min <= MON_LOW_US and mon_max >= MON_HIGH_US
    if ok then
        gcs:send_text(6, string.format("Para range %d-%d: PASS", mon_min, mon_max))
    else
        gcs:send_text(2, string.format("Para check FAILED: range %d-%d (need <=%d and >=%d)",
                      mon_min, mon_max, MON_LOW_US, MON_HIGH_US))
    end
    return ok
end

-- Main update loop, runs continuously
function update()
    local now = millis()

    -- 1. Pre-Arm Check Injection
    if auth_id then
        if mon_failed then
            arming:set_aux_auth_failed(auth_id, "Para check failed, rerun checks")
        elseif not checks_completed then
            arming:set_aux_auth_failed(auth_id, "Control surface checks required")
        else
            arming:set_aux_auth_passed(auth_id)
        end
    end

    -- 2. Track flights; in take-off mode, require the check again after landing
    local armed = arming:is_armed()
    if armed and vehicle:get_likely_flying() then
        flew_this_arm = true
    end
    if was_armed and not armed then
        if CTSF_REQ:get() == REQ_EVERY_TAKEOFF and flew_this_arm then
            checks_completed = false
            last_reminder_time = now
            gcs:send_text(4, "Landed: run control surface checks before next takeoff")
        end
        flew_this_arm = false
    end
    was_armed = armed

    -- Prevent running the check (or nagging) while armed
    if armed then
        if running then
            gcs:send_text(4, "Armed during control surface check. Aborting.")
            stop_sequence()
        end
        return update, 100
    end

    -- Nag Reminder Logic (Runs every REMINDER_INTERVAL_MS if checks aren't done)
    if not checks_completed and not running and (now - last_reminder_time >= REMINDER_INTERVAL_MS) then
        gcs:send_text(4, "Please run control surface checks before flight")
        last_reminder_time = now
    end

    -- 3. Check for the trigger switch activation (rising edge)
    local trig_chan = CTSF_TRIG_CH:get()
    local rc_pwm = rc:get_pwm(trig_chan)
    local trigger_activated = false
    if rc_pwm and rc_pwm >= RC_VALID_MIN_US then
        local switch_is_on = rc_pwm > 1800
        trigger_activated = switch_is_on and not prev_switch_state
        prev_switch_state = switch_is_on
    end

    -- Start the sequence if triggered and not already running
    if trigger_activated and not running then
        running = true
        step = 0
        last_step_time = now
        mon_active, mon_min, mon_max = false, nil, nil
        checks_completed = false -- A rerun clears any previous pass so arming stays blocked until it finishes
        mon_failed = false
        gcs:send_text(1, "Starting control surface check")
    end

    -- 4. Execute the test sequence if it's running
    if not running then
        return update, 100 -- If not running, check again in 100ms
    end

    -- Sample the monitored channel every loop while its steps are active
    if mon_active then
        sample_monitor()
    end

    -- State Machine
    if step == 0 then
        -- Attempt to change to MANUAL mode
        if change_mode(MODE_MANUAL, "MANUAL") then
            step = 1 -- Move to next step to wait for confirmation
            last_step_time = now
            param:set('Q_TAILSIT_VFGAIN', 1)
            gcs:send_text(6, "Set Q_TAILSIT_VFGAIN to 1 (VTOL mode)")
        else
            running = false -- Abort if command fails
        end
    elseif step == 1 then
        -- Wait for MANUAL mode to be active
        if vehicle:get_mode() == MODE_MANUAL then
            gcs:send_text(1, "Mode Changed to Manual")
            step = 2 -- Mode confirmed, start movements
            last_step_time = now
        elseif (now - last_step_time > MODE_CHANGE_TIMEOUT_MS) then
            gcs:send_text(4, "TIMEOUT: Failed to enter MANUAL mode. Aborting.")
            stop_sequence()
        end
    elseif step == 12 then
        -- Wait for the real Para switch to read LOW before leaving MANUAL
        local pwm = rc:get_pwm(RC_CHAN_MON)
        if pwm and pwm >= RC_VALID_MIN_US and pwm <= MON_LOW_US then
            gcs:send_text(6, string.format("Para switch LOW (%d)", pwm))
            gcs:send_text(1, "All checks complete. Releasing controls.")
            stop_sequence() -- Restores VFGAIN/QLOITER and releases overrides back to TX

            -- Only clear the pre-arm block & reminder if the monitored channel passed
            mon_failed = not monitor_passed()
            checks_completed = not mon_failed
        elseif now - last_step_time > PARA_LOW_TIMEOUT_MS then
            gcs:send_text(2, "Para switch not LOW: check FAILED, staying in MANUAL")
            stop_sequence(true) -- Stay in MANUAL so the held parachute press is not released
            mon_failed = true
            checks_completed = false
        elseif now - last_para_prompt >= STEP_DELAY_MS then
            gcs:send_text(4, "Para switch HIGH - move to LOW")
            last_para_prompt = now
        end
    elseif step >= 2 then
        -- Execute timed movements
        if (now - last_step_time < STEP_DELAY_MS) then
            return update, 50 -- Not yet time for the next movement
        end
        last_step_time = now  -- Reset timer for the next step

        if step == 2 then
            -- Pitch Up
            gcs:send_text(1, "Pitch Up")
            rc_pitch_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            step = step + 1
        elseif step == 3 then
            -- Pitch Down
            gcs:send_text(1, "Pitch Down")
            rc_pitch_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 4 then
            -- Pitch Neutral
            gcs:send_text(1, "Pitch Neutral")
            rc_pitch_chan:set_override(RC_NEUTRAL_US)
            step = step + 1
        elseif step == 5 then
            -- Roll Right
            gcs:send_text(1, "Roll Right")
            rc_roll_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 6 then
            -- Roll Left
            gcs:send_text(1, "Roll Left")
            rc_roll_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            step = step + 1
        elseif step == 7 then
            -- Door Closed
            gcs:send_text(1, "Cargo bay door closed")
            -- Also neutralising roll from the previous step
            rc_roll_chan:set_override(RC_NEUTRAL_US)
            rc_flap_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 8 then
            -- Door Open
            gcs:send_text(1, "Cargo bay door open")
            rc_flap_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            step = step + 1
        elseif step == 9 then
            -- Monitored channel Low (start sampling)
            gcs:send_text(1, "Para Low")
            rc_mon_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            mon_active = true
            step = step + 1
        elseif step == 10 then
            -- Monitored channel High
            gcs:send_text(1, "Para High")
            rc_mon_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 11 then
            -- Hand the Para channel back to the TX; step 12 waits for the real switch to read LOW
            rc_mon_chan:set_override(0)
            mon_active = false
            gcs:send_text(4, "Para: confirm switch LOW")
            last_para_prompt = now
            step = 12
        end
    end

    return update, 50 -- Schedule the next run
end

-- Confirm script loaded
gcs:send_text(6, "Airbound Lua: Control Surface Check Script Loaded")

-- Initial call to start the scripting
return update()