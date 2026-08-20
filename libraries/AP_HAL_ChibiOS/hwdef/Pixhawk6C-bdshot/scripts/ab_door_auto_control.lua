--[[
  Auto door opening and closing Lua Script
  This script works only with Airbound 4.5.7.5 - hf5 or above
]]

-- ###############################################################################
-- # CONSTANTS AND GLOBALS
-- ###############################################################################
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local LOOP_INTERVAL_MS = 20
local CONFIG_UPDATE_INTERVAL_MS = 5000
local RC_SWITCH_THRESHOLD = 1500
local PWM_SAFE_MIN = 800
local PWM_SAFE_MAX = 2200
local backtransition_complete_time_ms = nil

-- Modes for landing
local QMODES = {
    [17] = true,  -- QSTABILIZE
    [18] = true,  -- QHOVER
    [19] = true,  -- QLOITER
    [20] = true,  -- QLAND
    [21] = true,  -- QRTL
    [22] = true,  -- QAUTOTUNE
    [23] = true,  -- QACRO
}

-- Mission commands for landing phase
local NAV_CMDS = {
    [17] = true, -- MAV_CMD_NAV_LOITER_UNLIM
    [19] = true, -- MAV_CMD_NAV_LOITER_TIME
    [31] = true, -- MAV_CMD_NAV_LOITER_TO_ALT
    [85] = true, -- MAV_CMD_NAV_VTOL_LAND
}

-- ###############################################################################
-- # PARAMETER DEFINITIONS
-- ###############################################################################
local PARAM_TABLE_KEY = 88
local PARAM_PREFIX = "DOOR_"
local NUM_PARAMS = 17
assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, NUM_PARAMS), "Failed to create param table")

local PARAM_IDX = {
    -- Basic control parameters (1-5)
    MAN_CMD_CH = 1, MAN_TIMEOUT = 2, ALT_TRIG_M = 3,
    
    -- Servo 1 (6-8)
    S1_FUNC = 4, S1_OPEN = 5, S1_CLOSE = 6,
    
    -- Servo 2 (9-11)  
    S2_FUNC = 7, S2_OPEN = 8, S2_CLOSE = 9,
    
    -- Servo 3 (12-14)
    S3_FUNC = 10, S3_OPEN = 11, S3_CLOSE = 12,
    
    -- Servo 4 (15-17)
    S4_FUNC = 13, S4_OPEN = 14, S4_CLOSE = 15,
    
    -- Slew rate parameter (18)
    SLEW_RATE = 16,

    -- OVERRIDE (19)
    OVERRIDE = 17
}

--[[ @Param: DOOR_MAN_CMD_CH, @DisplayName: Door Manual Command Chan, @Range: 1 16 --]]
assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX.MAN_CMD_CH, "MAN_CMD_CH", 9))
--[[ @Param: DOOR_MAN_TIMEOUT, @DisplayName: Manual Override Timeout, @Units: s, @Range: 0 300 --]]
assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX.MAN_TIMEOUT, "MAN_TIMEOUT", 5))
--[[ @Param: DOOR_ALT_TRIG_M, @DisplayName: Take-off Altitude Trigger, @Units: m, @Range: 1 100 --]]
assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX.ALT_TRIG_M, "ALT_TRIG_M", 3))

-- Servo function and PWM parameters
local open_pwms = {1000, 1900, 1000, 1900}
local closed_pwms = {1900, 1000, 1900, 1000}

for i=1,4 do
    assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX['S'..i..'_FUNC'], "S"..i.."_FUNC", 105+i))
    assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX['S'..i..'_OPEN'], "S"..i.."_OPEN", open_pwms[i]))
    assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX['S'..i..'_CLOSE'], "S"..i.."_CLOSE", closed_pwms[i]))
end

-- Slew rate parameter definition
--[[ @Param: DOOR_SLEW_RATE, @DisplayName: Door Servo Slew Rate, @Description: Speed of servo movement in PWM units per second. 0 disables slew and makes movement instant., @Units: PWM/s, @Range: 0 1000, @User: Standard --]]
assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX.SLEW_RATE, "SLEW_RATE", 1800))

-- Override parameter definition
--[[ @Param: DOOR_OVERRIDE, @DisplayName: Door Servo override, @Description: Enable/Disable for doors script auto logic, @Range: 0 1 --]]
assert(param:add_param(PARAM_TABLE_KEY, PARAM_IDX.OVERRIDE, "OVERRIDE", 0))



-- ###############################################################################
-- # SCRIPT INITIALIZATION & LOGIC
-- ###############################################################################
local state = {
    initialized = false,
    is_in_manual_override = false,
    manual_override_timer = 0,
    last_rc_pwm = 0,
    doors_are_commanded_closed = false,
    takeoff_climbout_complete = false,
    last_config_update = 0,
    was_armed = false,
    one_forward_transition_complete = false,
    -- State table for each servo's position
    servos = {}
}

local config = {
    man_cmd_ch = Parameter(PARAM_PREFIX .. "MAN_CMD_CH"),
    man_timeout = Parameter(PARAM_PREFIX .. "MAN_TIMEOUT"),
    alt_trig_m = Parameter(PARAM_PREFIX .. "ALT_TRIG_M"),
    slew_rate = Parameter(PARAM_PREFIX .. "SLEW_RATE"),
    man_override = Parameter(PARAM_PREFIX .. "OVERRIDE")
}

local servos_config = {}
for i = 1, 4 do
    servos_config[i] = {
        func = Parameter(PARAM_PREFIX .. "S" .. i .. "_FUNC"),
        open_pwm = Parameter(PARAM_PREFIX .. "S" .. i .. "_OPEN"),
        close_pwm = Parameter(PARAM_PREFIX .. "S" .. i .. "_CLOSE")
    }
end

local function clamp(value, min, max)
    return math.max(min, math.min(max, value))
end

local function log_message(severity, message)
    gcs:send_text(severity, "DoorCtrl: " .. message)
end

function update_config_cache()
    if (millis() - state.last_config_update) < CONFIG_UPDATE_INTERVAL_MS and state.initialized then
        return true
    end

    config.cache = {
        man_cmd_ch = config.man_cmd_ch:get(),
        man_timeout = config.man_timeout:get(),
        alt_trig_m = config.alt_trig_m:get(),
        slew_rate = config.slew_rate:get(),
        man_override = config.man_override:get()
    }

    if config.cache.man_cmd_ch < 1 or config.cache.man_cmd_ch > 16 then
        log_message(MAV_SEVERITY.ERROR, "Invalid manual channel")
        return false
    end

    config.cache.servos = {}
    for i = 1, 4 do
        local func_val = servos_config[i].func:get()
        local open_val = servos_config[i].open_pwm:get()
        local close_val = servos_config[i].close_pwm:get()

        config.cache.servos[i] = {
            func = func_val,
            open_pwm = open_val,
            close_pwm = close_val,
            enabled = (open_val > 0 and close_val > 0 and open_val ~= close_val)
        }

        if config.cache.servos[i].enabled and not SRV_Channels:find_channel(func_val) then
            log_message(MAV_SEVERITY.WARNING, "Servo "..i.." func "..func_val.." not found")
            config.cache.servos[i].enabled = false
        end
    end

    state.last_config_update = millis()
    return true
end

function set_door_state(state_name)
    if not config.cache then return false end

    local pwm_field = (state_name == "OPEN") and "open_pwm" or "close_pwm"

    local state_is_changing = (state.doors_are_commanded_closed == (state_name == "OPEN"))
    state.doors_are_commanded_closed = (state_name == "CLOSED")

    local doors_set = 0
    for i = 1, 4 do
        if config.cache.servos[i].enabled then
            state.servos[i].target_pwm = config.cache.servos[i][pwm_field]
            doors_set = doors_set + 1
        end
    end

    if doors_set > 0 and state_is_changing then
        log_message(MAV_SEVERITY.INFO, "Doors commanded: " .. state_name)
        return true
    elseif doors_set == 0 and state_is_changing then
        log_message(MAV_SEVERITY.WARNING, "Attempted to set doors, but none are configured.")
        return false
    end
    return doors_set > 0
end

function open_all_doors() return set_door_state("OPEN") end
function close_all_doors() return set_door_state("CLOSED") end

function update_servo_slew()
    if not config.cache or not state.initialized then return end

    if not config.cache.slew_rate or config.cache.slew_rate <= 0 then
        for i = 1, 4 do
            if config.cache.servos[i].enabled and state.servos[i].current_pwm ~= state.servos[i].target_pwm then
                local new_pwm = state.servos[i].target_pwm
                SRV_Channels:set_output_pwm(config.cache.servos[i].func, new_pwm)
                state.servos[i].current_pwm = new_pwm
            end
        end
        return
    end
    
    local max_pwm_change = config.cache.slew_rate * (LOOP_INTERVAL_MS / 1000.0)

    for i = 1, 4 do
        if config.cache.servos[i].enabled then
            local target = state.servos[i].target_pwm
            local current = state.servos[i].current_pwm

            if current ~= target then
                local new_pwm
                local diff = target - current
                if math.abs(diff) <= max_pwm_change then
                    new_pwm = target
                else
                    new_pwm = current + math.min(max_pwm_change, math.max(-max_pwm_change, diff))
                end

                new_pwm = clamp(new_pwm, PWM_SAFE_MIN, PWM_SAFE_MAX)
                SRV_Channels:set_output_pwm(config.cache.servos[i].func, new_pwm)
                state.servos[i].current_pwm = new_pwm
            end
        end
    end
end

function get_current_altitude_agl()
    local alt_down = ahrs:get_relative_position_D_home()
    if alt_down then
        return -alt_down
    end
    return nil
end

function in_vtol_flight()
    local vtol_active = not quadplane:tailsitter_in_vtol_transition() and quadplane:in_vtol_mode()
    if vtol_active then
        if backtransition_complete_time_ms == nil then
            backtransition_complete_time_ms = millis():tofloat()
        end
        return true
    end
    backtransition_complete_time_ms = nil
    return false
end

function run_auto_mode()
    if not arming:is_armed() then return end

    local current_mode = vehicle:get_mode()
    if not current_mode then return end

    local alt = get_current_altitude_agl()
    
    local is_vtol_flight = in_vtol_flight()

    -- Phase 1 - Check for takeoff
    if not state.takeoff_climbout_complete then
        if is_vtol_flight then
            if alt and alt >= config.cache.alt_trig_m then
                log_message(MAV_SEVERITY.INFO, "Takeoff alt " .. string.format("%.1f", alt) .. "m reached - closing doors")
                if close_all_doors() then
                    state.takeoff_climbout_complete = true
                end
            else
                -- Force doors open under alt_trig_m
                open_all_doors()
            end
        end
        return
    end

    -- Change Door state based on Flight Phase:FW, VTOL
    if is_vtol_flight then
        -- Only open doors in VTOL after first fixed wing transition is complete. Indicates VTOL takeoff is complete 
        if state.one_forward_transition_complete then
            open_all_doors()
        end
    else
        -- Non-VTOL Modes (FW, FBWA, MANUAL, etc.)
        -- Auto mode with mission commands other than Loiter/nav_vtol_land or other landing command ids
        -- VTOL Takoff complete is indicated by completion of first fixedwing transition
        if not state.one_forward_transition_complete then
            state.one_forward_transition_complete = true
        end
        close_all_doors()
    end -- landing phase checking

end

function check_manual_override()

    local rc_pwm = rc:get_pwm(config.cache.man_cmd_ch)
    if not rc_pwm then return false end

    if math.abs(rc_pwm - state.last_rc_pwm) > 100 then
        state.is_in_manual_override = true
        state.manual_override_timer = millis()
        log_message(MAV_SEVERITY.INFO, "Manual override active")
    end
    state.last_rc_pwm = rc_pwm

    if state.is_in_manual_override then
        if config.cache.man_override == 0 and config.cache.man_timeout > 0 and (millis() - state.manual_override_timer) > (config.cache.man_timeout * 1000) then
            log_message(MAV_SEVERITY.WARNING, "Manual override timeout")
            state.is_in_manual_override = false
            return false
        end
        if rc_pwm > RC_SWITCH_THRESHOLD then close_all_doors() else open_all_doors() end
        return true
    end
    return false
end

function initialize()
    if state.initialized then return true end
    if not update_config_cache() then return false end

    for i = 1, 4 do
        state.servos[i] = {
            current_pwm = config.cache.servos[i].open_pwm,
            target_pwm = config.cache.servos[i].open_pwm
        }
    end

    state.was_armed = arming:is_armed()
    local rc_pwm = rc:get_pwm(config.cache.man_cmd_ch)
    state.last_rc_pwm = rc_pwm or 0

    if not arming:is_armed() then
        log_message(MAV_SEVERITY.INFO, "Initializing - setting doors to OPEN")
        open_all_doors()
        update_servo_slew()
    end

    state.initialized = true
    log_message(MAV_SEVERITY.INFO, "Door controller initialized")
    return true
end

function update()
    if not state.initialized then
        if not initialize() then return update, 5000 end
    end

    if not update_config_cache() then return update, 5000 end

    local is_armed_now = arming:is_armed()
    if is_armed_now and not state.was_armed then
        log_message(MAV_SEVERITY.INFO, "Armed! Ready for takeoff.")
        state.takeoff_climbout_complete = false
        state.one_forward_transition_complete = false
    elseif not is_armed_now and state.was_armed then
        log_message(MAV_SEVERITY.INFO, "Disarmed. Resetting state and opening doors.")
        open_all_doors()
        state.is_in_manual_override = false
        state.takeoff_climbout_complete = false
        state.one_forward_transition_complete = false
    end
    state.was_armed = is_armed_now

    if not check_manual_override() then
        run_auto_mode()
    end
    
    update_servo_slew()

    return update, LOOP_INTERVAL_MS
end

log_message(MAV_SEVERITY.INFO, "Airbound Lua: Auto door controller loaded")
return update, 1000