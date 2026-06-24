-- Tailsitter Recovery Loop Script
-- Logic: Bad Pitch -> Save Mode -> QLoiter
local para_ahrs_pitch_threshold_max = -10
local para_ahrs_pitch_threshold_min = -50

-- 1. SETUP PARAMETER TABLE
local KEY = 110
assert(param:add_table(KEY, "AUTOB_", 14), "AUTOB table failed")

-- 2. ADD PARAMETERS
assert(param:add_param(KEY, 1, "PIT_LIM",  50),  'could not add AUTOB_PIT_LIM')   -- ATT pitch(VTOL frame) threshold to enter bailout (deg)
assert(param:add_param(KEY, 2, "ENABLE", 1),'could not add AUTOB_ENABLE')    -- 1 = Enabled
assert(param:add_param(KEY, 3, "BTRN_DLY", 2500), 'could not add AUTOB_BTRN_DLY') -- Delay (ms) before checking pitch
assert(param:add_param(KEY, 4, "PIT_TOUT", 100), 'could not add AUTOB_PIT_TOUT')  -- Sustained duration to enter bailout (ms)
assert(param:add_param(KEY, 5, "PARA_EN", 1),'could not add AUTOB_PARA_EN')    -- 1 = Enabled
assert(param:add_param(KEY, 6,"PARA_ANG", -15),'could not add AUTOB_PARA_ANG') -- AHRS Pitch(FW frame) threshold to trigger parachute(deg)
assert(param:add_param(KEY, 7,"PARA_TOUT", 100),'could not add AUTOB_PARA_TOUT') -- Parachute pitch threshold timeout in ms
assert(param:add_param(KEY, 8,  "LOOP_MS", 50), 'could not add AUTOB_LOOP_MS')  -- Loop rate (ms)
assert(param:add_param(KEY, 9,  "WIN_TIM",   5),   'could not add AUTOB_WIN_TIM')    -- Rolling window duration (s)
assert(param:add_param(KEY, 10, "WIN_SMP",   50),  'could not add AUTOB_WIN_SMP')    -- Calculated samples (output, read-only)
assert(param:add_param(KEY, 11,  "AVG_LIM", 20),'could not add AUTOB_AVG_LIM')    -- Pitch limit
assert(param:add_param(KEY, 12,"PEAK_LIM", 30),'could not add AUTOB_PEAK_LIM')
assert(param:add_param(KEY, 13, "DBG_EN", 1), 'could not add AUTOB_DBG_EN')  -- 1 = enable dataflash logging
assert(param:add_param(KEY, 14, "PRED_INT", 500), 'could not add AUTOB_PRED_INT')  -- Rate based VTOL pitch prediction interval

-- 3. BIND PARAMETERS
local function bind_param(name)
    local p = Parameter(name)
    if not p then
        gcs:send_text(2, "AUTOB: Reboot Required for " .. name)
    end
    return p
end

local p_enable = bind_param("AUTOB_ENABLE")
local p_avg_lim = bind_param("AUTOB_AVG_LIM")
local p_peak_lim = bind_param("AUTOB_PEAK_LIM")
local p_pit_lim = bind_param("AUTOB_PIT_LIM")
local p_btrn_dly = bind_param("AUTOB_BTRN_DLY")
local p_pitch_timeout = bind_param("AUTOB_PIT_TOUT")
local p_para_enable = bind_param("AUTOB_PARA_EN")
local p_para_ang = bind_param("AUTOB_PARA_ANG")
local p_para_timeout = bind_param("AUTOB_PARA_TOUT")
local p_loop_ms = bind_param("AUTOB_LOOP_MS")
local p_win_s   = bind_param("AUTOB_WIN_TIM")
local p_win_n   = bind_param("AUTOB_WIN_SMP")
local p_dbg_en = bind_param("AUTOB_DBG_EN")
local p_prediction_interval = bind_param("AUTOB_PRED_INT")

-- Read Parachute trigger channel number from FCU parameter list 


-- 4. MODE DEFINITIONS (ArduPlane)
local MODE_QLOITER = 19 -- Auto-Hover (Functionally same as QLoiter 50% Thr)
local AUTOBAILOUT_EXCLUDE_MODES = {
    [17] = true, --QSTABILIZE
    [18] = true,  -- QHOVER
    [19] = true,  -- QLOITER(cannot autobailout in QLOITER)
    [22] = true,  -- QAUTOTUNE
    [23] = true,  -- QACRO
}
-- 5. STATE VARIABLES
local autobailout_active = false
local last_mode_idx = 0
local pre_bailout_mode = nil
local first_pitch_exceeded_t = nil
local gcs_announce_autobailout = false

local WINDOW_SIZE     = 50
local pitch_error_buf = {}
local pitch_angle_buf = {}
local buf_idx         = 1
local post_bailout_sample_count = 0 --used to ensure sufficient samples have been collected to decide on resuming mode

local function buf_avg(buf)
    local sum, count = 0, 0
    for _, v in pairs(buf) do sum = sum + v; count = count + 1 end
    if count == 0 then return 0 end
    return sum / count
end

local function buf_max(buf)
    local mx = -math.huge
    for _, v in pairs(buf) do if v > mx then mx = v end end
    if mx == -math.huge then return 0 end
    return mx
end

-- Parachute state variables
local trigger_para_script = false
local first_para_pitch_exceeded_t = nil
local PARA_CHAN_HIGH = 1850
local last_para_warn_t = 0 
local backtransition_complete_time_ms = nil

-- Helper: Radians to Degrees
local function rad2deg(r) return r * 57.2958 end

function in_vtol_flight()
    local mode = vehicle:get_mode()
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

function is_parachute_angle_threshold_valid(threshold_angle)
-- check if the threshold angle set by user is valid
    if threshold_angle < para_ahrs_pitch_threshold_max and threshold_angle > para_ahrs_pitch_threshold_min then
        return true
    end
    return false
end

function para_deploy()
    
    if p_para_enable:get() ~= 1 then return end

    para_trigger_rc_chan = nil
    PARA_TRIG_CHAN = Parameter()
    PARA_TRIG_CHAN:init('PARA_TRIG_CH')
    if PARA_TRIG_CHAN then
        channel_num = PARA_TRIG_CHAN:get()
        -- gcs:send_text(2, "Channel num" .. tostring(channel_num))
        if channel_num == nil then
            local t = millis():tofloat()
            if (t - last_para_warn_t) > 3000 then
                gcs:send_text(2, "AUTOB: PARA_TRIG_CH not set")
                last_para_warn_t = t
            end
            return
        end  

        para_trigger_rc_chan = rc:get_channel(channel_num)
    end

    if para_trigger_rc_chan == nil then
        local t = millis():tofloat()
        if (t - last_para_warn_t) > 3000 then
            gcs:send_text(2, "AUTOB:Para rc channel is nil")
            last_para_warn_t = t
        end
        return
    end
    
    local para_threshold = p_para_ang:get() or -45
    if not is_parachute_angle_threshold_valid(para_threshold) then
        gcs:send_text(2, string.format("AUTOB:AUTB_PARA_ANG invalid. Range(%.1f, %.1f)",para_ahrs_pitch_threshold_min, para_ahrs_pitch_threshold_max))    
        return
    end

    local now = millis():tofloat()
    ahrs_pitch = rad2deg(ahrs:get_pitch() or 0)
    para_ang_timeout = p_para_timeout:get() or 200
    check_pitch = ahrs_pitch < para_threshold and (trigger_para_script == false) and quadplane:in_vtol_mode() and arming:is_armed()
    if check_pitch then
        if first_para_pitch_exceeded_t == nil then
            first_para_pitch_exceeded_t = millis():tofloat()
        else
            para_time_diff = now - first_para_pitch_exceeded_t
            if para_time_diff > para_ang_timeout then
                first_para_pitch_exceeded_t = nil
                trigger_para_script = true
                gcs:send_text(2, "AUTOB:trigger parachute via rc override")
            end
        end
    else
        first_para_pitch_exceeded_t = nil
    end
    
    if trigger_para_script then
        para_trigger_rc_chan:set_override(PARA_CHAN_HIGH)
    end
end

function trigger_autobailout(current_mode)
    if autobailout_active then
        gcs:send_text(2, "AUTOB: Autobailout already active")
        gcs:send_text(2, "AUTOB: Trigger rejected")
        return false
    end
    if vehicle:set_mode(MODE_QLOITER) then
        autobailout_active = true
        pre_bailout_mode = current_mode
        post_bailout_sample_count = 0 --Reset this variable only when autobailout is active
        gcs:send_text(2, "AUTOB: Switching to QLoiter" )
        return true
    end
    return false
end

function is_vtol_pitch_exceeding_limit(vtolpitch, is_vtol_flight, flightmode)
    local threshold = p_pit_lim:get() or 40
    local pitch_timeout = p_pitch_timeout:get() or 100

    --Dont check for pitch in AUTOBAILOUT_EXCLUDE_MODES
    if AUTOBAILOUT_EXCLUDE_MODES[flightmode] then
        return false
    end
    
    --dont check if not armed or not in vtol phase
    if not is_vtol_flight or not arming:is_armed() then
        -- Wait for Delay (settle time)
        return false
    end
    
    --dont check if within delay_ms post backtransition
    local current_time = millis():tofloat()
    local delay_ms = p_btrn_dly:get() or 1000
    if backtransition_complete_time_ms and (current_time - backtransition_complete_time_ms) < delay_ms then
        return false
    end

    if math.abs(vtolpitch) > threshold then
        if first_pitch_exceeded_t == nil then
            first_pitch_exceeded_t = millis():tofloat()
        else
            local time_diff = current_time - first_pitch_exceeded_t
            if time_diff > pitch_timeout then
                first_pitch_exceeded_t = nil  
                gcs:send_text(2, "AUTOB: VTOL pitch exceeding threshold: " .. tostring(vtolpitch))
                return true     
            end
        end
    else
        first_pitch_exceeded_t = nil
    end
    return false
end    

function is_predicted_vtol_pitch_exceeding_parathreshold(current_vtol_pitch_deg, current_vtol_pitch_rate, is_vtol_flight, flightmode)
    local pitch_prediction_interval = p_prediction_interval:get() or 0

    --Disable prediction based check
    if pitch_prediction_interval < 0 then
        return false
    end

    --Dont check for pitch in AUTOBAILOUT_EXCLUDE_MODES
    if AUTOBAILOUT_EXCLUDE_MODES[flightmode] then
        return false
    end

    --convert to VTOL frame
    local para_threshold = 90 - (p_para_ang:get() or -45) 

    if not is_vtol_flight or not arming:is_armed() then
        -- Wait for Delay (settle time)
        return false
    end
    
    --dont check if within delay_ms post backtransition
    local current_time = millis():tofloat()
    local delay_ms = p_btrn_dly:get() or 1000
    if backtransition_complete_time_ms and (current_time - backtransition_complete_time_ms) < delay_ms then
        return false
    end
    
    local predicted_next_instance_vtol_pitch = current_vtol_pitch_deg + current_vtol_pitch_rate * (pitch_prediction_interval/1000)

    if math.abs(predicted_next_instance_vtol_pitch) > para_threshold then
        gcs:send_text(2, "AUTOB: PredPitch exceeds threshold: " .. tostring(predicted_next_instance_vtol_pitch))
        return true
    end
    return false
end

function update()
    local loop_ms = p_loop_ms:get() or 100
    para_deploy()
    if trigger_para_script then return  update, loop_ms end

    -- Safety Check
    if p_enable:get() ~= 1 then return update, loop_ms end

    local current_mode = vehicle:get_mode()
    if not current_mode then return update, loop_ms end

    -- Detect Mode Changes
    if current_mode ~= last_mode_idx then
        last_mode_idx = current_mode
        first_pitch_exceeded_t = nil
        if not autobailout_active then        -- ADD THIS GUARD
            pitch_error_buf = {}
            pitch_angle_buf = {}
            buf_idx = 1
        end
    end

    local win_n   = math.max(1, math.floor((p_win_s:get() or 5) * 1000 / loop_ms))
    if win_n ~= WINDOW_SIZE then
        WINDOW_SIZE = win_n
        if not autobailout_active then
            pitch_error_buf = {}
            pitch_angle_buf = {}
            buf_idx = 1
        end
        p_win_n:set(WINDOW_SIZE)
    end

    local desired          = qp_att_desired()
    local actual           = qp_att_actual()
    local target_vtol_pitch_deg = desired and (desired.pitch_cd * 0.01) or 0
    local actual_vtol_pitch_deg = actual  and (actual.pitch_cd  * 0.01) or 0
    local vtol_pitch_rate_pid = qp_rate_pid_info(1)
    local actual_vtol_pitch_rate =  vtol_pitch_rate_pid and rad2deg(vtol_pitch_rate_pid.actual or 0) or 0

    pitch_error_buf[buf_idx] = math.abs(target_vtol_pitch_deg - actual_vtol_pitch_deg)
    pitch_angle_buf[buf_idx] = math.abs(actual_vtol_pitch_deg)
    buf_idx = (buf_idx % WINDOW_SIZE) + 1

    local avg_err  = buf_avg(pitch_error_buf)
    local peak_ang = buf_max(pitch_angle_buf)
    local pitch_deg = rad2deg(ahrs:get_pitch() or 0)

    if p_dbg_en:get() == 1 then
        logger:write('AUTB', 'AvgErr,PeakAng,PitchDeg,QPit,QRateP', 'fffff', avg_err, peak_ang, pitch_deg, actual_vtol_pitch_deg, actual_vtol_pitch_rate)
    end
    
    is_vtol_flight = in_vtol_flight()
    -- ==========================================================
    -- LOGIC: MONITORING (Checking Pitch)
    -- ==========================================================
    if not autobailout_active then
        if is_vtol_pitch_exceeding_limit(actual_vtol_pitch_deg, is_vtol_flight, current_mode) then
            trigger_autobailout(current_mode)
        elseif is_predicted_vtol_pitch_exceeding_parathreshold(actual_vtol_pitch_deg, actual_vtol_pitch_rate, is_vtol_flight,current_mode) then
            trigger_autobailout(current_mode)
        end
    -- ==========================================================
    -- LOGIC: RECOVERY
    -- ==========================================================
    elseif autobailout_active then
        if not gcs_announce_autobailout then
            gcs:send_text(2, "AUTOB: Autobailout Active")
            gcs_announce_autobailout = true
        end
        if current_mode ~= MODE_QLOITER then
            autobailout_active = false
            pre_bailout_mode = nil
            gcs_announce_autobailout = false
            gcs:send_text(6, "AUTOB: Manual Override Detected")
        else
            post_bailout_sample_count = post_bailout_sample_count + 1
            post_bailout_sample_count = math.min(post_bailout_sample_count, WINDOW_SIZE)
            local avg_lim  = p_avg_lim:get()  or 20
            local peak_lim = p_peak_lim:get() or 30
            if post_bailout_sample_count >= WINDOW_SIZE and avg_err < avg_lim and peak_ang < peak_lim then
                if pre_bailout_mode and vehicle:set_mode(pre_bailout_mode) then
                    local recovered_mode = pre_bailout_mode
                    autobailout_active = false
                    pre_bailout_mode = nil
                    gcs_announce_autobailout = false
                    gcs:send_text(2, "AUTOB: Recovering to mode " .. tostring(recovered_mode))
                end
            end
        end

    end
    return update, loop_ms

end

gcs:send_text(6, "Airbound Lua: Autobailout script loaded")
return update, 1000