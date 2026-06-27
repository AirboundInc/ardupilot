-- ab_fixed_wing_warnings.lua
-- ===========================================================================
-- Combined monitoring & WARNING system for fixed-wing flight.
--
-- Bundles four independent monitors that run at their own check rates:
--   1. Altitude deviation
--   2. Attitude deviation
--   3. Cross-track error
--   4. Climb / sink rate
--
-- Logs a consolidated message (FWM) containing all Trigger and Warn states.
-- ===========================================================================

local MAV_SEVERITY_WARN = 4         -- MAV_SEVERITY_WARNING
local MAV_SEVERITY_INFO = 6         -- MAV_SEVERITY_INFO (boot/status)
local RAD_TO_DEG        = 180.0 / math.pi

-- Monitor bit assignments (used in FWM_ENABLE only).
local BIT_ALT  = 1     -- Altitude
local BIT_ATT  = 2     -- Attitude
local BIT_CTE  = 4     -- Cross-track
local BIT_SINK = 8     -- Sink rate


-- ===========================================================================
-- PARAMETER TABLE SETUP
-- ===========================================================================

local PARAM_TABLE_KEY    = 74              -- must be unique vs other scripts
local PARAM_TABLE_PREFIX = "FWM_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16),
       "FW Monitor: could not add param table")

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
          string.format("FW Monitor: could not add param %s", name))
   local p = Parameter()
   assert(p:init(PARAM_TABLE_PREFIX .. name),
          string.format("FW Monitor: could not init param %s", name))
   return p
end

-- ---------------------------------------------------------------------------
-- GLOBAL PARAMETERS
-- ---------------------------------------------------------------------------
local FWM_ENABLE = bind_add_param("ENABLE", 1, 15)

-- Replaced LOG_MASK with a simple 0/1 enable toggle
local FWM_LOG_EN = bind_add_param("LOG_EN", 2, 1) 

local FWM_WARN_INTVL = bind_add_param("WARN_INTVL", 3, 2000)

-- ALTITUDE
local FWM_ALT_DEV_M = bind_add_param("ALT_DEV_M", 4, 10.0)
local FWM_ALT_RATE = bind_add_param("ALT_RATE", 5, 500)

-- ATTITUDE
local FWM_ATT_DEV = bind_add_param("ATT_DEV", 6, 30.0)
local FWM_ATT_RATE = bind_add_param("ATT_RATE", 7, 100)
local FWM_R_PERS = bind_add_param("R_PERS", 8, 350)
local FWM_P_PERS = bind_add_param("P_PERS", 9, 100)
local FWM_Y_PERS = bind_add_param("Y_PERS", 10, 6500)

-- CROSS-TRACK
local FWM_CTE_MULT = bind_add_param("CTE_MULT", 11, 2.0)
local FWM_CTE_RATE = bind_add_param("CTE_RATE", 12, 200)
local FWM_CTE_PERS = bind_add_param("CTE_PERS", 13, 9500)

-- SINK RATE
local FWM_SNK_LIM = bind_add_param("SNK_LIM", 14, 3.0)
local FWM_SNK_RATE = bind_add_param("SNK_RATE", 15, 200)
local FWM_SNK_PERS = bind_add_param("SNK_PERS", 16, 3000)


-- ===========================================================================
-- BITMASK HELPERS & GLOBAL LOG STATE
-- ===========================================================================

local function bit_is_set(mask_value, bit)
   return math.floor(mask_value / bit) % 2 == 1
end

local function warn_enabled(bit) return bit_is_set(FWM_ENABLE:get()   or 0, bit) end

-- Stores the most recent 0/1 states from the monitors
local fwm_log_state = {
    alt_tg = 0, alt_w = 0,
    r_tg = 0,   r_w = 0,
    p_tg = 0,   p_w = 0,
    y_tg = 0,   y_w = 0,
    cte_tg = 0, cte_w = 0,
    sr_tg = 0,  sr_w = 0
}


-- ===========================================================================
-- 1. ALTITUDE DEVIATION MONITOR
-- ===========================================================================

local alt_state = { last_warn_ms = 0 }

local function alt_reset_state()
   alt_state.last_warn_ms = 0
   fwm_log_state.alt_tg = 0
   fwm_log_state.alt_w = 0
end

local function altitude_monitor()
   if not arming:is_armed() then return end

   local target_alt  = vehicle:get_target_alt_rel_home_m()
   local current_alt = vehicle:get_current_alt_rel_home_m()

   local threshold = FWM_ALT_DEV_M:get() or 10.0
   local fw  = (target_alt ~= nil and current_alt ~= nil) and 1 or 0
   local tgt = target_alt  or 0.0
   local cur = current_alt or 0.0
   local dev = (fw == 1) and (cur - tgt) or 0.0

   local trig = (fw == 1 and math.abs(dev) > threshold) and 1 or 0
   local warn = 0

   if trig == 1 then
      local now_ms = millis():toint()
      local interval = FWM_WARN_INTVL:get() or 2000
      if now_ms - alt_state.last_warn_ms >= interval then
         local dir = dev > 0 and "ABOVE" or "BELOW"
         gcs:send_text(MAV_SEVERITY_WARN, string.format(
            "FW WARNING: %.1fm %s setpoint", math.abs(dev), dir))
         alt_state.last_warn_ms = now_ms
         warn = 1
      end
   end

   fwm_log_state.alt_tg = trig
   fwm_log_state.alt_w  = warn
end


-- ===========================================================================
-- 2. ATTITUDE DEVIATION MONITOR
-- ===========================================================================

local att_state = {
   roll  = { dev_start_ms = 0, last_warn_ms = 0 },
   pitch = { dev_start_ms = 0, last_warn_ms = 0 },
   yaw   = { dev_start_ms = 0, last_warn_ms = 0 },
}

local function att_reset_state()
   att_state.roll.dev_start_ms  = 0
   att_state.pitch.dev_start_ms = 0
   att_state.yaw.dev_start_ms   = 0
   fwm_log_state.r_tg, fwm_log_state.r_w = 0, 0
   fwm_log_state.p_tg, fwm_log_state.p_w = 0, 0
   fwm_log_state.y_tg, fwm_log_state.y_w = 0, 0
end

local function wrap_180(d)
   while d >  180.0 do d = d - 360.0 end
   while d < -180.0 do d = d + 360.0 end
   return d
end

local function check_attitude_axis(name, current_deg, target_deg, label,
                                   threshold, persist_ms, warn_interval_ms)
   local s = att_state[name]
   local dev = wrap_180(current_deg - target_deg)
   local now_ms = millis():toint()

   if math.abs(dev) <= threshold then
      s.dev_start_ms = 0
      return 0, 0
   end

   if s.dev_start_ms == 0 then
      s.dev_start_ms = now_ms
      return 1, 0
   end

   if now_ms - s.dev_start_ms < persist_ms then
      return 1, 0
   end

   if now_ms - s.last_warn_ms < warn_interval_ms then
      return 1, 0
   end

   gcs:send_text(MAV_SEVERITY_WARN, string.format(
      "FW WARNING: %s Deviation %.0fdeg",
      label, math.abs(dev), (now_ms - s.dev_start_ms) * 0.001))
   s.last_warn_ms = now_ms
   return 1, 1
end

local function attitude_monitor()
   if not arming:is_armed() then return end

   local threshold        = FWM_ATT_DEV:get()   or 30.0
   local warn_interval_ms = FWM_WARN_INTVL:get() or 2000
   local r_persist        = FWM_R_PERS:get()    or 100
   local p_persist        = FWM_P_PERS:get()    or 100
   local y_persist        = FWM_Y_PERS:get()    or 5000

   if not vehicle:is_fixed_wing_flight() then
      att_reset_state()
      return
   end

   local tgt_roll  = vehicle:get_target_roll_deg()
   local tgt_pitch = vehicle:get_target_pitch_deg()
   if tgt_roll == nil or tgt_pitch == nil then return end

    local cur_roll  = ahrs:get_roll()  * RAD_TO_DEG
    local cur_pitch = ahrs:get_pitch() * RAD_TO_DEG
    local cur_yaw   = ahrs:get_yaw()   * RAD_TO_DEG
   if cur_yaw < 0 then cur_yaw = cur_yaw + 360 end

   local r_trig, r_warn = check_attitude_axis("roll",  cur_roll,  tgt_roll,
                                              "ROLL",  threshold, r_persist, warn_interval_ms)
   local p_trig, p_warn = check_attitude_axis("pitch", cur_pitch, tgt_pitch,
                                              "PITCH", threshold, p_persist, warn_interval_ms)

   local y_trig, y_warn = 0, 0
   local tgt_bearing = vehicle:get_wp_bearing_deg()
   if tgt_bearing ~= nil then
      y_trig, y_warn = check_attitude_axis("yaw", cur_yaw, tgt_bearing,
                                           "YAW", threshold, y_persist, warn_interval_ms)
   else
      att_state.yaw.dev_start_ms = 0
   end

   fwm_log_state.r_tg = r_trig
   fwm_log_state.r_w  = r_warn
   fwm_log_state.p_tg = p_trig
   fwm_log_state.p_w  = p_warn
   fwm_log_state.y_tg = y_trig
   fwm_log_state.y_w  = y_warn
end


-- ===========================================================================
-- 3. CROSS-TRACK ERROR MONITOR
-- ===========================================================================

local WP_RADIUS = Parameter("WP_RADIUS")
local LAT_TO_M = 0.011131884502145034

local cte_prev_lat, cte_prev_lng             = nil, nil
local cte_cur_target_lat, cte_cur_target_lng = nil, nil

local cte_state = { dev_start_ms = 0, last_warn_ms = 0 }

local function cte_reset_state()
   cte_prev_lat, cte_prev_lng = nil, nil
   cte_cur_target_lat, cte_cur_target_lng = nil, nil
   cte_state.dev_start_ms = 0
   fwm_log_state.cte_tg = 0
   fwm_log_state.cte_w  = 0
end

local function cte_meters(a_lat, a_lng, b_lat, b_lng, p_lat, p_lng)
   local cos_lat = math.cos(math.rad(a_lat * 1e-7))
   local dn_AB = (b_lat - a_lat) * LAT_TO_M
   local de_AB = (b_lng - a_lng) * LAT_TO_M * cos_lat
   local dn_AP = (p_lat - a_lat) * LAT_TO_M
   local de_AP = (p_lng - a_lng) * LAT_TO_M * cos_lat

   local ab_len_sq = dn_AB*dn_AB + de_AB*de_AB
   if ab_len_sq < 0.01 then
      return math.sqrt(dn_AP*dn_AP + de_AP*de_AP)
   end

   return (dn_AB * de_AP - de_AB * dn_AP) / math.sqrt(ab_len_sq)
end

local function crosstrack_monitor()
   if not arming:is_armed() then
      cte_reset_state()
      return
   end

   local fw_active = vehicle:is_fixed_wing_flight() and 1 or 0
   local target  = vehicle:get_target_location()
   local current = ahrs:get_location()

   if target == nil or current == nil then
      cte_reset_state()
      return
   end

   local t_lat, t_lng = target:lat(), target:lng()

   if cte_prev_lat == nil then
      local home = ahrs:get_home()
      if home == nil then
         cte_reset_state()
         return
      end
      cte_prev_lat, cte_prev_lng = home:lat(), home:lng()
      cte_cur_target_lat, cte_cur_target_lng = t_lat, t_lng
   elseif t_lat ~= cte_cur_target_lat or t_lng ~= cte_cur_target_lng then
      cte_prev_lat, cte_prev_lng = cte_cur_target_lat, cte_cur_target_lng
      cte_cur_target_lat, cte_cur_target_lng = t_lat, t_lng
   end

   local cte = cte_meters(cte_prev_lat, cte_prev_lng,
                          cte_cur_target_lat, cte_cur_target_lng,
                          current:lat(), current:lng())

   local wp_radius        = WP_RADIUS:get() or 90.0
   local multiplier       = FWM_CTE_MULT:get() or 2.0
   local threshold        = multiplier * wp_radius
   local persist_ms       = FWM_CTE_PERS:get() or 1500
   local warn_interval_ms = FWM_WARN_INTVL:get() or 2000

   local abs_cte = math.abs(cte)
   local now_ms = millis():toint()

   local trig, warn = 0, 0

   if fw_active == 1 then
      if abs_cte <= threshold then
         cte_state.dev_start_ms = 0
      else
         trig = 1
         if cte_state.dev_start_ms == 0 then
            cte_state.dev_start_ms = now_ms
         elseif now_ms - cte_state.dev_start_ms >= persist_ms
            and now_ms - cte_state.last_warn_ms >= warn_interval_ms then
            gcs:send_text(MAV_SEVERITY_WARN, string.format(
               "FW WARNING: Cross Track Error %.0fm (limit %.0fm)",
               abs_cte, threshold, (now_ms - cte_state.dev_start_ms) * 0.001))
            cte_state.last_warn_ms = now_ms
            warn = 1
         end
      end
   else
      cte_state.dev_start_ms = 0
   end

   fwm_log_state.cte_tg = trig
   fwm_log_state.cte_w  = warn
end


-- ===========================================================================
-- 4. SINK / CLIMB RATE MONITOR
-- ===========================================================================

local sink_state = { dev_start_ms = 0, last_warn_ms = 0 }

local function sink_reset_state()
   sink_state.dev_start_ms = 0
   fwm_log_state.sr_tg = 0
   fwm_log_state.sr_w  = 0
end

local function sink_rate_monitor()
   if not arming:is_armed() then return end

   local sink_limit = FWM_SNK_LIM:get() or 3.0

   if not vehicle:is_fixed_wing_flight() then
      sink_reset_state()
      return
   end

   local vel_ned = ahrs:get_velocity_NED()
   if vel_ned == nil then
      sink_reset_state()
      return
   end

   local climb_rate = -vel_ned:z()
   local sink_rate  = -climb_rate
   local now_ms = millis():toint()

   local persist_ms       = FWM_SNK_PERS:get()  or 1000
   local warn_interval_ms = FWM_WARN_INTVL:get() or 2000

   local trig, warn = 0, 0

   if climb_rate > -sink_limit then
      sink_state.dev_start_ms = 0
   else
      trig = 1
      if sink_state.dev_start_ms == 0 then
         sink_state.dev_start_ms = now_ms
      elseif now_ms - sink_state.dev_start_ms >= persist_ms
         and now_ms - sink_state.last_warn_ms >= warn_interval_ms then
         gcs:send_text(MAV_SEVERITY_WARN, string.format(
            "FW WARNING: Sink %.1fm/s (limit %.1f)",
            sink_rate, sink_limit,
            (now_ms - sink_state.dev_start_ms) * 0.001))
         sink_state.last_warn_ms = now_ms
         warn = 1
      end
   end

   fwm_log_state.sr_tg = trig
   fwm_log_state.sr_w  = warn
end


-- ===========================================================================
-- MASTER SCHEDULER
-- ===========================================================================

local MASTER_TICK_MS = 100             -- fastest rate any monitor needs
local last_run_ms = { alt = 0, att = 0, cte = 0, sink = 0 }
local was_enabled = { alt = true, att = true, cte = true, sink = true }

local function master_tick()
   local now_ms = millis():toint()

   -- ALTITUDE
   local alt_on = warn_enabled(BIT_ALT)
   if not alt_on and was_enabled.alt then alt_reset_state() end
   was_enabled.alt = alt_on
   if alt_on then
      local rate = FWM_ALT_RATE:get() or 500
      if now_ms - last_run_ms.alt >= rate then
         last_run_ms.alt = now_ms
         altitude_monitor()
      end
   end

   -- ATTITUDE
   local att_on = warn_enabled(BIT_ATT)
   if not att_on and was_enabled.att then att_reset_state() end
   was_enabled.att = att_on
   if att_on then
      local rate = FWM_ATT_RATE:get() or 100
      if now_ms - last_run_ms.att >= rate then
         last_run_ms.att = now_ms
         attitude_monitor()
      end
   end

   -- CROSS-TRACK
   local cte_on = warn_enabled(BIT_CTE)
   if not cte_on and was_enabled.cte then cte_reset_state() end
   was_enabled.cte = cte_on
   if cte_on then
      local rate = FWM_CTE_RATE:get() or 200
      if now_ms - last_run_ms.cte >= rate then
         last_run_ms.cte = now_ms
         crosstrack_monitor()
      end
   end

   -- SINK RATE
   local snk_on = warn_enabled(BIT_SINK)
   if not snk_on and was_enabled.sink then sink_reset_state() end
   was_enabled.sink = snk_on
   if snk_on then
      local rate = FWM_SNK_RATE:get() or 200
      if now_ms - last_run_ms.sink >= rate then
         last_run_ms.sink = now_ms
         sink_rate_monitor()
      end
   end

   -- MASTER LOGGING
   -- If LOG_EN is 1, write the entire FWM state to dataflash
   if (FWM_LOG_EN:get() or 0) > 0 then
      logger:write("FWM",
         "AltTg,AltW,RoTg,RoW,PiTg,PiW,YaTg,YaW,CteTg,CteW,SrTg,SrW",
         "BBBBBBBBBBBB",
         "------------",
         "000000000000",
         fwm_log_state.alt_tg,
         fwm_log_state.alt_w,
         fwm_log_state.r_tg,
         fwm_log_state.r_w,
         fwm_log_state.p_tg,
         fwm_log_state.p_w,
         fwm_log_state.y_tg,
         fwm_log_state.y_w,
         fwm_log_state.cte_tg,
         fwm_log_state.cte_w,
         fwm_log_state.sr_tg,
         fwm_log_state.sr_w)
   end

   return master_tick, MASTER_TICK_MS
end


-- ===========================================================================
-- BOOT
-- ===========================================================================

local function bitmask_summary(mask_value)
   if mask_value == 0 then return "none" end
   local parts = {}
   if bit_is_set(mask_value, BIT_ALT)  then parts[#parts+1] = "Alt"      end
   if bit_is_set(mask_value, BIT_ATT)  then parts[#parts+1] = "Attitude" end
   if bit_is_set(mask_value, BIT_CTE)  then parts[#parts+1] = "CTE"      end
   if bit_is_set(mask_value, BIT_SINK) then parts[#parts+1] = "Sink"     end
   return table.concat(parts, ",")
end

local enable_mask = FWM_ENABLE:get()   or 15
local log_status  = (FWM_LOG_EN:get() or 1) > 0 and "ON" or "OFF"

gcs:send_text(MAV_SEVERITY_INFO, string.format(
   "FW Monitor: warn=[%s] log=%s",
   bitmask_summary(enable_mask), log_status))

return master_tick, 1000