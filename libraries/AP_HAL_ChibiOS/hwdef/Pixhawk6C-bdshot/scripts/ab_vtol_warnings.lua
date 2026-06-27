-- ab_vtol_warnings.lua
-- ===========================================================================
-- VTOL flight monitoring & WARNING system for tailsitter.
--
-- Currently monitors two VTOL-specific conditions:
--   1. Pitch angle exceeded (|pitch| > limit, sustained)
--   2. Pitch control effort exceeded (|P+D+FF| > limit, sustained)
--
-- Active only while quadplane:in_vtol_mode() is true and vehicle is ARMED. 
-- In fixed-wing flight or when DISARMED, the script polls slowly and emits nothing.
--
-- Logs a consolidated message (VTM) containing only the Trigger and Warn states.
-- ===========================================================================

local MAV_SEVERITY_WARN = 4         -- MAV_SEVERITY_WARNING
local MAV_SEVERITY_INFO = 6         -- MAV_SEVERITY_INFO

-- Monitor bit assignments (used in VTM_ENABLE).
local BIT_PITCH_ANG = 1     -- Pitch angle warning
local BIT_PITCH_EFF = 2     -- Pitch effort warning

local BIT_VTOL_FENCE = 4    -- VTOL geofence (home distance) warning


-- ===========================================================================
-- PARAMETER TABLE SETUP
-- ===========================================================================
local PARAM_TABLE_KEY    = 75              -- unique vs FWM_ (which uses 74/76)
local PARAM_TABLE_PREFIX = "VTM_"

-- Table size kept at 13 to safely preserve your existing memory allocation
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 13),
       "VTOL Monitor: could not add param table")

-- Helper: add a parameter and return a Parameter object bound to it.
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
          string.format("VTOL Monitor: could not add param %s", name))
   local p = Parameter()
   assert(p:init(PARAM_TABLE_PREFIX .. name),
          string.format("VTOL Monitor: could not init param %s", name))
   return p
end


-- ---------------------------------------------------------------------------
-- GLOBAL PARAMETERS
-- (Note: Index 9 is skipped to prevent memory corruption)
-- ---------------------------------------------------------------------------
local VTM_ENABLE = bind_add_param("ENABLE", 1, 7)
local VTM_LOG_EN = bind_add_param("LOG_EN", 2, 1)
local VTM_RATE   = bind_add_param("RATE", 3, 20)

-- PITCH ANGLE
local VTM_PANG_LIM   = bind_add_param("PANG_LIM", 4, 50.0)
local VTM_PANG_PERS  = bind_add_param("PANG_PERS", 5, 300)

-- UNIFIED WARNING INTERVAL
local VTM_WARN_INTVL = bind_add_param("WARN_INTVL", 6, 2000)

-- PITCH EFFORT
local VTM_PEFF_LIM   = bind_add_param("PEFF_LIM", 7, 0.5)
local VTM_PEFF_PERS  = bind_add_param("PEFF_PERS", 8, 200)
-- (Index 9 was PEFF_INTVL - Removed)

-- VTOL GEOFENCE (home-distance warning)
local VTM_FENCE_RAD  = bind_add_param("FENCE_RAD",  10, 50.0)   -- radius in metres
local VTM_FENCE_PERS = bind_add_param("FENCE_PERS", 11, 1000)   -- ms outside radius before warning


-- ===========================================================================
-- BITMASK HELPERS
-- ===========================================================================
local function bit_is_set(mask_value, bit)
   return math.floor(mask_value / bit) % 2 == 1
end

local function warn_enabled(bit) return bit_is_set(VTM_ENABLE:get() or 0, bit) end


-- ===========================================================================
-- PER-WARNING STATE
-- ===========================================================================
local angle_state  = { dev_start_ms = 0, last_warn_ms = 0 }
local effort_state = { dev_start_ms = 0, last_warn_ms = 0 }
local fence_state  = { dev_start_ms = 0, last_warn_ms = 0 }

local function reset_all_states()
   angle_state.dev_start_ms  = 0
   effort_state.dev_start_ms = 0
   fence_state.dev_start_ms  = 0
end

local was_enabled = { ang = true, eff = true, fence = true }


-- ===========================================================================
-- THRESHOLD CHECK HELPER
-- ===========================================================================
local function check_threshold(state, value, limit, persist_ms,
                               warn_interval_ms, label, unit_str)
   local now_ms = millis():toint()

   if math.abs(value) <= limit then
      state.dev_start_ms = 0
      return 0, 0
   end

   if state.dev_start_ms == 0 then
      state.dev_start_ms = now_ms
      return 1, 0
   end

   if now_ms - state.dev_start_ms < persist_ms then
      return 1, 0
   end

   if now_ms - state.last_warn_ms < warn_interval_ms then
      return 1, 0
   end

   gcs:send_text(MAV_SEVERITY_WARN, string.format(
      "VTOL WARNING: %s %.2f%s",
      label, math.abs(value), unit_str,
      (now_ms - state.dev_start_ms) * 0.001))
   state.last_warn_ms = now_ms
   return 1, 1
end


-- ===========================================================================
-- MAIN UPDATE
-- ===========================================================================
local function update()
   -- Handle enable -> disable edges
   local ang_on = warn_enabled(BIT_PITCH_ANG)
   local eff_on = warn_enabled(BIT_PITCH_EFF)
   if not ang_on and was_enabled.ang then angle_state.dev_start_ms  = 0 end
   if not eff_on and was_enabled.eff then effort_state.dev_start_ms = 0 end
   was_enabled.ang, was_enabled.eff = ang_on, eff_on

   -- SKIP if DISARMED or if NOT IN VTOL MODE
   if not arming:is_armed() or not quadplane:in_vtol_mode() then
      reset_all_states()
      return update, 1000
   end

   local rate_ms = VTM_RATE:get() or 20
   local log_en  = (VTM_LOG_EN:get() or 1) > 0

   -- Initialize variables so they exist in scope
   local pitch_deg, pitch_effort = 0, 0
   local ang_trig, ang_warn = 0, 0
   local eff_trig, eff_warn = 0, 0

   -- 1. PITCH ANGLE CHECKS
   if ang_on or log_en then
      local att_actual = qp_att_actual()
      if att_actual then
          pitch_deg = att_actual.pitch_cd * 0.01
      end
      
      if ang_on then
         local limit       = VTM_PANG_LIM:get()   or 50.0
         local persist_ms  = VTM_PANG_PERS:get()  or 100
         local intvl_ms    = VTM_WARN_INTVL:get() or 2000
         ang_trig, ang_warn = check_threshold(angle_state, pitch_deg,
            limit, persist_ms, intvl_ms, "PITCH Angle", "deg")
      end
   end

   -- 2. PITCH EFFORT CHECKS
   if eff_on or log_en then
      local pitch_pid = qp_rate_pid_info(1)
      if pitch_pid then
          pitch_effort = pitch_pid.P + pitch_pid.D + pitch_pid.FF
      end
      
      if eff_on then
         local limit       = VTM_PEFF_LIM:get()   or 0.5
         local persist_ms  = VTM_PEFF_PERS:get()  or 100
         local intvl_ms    = VTM_WARN_INTVL:get() or 2000
         eff_trig, eff_warn = check_threshold(effort_state, pitch_effort,
            limit, persist_ms, intvl_ms, "PITCH Effort", "")
      end
   end

   -- 3. VTOL GEOFENCE (horizontal distance from home) CHECK
   local fnc_on = warn_enabled(BIT_VTOL_FENCE)
   if not fnc_on and was_enabled.fence then fence_state.dev_start_ms = 0 end
   was_enabled.fence = fnc_on

   local fnc_trig, fnc_warn = 0, 0
   local horiz_dist = 0
   if fnc_on or log_en then
      local current = ahrs:get_location()
      local home    = ahrs:get_home()
      if current and home then
         horiz_dist = current:get_distance(home)   -- metres, horizontal only
      end

      if fnc_on then
         local radius     = VTM_FENCE_RAD:get()  or 50.0
         local persist_ms = VTM_FENCE_PERS:get() or 1000
         local intvl_ms   = VTM_WARN_INTVL:get() or 2000
         local now_ms     = millis():toint()

         if horiz_dist <= radius then
            fence_state.dev_start_ms = 0
         else
            fnc_trig = 1
            if fence_state.dev_start_ms == 0 then
               fence_state.dev_start_ms = now_ms
            elseif now_ms - fence_state.dev_start_ms >= persist_ms
               and now_ms - fence_state.last_warn_ms >= intvl_ms then
               gcs:send_text(MAV_SEVERITY_WARN, string.format(
                  "VTOL WARNING: %.0fm from home (limit %.0fm)",
                  horiz_dist, radius))
               fence_state.last_warn_ms = now_ms
               fnc_warn = 1
            end
         end
      end
   end

   -- 4. MASTER LOGGING
   if log_en then
      logger:write("VTM",
         "PAngTg,PAngW,EffPTg,EffPW,FncTg,FncW",
         "BBBBBB",
         "------",
         "000000",
         ang_trig,
         ang_warn,
         eff_trig,
         eff_warn,
         fnc_trig,
         fnc_warn)
   end

   return update, rate_ms
end


-- ===========================================================================
-- BOOT
-- ===========================================================================
local function warn_summary(mask_value)
   if mask_value == 0 then return "none" end
   local parts = {}
   if bit_is_set(mask_value, BIT_PITCH_ANG)  then parts[#parts+1] = "PAng"  end
   if bit_is_set(mask_value, BIT_PITCH_EFF)  then parts[#parts+1] = "PEff"  end
   if bit_is_set(mask_value, BIT_VTOL_FENCE) then parts[#parts+1] = "Fence" end
   return table.concat(parts, ",")
end

local enable_mask = VTM_ENABLE:get() or 7
local log_status  = (VTM_LOG_EN:get() or 1) > 0 and "ON" or "OFF"

gcs:send_text(MAV_SEVERITY_INFO, string.format(
   "VTOL Monitor: warn=[%s] log=%s",
   warn_summary(enable_mask), log_status))

return update, 1000