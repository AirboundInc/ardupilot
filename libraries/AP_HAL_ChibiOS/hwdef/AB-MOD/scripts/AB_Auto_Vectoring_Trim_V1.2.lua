--[[
   Axis 2 (independent vectoring) trim switcher for dual-axis tiltrotor quadplanes (Q_TILT_TYPE=4).

   The left/right Axis 2 vectoring servos (SERVO_FUNCTION 190/191) need a
   different neutral (SERVOx_TRIM) in VTOL flight vs fixed wing flight.
   This script saves the VTOL trim on the way out to fixed wing, applies
   VECTRIM_FW_L/VECTRIM_FW_R while fixed wing, and restores the saved VTOL
   trim on the way back.

   The fixed wing trims are only applied once Axis 1 is fully forward,
   read from quadplane:get_tilt() (Tiltrotor current_tilt, 0 = vertical,
   1 = fully forward), so the VTOL trim is kept for the whole forward
   transition while the rotors tilt. Requires firmware with the
   quadplane:get_tilt() scripting binding.
--]]

local MAV_SEVERITY_INFO  = 6
local MAV_SEVERITY_ERROR = 3

local PARAM_TABLE_KEY = 101   -- change if this collides with another loaded script's table key
local PARAM_TABLE_PREFIX = "VECTRIM_"

-- bind a parameter to a variable
local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('VecTrim: could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('VecTrim: could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'VecTrim: could not add param table')

--[[
  // @Param: VECTRIM_ENABLE
  // @DisplayName: Axis2 trim switcher enable
  // @Description: Enables switching Axis 2 (independent vectoring) servo trims between VTOL and fixed wing flight
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local VECTRIM_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: VECTRIM_FW_L
  // @DisplayName: Axis2 left vectoring fixed wing trim
  // @Description: SERVOx_TRIM PWM applied to the left Axis 2 vectoring servo (SERVO_FUNCTION 190) while in fixed wing flight
  // @Units: PWM
  // @Range: 800 2200
  // @Increment: 1
  // @User: Standard
--]]
local VECTRIM_FW_L = bind_add_param('FW_L', 2, 1500)

--[[
  // @Param: VECTRIM_FW_R
  // @DisplayName: Axis2 right vectoring fixed wing trim
  // @Description: SERVOx_TRIM PWM applied to the right Axis 2 vectoring servo (SERVO_FUNCTION 191) while in fixed wing flight
  // @Units: PWM
  // @Range: 800 2200
  // @Increment: 1
  // @User: Standard
--]]
local VECTRIM_FW_R = bind_add_param('FW_R', 3, 1500)

local K_TILTMOTOR_LEFT_VEC  = 190
local K_TILTMOTOR_RIGHT_VEC = 191

local TILT_FULL_FWD = 0.999   -- current_tilt >= this is treated as Axis 1 fully forward

local UPDATE_PERIOD_MS = 200

-- check the firmware has the quadplane:get_tilt() binding before doing anything
if not pcall(function() return quadplane:get_tilt() end) then
   gcs:send_text(MAV_SEVERITY_ERROR, "VecTrim: quadplane:get_tilt() binding missing, stopping")
   return
end

-- resolve the SERVOx_TRIM parameter name for a given SERVO_FUNCTION, or nil if not assigned
local function trim_param_name(servo_function)
   local chan = SRV_Channels:find_channel(servo_function)
   if not chan then
      return nil
   end
   return string.format("SERVO%d_TRIM", chan + 1)
end

local left_trim_param  = trim_param_name(K_TILTMOTOR_LEFT_VEC)
local right_trim_param = trim_param_name(K_TILTMOTOR_RIGHT_VEC)

if not left_trim_param or not right_trim_param then
   gcs:send_text(MAV_SEVERITY_ERROR, "VecTrim: Axis2 left/right servo function not assigned, stopping")
   return
end

-- trims as configured for VTOL flight, captured just before the switch to fixed wing
local vtol_trim_left = nil
local vtol_trim_right = nil

-- true once the fixed wing trims have been applied
local in_fw_trim = false

local function restore_vtol_trims(reason)
   param:set(left_trim_param, vtol_trim_left)
   param:set(right_trim_param, vtol_trim_right)
   in_fw_trim = false
   gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: %s, VTOL trims restored (L=%.0f R=%.0f)",
                  reason, vtol_trim_left, vtol_trim_right))
end

local function apply_fw_trims(tilt)
   vtol_trim_left = param:get(left_trim_param)
   vtol_trim_right = param:get(right_trim_param)
   param:set(left_trim_param, VECTRIM_FW_L:get())
   param:set(right_trim_param, VECTRIM_FW_R:get())
   in_fw_trim = true
   gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: Axis1 %.0fdeg, FW trims applied (L=%.0f R=%.0f), VTOL trims saved (L=%.0f R=%.0f)",
                  tilt * 90, VECTRIM_FW_L:get(), VECTRIM_FW_R:get(), vtol_trim_left, vtol_trim_right))
end

local function update()
   if VECTRIM_ENABLE:get() <= 0 then
      if in_fw_trim then
         restore_vtol_trims("disabled")
      end
      return update, UPDATE_PERIOD_MS
   end

   local is_vtol   = quadplane:in_vtol_mode()
   local tilt      = quadplane:get_tilt()
   local axis1_fwd = tilt >= TILT_FULL_FWD

   -- FW trims only in fixed wing flight with Axis 1 fully forward, VTOL trims otherwise
   local want_fw = (not is_vtol) and axis1_fwd

   if want_fw and not in_fw_trim then
      apply_fw_trims(tilt)
   elseif not want_fw and in_fw_trim then
      restore_vtol_trims(is_vtol and "VTOL mode" or "Axis1 not fwd")
   end

   return update, UPDATE_PERIOD_MS
end

-- protected wrapper: on any script error put the VTOL trims back instead of leaving FW trims stuck
local function protected_update()
   local ok, ret, period = pcall(update)
   if not ok then
      gcs:send_text(MAV_SEVERITY_ERROR, "VecTrim: error: " .. tostring(ret))
      if in_fw_trim and vtol_trim_left and vtol_trim_right then
         param:set(left_trim_param, vtol_trim_left)
         param:set(right_trim_param, vtol_trim_right)
         in_fw_trim = false
      end
      return protected_update, 1000
   end
   return protected_update, period
end

gcs:send_text(MAV_SEVERITY_INFO, "AB VecTrim: loaded")

return protected_update, 1000
