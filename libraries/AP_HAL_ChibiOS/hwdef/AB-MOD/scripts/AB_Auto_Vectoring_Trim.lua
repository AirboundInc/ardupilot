--[[
   Axis 2 (independent vectoring) trim switcher for dual-axis tiltrotor quadplanes (Q_TILT_TYPE=4).

   The left/right Axis 2 vectoring servos (SERVO_FUNCTION 190/191) need a
   different neutral (SERVOx_TRIM) in VTOL flight vs fixed wing flight.
   This script saves the VTOL trim on the way out to fixed wing, applies
   VECTRIM_FW_L/VECTRIM_FW_R while fixed wing, and restores the saved VTOL
   trim on the way back.
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

local UPDATE_PERIOD_MS = 200

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

-- trims as configured for VTOL flight, captured just before the first switch to fixed wing
local vtol_trim_left = nil
local vtol_trim_right = nil

-- true once the fixed wing trims have been applied
local in_fw_trim = false

local function update()
   if VECTRIM_ENABLE:get() <= 0 then
      if in_fw_trim then
         param:set(left_trim_param, vtol_trim_left)
         param:set(right_trim_param, vtol_trim_right)
         gcs:send_text(MAV_SEVERITY_INFO, "VecTrim: disabled, VTOL trims restored")
         in_fw_trim = false
      end
      return update, UPDATE_PERIOD_MS
   end

   local is_vtol = quadplane:in_vtol_mode()

   if is_vtol then
      if in_fw_trim then
         param:set(left_trim_param, vtol_trim_left)
         param:set(right_trim_param, vtol_trim_right)
         gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: VTOL trims restored (L=%d R=%d)", vtol_trim_left, vtol_trim_right))
         in_fw_trim = false
      end
   else
      if not in_fw_trim then
         vtol_trim_left = param:get(left_trim_param)
         vtol_trim_right = param:get(right_trim_param)
         param:set(left_trim_param, VECTRIM_FW_L:get())
         param:set(right_trim_param, VECTRIM_FW_R:get())
         gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: FW trims applied (L=%d R=%d), VTOL trims saved (L=%d R=%d)",
                        VECTRIM_FW_L:get(), VECTRIM_FW_R:get(), vtol_trim_left, vtol_trim_right))
         in_fw_trim = true
      end
   end

   return update, UPDATE_PERIOD_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "VecTrim: loaded")

return update, 1000
