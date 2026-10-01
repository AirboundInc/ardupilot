#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

// enabled in the hwdef of boards that use it; provisioned and read over
// MAVLink, so it needs a GCS
#ifndef AP_CUSTOM_STORAGE_ENABLED
#define AP_CUSTOM_STORAGE_ENABLED (HAL_GCS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
