#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Master enable for the Danger Zone framework.
// Default-on so the library and its unit tests build everywhere; gating to
// specific vehicles/boards can be tightened later.
#ifndef AP_DANGERZONE_ENABLED
#define AP_DANGERZONE_ENABLED 1
#endif
