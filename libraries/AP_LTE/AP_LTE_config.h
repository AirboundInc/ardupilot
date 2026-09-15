#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// LTE modem driver. Ported from the LTE_modem.lua applet; transparent-mode
// socket only (no CMUX, no direct push, no PPP).
//
// On unconditionally, on every board and flash size. Only ArduPlane links the
// library (see ArduPlane/wscript), so this is Plane-only in practice.
//
// Gating the macro itself on APM_BUILD_TYPE does not work: waf decides whether
// a library is vehicle-dependent by grepping each .cpp for that macro (see
// _depends_on_vehicle in Tools/ardupilotwaf/ap_library.py), and a reference
// from a header does not count -- AP_LTE would be built once with no vehicle
// defined and compile itself away. The side effect is that
// AP_SERIALMANAGER_REGISTER_ENABLED becomes true for every vehicle, which
// costs the other ones a small amount of flash for registered-port support
// they do not use.
#ifndef AP_LTE_ENABLED
#define AP_LTE_ENABLED 1
#endif

// Size of the shared AT-reply accumulator. The largest single reply the
// driver has to hold whole is the HTTPS auth JSON body; everything else is
// trimmed as it arrives.
#ifndef AP_LTE_RXBUF_SIZE
#define AP_LTE_RXBUF_SIZE 2048
#endif

// Virtual port buffers. The port carries the full MAVLink stream between the
// GCS backend and the modem, so it is sized like a real telemetry UART.
#ifndef AP_LTE_VPORT_RXBUF
#define AP_LTE_VPORT_RXBUF 2048
#endif
#ifndef AP_LTE_VPORT_TXBUF
#define AP_LTE_VPORT_TXBUF 2048
#endif
