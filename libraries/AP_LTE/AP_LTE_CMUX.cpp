/*
  GSM 07.10 / 3GPP 27.010 basic-mode multiplexing for AP_LTE.

  AT+CMUX=0 puts the modem's single UART into two logical channels: DLC 1
  carries AT commands and DLC 2 carries the socket stream. That is what lets
  signal quality be polled while a transparent socket is open, which is
  otherwise impossible -- in plain transparent mode the socket owns the port.

  Framing is: FLAG | address | control | length | payload | FCS | FLAG, with
  the FCS covering the header only for UIH frames.
 */

#include "AP_LTE.h"

#if AP_LTE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

// Reflected CRC-8, polynomial 0xE0. Checked byte for byte against the table
// the Lua applet carried.
static const uint8_t cmux_fcs_table[256] = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
    0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
    0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
    0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
    0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
    0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
    0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
    0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
    0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
    0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
    0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
    0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
    0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
    0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
    0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
    0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
    0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf,
};

/*
  Firmware revisions that accept AT+CMUX=0 and then silently ignore the SABM
  frames, so the channels never open. Detecting them at identification time
  saves the ~25 second broken-mux recovery cycle on every boot.
*/
bool AP_LTE::is_known_broken_cmux(const char *rev)
{
    if (rev == nullptr || rev[0] == 0) {
        return false;
    }
    static const char *const broken[] = {
        // "EC25EFAR02A08M4G",   // suspected, never confirmed
        nullptr
    };
    for (uint8_t i = 0; broken[i] != nullptr; i++) {
        if (strstr(rev, broken[i]) != nullptr) {
            return true;
        }
    }
    return false;
}

// EC25 families with CMUX confirmed working on the bench. Anything EC25 that
// is neither known-good nor known-broken is used anyway, but warned about, so
// an unverified revision that turns out to hang is diagnosable.
bool AP_LTE::is_known_good_ec25(const char *rev)
{
    if (rev == nullptr) {
        return false;
    }
    static const char *const good[] = { "EC25EFAR06", "EC25EFAR08", nullptr };
    for (uint8_t i = 0; good[i] != nullptr; i++) {
        if (strncmp(rev, good[i], strlen(good[i])) == 0) {
            return true;
        }
    }
    return false;
}

uint8_t AP_LTE::cmux_fcs(const uint8_t *data, uint8_t len)
{
    uint8_t fcs = 0xff;
    for (uint8_t i = 0; i < len; i++) {
        fcs = cmux_fcs_table[fcs ^ data[i]];
    }
    return uint8_t(~fcs);
}

/*
  Whether the AT stream is currently multiplexed.

  found_cmux wins over the option bit: a modem left in CMUX by a previous boot
  is speaking frames whatever the parameter says, so the only way to talk to it
  is to frame too.
*/
bool AP_LTE::cmux_enabled() const
{
    if (_cmux_force_disabled) {
        return false;
    }
    if (_found_cmux) {
        return true;
    }
    return _modem->cmux != nullptr && !option_enabled(OPT_NOMUX);
}

/*
  Frame one payload. Returns bytes written, or 0 if it would not fit.
*/
uint16_t AP_LTE::cmux_encode(uint8_t dlc, uint8_t type,
                             const uint8_t *data, uint16_t len,
                             uint8_t *out, uint16_t out_len)
{
    uint8_t hdr[4];
    uint8_t hdr_len = 0;
    hdr[hdr_len++] = uint8_t((dlc << 2) | CMUX_EA | CMUX_CR);
    hdr[hdr_len++] = uint8_t(type | 0x10);
    if (len <= 127) {
        hdr[hdr_len++] = uint8_t((len << 1) | CMUX_EA);
    } else {
        // Extended two-octet length: L1 clears EA and carries the low 7 bits,
        // L2 the rest. Only needed for payloads over 127 bytes, which in
        // practice means the CA certificate upload during HTTPAUTH.
        hdr[hdr_len++] = uint8_t((len & 0x7f) << 1);
        hdr[hdr_len++] = uint8_t((len >> 7) & 0xff);
    }

    const uint16_t total = uint16_t(1 + hdr_len + len + 1 + 1);
    if (total > out_len) {
        return 0;
    }

    uint16_t o = 0;
    out[o++] = CMUX_FLAG;
    memcpy(&out[o], hdr, hdr_len);
    o += hdr_len;
    if (len > 0) {
        memcpy(&out[o], data, len);
        o += len;
    }
    out[o++] = cmux_fcs(hdr, hdr_len);   // UIH: header only
    out[o++] = CMUX_FLAG;
    return o;
}

/*
  Queue a payload as one or more frames.

  AT+CMUX=0 leaves N1 at each chipset's own default: EC200 takes the whole
  ~1.9KB certificate in a single frame, SIM7600 silently wedges on it. Capping
  outgoing frames at 127 bytes is safe for any N1, and the receiving AT parser
  sees one continuous stream either way.
*/
void AP_LTE::cmux_send(uint8_t dlc, const uint8_t *data, uint16_t len)
{
    if (_txq == nullptr) {
        return;
    }
    uint8_t frame[CMUX_MAX_FRAME + 8];

    if (len == 0) {
        const uint16_t n = cmux_encode(dlc, CMUX_UIH, nullptr, 0, frame, sizeof(frame));
        _txq->write(frame, n);
        return;
    }

    uint16_t pos = 0;
    while (pos < len) {
        const uint16_t chunk = MIN(uint16_t(len - pos), uint16_t(CMUX_MAX_FRAME));
        const uint16_t n = cmux_encode(dlc, CMUX_UIH, &data[pos], chunk,
                                       frame, sizeof(frame));
        if (n == 0) {
            return;
        }
        _txq->write(frame, n);
        pos += chunk;
    }
}

// Open the control channel and both data link channels.
void AP_LTE::cmux_send_sabm()
{
    uint8_t frame[8];
    for (uint8_t dlc = 0; dlc <= 2; dlc++) {
        const uint16_t n = cmux_encode(dlc, CMUX_SABM, nullptr, 0, frame, sizeof(frame));
        if (n > 0 && _txq != nullptr) {
            _txq->write(frame, n);
        }
    }
}

/*
  De-frame a UART read.

  DLC 1 payload is appended to _chunk, so every step above this reads AT
  replies the same way whether or not the stream is multiplexed. DLC 2 payload
  goes to _dlc2 for the CONNECTED path. Anything left over is an incomplete
  frame and is carried to the next read.
*/
void AP_LTE::cmux_feed(const uint8_t *data, uint16_t len)
{
    if (_cmux_partial_len + len > sizeof(_cmux_partial)) {
        // A frame larger than the buffer, or a desync that never resolves.
        // Drop and resync on the next FLAG rather than wedging here.
        _cmux_partial_len = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: CMUX buffer overflow, resyncing");
        return;
    }
    memcpy(&_cmux_partial[_cmux_partial_len], data, len);
    _cmux_partial_len += len;

    uint16_t pos = 0;
    while (pos < _cmux_partial_len) {
        // find the opening flag
        while (pos < _cmux_partial_len && _cmux_partial[pos] != CMUX_FLAG) {
            pos++;
        }
        if (pos >= _cmux_partial_len) {
            break;
        }
        const uint8_t *f = &_cmux_partial[pos];
        const uint16_t avail = uint16_t(_cmux_partial_len - pos);
        if (avail < 6) {
            break;                      // too short to judge yet
        }

        const uint8_t addr = f[1];
        const uint8_t ctrl = f[2];
        const uint8_t l1 = f[3];
        uint16_t payload_len;
        uint8_t hdr_len;
        if (l1 & CMUX_EA) {
            payload_len = uint16_t(l1 >> 1);
            hdr_len = 3;                // addr, ctrl, one length octet
        } else {
            if (avail < 7) {
                break;
            }
            payload_len = uint16_t((l1 >> 1) | (uint16_t(f[4]) << 7));
            hdr_len = 4;
        }

        const uint16_t frame_len = uint16_t(1 + hdr_len + payload_len + 1 + 1);
        if (avail < frame_len) {
            break;                      // still arriving
        }
        if (f[frame_len - 1] != CMUX_FLAG) {
            // Not a frame after all: step over this flag and rescan.
            pos++;
            continue;
        }

        if ((ctrl & 0xef) == CMUX_UIH && cmux_fcs(&f[1], hdr_len) == f[frame_len - 2]) {
            const uint8_t dlc = (addr >> 2) & 0x3f;
            const uint8_t *payload = &f[1 + hdr_len];
            if (dlc == 0) {
                // MSC for DLC 2: the FC bit is the modem telling us to stop
                // sending there. Ignoring it pushed 144KB into a stopped DLC 2
                // (log 0097) and the AT channel died.
                if (payload_len >= 4 && payload[0] == 0xE3 &&
                    (payload[2] >> 2) == CMUX_DLC_DATA) {
                    if ((payload[3] & 0x02) != 0) {
                        if (_fc_on_ms == 0) {
                            _fc_on_ms = AP_HAL::millis();
                        }
                    } else {
                        _fc_on_ms = 0;
                    }
                }
            } else if (dlc == CMUX_DLC_AT) {
                const uint16_t space = uint16_t(sizeof(_chunk) - _chunk_len);
                const uint16_t n = MIN(payload_len, space);
                memcpy(&_chunk[_chunk_len], payload, n);
                _chunk_len += n;
            } else if (dlc == CMUX_DLC_DATA) {
                const uint16_t space = uint16_t(sizeof(_dlc2) - _dlc2_len);
                const uint16_t n = MIN(payload_len, space);
                memcpy(&_dlc2[_dlc2_len], payload, n);
                _dlc2_len += n;
            }
        }
        // SABM answers and bad-FCS frames are consumed without a payload.
        pos += frame_len;
    }

    // carry the remainder
    if (pos > 0) {
        _cmux_partial_len = uint16_t(_cmux_partial_len - pos);
        memmove(_cmux_partial, &_cmux_partial[pos], _cmux_partial_len);
    }
}

/*
  Negotiate the mux. Entered only when cmux_enabled() says we should be
  multiplexed and we have not yet seen a frame from the modem.
*/
void AP_LTE::step_CMUX()
{
    if (_found_cmux) {
        cmux_send_sabm();
        log_event("{CMUX mode set}");
        _cmux_was_set = true;
        _step = Step::BAUD;
        return;
    }

    // Read raw: until the mux is up there is nothing to de-frame, and the
    // reply to AT+CMUX=0 arrives as plain text.
    // Accumulated, not matched within a single read: the acknowledgement is
    // "AT+CMUX=0\r\r\nOK\r\n" and the UART hands that over in however many
    // pieces it likes. Matching per read can see the echo without the OK,
    // conclude nothing happened, and re-send into a modem that has already
    // switched to framing and will never answer plain bytes again.
    const uint16_t n = uart_read_raw();
    if (n > 0) {
        rx_append(_raw, n);
    }

    if (rx_find("CME ERROR")) {
        rx_reset();
        uart_write_raw("AT+CFUN=1\r\n");
        return;
    }

    // A flag byte means the modem is already framing; the echoed OK means it
    // accepted the command and is about to. Either way the mux is up.
    const bool sees_frame = _rx_len >= 4 &&
                            memchr(_rx, CMUX_FLAG, _rx_len) != nullptr;
    const bool sees_ack = rx_find("CMUX=0") && rx_find("OK");
    if (sees_frame || sees_ack) {
        log_event("{CMUX mode set}");
        if (sees_frame) {
            _found_cmux = true;
        }
        _cmux_was_set = true;
        rx_reset();
        cmux_send_sabm();
        _step = Step::BAUD;
        return;
    }

    uart_write_raw(_modem->cmux);
}

#endif  // AP_LTE_ENABLED
