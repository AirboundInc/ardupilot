#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_LTE {
public:
    AP_LTE();

    void init();
    void update();
    bool is_initialized() const { return _initialized; }
    bool is_connected()   const { return _connected; }

    static const struct AP_Param::GroupInfo var_info[];

    enum class State : uint8_t {
        INIT = 0,
        WAIT_BOOT,
        SEND_AT,
        WAIT_AT_OK,
        DISABLE_ECHO,
        WAIT_ECHO,
        CHECK_SIM,
        WAIT_SIM_OK,
        CHECK_NETWORK,
        WAIT_NETWORK,
        ACTIVATE_PDP,
        WAIT_PDP,
        WAIT_ACT,
        CHECK_ACT,
        WAIT_ACT_CHECK,
        CLOSE_SOCKET,
        WAIT_CLOSE,
        OPEN_SOCKET,
        WAIT_SOCKET,
        CONNECTED,
        ERROR
    };

private:
    enum class BridgeState : uint8_t {
        IDLE = 0,
        SEND,
        READ,
    };

    void     handle_state_machine();
    void     bridge_data();
    uint16_t build_mavlink_packet(uint8_t *buf, uint16_t buflen, uint32_t now);
    void     send_at(const char *cmd);
    bool     check_response(const char *expected);
    void     change_state(State new_state);

    // ── UARTs ──────────────────────────────────────────────────
    AP_HAL::UARTDriver *_uart_modem{nullptr};
    AP_HAL::UARTDriver *_uart_telem{nullptr};  // SERIAL2 MAVLink2 (downlink only)

    // ── State ──────────────────────────────────────────────────
    State    _state{State::INIT};
    uint32_t _state_start_ms{0};
    uint32_t _last_poll_ms{0};

    bool     _connected{false};
    bool     _initialized{false};

    // ── MAVLink packet generation timers ───────────────────────
    uint32_t _mav_last_hb_ms{0};
    uint32_t _mav_last_att_ms{0};
    uint32_t _mav_last_pos_ms{0};
    uint32_t _mav_last_sys_ms{0};
    uint8_t  _mav_seq{0};

    // ── Bridge ─────────────────────────────────────────────────
    BridgeState _bridge_state{BridgeState::IDLE};
    uint32_t    _bridge_state_ms{0};
    uint8_t     _pending_tx_buf[512];
    uint16_t    _pending_tx_len{0};

    // ── Shared RX buffer ───────────────────────────────────────
    char     _rx_buf[512];
    uint16_t _rx_idx{0};

    // ── Parameters ─────────────────────────────────────────────
    AP_Int8  _enabled;
    AP_Int8  _debug_level;
};