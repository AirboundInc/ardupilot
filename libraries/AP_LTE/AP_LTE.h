#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL_Boards.h>

// ─────────────────────────────────────────────────────────────────────────────
// AP_LTE_VirtualPort
//
// writebuffer: ArduPlane GCS _write() → vport_read()  (uplink: Pixhawk → modem)
// readbuffer:  vport_write()          → GCS _read()   (downlink: modem → Pixhawk)
// ─────────────────────────────────────────────────────────────────────────────
class AP_LTE_VirtualPort : public AP_SerialManager::RegisteredPort {
public:
    AP_LTE_VirtualPort() {}
    bool     init_buffers(uint32_t rxS, uint32_t txS);
    void     begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
        _begin(baud, rxS, txS);
    }
    uint32_t vport_available();
    ssize_t  vport_read(uint8_t *buf, uint16_t count);
    size_t   vport_write(const uint8_t *buf, size_t size);

private:
    bool     is_initialized() override { return true; }
    bool     tx_pending()     override { return false; }
    uint32_t txspace()        override;
    void     _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t   _write(const uint8_t *buffer, size_t size) override;
    ssize_t  _read(uint8_t *buffer, uint16_t count) override;
    uint32_t _available() override;
    void     _end()   override {}
    void     _flush() override {}
    bool     _discard_input() override;

    ByteBuffer   *writebuffer{nullptr};
    ByteBuffer   *readbuffer{nullptr};
    uint32_t      last_size_tx{0};
    uint32_t      last_size_rx{0};

};

// ─────────────────────────────────────────────────────────────────────────────
// AP_LTE — EC200UCN driver (no CMUX, transparent access mode 2)
//
// State machine:
//   WAIT_BOOT → SEND_AT → WAIT_AT_OK → DISABLE_ECHO → WAIT_ECHO
//   → CHECK_SIM → WAIT_SIM_OK → CHECK_NETWORK → WAIT_NETWORK
//   → CLOSE_SOCKET → WAIT_CLOSE → OPEN_SOCKET → WAIT_SOCKET → CONNECTED
//
// Data flow when CONNECTED (transparent mode — raw byte passthrough):
//   Uplink:   vport (GCS MAVLink) → raw bytes → modem UART → relay → MP
//   Downlink: raw bytes from modem UART → vport (GCS MAVLink)
// ─────────────────────────────────────────────────────────────────────────────
class AP_LTE {
public:
    AP_LTE();

    void pre_init();   // call BEFORE gcs().setup_uarts()
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
        CLOSE_SOCKET,
        WAIT_CLOSE,
        OPEN_SOCKET,
        WAIT_SOCKET,
        CONNECTED,
        ERROR
    };

private:
    void  handle_state_machine();
    void  bridge_data();
    void  send_at(const char *cmd);
    bool  check_response(const char *expected);
    void  change_state(State s);

    AP_HAL::UARTDriver *_uart_modem{nullptr};
    AP_LTE_VirtualPort  _vport;

    State    _state{State::INIT};
    uint32_t _state_start_ms{0};

    bool     _connected{false};
    bool     _initialized{false};
    bool     _vport_registered{false};

    static constexpr uint16_t RX_BUF_SIZE = 2048;
    uint8_t  _rx_buf[RX_BUF_SIZE];
    uint16_t _rx_idx{0};

    uint8_t  _tx_buf[256];  // uplink chunk buffer

    AP_Int8  _enabled;
    AP_Int8  _debug_level;
};