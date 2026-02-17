#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>

class AP_LTE {
public:
    AP_LTE();

    void init();
    void update();
    bool is_initialized() { return _initialized; }
    bool is_connected() const { return _connected; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class State {
        INIT,
        WAIT_BOOT,
        SEND_AT,
        WAIT_AT_OK,
        CHECK_SIM,
        WAIT_SIM_OK,
        CHECK_NETWORK,
        WAIT_NETWORK,
        OPEN_SOCKET,
        WAIT_SOCKET,
        SET_TRANSPARENT,
        WAIT_TRANSPARENT,
        CONNECTED,
        ERROR
    };


    AP_HAL::UARTDriver *_uart_modem;
    AP_HAL::UARTDriver *_uart_mavlink;
    
    State _state;
    uint32_t _state_start_ms;
    bool _connected;
    
    char _rx_buf[512];
    uint16_t _rx_idx;
    
    // Debug logging
    void debug_log(const char *msg);
    void debug_log_rx(const char *data, uint16_t len);
    void debug_log_tx(const char *data, uint16_t len);
    
    // Parameters
    AP_Int8 _enabled;
    AP_Int8 _debug_level;  // 0=off, 1=info, 2=verbose
    bool _initialized=false;
    void handle_state_machine();
    void bridge_data();
    
    void send_at(const char *cmd);
    bool check_response(const char *expected);
    void change_state(State new_state);
};
