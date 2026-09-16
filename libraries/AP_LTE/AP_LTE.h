#pragma once

#include "AP_LTE_config.h"

#if AP_LTE_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "AP_LTE_Modem.h"

/*
  Virtual serial port presented to the rest of the vehicle as a MAVLink
  telemetry link. The GCS backend sees an ordinary UARTDriver; the driver
  thread pumps bytes between it and the modem.

    writebuffer: GCS _write()    -> device_read()   (uplink: vehicle -> modem)
    readbuffer:  device_write()  -> GCS _read()     (downlink: modem -> vehicle)

  This replaces the SCR_SDEV simulated device the Lua applet attached to.
*/
class AP_LTE_Port : public AP_SerialManager::RegisteredPort {
public:
    friend class AP_LTE;

    bool init_buffers(uint32_t size_rx, uint32_t size_tx);

    // device side, called only from the driver thread
    size_t   device_write(const uint8_t *buffer, size_t size);
    ssize_t  device_read(uint8_t *buffer, uint16_t count);
    uint32_t device_available();
    void     device_clear();

private:
    bool     is_initialized() override { return true; }
    bool     tx_pending() override { return false; }
    uint32_t txspace() override;
    void     _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t   _write(const uint8_t *buffer, size_t size) override;
    ssize_t  _read(uint8_t *buffer, uint16_t count) override;
    uint32_t _available() override;
    void     _end() override {}
    void     _flush() override {}
    bool     _discard_input() override;

    ByteBuffer *readbuffer;
    ByteBuffer *writebuffer;
    HAL_Semaphore sem;
};

/*
  AT-command LTE modem driver, ported from the LTE_modem.lua applet.

  Supported chipsets: SIM7600/R1951 (SimCom), EC200/EC20/EC25/BG95/EG800Q
  (Quectel). Transparent-mode socket only -- the CMUX, direct-push and PPP
  transports the applet also carried are not part of this driver.

  Step order on a clean boot:
    ATI -> BAUD -> CPIN -> CONFIG -> CREG -> SIMINFO -> HTTPAUTH
        -> SIGNAL_GATE -> [QENG] -> [CIPMODE -> NETOPEN]
        -> SOCKET_STATE -> [CIPCLOSE] -> CIPOPEN -> CONNECTED

  Step names are emitted verbatim as "LTE_modem: step <NAME>" statustext, which
  is what Tools/autotest/Lte_modem_test.py asserts against.
*/
class AP_LTE {
public:
    AP_LTE();

    CLASS_NO_COPY(AP_LTE);

    static AP_LTE *get_singleton() { return _singleton; }

    // register the virtual port. Must run before gcs().setup_uarts(), or the
    // port exists but no MAVLink backend is ever attached to it.
    void pre_init();

    // claim the modem UART and start the driver thread
    void init();

    static const struct AP_Param::GroupInfo var_info[];

    // Hand-rolled scanning. ChibiOS builds have no sscanf, and pulling it in
    // for a handful of "<int>,<int>" replies would not be worth the flash even
    // if they did. Each advances p past what it consumed. Public only so the
    // file-local parsing helpers can reach them.
    static void skip_ws(const char *&p);
    static bool parse_int(const char *&p, int32_t &v);
    static bool expect_char(const char *&p, char c);
    static bool parse_int_after(const char *&p, char sep, int32_t &v);
    // "<a>.<b>.<c>.<d>", each octet 0-255
    static bool parse_ipv4(const char *&p, uint8_t ip[4]);
    // step over a "..." field, leaving p just past the closing quote
    static bool skip_quoted(const char *&p);

    enum class Step : uint8_t {
        ATI = 0,
        BAUD,
        CMUX,
        CPIN,
        CONFIG,
        CREG,
        SIMINFO,
        HTTPAUTH,
        SIGNAL_GATE,
        QENG,
        CEREG_CHECK,
        SOCKET_STATE,
        CIPCLOSE,
        CGACT,
        PPPOPEN,
        CIPMODE,
        NETOPEN,
        CIPOPEN,
        CONNECTED,
        HALT,
    };

private:
    static AP_LTE *_singleton;

    // LTE_OPTIONS bits. DPUSH is inert: this driver has no direct-push (QISEND)
    // transport. The bit keeps its value so a fleet-wide LTE_OPTIONS carried
    // over from the Lua applet still means the same thing.
    enum Option : uint16_t {
        OPT_LOGALL     = (1U << 0),
        OPT_SIGNALS    = (1U << 1),
        OPT_NOMUX      = (1U << 2),
        OPT_NOSIGQUERY = (1U << 3),
        OPT_TCP        = (1U << 4),
        OPT_DPUSH      = (1U << 5),
    };

    enum class HttpSub : uint8_t {
        NONE = 0,
        TERM_WAIT,
        // Quectel
        CERT_LIST,
        CERT_UPLOAD_CMD,
        CERT_UPLOAD_DATA,
        CFG,
        CACERT,
        SECLEVEL,
        ACT,
        URL_CMD,
        URL_DATA,
        POST_CMD,
        POST_DATA,
        READ,
        // SimCom
        SC_CERT_LIST,
        SC_CERT_UPLOAD_CMD,
        SC_CERT_UPLOAD_DATA,
        SC_INIT,
        SC_CID,
        SC_CACERT,
        SC_SSL,
        SC_URL,
        SC_CONTENT,
        SC_DATA_CMD,
        SC_DATA_BODY,
        SC_ACTION,
        SC_READ,
    };

    // ---- thread / dispatch -------------------------------------------------
    void thread_main();
    uint32_t run_step();
    static const char *step_name(Step s);

    // ---- steps -------------------------------------------------------------
    void step_ATI();
    void step_BAUD();
    void step_CPIN();
    void step_CONFIG();
    void step_CREG();
    void step_SIMINFO();
    void step_HTTPAUTH();
    void step_SIGNAL_GATE();
    void step_QENG();
    void step_CEREG_CHECK();
    void step_SOCKET_STATE();
    void step_CIPCLOSE();
    void step_CGACT();
    void step_PPPOPEN();
    void step_CIPMODE();
    void step_NETOPEN();
    void step_CIPOPEN();
    void step_CONNECTED();

    Step next_after_registration() const;
    // LTE_PROTOCOL 48: hand the modem's PPP link to the IP stack instead of
    // opening a socket ourselves.
    bool ppp_mode() const;
    // Where PPP bring-up starts: the context check, or straight to the dial.
    Step ppp_entry() const;

    // ---- CMUX (GSM 07.10 basic mode) ---------------------------------------
    enum : uint8_t {
        CMUX_FLAG      = 0xF9,
        CMUX_UIH       = 0xEF,
        CMUX_SABM      = 0x2F,
        CMUX_EA        = 0x01,
        CMUX_CR        = 0x02,
        CMUX_DLC_AT    = 1,
        CMUX_DLC_DATA  = 2,
        // safe for any chipset's N1 default; see cmux_send()
        CMUX_MAX_FRAME = 127,
    };

    bool cmux_enabled() const;
    static uint8_t cmux_fcs(const uint8_t *data, uint8_t len);
    uint16_t cmux_encode(uint8_t dlc, uint8_t type, const uint8_t *data, uint16_t len,
                         uint8_t *out, uint16_t out_len);
    void cmux_send(uint8_t dlc, const uint8_t *data, uint16_t len);
    void cmux_send_sabm();
    void cmux_feed(const uint8_t *data, uint16_t len);
    void step_CMUX();
    static bool is_known_broken_cmux(const char *rev);
    static bool is_known_good_ec25(const char *rev);

    // ---- UART / framing ----------------------------------------------------
    // Leaves de-framed AT-channel bytes in _chunk (and socket bytes in _dlc2)
    // when multiplexed, so every step reads replies the same way either way.
    uint16_t uart_read();
    // Raw and unframed, for the phases that run before the mux exists.
    uint16_t uart_read_raw();
    void uart_write_raw(const char *s);
    void send_raw(const uint8_t *data, uint16_t len);
    // socket stream: DLC 2 when multiplexed, straight at the UART otherwise
    void data_send(const uint8_t *data, uint16_t len);
    void at_send(const char *cmd);
    void at_sendf(const char *fmt, ...) FMT_PRINTF(2, 3);
    // Same, but on the socket channel: DLC 2 when multiplexed.
    void data_sendf(const char *fmt, ...) FMT_PRINTF(2, 3);
    void rx_append_all();
    void uart_write_pending();
    void send_data_reset();
    bool handle_error();                // ERROR in _rx -> reset to ATI

    // search that does not stop at a NUL, for buffers holding socket bytes
    static const char *mem_find(const char *hay, uint16_t hay_len, const char *needle);
    bool chunk_find(const char *needle) const;
    void flush_fc();                    // downlink the vport could not take

    // ---- reply accumulator -------------------------------------------------
    void rx_reset();
    void rx_append(const uint8_t *data, uint16_t len);
    // binary-safe: the accumulator carries raw socket bytes in CONNECTED
    bool rx_find(const char *needle) const;
    const char *rx_at(const char *needle) const;
    void rx_consume_through(const char *needle);

    // ---- parsers -----------------------------------------------------------
    void handle_AT_reply();
    bool check_CSQ();
    bool check_CGACT();
    bool check_CPSI();
    bool check_QENG();
    bool check_QCSQ();
    void check_modem_banner();
    void check_modem_revision(const char *s);
    bool parse_auth_response(const char *body, uint8_t ip[4], uint16_t &port) const;
    void apply_auth_identity(const char *body);

    // ---- helpers -----------------------------------------------------------
    bool option_enabled(uint16_t opt) const { return (uint16_t(_options.get()) & opt) != 0; }
    void change_step(Step s) { _step = s; }
    void reset_state();
    void reset_to_ATI();
    void reset_buffers();
    void set_MCCMNC();
    void set_BAND();
    void effective_server(uint8_t ip[4], uint16_t &port) const;
    void http_fail(const char *reason, bool no_retry = false);
    bool build_auth_body();
    void log_raw(const uint8_t *data, uint16_t len, const char *marker);
    void log_open();

    // ---- port / uart -------------------------------------------------------
    AP_HAL::UARTDriver *_uart;
    AP_LTE_Port _port;
    bool _port_registered;
    ByteBuffer *_txq;                   // pending bytes for the modem UART

    // ---- state -------------------------------------------------------------
    const AP_LTE_ModemInfo *_modem;

    Step _step;
    Step _last_step;
    uint32_t _step_timer_ms;
    bool _reset_recorded;

    struct {
        const char *name;
        uint32_t ms;
    } _step_times[16];
    uint8_t _step_times_n;

    uint32_t _change_baud;              // non-zero: apply at the top of run_step
    uint32_t _ati_sequence;
    uint32_t _ati_dbg_ms;

    bool _cops_zero_sent;
    uint8_t _qcsq_tries;
    uint8_t _cipopen_retry;
    bool _cipopen_sent;
    uint32_t _cipopen_sent_ms;
    bool _cipopen_preclosed;
    bool _cipmode_sent;
    uint8_t _hard_reset_strikes;

    uint8_t _sim_probe_count;
    uint8_t _sim_missing_count;
    uint32_t _cpin_probe_n;

    uint32_t _cereg_drop_ms;            // 0 = registered
    uint32_t _disconnect_ms;            // 0 = no outage in progress
    uint32_t _creg_search_ms;
    uint32_t _last_data_ms;
    uint32_t _last_CSQ_ms;
    uint32_t _last_CSQ_reply_ms;
    uint32_t _last_send_data_ms;
    uint32_t _ltei_ms;
    bool _post_reset;
    bool _csq_toggle;

    bool _siminfo_sent;
    bool _siminfo_printed;

    int32_t _last_mccmnc;
    int32_t _last_band;

    // band/cell tracking, -1 = not yet seen
    int32_t _track_band;
    int32_t _track_cid;

    // identity strings pulled out of ATI/COPS/SPN
    char _modem_revision[24];
    char _fw_short[20];
    char _sim_spn[20];
    char _sim_oper[20];
    char _sim_name[20];

    // ---- HTTPS auth --------------------------------------------------------
    HttpSub _http_sub;
    uint8_t _http_cfg_i;
    uint32_t _http_deadline;
    uint8_t _http_retry_count;
    bool _term_reset_done;
    char _auth_body[80];                // aircraftId=...&password=...
    uint8_t _auth_ip[4];
    uint16_t _auth_port;
    bool _auth_done;
    const char *_halt_reason;

    int8_t _authkey_seen;               // -1 = not yet acted on

    // ---- buffers -----------------------------------------------------------
    uint8_t _raw[512];                  // as it came off the UART
    uint16_t _raw_len;
    uint8_t _chunk[512];                // AT channel, de-framed when multiplexed
    uint16_t _chunk_len;
    uint8_t _dlc2[512];                 // socket channel, CMUX only
    uint16_t _dlc2_len;
    uint8_t _cmux_partial[1024];        // frame still arriving
    uint16_t _cmux_partial_len;

    bool _found_cmux;                   // the modem is speaking frames
    bool _cmux_was_set;                 // per attempt: step_CMUX reached BAUD
    // Sticky for the life of the boot. Set either by firmware revision
    // auto-detect, or by CPIN failing repeatedly after CMUX supposedly came
    // up -- firmware that accepts AT+CMUX=0 and then never opens the channels.
    bool _cmux_force_disabled;
    // Consecutive muxed attempts that got the mux up and then timed out on
    // CPIN. Survives a reset, cleared once a connection comes up muxed.
    uint8_t _cmux_cpin_strikes;

    char _rx[AP_LTE_RXBUF_SIZE + 1];
    uint16_t _rx_len;

    uint8_t _fc[1024];                  // downlink bytes the vport could not take
    uint16_t _fc_len;

    uint32_t _bytes_in;
    uint32_t _bytes_out;

    int _log_fd;

    // ---- parameters --------------------------------------------------------
    //
    // Every one of these is AP_Float, which looks wrong for what are all
    // integers. It is deliberate. The Lua applet declared them through
    // param:add_table(106, "LTE_", ...), and a dynamic table stores its
    // members as AP_PARAM_FLOAT at storage key 106 + AP_PARAM_DYNAMIC_KEY_BASE
    // = 406. AP_Param::scan() matches on type as well as key and group
    // element, so typed params here would not find any of those stored values
    // and every provisioned vehicle would silently fall back to defaults on
    // the first flash of this firmware. Float members at key 406 with the
    // original indices read the existing entries exactly.
    AP_Float _enable;
    AP_Float _serport;
    AP_Float _scrport;
    AP_Float _server_ip0;
    AP_Float _server_ip1;
    AP_Float _server_ip2;
    AP_Float _server_ip3;
    AP_Float _server_port;
    AP_Float _baud;
    AP_Float _timeout;
    AP_Float _protocol;
    AP_Float _options;
    AP_Float _ibaud;
    AP_Float _mccmnc;
    AP_Float _tx_rate;
    AP_Float _band;
    AP_Float _grace;
    AP_Float _stuck_t;
    AP_Float _httpauth;
    AP_Float _authsys;
    AP_Float _authkey;

    // the params are floats but every use of them is integral
    int32_t pget(const AP_Float &p) const { return int32_t(p.get()); }
};

namespace AP {
    AP_LTE *lte();
};

#endif  // AP_LTE_ENABLED
