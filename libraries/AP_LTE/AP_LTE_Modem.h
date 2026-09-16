#pragma once

#include "AP_LTE_config.h"

#if AP_LTE_ENABLED

#include <stdint.h>

/*
  Per-chipset AT command tables, ported from the modem_list/quectel_http/
  simcom_http tables in LTE_modem.lua.

  A null member means "this family does not have that command"; the state
  machine branches on presence exactly as the Lua version did. Format
  specifiers are C, not Lua: band masks are %llx against a uint64_t because
  a band above 32 does not fit an unsigned.
*/

struct AP_LTE_HTTP {
    bool simcom;                    // false = Quectel dialect
    const char *cert_filename;
    const char *cert_list;
    const char *cert_upload;        // fmt: byte count
    const char *const *cfg;         // null-terminated, Quectel only
    const char *cacert;
    const char *seclevel;           // Quectel: turns on cert verification
    const char *act;                // Quectel: PDP context activate
    const char *url;                // Quectel: fmt url length. SimCom: literal
    const char *post;               // Quectel: fmt body length
    const char *read;               // Quectel: literal. SimCom: fmt byte count
    const char *init;               // SimCom only, below
    const char *cid;
    const char *ssl;
    const char *content;
    const char *data;               // fmt: body length
    const char *action;
    const char *term;
};

struct AP_LTE_ModemInfo {
    const char *name;               // table key in the Lua original
    const char *banner;             // substring identifying the family in ATI
    const char *cmux;               // null = this family cannot multiplex
    const char *spn;
    const char *setbaud;            // fmt: baud
    const char *cpsi;
    const char *preflight;
    const char *cpin;
    const char *reset;
    const char *cipmode;
    const char *cipopen_tcp;        // fmt: 4 octets then port
    const char *cipopen_udp;
    const char *cipclose;
    const char *pppopen;            // dial string for LTE_PROTOCOL=48
    const char *cgact;              // null = go straight to pppopen
    const char *netopen;
    const char *netclose;
    const char *caswitch;
    const char *config_extra;
    const char *mccmnc;             // fmt: MCCMNC
    const char *setband;            // fmt: band number
    const char *setband_mask;       // fmt: band bitmask
    const char *setband_all;
    const char *sim_probe;
    const char *csq_gate;
    const char *socket_state;
    const char *cfun;
    bool fast_connect;
    bool reset_not_baudrate;
    const AP_LTE_HTTP *http;
};

extern const AP_LTE_ModemInfo AP_LTE_modem_list[];
extern const uint8_t AP_LTE_modem_count;

// Used until ATI identifies the family.
extern const AP_LTE_ModemInfo AP_LTE_default_modem;

// ISRG Root X1, the root of the auth endpoint's chain.
extern const char AP_LTE_ca_cert_pem[];
extern const uint16_t AP_LTE_ca_cert_len;

extern const char AP_LTE_auth_url[];

#endif  // AP_LTE_ENABLED
