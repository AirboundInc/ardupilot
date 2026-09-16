#include "AP_LTE_Modem.h"

#if AP_LTE_ENABLED

#include <AP_Common/AP_Common.h>

const char AP_LTE_auth_url[] = "https://poc.rudra.airbound.com/api/v1/users/aircraft-login";

// ISRG Root X1 (Let's Encrypt), the root of poc.rudra.airbound.com's chain.
// Pinning the root rather than the intermediate survives LE intermediate
// rotation; only a change of issuer needs this replaced on every modem.
// Valid to 2035-06-04. Source: https://letsencrypt.org/certs/isrgrootx1.pem
const char AP_LTE_ca_cert_pem[] =
    "-----BEGIN CERTIFICATE-----\n"
    "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\n"
    "-----END CERTIFICATE-----\n";

const uint16_t AP_LTE_ca_cert_len = sizeof(AP_LTE_ca_cert_pem) - 1;

// Quectel HTTP(S) configuration, sent one command per OK/ERROR.
static const char *const quectel_http_cfg[] = {
    "AT+QHTTPCFG=\"contextid\",1\r\n",
    "AT+QHTTPCFG=\"responseheader\",0\r\n",
    // 0 = x-www-form-urlencoded, which is what the auth body is. Left at the
    // firmware default it posts the wrong content type.
    "AT+QHTTPCFG=\"contenttype\",0\r\n",
    "AT+QHTTPCFG=\"sslctxid\",1\r\n",
    "AT+QSSLCFG=\"sslversion\",1,4\r\n",
    "AT+QSSLCFG=\"ciphersuite\",1,0XFFFF\r\n",
    // Required: the endpoint is SNI-virtual-hosted, and without this the wrong
    // cert can be served and hostname validation fails.
    "AT+QSSLCFG=\"sni\",1,1\r\n",
    nullptr
};

// Bench-verified 2026-08-17 on EC200U (EC200UCNAAR03A15M08, UNISOC platform).
// Not re-proven on the Qualcomm-dialect Quectel parts.
// QSSLCFG/QFUPL reject a "UFS:" filename prefix even though QFLST echoes one,
// so every input path below passes the bare filename.
static const AP_LTE_HTTP quectel_http = {
    .simcom = false,
    .cert_filename = "isrgrootx1.pem",
    .cert_list = "AT+QFLST=\"*\"\r\n",
    .cert_upload = "AT+QFUPL=\"isrgrootx1.pem\",%u,300\r\n",
    .cfg = quectel_http_cfg,
    // cacert and seclevel are hard-fail steps of their own rather than part of
    // cfg above: these two are what actually bind and enforce certificate
    // verification, so an ERROR must halt rather than quietly continue on a
    // connection that is not verified.
    .cacert = "AT+QSSLCFG=\"cacert\",1,\"isrgrootx1.pem\"\r\n",
    .seclevel = "AT+QSSLCFG=\"seclevel\",1,1\r\n",
    .act = "AT+QIACT=1\r\n",
    .url = "AT+QHTTPURL=%u,30\r\n",
    .post = "AT+QHTTPPOST=%u,80,80\r\n",
    .read = "AT+QHTTPREAD=80\r\n",
    .init = nullptr,
    .cid = nullptr,
    .ssl = nullptr,
    .content = nullptr,
    .data = nullptr,
    .action = nullptr,
    .term = nullptr,
};

// Bench-verified 2026-08-17 on SIMCOM_SIM7600G-H, firmware SIM7600G_V2.0.2.
// SimCom takes the URL as a quoted HTTPPARA string rather than a separate
// upload, and reports the POST result asynchronously via +HTTPACTION.
static const AP_LTE_HTTP simcom_http = {
    .simcom = true,
    .cert_filename = "isrgrootx1.pem",
    .cert_list = "AT+CCERTLIST\r\n",
    .cert_upload = "AT+CCERTDOWN=\"isrgrootx1.pem\",%u\r\n",
    .cfg = nullptr,
    // Context 0 is the default HTTP(S) app context. cacert without authmode=1
    // is a silent no-op: the CA loads but is never consulted.
    .cacert = "AT+CSSLCFG=\"cacert\",0,\"isrgrootx1.pem\"\r\n",
    .seclevel = nullptr,
    .act = nullptr,
    .url = "AT+HTTPPARA=\"URL\",\"https://poc.rudra.airbound.com/api/v1/users/aircraft-login\"\r\n",
    .post = nullptr,
    .read = "AT+HTTPREAD=0,%u\r\n",
    .init = "AT+HTTPINIT\r\n",
    .cid = "AT+HTTPPARA=\"CID\",1\r\n",
    .ssl = "AT+CSSLCFG=\"authmode\",0,1\r\n",
    .content = "AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"\r\n",
    .data = "AT+HTTPDATA=%u,10000\r\n",
    .action = "AT+HTTPACTION=1\r\n",
    .term = "AT+HTTPTERM\r\n",
};

// The Quectel families differ only in their band syntax, so the rest of the
// table is shared. Members stay in declaration order for the designated
// initialisers; EC25 appends its own preflight after this expands.
#define QUECTEL_COMMON(NAME, BANNER, PREFLIGHT, BANDMASK, BANDALL)            \
    .name = NAME,                                                             \
    .banner = BANNER,                                                         \
    .cmux = "AT+CMUX=0\r\n",                                                 \
    .spn = "AT+QSPN\r\n",                                                     \
    .setbaud = nullptr,                                                       \
    .cpsi = "AT+QENG=\"servingcell\"\r\n",                                    \
    .preflight = PREFLIGHT,                                                   \
    .cpin = "AT+CPIN?\r\n",                                                   \
    .reset = "AT+CFUN=1,1\r\n",                                               \
    .cipmode = nullptr,                                                       \
    .cipopen_tcp = "AT+QIOPEN=1,0,\"TCP\",\"%d.%d.%d.%d\",%d,0,2\r\n",        \
    .cipopen_udp = "AT+QIOPEN=1,0,\"UDP\",\"%d.%d.%d.%d\",%d,6001,2\r\n",     \
    .cipclose = "AT+QICLOSE=0\r\n",                                           \
    .pppopen = "ATD*99#\r",                                                   \
    .cgact = nullptr,                                                         \
    .netopen = nullptr,                                                       \
    .netclose = nullptr,                                                      \
    .caswitch = nullptr,                                                      \
    .config_extra = nullptr,                                                  \
    .mccmnc = "AT+COPS=4,2,\"%u\"\r\n",                                       \
    .setband = nullptr,                                                       \
    .setband_mask = BANDMASK,                                                 \
    .setband_all = BANDALL,                                                   \
    .sim_probe = "AT+QCCID\r\n",                                              \
    .csq_gate = "AT+QCSQ\r\n",                                                \
    .socket_state = "AT+QISTATE?\r\n",                                        \
    .cfun = nullptr,                                                          \
    .fast_connect = true,                                                     \
    .reset_not_baudrate = false,                                              \
    .http = &quectel_http

// The 128-bit band families differ only in name, so they get their own
// wrapper: a macro that expanded to two arguments would be counted as one
// at the call site.
#define QUECTEL_COMMON_128(NAME, BANNER)                                      \
    QUECTEL_COMMON(NAME, BANNER, nullptr,                                     \
                   "AT+QCFG=\"band\",0,0x%llx\r\n",                           \
                   "AT+QCFG=\"band\",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n")

const AP_LTE_ModemInfo AP_LTE_modem_list[] = {
    {
        .name = "SimCom",
        .banner = "SIMCOM",
        .cmux = "AT+CMUX=0\r\n",
        .spn = "AT+CSPN?\r\n",
        .setbaud = "AT+IPR=%u\r\n",
        .cpsi = "AT+CPSI?\r\n",
        .preflight = nullptr,
        .cpin = "AT+CPIN?\r\n",
        .reset = "AT+CFUN=1,1\r\n",
        .cipmode = "AT+CIPMODE=1\r\n",
        .cipopen_tcp = "AT+CIPOPEN=0,\"TCP\",\"%d.%d.%d.%d\",%d\r\n",
        .cipopen_udp = "AT+CIPOPEN=0,\"UDP\",\"%d.%d.%d.%d\",%d,6001\r\n",
        .cipclose = "AT+CIPCLOSE=0\r\n",
        .pppopen = "ATD*99#\r",
        .cgact = nullptr,
        .netopen = "AT+NETOPEN\r\n",
        .netclose = nullptr,
        .caswitch = nullptr,
        .config_extra = "ATH\r\n",
        .mccmnc = "AT+COPS=1,2,\"%u\"\r\n",
        .setband = nullptr,
        .setband_mask = "AT+CNBP=,0x%llx\r\n",
        .setband_all = "AT+CNBP=,0x480000000000000000000000000000000000000000000042000007FFFFDF3FFF\r\n",
        .sim_probe = "AT+CICCID\r\n",
        .csq_gate = "AT+CPSI?\r\n",
        .socket_state = "AT+CIPOPEN?\r\n",
        .cfun = nullptr,
        .fast_connect = true,
        .reset_not_baudrate = false,
        .http = &simcom_http,
    },
    {
        .name = "SimCom2",
        .banner = "R1951",
        .cmux = "AT+CMUX=0\r\n",
        .spn = "AT+CSPN?\r\n",
        .setbaud = "AT+IPR=%u\r\n",
        .cpsi = "AT+CPSI?\r\n",
        .preflight = nullptr,
        .cpin = "AT+CPIN?\r\n",
        .reset = "AT+CFUN=1,1\r\n",
        .cipmode = "AT+CACID=0\r\n",
        .cipopen_tcp = "AT+CAOPEN=0,0,\"TCP\",\"%d.%d.%d.%d\",%d\r\n",
        .cipopen_udp = "AT+CAOPEN=0,0,\"UDP\",\"%d.%d.%d.%d\",%d\r\n",
        .cipclose = nullptr,
        .pppopen = "ATD*99#\r",
        // Queried, not set: this family reports context state and PPPOPEN
        // waits for it to read active before dialling.
        .cgact = "AT+CGACT?\r\n",
        .netopen = "AT+CNACT=0,1\r\n",
        .netclose = "AT+CNACT=0,0\r\n",
        .caswitch = "AT+CASWITCH=0,1\r\n",
        .config_extra = nullptr,
        .mccmnc = "AT+COPS=4,2,\"%u\"\r\n",
        .setband = "AT+CBANDCFG=\"CAT-M\",%d\r\n",
        .setband_mask = nullptr,
        .setband_all = "AT+CBANDCFG=\"CAT-M\",1,2,3,4,5,8,12,13,14,18,19,20,25,26,27,28,66,85\r\n",
        .sim_probe = nullptr,
        .csq_gate = nullptr,
        .socket_state = nullptr,
        .cfun = "AT+CFUN=1\r\n",
        .fast_connect = false,
        .reset_not_baudrate = true,
        .http = nullptr,
    },
    {
        // Kept for identification only: this family has no socket-open command,
        // so it only ever connected over PPP. CIPOPEN halts on it and says so.
        .name = "Air780",
        .banner = "AirM2M_780E",
        .cmux = nullptr,
        .spn = nullptr,
        .setbaud = "AT+IPR=%u\r\n",
        .cpsi = nullptr,
        .preflight = nullptr,
        .cpin = "AT+CPIN?\r\n",
        .reset = "AT+CFUN=1,1\r\n",
        .cipmode = "AT+CIPMODE=1\r\n",
        .cipopen_tcp = nullptr,
        .cipopen_udp = nullptr,
        .cipclose = nullptr,
        .pppopen = "ATD*99#\r",
        .cgact = "AT+CGACT=1,1\r\n",
        .netopen = nullptr,
        .netclose = nullptr,
        .caswitch = nullptr,
        .config_extra = nullptr,
        .mccmnc = nullptr,
        .setband = nullptr,
        .setband_mask = nullptr,
        .setband_all = nullptr,
        .sim_probe = nullptr,
        .csq_gate = nullptr,
        .socket_state = nullptr,
        .cfun = nullptr,
        .fast_connect = false,
        .reset_not_baudrate = false,
        .http = nullptr,
    },
    { QUECTEL_COMMON_128("EC200", "EC200") },
    { QUECTEL_COMMON_128("BG95", "BG95") },
    { QUECTEL_COMMON_128("EG800Q", "EG800Q") },
    {
        QUECTEL_COMMON("EC20", "EC20C", nullptr,
                       "AT+QCFG=\"band\",0,%llx,0\r\n",
                       "AT+QCFG=\"band\",0,7FFFFFFFFFFFFFFF,0\r\n"),
    },
    {
        // EC25 alone runs a servingcell preflight before opening the socket.
        QUECTEL_COMMON("EC25", "EC25", "AT+QENG=\"servingcell\"\r\n",
                       "AT+QCFG=\"band\",0,%llx,0\r\n",
                       "AT+QCFG=\"band\",bff,00b0e18df,0\r\n"),
    },
};

const uint8_t AP_LTE_modem_count = ARRAY_SIZE(AP_LTE_modem_list);

// Everything null but the reset, matching default_modem in the Lua original.
const AP_LTE_ModemInfo AP_LTE_default_modem = {
    .name = "unknown",
    .banner = nullptr,
    .cmux = nullptr,
    .spn = nullptr,
    .setbaud = nullptr,
    .cpsi = nullptr,
    .preflight = nullptr,
    .cpin = nullptr,
    .reset = "AT+CFUN=1,1\r\r",
    .cipmode = nullptr,
    .cipopen_tcp = nullptr,
    .cipopen_udp = nullptr,
    .cipclose = nullptr,
    .pppopen = nullptr,
    .cgact = nullptr,
    .netopen = nullptr,
    .netclose = nullptr,
    .caswitch = nullptr,
    .config_extra = nullptr,
    .mccmnc = nullptr,
    .setband = nullptr,
    .setband_mask = nullptr,
    .setband_all = nullptr,
    .sim_probe = nullptr,
    .csq_gate = nullptr,
    .socket_state = nullptr,
    .cfun = nullptr,
    .fast_connect = false,
    .reset_not_baudrate = false,
    .http = nullptr,
};

#endif  // AP_LTE_ENABLED
