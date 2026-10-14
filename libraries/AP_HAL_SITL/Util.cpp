#include "Util.h"
#include <sys/time.h>
#include <AP_Param/AP_Param.h>
#include "RCOutput.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <AP_Common/ExpandingString.h>
#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE==1
#include <AP_CustomStorage/AP_CustomStorage.h>
#endif

extern const AP_HAL::HAL& hal;

#ifdef WITH_SITL_TONEALARM
HALSITL::ToneAlarm_SF HALSITL::Util::_toneAlarm;
#endif

uint64_t HALSITL::Util::get_hw_rtc() const
{
#ifndef CLOCK_REALTIME
    struct timeval ts;
    gettimeofday(&ts, nullptr);
    return ((long long)((ts.tv_sec * 1000000) + ts.tv_usec));
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t seconds = ts.tv_sec;
    const uint64_t nanoseconds = ts.tv_nsec;
    return (seconds * 1000000ULL + nanoseconds/1000ULL);
#endif
}

/*
  get a (hopefully unique) machine ID
 */
bool HALSITL::Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    char *cbuf = (char *)buf;

    // try first to use machine-id file. Most systems will have this
    const char *paths[] = { "/etc/machine-id", "/var/lib/dbus/machine-id" };
    for (uint8_t i=0; i<ARRAY_SIZE(paths); i++) {
        int fd = open(paths[i], O_RDONLY);
        if (fd == -1) {
            continue;
        }
        ssize_t ret = read(fd, buf, len);
        close(fd);
        if (ret <= 0) {
            continue;
        }
        if (ret == len) {
            cbuf[len-1] = '\0';
        } else {
            cbuf[ret] = '\0';
        }
        len = ret;
        char *p = strchr(cbuf, '\n');
        if (p) {
            *p = 0;
        }
        len = strnlen(cbuf, len);
        buf[0] += sitlState->get_instance();
        return true;
    }

    // fallback to hostname
    if (gethostname(cbuf, len) != 0) {
        // use a default name so this always succeeds. Without it we can't
        // implement some features (such as UAVCAN)
        snprintf(cbuf, len, "sitl-unknown-%d", sitlState->get_instance());
    } else {
        // To ensure separate ids for each instance
        cbuf[0] += sitlState->get_instance();
    }
    len = strnlen(cbuf, len);
    return true;
}

/*
  as get_system_id_unformatted will already be ascii, we use the same
  ID here
 */
bool HALSITL::Util::get_system_id(char buf[50])
{
    uint8_t len = 40;
    return get_system_id_unformatted((uint8_t *)buf, len);
}

#if !defined(HAL_BUILD_AP_PERIPH)
enum AP_HAL::Util::safety_state HALSITL::Util::safety_switch_state(void)
{
#define HAL_USE_PWM 1
#if HAL_USE_PWM
    return ((RCOutput *)hal.rcout)->_safety_switch_state();
#else
    return SAFETY_NONE;
#endif
}

void HALSITL::Util::set_cmdline_parameters()
{
    for (uint16_t i=0; i<sitlState->cmdline_param.available(); i++) {
        const auto param = sitlState->cmdline_param[i];
        if (param != nullptr) {
            AP_Param::set_default_by_name(param->name, param->value);
        }
    }

#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE==1
    // Craft identity goes into custom storage once, so the setters only run
    // after init() has opened the storage area. Values are written on every
    // start, so the command line is the source of truth.
    static bool identity_applied;
    if (identity_applied) {
        return;
    }
    identity_applied = true;

    if (sitlState->craft_id_arg != nullptr || sitlState->craft_password_arg != nullptr ||
        sitlState->uin_arg != nullptr) {
        g_custom_storage.init();
        // no values are printed: the password must not reach the console
        if (sitlState->craft_id_arg != nullptr && !g_custom_storage.set_craft_id(sitlState->craft_id_arg)) {
            fprintf(stderr, "SITL: failed to store craft ID in custom storage\n");
            exit(1);
        }
        if (sitlState->craft_password_arg != nullptr && !g_custom_storage.set_password(sitlState->craft_password_arg)) {
            fprintf(stderr, "SITL: failed to store password in custom storage\n");
            exit(1);
        }
        if (sitlState->uin_arg != nullptr && !g_custom_storage.set_uin(sitlState->uin_arg)) {
            fprintf(stderr, "SITL: failed to store UIN in custom storage\n");
            exit(1);
        }
        printf("SITL: custom storage identity set (craft ID %s)\n",
               sitlState->craft_id_arg != nullptr ? sitlState->craft_id_arg : "unchanged");
    }

#endif
}
#endif

/**
   return commandline arguments, if available
*/
void HALSITL::Util::commandline_arguments(uint8_t &argc, char * const *&argv)
{
    argc = saved_argc;
    argv = saved_argv;
}

/**
 * This method will read random values with set size.
 */
bool HALSITL::Util::get_random_vals(uint8_t* data, size_t size)
{
    int dev_random = open("/dev/urandom", O_RDONLY);
    if (dev_random < 0) {
        return false;
    }
    ssize_t result = read(dev_random, data, size);
    if (result < 0) {
        close(dev_random);
        return false;
    }
    close(dev_random);
    return true;
}

#if HAL_UART_STATS_ENABLED
// request information on uart I/O
void HALSITL::Util::uart_info(ExpandingString &str)
{
    // Calculate time since last call
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt_ms = now_ms - sys_uart_stats.last_ms;
    sys_uart_stats.last_ms = now_ms;

    // a header to allow for machine parsers to determine format
    str.printf("UARTV1\n");
    for (uint8_t i = 0; i < hal.num_serial; i++) {
        if (i >= ARRAY_SIZE(sitlState->_serial_path)) {
            continue;
        }
        auto *uart = hal.serial(i);
        if (uart) {
            str.printf("SERIAL%u ", i);
            uart->uart_info(str, sys_uart_stats.serial[i], dt_ms);
        }
    }
}

#if HAL_LOGGING_ENABLED
// Log UART message for each serial port
void HALSITL::Util::uart_log()
{
    // Calculate time since last call
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt_ms = now_ms - log_uart_stats.last_ms;
    log_uart_stats.last_ms = now_ms;

    // Loop over all ports
    for (uint8_t i = 0; i < hal.num_serial; i++) {
        auto *uart = hal.serial(i);
        if (uart) {
            uart->log_stats(i, log_uart_stats.serial[i], dt_ms);
        }
    }
}
#endif // HAL_LOGGING_ENABLED
#endif // HAL_UART_STATS_ENABLED
