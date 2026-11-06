/* Temporary diagnostic preload. No hardware operations occur at load time.
 * Built from the repository root with:
 *   cc -shared -fPIC -O2 -Wall -Wextra -Werror -I driver \
 *      -o build/u25-tx-diagnosis-20261106/tx_timing_probe.so \
 *      firmware/diagnostics/tx_timing_probe.c -ldl
 *
 * CARIBOULITE_TX_WRITE_TIMING=setup,strobe,hold overrides only the
 * hardware-paced TX request 1/4/2. The caller's mutable expected-settings
 * object is updated, so production readback verification checks the actual
 * experimental tuple. Initial settings and normal restore requests pass
 * through unchanged. Without that environment variable this only logs.
 *
 * Intended only for this backend on Linux/aarch64. Its ioctl calls supply
 * a third argument, either a pointer or a scalar. Unknown requests retain
 * that argument; known Linux no-argument requests do not consume varargs.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "bcm2835_smi.h"

static int (*next_ioctl)(int, unsigned long, ...);
static int (*next_gap)(void *, uint8_t *);
static int override_enabled;
static int write_setup, write_strobe, write_hold;

__attribute__((constructor)) static void probe_init(void)
{
    int saved_errno = errno;
    next_ioctl = dlsym(RTLD_NEXT, "ioctl");
    next_gap = dlsym(RTLD_NEXT, "caribou_fpga_get_sys_ctrl_tx_sample_gap");
    const char *text = getenv("CARIBOULITE_TX_WRITE_TIMING");
    if (text && *text) {
        char trailing;
        if (sscanf(text, "%d,%d,%d%c", &write_setup, &write_strobe,
                   &write_hold, &trailing) != 3 ||
            write_setup < 0 || write_setup > 63 ||
            write_strobe < 1 || write_strobe > 127 ||
            write_hold < 0 || write_hold > 63) {
            fprintf(stderr, "[tx_timing_probe] invalid timing '%s'; exiting\n", text);
            _exit(125);
        }
        override_enabled = 1;
        fprintf(stderr, "[tx_timing_probe] TX timing override=%d/%d/%d\n",
                write_setup, write_strobe, write_hold);
    } else {
        fprintf(stderr, "[tx_timing_probe] baseline; timing unchanged\n");
    }
    errno = saved_errno;
}

static int no_argument(unsigned long request)
{
    switch (request) {
#ifdef FIOCLEX
    case FIOCLEX:
#endif
#ifdef FIONCLEX
    case FIONCLEX:
#endif
#ifdef TIOCNOTTY
    case TIOCNOTTY:
#endif
        return 1;
    default:
        return 0;
    }
}

int ioctl(int descriptor, unsigned long request, ...)
{
    uintptr_t argument = 0;
    if (!no_argument(request)) {
        va_list args;
        va_start(args, request);
        argument = va_arg(args, uintptr_t);
        va_end(args);
    }
    int input_errno = errno;
    struct smi_settings *settings = (struct smi_settings *)argument;
    int changed = 0;
    if (request == BCM2835_SMI_IOC_WRITE_SETTINGS && settings) {
        if (override_enabled && settings->write_setup_time == 1 &&
            settings->write_strobe_time == 4 && settings->write_hold_time == 2) {
            settings->write_setup_time = write_setup;
            settings->write_strobe_time = write_strobe;
            settings->write_hold_time = write_hold;
            changed = 1;
        }
    }
    errno = input_errno;
    int result = next_ioctl ? next_ioctl(descriptor, request, argument)
                            : syscall(SYS_ioctl, descriptor, request, argument);
    int output_errno = errno;
    if (request == BCM2835_SMI_IOC_WRITE_SETTINGS && settings) {
        fprintf(stderr, "[tx_timing_probe] WRITE_SETTINGS fd=%d tuple=%d/%d/%d pace=%d "
                "overridden=%d result=%d errno=%d\n", descriptor,
                settings->write_setup_time, settings->write_strobe_time,
                settings->write_hold_time, settings->write_pace_time,
                changed, result, result < 0 ? output_errno : 0);
    }
    errno = output_errno;
    return result;
}

/* Pointer ABI matches caribou_fpga_st * without importing unrelated headers. */
int caribou_fpga_get_sys_ctrl_tx_sample_gap(void *device, uint8_t *gap)
{
    if (!next_gap) {
        next_gap = dlsym(RTLD_NEXT, "caribou_fpga_get_sys_ctrl_tx_sample_gap");
        if (!next_gap) {
            errno = ENOSYS;
            return -1;
        }
    }
    int result = next_gap(device, gap);
    int saved_errno = errno;
    fprintf(stderr, "[tx_timing_probe] FPGA sample_gap=%u result=%d\n",
            gap ? (unsigned)*gap : 255u, result);
    errno = saved_errno;
    return result;
}
