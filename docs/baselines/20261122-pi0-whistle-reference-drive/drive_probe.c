#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/* Process-local diagnostic: override CLKO drive only; preserve clock selection.
 * The opaque pointer/int arguments match the exported device/enum ABI.
 * No per-sample interception or active-window polling. */
static int override_drive = -1;
__attribute__((constructor)) static void configure(void)
{
    const char *value = getenv("WHISTLE_REF_DRIVE_MA");
    if (!value) return;
    char *end;
    long ma = strtol(value, &end, 10);
    if (!*value || *end || (ma != 2 && ma != 4 && ma != 6 && ma != 8)) {
        fprintf(stderr, "Invalid WHISTLE_REF_DRIVE_MA\n");
        _exit(98);
    }
    override_drive = (int)ma / 2 - 1;
}

void at86rf215_set_clock_output(void *dev, int drive, int clock)
{
    void (*set_clock)(void *, int, int);
    int (*read_byte)(void *, uint16_t);
    *(void **)(&set_clock) = dlsym(RTLD_NEXT, "at86rf215_set_clock_output");
    *(void **)(&read_byte) = dlsym(RTLD_NEXT, "at86rf215_read_byte");
    if (!set_clock || !read_byte) abort();
    int selected = override_drive < 0 ? drive : override_drive;
    set_clock(dev, selected, clock);
    int actual = read_byte(dev, 0x0007); /* RF_CLKO: DRV[4:3], OS[2:0]. */
    fprintf(stderr, "CLOCK_DRIVE original=%d selected=%d os=%d readback=%d\n",
            drive, selected, clock, actual);
}

/* Observe existing startup PLL-status reads; introduce no periodic polling. */
void rffc507x_readback_status(void *dev, void *id, void *status)
{
    void (*read_status)(void *, void *, void *);
    *(void **)(&read_status) = dlsym(RTLD_NEXT, "rffc507x_readback_status");
    if (!read_status) abort();
    read_status(dev, id, status);
    if (status) {
        uint16_t raw;
        memcpy(&raw, status, sizeof(raw));
        fprintf(stderr, "PLL_STATUS raw=%u lock=%u\n", (unsigned)raw,
                (unsigned)((raw >> 15) & 1));
    }
}
