#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
/* Observe configuration only; forward every mode selection unchanged. This
 * runs before streaming starts, never for each sample or capture callback.
 * Opaque device pointer and int enum retain the exported library ABI. */
int caribou_fpga_set_io_ctrl_mode(void *dev, uint8_t debug, int mode)
{
    int (*set_mode)(void *, uint8_t, int);
    int (*get_mode)(void *, uint8_t *, int *);
    *(void **)(&set_mode) = dlsym(RTLD_NEXT, "caribou_fpga_set_io_ctrl_mode");
    *(void **)(&get_mode) = dlsym(RTLD_NEXT, "caribou_fpga_get_io_ctrl_mode");
    if (!set_mode || !get_mode) abort();
    int rc = set_mode(dev, debug, mode);
    uint8_t actual_debug = 0;
    int actual_mode = -1;
    int read_rc = rc ? -1 : get_mode(dev, &actual_debug, &actual_mode);
    fprintf(stderr, "BYPASS_ROUTE requested=%d set_rc=%d actual=%d debug=%u read_rc=%d\n",
            mode, rc, actual_mode, (unsigned)actual_debug, read_rc);
    return rc;
}
