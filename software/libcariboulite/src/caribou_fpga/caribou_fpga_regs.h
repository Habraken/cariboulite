#ifndef CARIBOU_FPGA_REGS_H
#define CARIBOU_FPGA_REGS_H
#include <stdint.h>

/* Readback whitelist for the current firmware. Reads do not clear or write
 * registers. Undefined/stale output bits are excluded by defined_mask;
 * dynamic_mask identifies FIFO flags/physical inputs that may change without a write. */
#define CARIBOU_FPGA_REG_COUNT 10
#define CARIBOU_FPGA_REG_ALL ((1U << CARIBOU_FPGA_REG_COUNT) - 1U)
typedef struct { const char *name; uint8_t module, ioc, defined_mask, dynamic_mask; } caribou_fpga_reg_desc_st;
static const caribou_fpga_reg_desc_st caribou_fpga_reg_desc[CARIBOU_FPGA_REG_COUNT] = {
    {"SYS.module_version", 0, 0, 0xff, 0},
    {"SYS.system_version", 0, 1, 0xff, 0},
    {"SYS.manufacturer", 0, 2, 0xff, 0},
    {"SYS.gap_sync_source", 0, 6, 0xff, 0},
    {"IO.module_version", 1, 0, 0xff, 0},
    {"IO.mode", 1, 1, 0x1f, 0},
    {"IO.digital", 1, 2, 0xfb, 0xf8},
    /* Internal RF control state; mixer pad output is forced high in top.v. */
    {"IO.rf_state", 1, 5, 0xff, 0},
    {"SMI.module_version", 2, 0, 0xff, 0},
    {"SMI.fifo_channel_direction", 2, 1, 0xff, 0x03},
};
typedef struct {
    uint16_t valid_mask;
    uint8_t value[CARIBOU_FPGA_REG_COUNT];
} caribou_fpga_regs_st;
#endif
