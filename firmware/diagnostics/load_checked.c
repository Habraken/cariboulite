#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "cariboulite_setup.h"

CARIBOULITE_CONFIG_DEFAULT(board);

static int read_register(unsigned char opcode, unsigned char *value)
{
    unsigned char tx[2] = {opcode, 0}, rx[2] = {0, 0};
    int result = io_utils_spi_transmit(board.fpga.io_spi,
        board.fpga.io_spi_handle, tx, rx, 2, io_utils_spi_read_write);
    if (result != 0) return -1;
    *value = rx[1];
    return 0;
}

static int load_and_check(const char *path)
{
    unsigned char values[5] = {0};
    const unsigned char opcodes[5] = {0x01, 0x02, 0x00, 0x20, 0x40};
    unsigned char mode = 0xff;
    printf("LOAD_FILE=%s\n", path);
    fflush(stdout);
    /* This API always performs configuration; an existing operational image
       cannot cause it to skip programming. No embedded image is used. */
    if (caribou_prog_configure(&board.fpga.prog_dev, path) != 0) return -1;
    if (caribou_fpga_soft_reset(&board.fpga) != 0) return -1;
    io_utils_usleep(100000);
    caribou_smi_setup_ios(&board.smi);
    if (io_utils_read_gpio(board.fpga.prog_dev.cdone_pin) != 1) return -1;
    for (unsigned i = 0; i < 5; ++i) {
        if (read_register(opcodes[i], &values[i]) != 0 || values[i] != 1) {
            fprintf(stderr, "SPI verification failed: opcode=0x%02x value=0x%02x\n",
                    opcodes[i], values[i]);
            return -1;
        }
    }
    if (caribou_fpga_set_io_ctrl_mode(&board.fpga, 0,
            caribou_fpga_io_ctrl_rfm_low_power) != 0 ||
        read_register(0x21, &mode) != 0 || (mode & 0x1f) != 0) return -1;
    printf("VERIFIED cdone=1 sys=%u manufacturer=%u sys_module=%u io_module=%u smi_module=%u low_power_mode=0x%02x\n",
           values[0], values[1], values[2], values[3], values[4], mode);
    fflush(stdout);
    return 0;
}

static bool valid_image(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode) && st.st_size == 32220 &&
           access(path, R_OK) == 0;
}

int main(int argc, char **argv)
{
    int result = 1;
    if (argc != 3 || !valid_image(argv[1]) || !valid_image(argv[2])) {
        fprintf(stderr, "Usage: %s <candidate.bin> <rollback.bin> (32220-byte images)\n", argv[0]);
        return 64;
    }
    /* The driver enforces exclusive open; claim it before touching GPIO/SPI. */
    board.ownership_fd = open("/dev/smi", O_RDWR | O_CLOEXEC);
    if (board.ownership_fd < 0) {
        fprintf(stderr, "Cannot claim /dev/smi: %s\n", strerror(errno));
        return 2;
    }
    board.ownership_claimed = true;
    if (cariboulite_setup_io(&board) != 0) {
        close(board.ownership_fd);
        return 3;
    }
    /* Setup holds modem and mixer in reset. Do not initialize radio channels,
       open streaming pipelines or activate RX/TX. */
    if (caribou_fpga_init(&board.fpga, &board.spi_dev) != 0) goto cleanup;
    if (load_and_check(argv[1]) == 0) {
        puts("CANDIDATE_PROGRAMMED=1 RF_TESTED=0");
        result = 0;
    } else {
        fprintf(stderr, "Candidate failed; restoring known rollback image.\n");
        if (load_and_check(argv[2]) == 0) {
            puts("ROLLBACK_RESTORED=1");
            result = 4;
        } else {
            fprintf(stderr, "ROLLBACK_RESTORED=0\n");
            result = 5;
        }
    }
cleanup:
    if (board.fpga.initialized) caribou_fpga_close(&board.fpga);
    cariboulite_release_io(&board);
    return result;
}
