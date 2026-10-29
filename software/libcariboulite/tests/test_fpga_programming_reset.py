#!/usr/bin/env python3
"""Check programming/reset ordering with mocked FPGA status and GPIO only."""
from pathlib import Path
import subprocess
import tempfile


source = (Path(__file__).resolve().parents[1] /
          "src/caribou_fpga/caribou_fpga.c").read_text()


def function(name):
    start = source.index("int " + name + "(")
    return source[start:source.index("\n}", start) + 2]


preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define ZF_LOGE(...) ((void)0)
#define ZF_LOGI(...) ((void)0)
#define CARIBOU_FPGA_CHECK_DEV(d, f) assert((d) && (d)->initialized)
enum { caribou_fpga_status_not_programmed, caribou_fpga_status_operational };
typedef struct {
    int initialized, soft_reset_pin, prog_dev, status;
} caribou_fpga_st;
static char events[32];
static size_t event_count;
static int failures_remaining;
static bool configured, leds_on, reset_asserted;

static void event(char value) {
    assert(event_count + 1 < sizeof(events));
    events[event_count++] = value;
    events[event_count] = '\0';
}
static int caribou_fpga_get_status(caribou_fpga_st *dev, void *unused) {
    (void)unused;
    event('Q');
    dev->status = configured ? caribou_fpga_status_operational
                             : caribou_fpga_status_not_programmed;
    return 0;
}
static int configure(char kind) {
    event(kind);
    if (failures_remaining > 0) {
        --failures_remaining;
        return -1;
    }
    configured = true;
    leds_on = false; /* Configuration alone leaves the LED flip-flops clear. */
    return 0;
}
static int caribou_prog_configure_from_buffer(int *dev, unsigned char *data,
                                             size_t length) {
    assert(dev && data && length);
    return configure('B');
}
static int caribou_prog_configure(int *dev, const char *filename) {
    assert(dev && filename);
    return configure('F');
}
static void io_utils_write_gpio_with_wait(int pin, int value, int wait) {
    assert(pin == 17 && wait == 1000);
    assert(configured); /* This harness only expects post-configuration resets. */
    event(value ? 'H' : 'L');
    if (!value) {
        assert(!reset_asserted);
        reset_asserted = true;
        leds_on = true; /* The firmware's reset value for both user LEDs. */
    } else {
        assert(reset_asserted);
        reset_asserted = false;
    }
}
static void io_utils_usleep(int delay) { assert(delay > 0); }
'''

main = r'''
static caribou_fpga_st setup(bool operational, int failures) {
    event_count = 0;
    events[0] = '\0';
    failures_remaining = failures;
    configured = operational;
    leds_on = false;
    reset_asserted = false;
    return (caribou_fpga_st){.initialized = 1, .soft_reset_pin = 17};
}
static int program(caribou_fpga_st *dev, bool file, bool force) {
    unsigned char image[] = {1, 2, 3};
    return file ? caribou_fpga_program_to_fpga_from_file(dev, "image.bin", force)
                : caribou_fpga_program_to_fpga(dev, image, sizeof(image), force);
}
int main(void) {
    for (int file = 0; file <= 1; ++file) {
        /* Fresh and forced programming must apply the same LED/reset defaults. */
        for (int operational = 0; operational <= 1; ++operational) {
            caribou_fpga_st dev = setup(operational, 0);
            assert(program(&dev, file, operational) == 0);
            assert(strcmp(events, file ? "QFLHQ" : "QBLHQ") == 0);
            assert(leds_on && !reset_asserted);
            assert(dev.status == caribou_fpga_status_operational);
        }
        /* Retaining a running image must preserve its state without a reset. */
        caribou_fpga_st dev = setup(true, 0);
        assert(program(&dev, file, false) == 0);
        assert(strcmp(events, "Q") == 0 && !leds_on);

        dev = setup(false, 3);
        /* Retry return-value bookkeeping is separate from reset consistency. */
        (void)program(&dev, file, false);
        assert(strcmp(events, file ? "QF" : "QBBB") == 0);
        assert(!configured && !leds_on && !reset_asserted);
    }
    /* A failed attempt must not reset; the later successful attempt must. */
    caribou_fpga_st dev = setup(false, 1);
    assert(program(&dev, false, false) == 0);
    assert(strcmp(events, "QBBLHQ") == 0 && leds_on && !reset_asserted);
    return 0;
}
'''

code = preamble + "\n".join(function(name) for name in (
    "caribou_fpga_soft_reset", "caribou_fpga_program_to_fpga",
    "caribou_fpga_program_to_fpga_from_file")) + main
with tempfile.TemporaryDirectory(prefix="fpga-programming-reset-") as directory:
    directory = Path(directory)
    fixture = directory / "test.c"
    executable = directory / "test"
    fixture.write_text(code)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    str(fixture), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True, timeout=5)

print("PASS FPGA blob/file programming reset, LED defaults, retries, and retained image")
