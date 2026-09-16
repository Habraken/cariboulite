# Overview
The SMI (Secondary Memory Interface) serves as the memory interface that facilitates the transfer of I/Q complex samples between the CaribouLite and the
 RPI Broadcom SoC. This interface plays a crucial role in streaming data from the AT86RF215 IC's ADC (Analog-to-Digital Converter) to an FPGA at a
 maximum sample rate of 4 MSPS (Mega Samples Per Second).

Each ADC sample consists of 13 bits for the I (In-phase) component and 13 bits for the Q (Quadrature) component. These samples are then transmitted
to the FPGA through the modem LVDS interface; SMI connects the FPGA to the Pi. To accommodate the data requirements, each RF channel (CaribouLite has two channels) necessitates 4 bytes per
sample, which are padded to 32 bits. This translates to a data rate of 16 MBytes/sec, equivalent to 128 MBits/sec.

In addition to the I/Q data, the Tx/Rx (Transmit/Receive) streams include flow control and configuration bits. The Microchip AT86RF215 IC, responsible
for the modem functionality, features two RX I/Q outputs from its ADCs (one for each physical channel - sub-1GHz and 2.4GHz) and a single TX I/Q input
directed to the DACs.

While Broadcom's Linux distribution provides the SMI interface through a device driver module embedded upon request from the Device-Tree, it lacks
support for essential streaming device file APIs such as "poll," "select," and asynchronous I/O. Consequently, a custom kernel module was developed
to enable uninterrupted sample streaming.

However, there are challenges associated with this approach. Developing, testing, and debugging kernel modules can be complex and time-consuming,
but portability is the bigger issue. Users update their machine's kernel and each kernel update requires recompiling the kernel module using the updated
kernel headers. Additionally, users with different kernel versions may need to recompile and link the kernel module locally to ensure compatibility.

Kernel mainline integration is a historical proposal, not an available installation path.
Maintenance/packaging options remain [DOC-02](../../ROADMAP.md#documentation-validation-backlog).
This also is the main issue while switching between the mainline Raspbian to DragonOS or other RPI distributions.

Currently the "smi-stream-dev" is stable but is still has certain issues to solve:
- the usage of "single-shot" DMA request rather than "cyclic" DMA requests. As a result, depending on the systems compute load, it may miss samples and
needs resynchronization. This resync process is done on the software side, but it still misses a few samples. This applies both to RX and TX, while the
FPGA FIFO depth enlargement solves this problem (most of the time), this development process still needs to be visited as it will certainly make the
module better and the requirements from the FPGA less constrained.
- generalization problem: the SMI-STREAM-DEV name was given to the module for a good reason - no mentioning CaribouLite. This module should be built
with no specific relation with CaribouLite and should support high speed streaming of information in the RPI (whether its a camera, display, ultrasound
receiver, or SDR). Currently the module is not enough general.
- Bugs - solving them is an iterative task. Though some of the bugs pop out frequently, tracking their source is time consuming.

# Steps to Compiling
When the kernel module is not applicable to the kernel version the following is expected when starting the system:
```
...
      05-30 17:02:39.490  2342  2342 I CARIBOU_SMI caribou_smi_init@caribou_smi.c:493 initializing caribou_smi
      05-30 17:02:39.505  2342  2342 D CARIBOU_SMI_MODULES caribou_smi_check_modules@caribou_smi_modules.c:112 Loading smi-stream module
-->   05-30 17:02:39.506  2342  2342 E CARIBOU_SMI_MODULES caribou_smi_insert_smi_modules@caribou_smi_modules.c:71 Module insertion 'smi_stream_dev' failed
-->   05-30 17:02:39.506  2342  2342 E CARIBOU_SMI caribou_smi_init@caribou_smi.c:502 Problem reloading SMI kernel modules
-->   05-30 17:02:39.506  2342  2342 E CARIBOULITE Setup cariboulite_init_submodules@cariboulite_setup.c:286 Error setting up smi submodule
      05-30 17:02:39.506  2342  2342 D CARIBOULITE Setup cariboulite_release_submodules@cariboulite_setup.c:442 CLOSE FPGA communication
      05-30 17:02:39.506  2342  2342 I IO_UTILS_SPI io_utils_spi_remove_chip@io_utils_spi.c:475 removing an spi device with handle 0
...
```

This historical log shows a module insertion failure, which can have several
causes. Inspect the kernel log and module/kernel versions before concluding
that recompilation is sufficient. Current sources and the driver-only procedure
are in [driver/README.md](../../driver/README.md); the old userspace `kernel/`
directory holds generated artifacts and is not the build source.

The original requests for easier updates, generic streaming support and timing
improvements are tracked in [DOC-02](../../ROADMAP.md#documentation-validation-backlog).

# API
Our implemented API (user-mode) driver is located in: [Caribou-SMI API Driver](../../software/libcariboulite/src/caribou_smi/README.md)
