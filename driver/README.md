# SMI stream driver

The current kernel module is `smi_stream_dev`, backed by `bcm2835_smi`.
Its source of truth is this directory. The headers under the userspace
`caribou_smi/kernel/` directory are generated/copied installation artifacts.

## Build and installation

Run the repository's `./install.sh` from the repository root as a normal user.
It builds `software/utils/generate_bin_blob` before invoking this directory's
installer. The top-level script also pulls Git changes and installs dependencies;
review it before using it on a development checkout.

For a driver-only installation after the blob generator and matching kernel
headers are available, run from the repository root:

```sh
./driver/install.sh install 6 2 3
```

The arguments are FIFO MTU multiplier, direction address offset and channel
address offset. The driver-only defaults are `16 2 3`; the top-level installer
chooses multiplier 6 or 2 based on available memory. These are script defaults,
not measured optimal settings for every Pi.

The installer clears `driver/build`, compiles against the running kernel's
`/lib/modules/$(uname -r)/build`, checks the module's vermagic, compresses the
module and generates the userspace blob/header copies. It installs into the
running kernel's existing `bcm2835_smi_dev` directory and runs `depmod`. It then
writes:

- `/etc/modprobe.d/blacklist-bcm_smi.conf`: blacklist `bcm2835_smi_dev`.
- `/etc/modules-load.d/smi_stream_mod.conf`: load `smi_stream_dev` at boot.
- `/etc/modprobe.d/smi_stream_mod_cariboulite.conf`: module parameters.
- `/etc/udev/rules.d/40-cariboulite.rules`: device permissions.

Rebuild after a kernel change or a driver source change. The installer still
requests `raspberrypi-kernel-headers`; OS-specific package compatibility needs
revalidation (roadmap DOC-01). Installing a module does not replace an already
loaded copy; reboot before checking the installed setup.

```sh
uname -r
lsmod | grep smi
modinfo -F vermagic smi_stream_dev
ls -l /dev/smi
```

Expect `smi_stream_dev` and `bcm2835_smi`; module sizes and device major numbers
vary. These checks establish installation state, not successful sample streaming.

## Udev rules

The exact rules are in [udev/40-cariboulite.rules](udev/40-cariboulite.rules).
They currently set mode `0666` for SMI, I2C, SPI, GPIO memory and raw memory
matches, with groups on some rules. This is broader than SMI-only access.
The helper [udev/install.sh](udev/install.sh) copies/removes the rules and reloads
udev. A narrower permission policy and unprivileged operation tests are tracked
as [DOC-01](../roadmap.md#documentation-validation-backlog).
