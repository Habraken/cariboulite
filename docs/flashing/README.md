# EEPROM
Each CaribouLite is pre-configured by the contract manufacturer before shipping. The Full and the ISM versions are further distinguishable by the configuration resistors placed on the back side of the board (marked as "**CFG/1234**").

CaribouLite doesn't have an FPGA configuration flash device, as the ICE40 device is dynamically configured by the RPI quickly and on-demand. It rather has a general board configuration device (EEPROM) as required by RPi's HAT device rules.

For current FPGA image loading and reset commands, use the
[radio controller procedure](../radio-control.md#fpga-programming-after-reboot)
or the explicit programming targets in [firmware/README.md](../../firmware/README.md).
Those operations configure the volatile FPGA; they do not change EEPROM data.

## EEPROM Data Structure
The code is located [here](../../software/libcariboulite/src/hat/) and is based on [RPi EERPROM Utils Tools](https://github.com/raspberrypi/hats/tree/master/eepromutils) provided by RaspberryPi. The ID EEPROM internal structure is described [here](https://github.com/raspberrypi/hats/blob/master/eeprom-format.md).

The general structure is as follows:
1. **HEADER**: containing a valid header key (SIGN), its version, the number of ATOMs that follow the header and the total size of the EEPROM contents.
2. **VENDOR INFO**: Product information and Vendor information, and the product UID (unique 128bit identifier / serial number).
3. **GPIO MAP**: a listing of CaribouLite's RPi GPIO states and flags, for each GPIO pin (GPIO2-GPIO27).
4. **DEVICE TREE**: as the system is booting, the RPi kernel automatically reads this ATOM from the EEPROM and sets it as an overlay. As a result, the kernel modules needed for CaribouLite to operate (SMI, SPI, etc.) are probed on startup and configured according to the overlay.
5. **PROPRIETARY DATA**: historically proposed for IQ, TX-power and reference-clock calibration and GPIO/PMOD settings. The populated layout and calibration use are not established by this guide; validation remains DOC-05.

## Identity for reproduction boards

Requirement recorded 2026-11-06: the new reproduction boards must be identifiable
as our own production run when their EEPROM images are prepared. The selected
vendor string is `CaribouLite Reproduction 2026Q4 PD9JH`, including the owner's
callsign and production-run marker.
Keep the string below 64 bytes, including its terminating NUL, to fit the current
library's board-info buffers.

Keep the product string exactly `CaribouLite RPI Hat`: the current
[SoapySDR discovery code](../../software/libcariboulite/src/soapy_api/SoapyCariboulite.cpp)
matches it literally, while it only reports the vendor string. Preserve the
variant's product ID (`0x0001` for Full, `0x0002` for ISM) and the existing
product version (`0x0001`) for the unchanged first reproduction. Generate and
record a unique, valid UUID for each board, and reuse that UUID if the board is
reprogrammed. The ownership marker belongs in the vendor text, not in the UUID.
This records the image-preparation requirement; a general production procedure
remains tracked under DOC-05. The first blank-board initialization, readback
and post-reboot detection passed as recorded below.

## Gracefully Accessing Data
To access the EEPROM data, one can use our code which also validates and decodes the information. The easier way to access the information within the programmed EEPROM is to use the boot-time device-tree view under `/proc/device-tree` as follows:

```
cd /proc/device-tree/hat/

# name
cat name

# product: "CaribouLite RPI Hat"
cat product

# product_id: "0x0001"
cat product_id

# product_ver: "0x0001"
cat product_ver

# uuid: "AAAAAAAA-BBBB-CCCC-DDDD-EEEEEEEEEEEE" GUID format
#       which is also the board's raw serial number
cat uuid

# vendor: "CaribouLabs.co"
cat vendor
```

## Reprogramming the EEPROM
The implementation is under [hat](../../software/libcariboulite/src/hat/) and
[cariboulite_production.c](../../software/libcariboulite/src/cariboulite_production.c).
General programming/restore instructions and write-protection handling remain tracked as
[DOC-05](../../ROADMAP.md#documentation-validation-backlog). Do not substitute
FPGA programming for EEPROM programming; they target different devices.

### First blank-board initialization, 2026-11-06

The installed Full board's CAT24C32 responded on I2C bus 0 at address `0x50`.
Two complete 4096-byte reads before programming were identical and entirely
`FF`. Initialization then passed with:

| Identity field | Programmed value |
| --- | --- |
| Vendor | `CaribouLite Reproduction 2026Q4 PD9JH` |
| Product | `CaribouLite RPI Hat` |
| Product ID / version | `0x0001` / `0x0001` |
| Board UUID | `d8810044-c444-46f4-a548-703412cbf971` |

The image was generated using the installed Raspberry Pi `eepmake -v1`, with
three legacy HAT atoms: vendor identity, GPIO-bank-0 map and the existing
1100-byte `software/devicetrees/cariboulite.dtbo`. Unused GPIO entries were
zeroed; the used GPIO settings match the library's map. The overlay also passed
offline application against this host's Pi 4 base device tree.

The 1250-byte payload was padded with `FF` to 4096 bytes. Programming preserved
a full blank-device backup, respected 32-byte page boundaries, waited 10 ms
after each write and verified each page. Page zero, containing the HAT header,
was written last. Two complete final readbacks matched the prepared image;
independent parsing verified the identity, atom lengths/CRCs and embedded
overlay. `eepdump` also decoded the readback successfully.

Full programmed-image SHA-256:
`508c2ee1ae08119e3f006cc563da8391bb598505bb2b4771d7f5b93a6513aa3c`.
Blank backup SHA-256:
`f47a8ec3e9aff2318d896942282ad4fe37d6391c82914f54a5da8a37de1300c6`.

Local evidence is retained under `build/eeprom-init-20261106/pd9jh-full/`:
`settings.txt`, `manifest.json`, `image.eep`, `image.bin`, `before.bin`,
`after.bin`, decoded data and `programming-receipt.json`. The preparation and
checked single-board writer scripts are in the parent directory. These are
local, ignored build artifacts. Preserve this board's UUID when reprogramming;
generate a new UUID for each additional board rather than copying this image.

After reboot, `/proc/device-tree/hat` reported the programmed product, vendor
and UUID, and the EEPROM overlay created the SMI controller/device-tree nodes.
The initial backend failure came from the missing custom `smi_stream_dev`
module after the host moved from kernel `6.18.50+rpt-rpi-v8` to
`6.18.60+rpt-rpi-v8`. Rebuilding, installing and loading the matching module
restored `/dev/smi`; backend initialization then passed without reprogramming
the EEPROM. See the [driver recovery guide](../../driver/README.md#missing-devsmi-after-a-kernel-update).

This establishes blank-device initialization, readback and post-reboot
recognition on this host. Restoration and a general production procedure
remain unvalidated. Subsequent [RX/TX and clock checks](../u25-tx-timing-diagnosis-2026-11-06.md)
record the separate U25 frequency mismatch on this board.

# License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
