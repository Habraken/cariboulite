# Raw SMI RX framing diagnostic

This opt-in diagnostic captures bytes returned by normal SMI RX reads before
framing validation. It requires no FPGA firmware change and no public structure
layout change. It does not change framing acceptance, retries, or decoded data.

Set `CARIBOULITE_SMI_RAW_CAPTURE` to an absolute, new file path in the backend's
process environment **before starting the backend**. If using sudo, pass the
variable through `sudo env`, for example:

```sh
sudo env CARIBOULITE_SMI_RAW_CAPTURE=/tmp/board4-led1-high.bin \
  build/radio-backend/cariboulite_radio_backend <usual backend arguments>
```

Use the normal controller to set the LEDs while stopped, start RX, observe the
result, stop RX, and shut down the backend cleanly. The file is written during
SMI device close. A crash or forced kill can leave the initially empty file
without captured data. Stderr reports whether arming/saving succeeded. A capture
failure does not fail radio initialization or RX.

The first 1 MiB of incoming bytes is retained in preallocated memory, including
bytes the decoder later rejects. Every positive normal RX read is appended once,
regardless of how many decoder calls consume it. No disk I/O or allocation occurs
in the capture receive hook. Loopback/debug read paths are excluded. Memory copies
can affect timing, so compare with the diagnostic disabled if behavior changes.
There is one capture owner per process, consistent with the single SMI device.
Existing files and symlinks are refused; files are created with permissions 0600.
No environment variable means no allocation or file activity.

For comparison, start a fresh backend and use a fresh path for each LED1 state:
0, then 1, then 0. Keep LED0 and other settings fixed, start no TX, and restore
LED1=0 afterward. Starting multiple RX sessions in one process concatenates
reads without session markers; use one RX session per capture. The first-byte
position is arbitrary. This is a startup capture, not a rolling pre-trigger
capture of failures occurring after the buffer fills.

Analyze each saved file:

```sh
python3 software/libcariboulite/tools/analyze_smi_raw.py \
  /tmp/board4-led1-high.bin > /tmp/board4-led1-high.json
```

JSON includes SHA-256, all four byte phases, per-framing-bit mismatch counts,
valid-word totals and longest runs, bit occupancies, stuck-bit candidates,
adjacent repetitions, zero/all-one words, and byte/halfword-order hypotheses.
The checked mask is `0xC001C000`, expected `0x80004000` (bits 14,15,16,30,31).
Bit counts are phase-dependent: interpret them together with alignment evidence.
Legitimate payloads may contain constant bits or repeated words. Aggregate valid
runs do not reproduce the decoder's acquisition-window boundaries or prove that
it should have locked. Captures alone cannot locate corruption before versus
after the FPGA FIFO, or identify an electrical cause.

Hardware-free regressions:

```sh
python3 software/libcariboulite/tests/test_smi_raw_capture.py
python3 software/libcariboulite/tests/test_smi_rx_stream.py
```
