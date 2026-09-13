# CaribouLite code review — 2026-09-12

Reviewed revision: `6236ae1` on `dev`. Host: Raspberry Pi 4 Model B,
Linux `6.18.39+rpt-rpi-v8`, GCC 14.2.0.

The application and kernel module build successfully, but several error paths,
streaming APIs, and firmware build dependencies need correction before treating
this revision as a reproducible reference for five new boards. These findings
do not contradict the owner's working RX/TX setup: many involve alternative
APIs, partial failures, or rebuilding firmware.

This is a broad source and build review, not an exhaustive line-by-line audit or
RF validation. Detailed inspection concentrated on the application pipelines,
SMI kernel/userspace transport, C/C++ and Soapy streaming interfaces, FPGA HDL,
SPI teardown, and build/install scripts. GNU Radio integration and supporting
data structures received targeted inspection. Chip register tables, EEPROM
formats, production tooling, bundled third-party code, and examples have not
received exhaustive functional verification. Manufacturing files were outside
this code review's scope.

P1 means fix before depending on the affected path; P2 means a concrete
correctness/reproducibility issue to address next. Findings below are ordered
by priority, with source locations referring to the reviewed revision.

## Findings

### 1. P1 — Multiple opens can leave the kernel using freed FIFO memory

`driver/smi_stream_dev.c:1261–1323`

Every `open()` allocates buffers into the same global `inst` and reinitializes
its FIFOs. There is no exclusive-open guard or per-file ownership. Opening a
second client replaces the first client's buffers; closing either client frees
the current buffers even while the other descriptor remains usable. Subsequent
reads/writes through that descriptor can access freed kernel memory. Opening
while DMA runs also reinitializes FIFO state before stopping the old stream.

Reject concurrent opens with `-EBUSY`, or implement explicit shared ownership
and serialize buffer lifetime with stream shutdown. Source-confirmed; a
multi-client test was deliberately not run against the live driver.

Update 2026-09-13: implemented exclusive open using a mutex and ownership flag.
The mutex covers buffer allocation and final release; another open returns
`-EBUSY` before touching shared state. Allocation failures release the mutex
and free any partial allocation. Final release stops streaming and frees the
buffers before allowing a new owner. Duplicated/inherited descriptors share
the same open file description and retain ownership until its final release.

Validation: `python3 driver/tests/test_exclusive_open.py` passed. It compiles
the actual open/release function bodies with mocked kernel services and checks
second-open rejection without changing the owner's buffers or stream,
both allocation failure paths and recovery, exclusion during allocation and
cleanup, and 100 rounds of 16 concurrent opens with exactly one winner.
A fresh module build in `/tmp/cariboulite-issue1-driver` passed for
`6.18.39+rpt-rpi-v8`. The new module has not been installed or loaded;
live exclusive-open and post-change radio validation remain pending.

Live follow-up 2026-09-13: the owner loaded the new module (sysfs source version
`12F9D2980B1137B92139E7F`) and started a second test application while the first
was transmitting through option 11. The second application's log confirms
`/dev/smi` open failed with `Device or resource busy`. However, the owner
reported TX stopped and the first application appeared hung. Startup reaches
`cariboulite_setup_io()` before attempting SMI open; that function drives the
modem and mixer reset pins low. Thus the kernel guard rejects the second
client too late to prevent application-level hardware interference. An early
ownership claim before any hardware setup, retained through hardware cleanup,
is needed in the library as well. The exact hang mechanism is not established.
Captured logs are local under `installations/issue1-validation/`.

Follow-up implementation: library initialization now claims `/dev/smi` before
signal registration, board detection, GPIO setup, or FPGA communication.
SMI initialization duplicates this descriptor with close-on-exec semantics;
it does not perform a second open. The ownership descriptor stays open until
hardware cleanup finishes, including initialization failure paths. SMI close
explicitly stops streaming because closing its duplicate alone no longer
invokes the kernel's final release. Production/minimal initialization paths
also require the SMI device to be available for this ownership claim.

Validation of the library follow-up:

- Complete local application/library/Soapy build passed.
- `python3 software/libcariboulite/tests/test_early_ownership.py` passed against
  the rebuilt application with interposed device/GPIO calls: busy rejection
  makes no GPIO setup call; injected GPIO setup failure releases ownership.
- The kernel open/release regression tests still pass.
- Live test with `/dev/smi` held open: the rebuilt application exited before
  GPIO setup with "hardware setup skipped". A duplicated descriptor retained
  ownership after closing the original; reopen succeeded after final close.
- The rebuilt single application reached the menu and quit with exit code 0.
  No transmit option was selected during these follow-up checks.

Use the rebuilt `build/cariboulite_test_app` for both sessions in the next
TX regression check, with separate stderr log files. Installed libraries under
`/usr/local` have not been replaced; older applications/libraries can still
touch hardware before their SMI open is rejected. The original two-application
TX scenario and reported hang had not yet been revalidated at that point.

Owner validation 2026-09-13, 16:14 local time: the repeated two-session TX
test passed. The first application logged TX activation at 16:14:27.954.
At 16:14:42.110 the second application was rejected with `Device or resource
busy; hardware setup skipped`, with no subsequent hardware initialization.
The first application logged TX deactivation at 16:14:52.392, then a normal
menu quit and completed driver release at 16:14:57.399. The owner confirmed
the test worked; the first log contains no error or warning entries.
Preserved logs: `installations/issue1-validation/passed-first.log` and
`passed-second.log`. Issue 1 is implemented and validated for this scenario;
permanent installation and committing the changes remain separate steps.

Permanent driver installation completed 2026-09-13 for the running kernel
`6.18.39+rpt-rpi-v8` only. Installed the tested compressed module at
`/lib/modules/6.18.39+rpt-rpi-v8/kernel/drivers/char/broadcom/smi_stream_dev.ko.xz`
and successfully ran `depmod -a 6.18.39+rpt-rpi-v8`. Verified the installed
bytes against the tested module and source version against the already loaded
module (`12F9D2980B1137B92139E7F`). `modprobe --show-depends` resolves the base
SMI dependency and existing parameters `6, 2, 3`. Boot module loading was
already configured. No reload or reboot was performed.
Rollback copy and installation hashes are under `installations/issue1-driver/`.
Installed userspace libraries remain unchanged; use the rebuilt local app.

### 2. P1 — RX destruction uses an uncreated or already joined thread

`software/libcariboulite/src/app_menu.c:1987–2044`

RX initialization starts audio and demodulation workers, but creates the reader
only on RX start. Destroy nevertheless unconditionally cancels and joins the
reader. Entering the RX menu and quitting without starting RX therefore uses
the zero-initialized thread handle. After RX has run, `rx_pipeline_stop()` joins
the reader and destroy cancels/joins the same handle again. These operations
have invalid thread lifetime assumptions and can crash or target a reused
handle. Reader allocation and `pthread_create()` failures are also unchecked.

Track successful creation for each worker, clear that state after join, and
make stop/destroy safe after any partial initialization. Thread cancellation
also needs cleanup handlers for FIFO mutex ownership and reader metadata.

### 3. P1 — C++ synchronous reads can overflow internal buffers

`software/libcariboulite/src/CaribouLiteRadioCpp.cpp:108–126, 178–194`

The constructor allocates one MTU of sample and metadata storage. `ReadSamples`
passes unrestricted `num_to_read` to the C reader using that storage. The C
reader processes the requested length in chunks; it does not cap the total to
one MTU. An oversized request writes beyond both internal allocations. The
float overload uses the same path. GNU Radio's `work()` forwards
`noutput_items` without applying its stored MTU limit.

Clamp the request, resize the buffers, or process bounded chunks. An isolated
test of the actual wrapper body with a mock reader and a protected guard page
reproduced SIGSEGV for a five-sample request into four-sample internal storage.
No board access was involved.

### 4. P1 — Kernel TX transition takes a sleeping mutex under a spinlock

`driver/smi_stream_dev.c:359–375`

`set_state()` holds `state_lock` while acquiring `write_lock` with
`mutex_lock_interruptible()`. The write path holds that mutex across
`kfifo_from_user()`, which can fault/sleep. A concurrent TX transition can
therefore sleep in atomic context. Moving DMA shutdown outside the spinlock
has not removed this remaining problem.

Serialize the whole transition with a process-context mutex and use spinlocks
only for short state updates that cannot sleep. Also serialize concurrent
ioctl transitions across stop/start, not just the final state assignment.
Source-confirmed; no live contention test was run.

### 5. P1 — TX stop can wait forever before disabling transmission

`software/libcariboulite/src/app_menu.c:1762–1774, 1844–1862, 2758–2762`

Stopping TX first waits for the DSP worker to consume injected tail-tone
frames. That wait has no deadline or worker-failure check. If the TX writer
has set `nbfm_tx_active=false` after a hard error, the producer stops consuming
injections, but the pipeline's `running` flag remains true. A subsequent stop
or quit waits forever. A producer blocked on audio capture or a full TX FIFO
can cause the same problem. Hardware shutdown occurs only after that wait.

Give tone injection a bounded wait and a failure/cancellation path; always
complete hardware shutdown even when the tail tone cannot be delivered.

### 6. P1 — Partial streaming writes are omitted from the returned count

`software/libcariboulite/src/caribou_smi/caribou_smi.c:1086–1128`

`caribou_smi_write_samples()` tracks progress inside the current chunk in
`off`, but four subsequent timeouts jump directly to `done`, bypassing the
update of `consumed_samples`. The hard-error path has the same accounting
problem. The caller retries samples already queued to DMA, corrupting the
sample sequence under backpressure.

A mock-I/O test using the actual function accepted two samples and then
returned four timeouts: the function reported zero samples consumed. Include
current-chunk progress in every return path and retain any partial-byte state.

### 7. P2 — Public TX API inserts samples and reports more than requested

`software/libcariboulite/src/caribou_smi/caribou_smi.c:903–981`

The public C radio API calls `caribou_smi_write()`, which pads every short DMA
quarter with repeated samples and counts the padding as caller data consumed.
This changes the waveform and violates the write-count contract. C++ and
Soapy callers inherit it; the custom app's newer writer uses a different
function and is affected by finding 6 instead.

With a mocked 64-byte native batch, requesting one sample emits four and
returns four. Preserve tails between calls, or support arbitrary aligned
writes, and return only the caller's samples actually accepted.

### 8. P2 — FIFO timeout deadlines use the wrong clock

`software/libcariboulite/src/app_menu.c:943–984, 1232–1234, 1291–1346`

Audio and RF condition variables are initialized with default attributes
(realtime clock), while their timed waits construct absolute deadlines from
`CLOCK_MONOTONIC`. The deadlines are already expired in the realtime clock,
so full/empty FIFO waits return immediately instead of honoring the requested
timeout. Infinite waits are unaffected.

An extracted audio-FIFO test requested 100 ms and returned in approximately
0.060 ms. Set the condition-variable clock to monotonic or calculate deadlines
using its actual configured clock.

### 9. P2 — Partial pipeline initialization is not unwound

`software/libcariboulite/src/app_menu.c:1749–1755, 1921–1943, 3113–3114`

TX and RX initialization return directly on thread-creation failures after
allocating resources and potentially starting earlier workers. `inited` is
still false, so the normal destroy routines do not clean up. In the standalone
menus, returning after such a failure can leave a worker referencing a pipeline
object that was on the caller's stack. The combined monitor ignores both init
return values and proceeds to display statistics from possibly destroyed FIFOs
when opening its hard-coded ALSA devices fails.

Use staged cleanup and check both initialization results before entering the
monitor. Exercise missing ALSA devices and injected thread/allocation failures.

### 10. P2 — FPGA TX configuration outputs are disconnected

`firmware/sys_ctrl.v:18–19, 57–59, 72–85`; `firmware/top.v:168–169, 449–451`

`debug_loopback_tx` and `tx_sample_gap` are written and, for the gap, readable
over SPI, but never assigned to their corresponding output ports. The LVDS
transmitter consumes those undriven ports. Software can read back a changed
gap register without that value reaching the transmitter.

Yosys confirms undriven `i_sample_gap` and `i_debug_lb` in the flattened design.
Connect the registers to the outputs and verify gap/loopback behavior in HDL
tests before programming hardware.

### 11. P2 — LVDS TX reset refers to an undefined state

`firmware/lvds_tx.v:76`

Reset assigns `r_state <= INIT`, but the defined states are `IDLE`, `TX_FRAME`,
`TX_GAP`, and `LOOPBACK`. Verilog implicitly creates an undriven `INIT` wire.
Yosys reports both the implicit wire and a nonconstant asynchronous reset.
The default case may eventually recover, but the reset state is not defined
as intended. Use a declared reset state and enable checks for implicit nets.

### 12. P2 — Editing HDL does not trigger rebuilding the bitstream

`firmware/Makefile:5–17`

The `top.bin` target has no prerequisites. When it exists, changing an included
HDL file or `io.pcf` does not rebuild it; `build` can then regenerate and copy
a header from the stale binary. A dry run with `make -n -W lvds_tx.v top.bin`
reported that the binary was up to date.

Declare HDL/constraint dependencies and separate synthesis, routing, packing,
and header generation with proper dependencies. Routing is additionally piped
through `tee` without preserving its failure status, and uses
`--timing-allow-fail`; a nominally successful build is not timing acceptance.

### 13. P2 — SPI close misinterprets timed locking and destroys a locked mutex

`software/libcariboulite/src/io_utils/io_utils_spi.c:346–361`

The timeout is the absolute timestamp one second after the Unix epoch rather
than one second from now. The code checks for negative pthread error values,
although pthread errors are positive. If another thread holds the mutex,
timeout is ignored and resources can be freed while in use. If locking succeeds,
the mutex is destroyed while still locked.

Coordinate shutdown with users, use a valid absolute deadline and correct error
checks, then release ownership before destroying the mutex.

### 14. P2 — Soapy synchronous streaming ignores timeout and inactive state

`software/libcariboulite/src/soapy_api/CaribouliteStream.cpp:192–209, 290–309`

With `USE_ASYNC` disabled, reads/writes delegate directly to C functions without
using `timeout_us` or checking stream activation. Negative read errors are
converted to zero. This does not implement the timeout/error behavior documented
in the adjacent Soapy stream interface and can make callers spin or wait beyond
their requested deadline.

Implement deadlines and inactive-stream handling and translate transport failures
into the appropriate Soapy errors. Add mock transport tests for zero/short
timeouts and inactive streams.

### 15. P2 — Optional Soapy dependency removes the main application targets

`software/libcariboulite/CMakeLists.txt:98–102, 148–199`

When SoapySDR is missing, the top-level `return()` also skips creation of
`cariboulite_test_app`, utilities, and library installation rules. Configuration
only warns that Soapy support is being skipped, so this is misleading on a
fresh machine intended to run the standalone app.

Make only the Soapy module conditional. Keep the app, utilities, and library
installation independent of that optional dependency.

### 16. P2 — Driver installer copies one kernel's module into other kernels

`driver/install.sh:50–72`

The build targets the running kernel, but installation searches all of
`/lib/modules` for base SMI modules and copies the same generated `.ko.xz` into
every matching directory. Hosts retaining multiple kernels receive a module
with the wrong version in the other kernels' trees.

Resolve the destination strictly within the kernel version used for compilation.
Also stop immediately on compilation/packaging failures before changing module
blacklists and boot-loading configuration. Installation was reviewed, not run.

## Verification performed

- Fresh C/C++ configure and complete default build in
  `/tmp/cariboulite-review-build`: passed, including app, static/shared libraries,
  utilities, and Soapy module.
- Fresh kernel module build in `/tmp/cariboulite-review-driver`: passed and
  produced `smi_stream_dev.ko` for the running kernel. It was not installed.
- Yosys hierarchy/process checks with iCE40 primitive definitions, and actual
  `synth_ice40`: completed, with the warnings described above. Synthesis later
  optimizes undriven nets away; its final zero-problem check does not invalidate
  those earlier diagnostics. Place-and-route/timing closure was not performed.
- Parsed all 30 tracked Python/shell scripts with Python AST or `bash -n`:
  no syntax errors. This does not test script behavior or dependencies.
- `ctest -N` in the fresh main build: zero registered tests.
- Isolated source-body reproductions: FIFO timeout mismatch, public TX count
  inflation, streaming TX partial-count loss, and C++ oversized-read fault.
  Harnesses and binaries are under `/tmp/cariboulite-review-checks`.
- AddressSanitizer could not initialize its address space in this execution
  environment. The C++ boundary test instead used an OS-protected guard page
  and a mock radio read; it terminated with SIGSEGV as expected.
- Build-time `SoapySDRUtil --info` loaded the installed plugin and printed a
  missing `/dev/gpiomem` message in this tool environment. That is not evidence
  that the owner's board/setup is broken. No RF application was intentionally
  launched, no transmission was requested, and no firmware was programmed.

## Next work

First preserve the exact working bitstream and installed software/module hashes.
Then fix lifecycle and transport accounting in small changes, verifying each
against the reference setup. Add automated tests for the failures above before
refactoring the large app menu. Treat FPGA wiring/build fixes as a separate
change requiring HDL tests and subsequent board validation.

Further hardware-focused checks should cover repeated RX/TX switching, clean
quit before first RX, missing audio devices, backpressure, restart without power
cycling, both radio paths, and sustained streaming. The source also contains
historical code and control/register assumptions that need comparison against
the actual working bitstream; this review does not certify RF register values,
clock-domain timing, or spectral performance.

Only this review document was added to the repository. Functional source,
existing build outputs, branch history, and hardware configuration were not
changed.
