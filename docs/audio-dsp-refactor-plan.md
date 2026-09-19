# Audio and DSP refactoring plan

Status: H0 accepted by Jan on 2026-09-19; implementation has not started. Each numbered step is a small,
reviewable change. The existing `nbfm_mod` rename and `nbfm_demod` extraction are
already complete; the demodulator still depends on application FIFOs and ALSA.

Target paths:

```text
alsa_source (or tone_source) -> audio -> nbfm_mod -> IQ -> radio
radio -> IQ -> nbfm_demod -> audio -> alsa_sink
```

Interface reference: [Audio and DSP interfaces](audio-dsp-interfaces.md).
Update that reference in the same change as each interface modification. Mark
proposed contracts separately from implemented ones. Do not change modulation,
filtering, audio levels, defaults or radio routing as part of a mechanical move.

## Working rhythm

For every step: inspect current code, make the smallest change, update interface
documentation, build, run relevant software checks, and review the diff. At each
hardware checkpoint below, stop before the next implementation step until Jan
has confirmed the physical result (or explicitly deferred the checkpoint).
Record failures and resolve them before proceeding. Commit accepted increments
individually when requested; do not combine several checkpoints into one change.

Software checks currently available:

```sh
cmake --build build --target cariboulite_test_app -j2
python3 software/libcariboulite/tests/test_nbfm_rate.py
python3 software/libcariboulite/tests/test_rx_lifecycle.py
python3 software/libcariboulite/tests/test_tx_stop_deadline.py
```

Add focused checks when a new boundary creates a meaningful failure mode. These
checks do not establish physical RF performance.

## Steps

### 0. Record the physical baseline

- [ ] Record revision, Pi model, Raspberry Pi OS/kernel, FPGA image, ALSA device
  names, RF modem/connector route, frequency, gains, and the actual RF test setup.
- [x] Accept H0 from Jan's clean rebuild and physical radio test (2026-09-19).
  This is a functional baseline; see the evidence log for its recorded scope.
- [ ] Resolve documentation/configuration mismatches in the recorded procedure.
  In particular, current monitor initialization selects `radio_high` (RF24),
  while some existing documentation describes RF09. Do not silently change it.

Exit: a reproducible baseline with known limitations and measured comparisons.

### 1. Name the existing ALSA capture adapter clearly

- [ ] Rename `alsa48k_source.c/.h` and its symbols to `alsa_source`.
- [ ] Keep existing 48 kHz mono capture behavior and configuration unchanged.
- [ ] Update callers, tests, build inputs and interface reference.

Checks: build and lifecycle tests; no separate physical checkpoint required.

### 2. Introduce the audio-source boundary

- [ ] Define a small `audio_source` interface with explicit format, read result,
  buffer ownership, and close semantics; adapt ALSA capture to it.
- [ ] Preserve the existing read/stop behavior; test partial reads and failures.
- [ ] Keep supported audio at 48 kHz mono initially; reject unsupported formats.

**H1:** verify real ALSA/loopback audio TX at both RF rates and repeated TX stop/start.

### 3. Consolidate tone generation

- [ ] Implement `tone_source` through the same audio-source interface.
- [ ] Route normal TX tone generation through it, preserving amplitude and phase.
- [ ] Then migrate self-test tone generation and transient tone/silence injection,
  preserving override priority, duration, phase continuity and shutdown deadlines.
  Split these callers into separate commits if needed.
- [ ] Remove the unused old tone interface only after all callers are migrated.
- [ ] Test tone pitch, amplitude, continuity across blocks and source switching.

**H2:** compare normal tone TX, self-test audio, injected tones/silence and shutdown
with H0; also confirm ALSA audio still works.

### 4. Extract ALSA playback without changing the audio pipeline

- [ ] Move playback open/configure/write/recovery/close into `alsa_sink.c/.h`.
- [ ] Introduce `audio_sink` and route playback through it.
- [ ] Keep worker threads, queues and their sizes in their existing owner for now.
- [ ] Test partial writes, xrun recovery, failure cleanup and stop behavior.

**H3:** verify physical RX audio at both RF rates, including restart and retune.

### 5. Separate demodulator DSP from its worker thread

- [ ] Create explicit NBFM DSP state with create/process/reset/destroy operations.
- [ ] Move FIFO access, ALSA diagnostics, scheduling and thread control into a
  pipeline worker; remove ALSA/FIFO dependencies from the DSP header and source.
- [ ] Preserve filtering, resampler state, reset semantics and output levels.
- [ ] Let the worker measure FIFO depth and provide the existing clock correction;
  keep the fractional resampler in the DSP for this increment.
- [ ] Compare deterministic IQ-to-audio output before/after at 2 and 4 MS/s,
  including varying input blocks, output capacities, reset and correction values.

**H4:** run the full physical comparison, especially sustained RX audio and FIFO
stability. Do not claim clock synchronization from a short tone test alone.

### 6. Make the modulator boundary explicit

- [ ] Document and normalize configuration, reset, processing progress, errors,
  buffer lifetime and supported rates in `nbfm_mod`.
- [ ] Establish explicit audio and IQ formats shared with the demodulator.
- [ ] Preserve IQ scale and hardware-specific TX bit packing at the radio boundary.
- [ ] Test consumed/produced counts, small buffers and block continuity.

**H5:** verify ALSA and tone TX at both RF rates and the modem self-test.

### 7. Move pipeline coordination out of the menu

- [ ] Extract shared FIFO implementation into an internal transport module.
- [ ] Extract TX lifecycle and worker coordination into `tx_pipeline.c/.h`.
- [ ] Extract RX lifecycle and worker coordination into `rx_pipeline.c/.h`.
- [ ] Keep each extraction separately reviewable; run lifecycle/stop tests after each.
- [ ] Leave configuration selection and status display in `app_menu.c`.

**H6:** full physical comparison, repeated start/stop, rate changes while stopped,
retuning and sustained audio. Confirm menu exit releases resources.

### 8. Prove interchangeability and finish the documentation

- [ ] Exercise DSP with memory-backed audio source/sink implementations without
  ALSA or hardware, reusing the documented sample contracts.
- [ ] Document module dependencies, error paths, threading, supported formats,
  ownership and an example showing how a new source/sink is connected.
- [ ] Document how a future modem plugs into the pipelines. Introduce a shared
  modem operations interface with the second actual modem, unless an earlier
  increment demonstrates a concrete need for one.

Exit: clear source -> modulator and demodulator -> sink paths; no ALSA, UI or radio
control dependencies inside DSP; physical evidence tied to the final revision.

## Physical checkpoints

H0 and H6 use the full procedure. Intermediate checkpoints use the indicated
subset, always with the same recorded setup and newly built binary.

1. Confirm the intended FPGA image, modem/connector route and ALSA routes.
2. At 4 MS/s, send a known audio tone and real audio through TX; observe the RF
   with the recorded receiver/analyzer. Record pitch, level and audible quality.
3. Feed a known RF signal into RX and observe/record ALSA output. Check pitch,
   level, intelligibility and continuity. Repeat TX and RX at 2 MS/s.
4. Stop streaming before changing sample rate. Repeat start/stop five times and
   retune RX; check for hangs, stuck streaming, missing audio and new errors.
5. Exercise menu self-test and monitor controls used in the baseline. Digital
   interface loopback is useful additional evidence, but does not verify RF audio.
6. For full checkpoints and H4, run RX and TX separately for at least five minutes
   per rate; observe underruns/overruns, FIFO depth and drift where reported.
7. Exit normally and restart the app; verify devices can be reopened.

Agree baseline tolerances for pitch, level and stop time at H0, using available
instrumentation. Later results must remain within those tolerances, with no new
hangs, dropouts or errors. Record pre-existing failures as such; a failed baseline
case is not a pass. Do not mark an untested combination as verified.

## Evidence log

| Checkpoint | Revision / dirty diff | Setup / procedure | Measurements and result | Confirmed by |
| --- | --- | --- | --- | --- |
| H0 | `0af6f2d`; working tree clean when recorded | Raspberry Pi OS; Jan deleted the build folder, rebuilt the app using the install script, and physically tested with a radio | Jan reports everything works as expected; accepted functional baseline, no numerical measurements supplied | Jan, 2026-09-19 |
| H1–H6 | Pending | Pending | Not run | — |

Link longer logs or recordings here. Record software and hardware results
separately, including any explicitly deferred tests and remaining limitations.

### H0 acceptance notes

Jan accepted this as a good baseline after the clean rebuild and physical test.
This satisfies the user-confirmation gate for beginning step 1. It does not
establish that every item in the proposed full H0 procedure was exercised.
Specific tested sample rates, TX/RX cases, duration, restart counts, hardware and
firmware details, ALSA routes and numerical tolerances were not supplied. The
remaining baseline checklist items track documentation follow-up, not a request
to repeat the accepted test before step 1. Record those details when available
and use Jan's working setup for subsequent physical comparisons.
