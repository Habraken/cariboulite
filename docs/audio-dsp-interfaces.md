# Audio and DSP module interfaces

Living reference for the [incremental refactoring plan](audio-dsp-refactor-plan.md).
Status: current inventory plus proposed contracts; proposed APIs are not implemented.
Update this file with every interface-changing increment.

## Current implementation

| Module | Inputs / outputs | Current coupling |
| --- | --- | --- |
| `alsa_source.c/.h` | ALSA capture -> mono float audio at 48 kHz | Owns capture handle, conversion and buffering; used by TX |
| `audio48k_source.h`, `tone48k.c` | Frequency/amplitude -> 48 kHz sine samples | Standalone generator compiled into app; current app generates tones elsewhere |
| `nbfm_mod.c/.h` | Float audio -> packed signed 16-bit I/Q pairs | Configurable audio/RF rates; app uses 48 kHz audio and 2 or 4 MS/s IQ |
| `nbfm_demod.c/.h` | Application IQ FIFO -> application audio FIFO | Owns DSP inside a thread; references ALSA for diagnostics, FIFO depth for clock correction |
| `app_pipeline_internal.h` | Shared RF/audio frame and FIFO types | Internal app/demodulator transport; not a reusable DSP API |
| `app_menu.c` | UI, radio control, audio and IQ streams | Owns pipeline coordination, FIFO implementations, ALSA playback and tone generation |

The modulator exposes create/destroy, push-audio and pull-IQ operations. The
current demodulator exposes a pthread entry point and a mutable control struct;
these are not yet symmetrical standalone DSP interfaces.

## Target data paths

```text
TX pipeline: audio_source.read -> nbfm_mod.process -> radio write
RX pipeline: radio read -> nbfm_demod.process -> audio_sink.write
```

`alsa_source` and `tone_source` implement the source contract. `alsa_sink`
implements the sink contract. Memory, file or network adapters can follow later.
Pipeline workers own pacing, queues, hardware-specific sample conversion and
coordination; DSP owns signal-processing state only.

## Proposed boundary contracts

These rules guide implementation. Record exact C declarations and supported
values here when each interface lands; do not treat these as existing guarantees.

### Audio source and sink

- Describe sample rate in Hz and channel count explicitly. Initially support
  mono float audio at exactly 48 kHz; reject unsupported configurations.
- Use nominal normalized float audio with a documented scale. Preserve current
  capture gain and TX amplitude. Define conversion/clipping at the sink when
  moving the current signed-16-bit RX audio output to the shared float format.
- Counts are audio frames, not bytes; one frame contains one sample per channel.
- Caller owns sample buffers. A read fills a caller buffer; a write consumes a
  caller buffer. Adapters must not retain that pointer after returning.
- Report actual transfer count separately from status. Specify partial transfer,
  end-of-stream, timeout/would-block, stopped and fatal-error behavior explicitly.
- Specify blocking and interruption behavior so shutdown has a bounded path.
- One pipeline worker owns each adapter. Close after its worker has stopped;
  document any operation explicitly allowed from the control thread.

### Modulator and demodulator

- Explicit configuration includes audio rate, IQ rate, deviation and applicable
  filtering/gain parameters. Document supported values and defaults, with units.
- Processing accepts input length and output capacity, and returns consumed and
  produced counts plus status. Buffered input/output and no-progress conditions
  must be documented so callers cannot spin or silently drop samples.
- Preserve state between blocks. Define reset separately from destruction and
  document which filter, phase and resampling states it clears.
- Use a shared IQ sample type with explicit I/Q order, signedness and scale.
  Initially preserve existing signed-16-bit IQ values; hardware control bits
  belong in radio transport conversion, not in generic DSP output.
- No ALSA handles, application FIFOs, radio access, UI calls or thread scheduling
  in DSP. Configuration updates and processing have one serialized owner.
- Allocate persistent storage at creation; document any processing-time allocation.
- The RX pipeline measures output-buffer fill and supplies clock correction.
  Preserve existing correction limits and timing during extraction; document the
  correction's sign and units. The demodulator need not know the destination.

### Pipeline coordination

- Own worker threads, transport queues, backpressure and radio activation order.
- Connect configured source/sink to a modem; validate compatible formats before
  starting hardware. No implicit arbitrary-rate or multichannel support.
- Distinguish requested stop, adapter failure and hardware failure. Define how
  blocked I/O is released, workers are joined and resources are destroyed.
- Preserve current restart/retune behavior and tested TX shutdown deadlines.
- Publish status for the UI without exposing mutable DSP internals.

## Documentation required for each implemented interface

For each module, add: header and implementation links, exact API declarations,
configuration/defaults, sample format and units, buffer/state ownership, thread
rules, partial-transfer/error semantics, reset/stop behavior, dependencies and a
short caller example. Link its software checks and physical checkpoint evidence.

## Open decisions to settle incrementally

- Exact transfer-result types and how stop interrupts blocking adapters (step 2).
- Tone phase behavior when switching sources or injecting a temporary tone (step 3).
- Sink normalization that preserves current demodulator PCM gain (steps 4–5).
- DSP output capacity, buffering and correction API (steps 5–6).
- Common modem operations required by a second actual implementation (step 8).

Choose each based on the existing behavior and tests at that step; avoid bundling
new audio features or DSP algorithm changes into the extraction.

## Implemented: ALSA capture adapter (step 1)

[Header](../software/libcariboulite/src/alsa_source.h) and
[implementation](../software/libcariboulite/src/alsa_source.c).
Step 1 renames the module and API only; it does not implement the proposed
common audio-source contract above or add configurable sample rates.

```c
typedef struct alsa_source alsa_source_t;
alsa_source_t* alsa_source_create(const char* device, float gain);
size_t alsa_source_read(alsa_source_t* s, float* dst, size_t max_frames);
void alsa_source_destroy(alsa_source_t* s);
```

- Creation opens blocking ALSA capture; a null device selects `default`.
  Configuration remains exactly 48 kHz mono, signed 16-bit little-endian input.
  Creation returns null on initialization failure.
- Output is mono float audio: input divided by 32768, multiplied by `gain`,
  and clipped to [-1, 1]. Counts are audio frames (one float per frame).
- The caller owns `dst`; the adapter writes at most `max_frames` samples and
  does not retain the pointer. Reads return the actual count, possibly short or
  zero. Errors are not reported separately from lack of samples.
- Reads drain the internal ring and attempt up to eight capture calls as needed.
  Blocking ALSA reads and suspend recovery mean this is not a guaranteed wall-clock
  timeout. Overflow drops the oldest buffered audio.
- One worker reads a source. The implementation also shares a static conversion
  buffer across instances, so concurrent reads from different instances are not
  supported. Stop/join the reading worker before destruction; destroy accepts null.
- The adapter owns the PCM handle and capture/ring storage. It depends on ALSA
  and the C runtime, not radio or modem code. There is no public reset operation.

Typical caller (inside the owning audio worker):

```c
alsa_source_t* source = alsa_source_create("plughw:Loopback,1,1", 1.0f);
if (source) {
    float audio[480];
    size_t frames = alsa_source_read(source, audio, 480);
    /* Feed only `frames` valid samples to the modulator. */
    alsa_source_destroy(source);
}
```

Validation: application build and `test_rx_lifecycle.py`; the implementation is
unchanged apart from names. The [step 1 physical retest](baselines/20260919T075122.967948Z/summary.json)
also completed successfully; Jan confirmed all tones at the correct pitch.
