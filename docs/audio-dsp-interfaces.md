# Audio and DSP module interfaces

Living reference for the [incremental refactoring plan](audio-dsp-refactor-plan.md).
Status: current inventory plus proposed contracts; proposed APIs are not implemented.
Update this file with every interface-changing increment.

## Current implementation

| Module | Inputs / outputs | Current coupling |
| --- | --- | --- |
| `audio_source.h` | Format + read-result + destroy operations | Common source boundary; currently used by ALSA TX capture |
| `alsa_source.c/.h` | ALSA capture -> mono float audio at 48 kHz | Owns capture handle, conversion and buffering; used by TX |
| `tone_source.c/.h` | Frequency/amplitude -> 48 kHz mono float audio | Common source implementation for TX, injection and self-test tones |
| `nbfm_mod.c/.h` | Float audio -> packed signed 16-bit I/Q pairs | Configurable audio/RF rates; app uses 48 kHz audio and 2 or 4 MS/s IQ |
| `audio_sink.h`, `alsa_sink.c/.h` | Mono S16 PCM at 48 kHz -> ALSA playback | Sink owns PCM configuration, stereo fallback, recovery and close |
| `nbfm_demod.c/.h` | IQ16 -> mono S16 PCM | Standalone stateful DSP; caller supplies fractional rate correction |
| `demod_worker.c/.h` | Application IQ FIFO -> application audio FIFO | Owns 480-frame packing, FIFO-depth servo, diagnostics and thread entry |
| `iq16.h` | Packed signed 16-bit I then Q | Shared existing IQ layout; no device dependencies |
| `app_pipeline_internal.h` | Shared RF/audio frame and FIFO types | Internal app/demodulator transport; not a reusable DSP API |
| `app_menu.c` | UI, radio control, audio and IQ streams | Owns pipeline coordination, FIFO implementations and source/sink workers |

The modulator exposes create/destroy, push-audio and pull-IQ operations. The
demodulator exposes create/process/reset/destroy and explicit transfer counts.
Their APIs are not yet symmetrical; step 6 addresses the modulator contract.

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

## Implemented: common audio source (step 2)

[Interface](../software/libcariboulite/src/audio_source.h). TX now holds an
`audio_source_t*` and calls `audio_source_read` / `audio_source_destroy`.
ALSA construction remains adapter-specific:

```c
audio_source_t* alsa_source_open(const char* device, float gain,
                                audio_format_t format);
```

The format contains `sample_rate` (Hz) and `channels`. Samples are interleaved
normalized floats, with counts in frames. `alsa_source_open` accepts only
`{48000, 1}`; other formats return NULL with `errno = EINVAL` before opening
hardware. Initialization failures return NULL with `errno = EIO`. The legacy
step 1 API remains available, but app capture no longer calls it directly.

A read returns `{frames, status, error}`. Only `frames` samples per channel are
valid; the caller owns storage and the adapter retains no pointer. `OK` can be a
short read. `AGAIN` means no samples presently available; `EOF` means source end;
`ERROR` includes a negative errno-style code. A final/error result may include
valid buffered frames. ALSA never returns EOF; its unrecovered capture failures
return ERROR with any buffered samples. Zero-length reads succeed without I/O;
invalid source/buffer arguments return ERROR/-EINVAL.

The TX worker retries AGAIN/short reads to fill its existing 480-frame block.
On ERROR or EOF it drops the unfinished block, logs failure and clears the TX
stream-active flag; normal pipeline stop/destroy performs final cleanup.
This avoids indefinite retries after a permanent capture failure. Existing
Quindar generation and normal sample conversion are unchanged.

The source has immutable format and operations pointers; an adapter embeds it
and implements read/destroy. No common factory, audio resampler or source-switch
logic is introduced yet. Step 3 replaces the old `audio48k_source` tone interface with `tone_source`. DSP modules do not depend on this I/O interface.

Blocking behavior is still adapter-specific: ALSA uses the existing blocking
reads, recovery, and worker cancellation. No new timeout or cross-thread stop API
is claimed. One reader owns the adapter; stop/join it before destroy. ALSA's
shared conversion buffer still excludes concurrent reads across instances.

```c
audio_source_t* source = alsa_source_open("plughw:Loopback,1,1", 1.0f,
                                         (audio_format_t){48000, 1});
if (source) {
    float block[480];
    audio_source_result_t r = audio_source_read(source, block, 480);
    /* Consume r.frames; handle r.status before requesting more. */
    audio_source_destroy(source); // after the reader has stopped
}
```

Software checks: `test_audio_source.py` scripts ALSA capture without hardware to
check format rejection, partial/error results, scaling and buffered reads;
application build, lifecycle and TX-stop tests also pass. H1 run `20260919T075807.545875Z` completed successfully. Jan confirmed correct
tones/pitch and microphone modulation at both RF rates. The runner's start/stop
cycles passed; additional manual repeated toggles were not separately reported.
[Physical results](baselines/20260919T075807.545875Z/summary.json).

## Implemented: tone source (step 3)

[Header](../software/libcariboulite/src/tone_source.h) and
[implementation](../software/libcariboulite/src/tone_source.c).

```c
audio_source_t* tone_source_open(float frequency, float amplitude,
                                audio_format_t format);
int tone_source_set(audio_source_t* source, float frequency, float amplitude);
audio_source_t* tone_source_open_cue(float frequency);
```

The regular source accepts 48 kHz mono, finite frequency in [0, 24000) Hz and
amplitude in [0, 1]. Invalid creation arguments return NULL/errno EINVAL;
allocation failures return NULL. `tone_source_set` returns 0 or -EINVAL and
leaves state unchanged on invalid input. It is specific to this adapter;
read/destroy use the common source API.

Reads produce exactly the requested frames with OK, without blocking or
allocating. The caller owns output storage; no pointer is retained. A single
worker owns reading and parameter changes. Creation allocates the oscillator,
and destroy releases it after the worker stops.

Regular tones preserve the former app float phase accumulator: advance phase
before each sample, wrap at 2*pi, emit amplitude*sin(phase). Changing frequency
or amplitude preserves phase. Frequency zero emits silence and freezes phase,
matching the old injection padding. TX's injector retains priority over normal
source selection, and normal microphone capture resumes afterward. The tone
state is allocated with the pipeline and freed after its workers are joined.
Tone restart/reset occurs by recreating the pipeline as before.

The self-test uses the regular source for its 600 Hz modulation. Its direct PCM
opening/closing cues use `tone_source_open_cue`, a compatibility mode preserving
the original indexed sine formula starting at zero phase. Sample index persists
across blocks; 0.6*32767 scaling and signed-16-bit rounding remain at the PCM
boundary. This avoids changing cue samples as part of an extraction.

```c
audio_source_t* tone = tone_source_open(600, 0.4f, (audio_format_t){48000, 1});
if (tone) {
    float block[480];
    audio_source_read(tone, block, 480);
    tone_source_set(tone, 0, 0.4f); // silence without advancing phase
    audio_source_read(tone, block, 480);
    audio_source_destroy(tone);
}
```

The unused `tone48k.c` and `audio48k_source.h` were removed. Tests:
`test_tone_source.py` compares samples with previous formulas across block sizes,
frequency changes, silence and PCM cues; lifecycle and TX-stop tests protect
allocation cleanup and injection deadlines. H2 passed on 2026-09-19: Jan confirmed the runner audio pitch and the separate
option 13 self-test. [Results](baselines/20260919T080542.756197Z/summary.json).

## Implemented: playback sink (step 4)

[Interface](../software/libcariboulite/src/audio_sink.h),
[ALSA header](../software/libcariboulite/src/alsa_sink.h) and
[implementation](../software/libcariboulite/src/alsa_sink.c).

```c
audio_sink_t* alsa_sink_open(const char* device, unsigned sample_rate);
audio_sink_result_t audio_sink_write(audio_sink_t*, const int16_t*, size_t frames);
void audio_sink_destroy(audio_sink_t*);
const char* audio_sink_state(audio_sink_t*);
unsigned alsa_sink_channels(const audio_sink_t*);
```

This incremental boundary deliberately retains **signed 16-bit mono PCM**,
range -32768 through 32767, without scaling, clipping or floating-point conversion.
The shared normalized-float target above remains proposed. Counts are mono input
frames, including when ALSA duplicates samples to stereo. The caller owns input
storage; the sink retains no pointer. `sample_rate` and operations are immutable.

Opening accepts only 48000 Hz, rejects a different negotiated rate, and returns
NULL with errno on failure. NULL/empty device selects `default`. Configuration
preserves blocking interleaved S16_LE playback, mono first with stereo fallback,
480-frame target periods, 2400-frame target buffer, start threshold of buffer
minus period, and availability threshold of one period. ALSA may adjust buffer
and period sizes. All configuration errors release the handle and parameter
storage. The sink owns its handle; no raw ALSA handle crosses the interface.

Writes return `{frames, status, error}`. `OK` reports positive progress, possibly
partial; the caller retries from that frame offset. Stereo writes consume at most
480 frames per call using bounded stack storage. `AGAIN` means zero progress:
ALSA returned zero/EAGAIN, or EPIPE recovery prepared the device successfully.
`ERROR` reports a negative errno-style code, including a failed prepare; suspend
errors remain fatal, as before. Zero-length writes return OK without I/O, and
invalid sink/buffer arguments return ERROR/-EINVAL. There is no EOF or drain API.

The pipeline helper retries partial writes, checks pthread cancellation and sleeps
1 ms on AGAIN. A fatal error is logged and ends the audio writer; the pipeline
owner must still perform normal stop/destroy to release remaining workers and
hardware. This does not introduce pipeline-wide error propagation. Successful
playback samples and normal queue scheduling remain unchanged.

One worker writes each sink. Writes may block in ALSA; stopping uses the existing
pthread cancellation/join path, not a new wall-clock timeout guarantee. Destruction
closes without explicit drain, after the writer and diagnostic reader have joined.
It accepts NULL. The diagnostic `state` operation is optional and returns a static
string; ALSA permits it alongside the writer while the sink remains alive. The
`demod_worker` thread uses it for its existing heartbeat; the standalone DSP
does not reference the sink. DSP calculations and FIFO sizes are
unchanged. Self-test cues use the sink before/after the writer, never concurrently.
The self-test allocation failure path now joins the writer before closing playback.

```c
audio_sink_t* sink = alsa_sink_open("null", 48000);
if (sink) {
    int16_t samples[480] = {0};
    audio_sink_result_t r = audio_sink_write(sink, samples, 480);
    /* Retry unconsumed frames, handle AGAIN and ERROR in the owning worker. */
    (void)r;
    audio_sink_destroy(sink); // after all users have stopped
}
```

Validation: `test_audio_sink.py` uses ALSA null with scripted writes/configuration
failures to check unchanged PCM, stereo duplication, partial progress, EPIPE
recovery and recovery failure, zero/EAGAIN, fatal errors, cleanup and cancellation
of the production retry helper. Application build and all existing source, tone,
rate, lifecycle and TX-stop checks passed. **H3 passed**: Jan confirmed the automated baseline, option 13, known-signal RX
at both RF rates and repeated RX start/stop. Interactive retuning is deferred
until that control exists. [Physical results](baselines/20260919T121032.048186Z/summary.json).

## Implemented: standalone demodulator DSP (step 5)

[Header](../software/libcariboulite/src/nbfm_demod.h),
[DSP](../software/libcariboulite/src/nbfm_demod.c) and
[worker](../software/libcariboulite/src/demod_worker.c).

```c
nbfm_demod_t* nbfm_demod_create(const nbfm_demod_config_t* config);
nbfm_demod_result_t nbfm_demod_process(nbfm_demod_t* dsp,
    const iq16_t* input, size_t count, int16_t* output, size_t capacity,
    double correction);
void nbfm_demod_reset(nbfm_demod_t* dsp);
int nbfm_demod_set_audio(nbfm_demod_t* dsp, float deemph_tau, float pcm_gain);
void nbfm_demod_destroy(nbfm_demod_t* dsp);
```

Configuration explicitly supplies RF rate (2,000,000 or 4,000,000 Hz), audio rate
(48,000 Hz), finite nonnegative de-emphasis tau in seconds (zero bypasses it),
and finite nonnegative PCM gain. Creation validates these and returns NULL/errno
EINVAL for invalid configuration or NULL on allocation failure. No implicit
configuration defaults are added. Existing application values remain 50 us and
8000 gain for RX, 12000 gain for self-test. RX initialization now rejects audio
rates other than 48 kHz before allocation.

`iq16_t` is the existing packed pair of signed 16-bit I then Q samples, moved
unchanged to `iq16.h` for use by both modems. RF transport still owns hardware
sample conversion. Output is mono signed-16-bit PCM with the existing gain,
clipping to [-32768, 32767] and `lrintf` rounding. This step does not introduce
float audio output or change any filtering or normalization constants.

Processing returns `{consumed, produced, error}`; counts are IQ pairs and mono
PCM frames respectively. The caller owns both nonoverlapping buffers, and no
pointer is retained. Processing allocates nothing and never blocks. Input may be
split arbitrarily; the caller advances by `consumed`, handles the `produced`
samples, and retries remaining input with more output capacity. Zero capacity
consumes nothing; zero input produces nothing. A short input block can be fully
consumed without producing audio because of decimation. The DSP retains filter/
decimator/resampler history, but no pending output samples. Invalid pointers,
nonfinite correction or correction outside ±0.0005 return -EINVAL with zero
progress and unchanged state.

Correction is a unitless fractional adjustment: the resampling increment is
`(48000.0 / 50000.0) * (1.0 + correction)`. Positive correction produces more
audio, negative less. The DSP has no FIFO-depth policy. One owner serializes
process, control updates, reset and destruction. `set_audio` preserves history
and rejects invalid values with -EINVAL; processing uses the current controls.
NULL is permitted for reset/destroy. Create allocates zeroed state; destroy frees
it after the pipeline worker is joined.

Reset preserves the old worker semantics: clear previous discriminator samples,
DC/de-emphasis/LPF history, interpolation endpoints and fractional phase, but
retain both integrate-and-dump accumulators/counters. Existing pipeline resets
occur between complete 10 ms RF blocks, where those accumulators are empty.
For a completely new stream at an arbitrary partial-decimation boundary, destroy
and recreate the state. Reset does not alter configuration. The worker separately
clears its partially packed output, correction, FIFO-depth EMA and servo engagement.

The worker retains the existing 200 kHz then 50 kHz decimation cadence indirectly
through complete RF frames. It measures FIFO depth immediately before processing
the last IQ pair of each 10 ms block, so the new correction applies to exactly the
same 500th intermediate sample as before. It stops processing whenever 480 output
samples are packed, queues them with the existing 10 ms timeout, and then resumes
unused input. Queue sizes, timeout/drop behavior, diagnostics and priority/CPU
placement are unchanged. The legacy priming fields remain bookkeeping; this
extraction does not add muting or a priming delay.

The FIFO-depth servo remains in the worker: EMA alpha 0.05, engagement at 35%
fill, 50% target, deadband of 1% of capacity, integral gain 2e-4, slew limit
10 ppm/update and ordinary clamp ±300 ppm. Existing emergency corrections reach
±500 ppm below 5% or above 95% fill. These are preserved constants, not a new
claim of measured clock synchronization. The worker guards zero capacity rather
than dividing by zero; valid application queues always have positive capacity.

The pipeline creates the DSP before starting its worker and destroys it only
after join, including failed startup. Self-test follows the same ownership rules.
The worker still uses the existing mutable application controls; synchronizing
that control protocol and moving all pipeline coordination are separate work.
DSP code depends only on the C runtime, math library and IQ definition, with no
ALSA, FIFO, pthread, UI or radio dependency.

```c
nbfm_demod_config_t cfg = {4000000, 48000, 50e-6f, 8000};
nbfm_demod_t* dsp = nbfm_demod_create(&cfg);
if (dsp) {
    iq16_t iq[80] = {{0}};
    int16_t pcm[2];
    nbfm_demod_result_t r = nbfm_demod_process(dsp, iq, 80, pcm, 2, 0);
    /* Consume r.produced audio frames; retry input after r.consumed. */
    (void)r;
    nbfm_demod_destroy(dsp);
}
```

`test_nbfm_demod.py` compares the old worker frozen from `b3da533` against the
new DSP/worker using deterministic IQ and FIFO depths. It compares output samples,
counts and ordering of FIFO puts/depth reads at both RF rates, with/without reset,
changing gain/de-emphasis, clipping and dropped FIFO writes: 345,600 PCM samples
match exactly. Separate block/capacity tests exercise zero and ±500 ppm correction,
reset during partial decimation, invalid inputs and zero-capacity behavior.
`test_rx_lifecycle.py` additionally checks DSP creation failure and joins real
waiting DSP/playback threads before destruction. **H4 passed**: Jan confirmed option 13, the baseline, correct pitch, clean
modulation and extended RX without unusual behaviour, and explicitly accepted
the checkpoint. [Physical results](baselines/20260919T123837.461278Z/summary.json).
