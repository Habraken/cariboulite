#ifndef SMI_RX_STREAM_H
#define SMI_RX_STREAM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Alignment must be unambiguous over a substantial window, not four payload-
 * dependent headers. Ambiguous input is retained, bounded, then rejected. */
#define SMI_RX_CONFIRM_WORDS 512u
#define SMI_RX_BUFFER_BYTES (8192u + 3u)
typedef struct {
    uint64_t acquisitions, alignment_failures, ambiguous_windows;
    uint64_t invalid_frames, discarded_bytes, emitted_samples, stream_resets;
} smi_rx_stats_t;
typedef struct {
    uint8_t bytes[SMI_RX_BUFFER_BYTES];
    size_t length;
    bool locked;
    smi_rx_stats_t stats; /* counters use atomic builtins; single decoder owner */
} smi_rx_stream_t;
typedef struct { size_t consumed, produced; int error; } smi_rx_result_t;
void smi_rx_stream_reset(smi_rx_stream_t *s);
static inline void smi_rx_stream_stats(const smi_rx_stream_t *s, smi_rx_stats_t *out)
{
#define SMI_RX_GET(f) out->f=__atomic_load_n(&s->stats.f,__ATOMIC_RELAXED)
    SMI_RX_GET(acquisitions); SMI_RX_GET(alignment_failures); SMI_RX_GET(ambiguous_windows);
    SMI_RX_GET(invalid_frames); SMI_RX_GET(discarded_bytes); SMI_RX_GET(emitted_samples); SMI_RX_GET(stream_resets);
#undef SMI_RX_GET
}
/* Input/output can be fragmented arbitrarily. Retains un-emitted complete
 * words and partial bytes. No allocation, interpolation or hidden disk I/O.
 * Output is packed native int16 I,Q; metadata is one sync byte per sample.
 * error=-3 reports framing failure; produced genuine samples remain valid. */
smi_rx_result_t smi_rx_stream_feed(smi_rx_stream_t *s, const uint8_t *input,
    size_t bytes, void *iq, uint8_t *metadata, size_t capacity, bool hif);
#endif
