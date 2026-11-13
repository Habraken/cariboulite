#include "smi_rx_stream.h"
#include <string.h>

#define ADD(s, field, n) __atomic_fetch_add(&(s)->stats.field, (uint64_t)(n), __ATOMIC_RELAXED)
static uint32_t word_at(const uint8_t *p)
{
    /* Wire format is little-endian; no alignment or aliasing assumptions. */
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static bool valid(uint32_t w) { return (w & 0xc001c000u) == 0x80004000u; }
static void emit(uint32_t w, void *iq, uint8_t *metadata, size_t n, bool hif)
{
    int16_t low=(w>>1)&8191, high=(w>>17)&8191;
    if (low>=4096) low-=8192;
    if (high>=4096) high-=8192;
    int16_t pair[2]={hif?low:high,hif?high:low};
    if (iq) memcpy((uint8_t*)iq+4*n,pair,sizeof(pair));
    if (metadata) metadata[n]=w&1;
}
void smi_rx_stream_reset(smi_rx_stream_t *s)
{
    s->length=0; s->locked=false;
    ADD(s,stream_resets,1);
}
smi_rx_result_t smi_rx_stream_feed(smi_rx_stream_t *s, const uint8_t *input,
    size_t bytes, void *iq, uint8_t *metadata, size_t capacity, bool hif)
{
    smi_rx_result_t r={0};
    while (r.produced<capacity) {
        if (!s->locked) {
            size_t take=bytes-r.consumed;
            if (take>sizeof(s->bytes)-s->length) take=sizeof(s->bytes)-s->length;
            if (take) { memcpy(s->bytes+s->length,input+r.consumed,take); s->length+=take; r.consumed+=take; }
            if (s->length<4*SMI_RX_CONFIRM_WORDS+3) break;
            int phase=-1; unsigned matches=0;
            for (unsigned p=0;p<4;++p) {
                size_t start=p, consecutive=0;
                for (size_t k=p;k+4<=s->length;k+=4) {
                    if (valid(word_at(s->bytes+k))) ++consecutive;
                    else {consecutive=0;start=k+4;}
                }
                // Startup may contain stale/partial FIFO bytes. Lock only a
                // unique phase with a long entirely valid remaining suffix.
                if (consecutive>=SMI_RX_CONFIRM_WORDS) {phase=(int)start;++matches;}
            }
            if (matches!=1) {
                if (s->length<sizeof(s->bytes)) break;
                ADD(s,alignment_failures,1);
                if (matches>1) ADD(s,ambiguous_windows,1);
                ADD(s,discarded_bytes,s->length);
                s->length=0; r.error=-3; break;
            }
            if (phase) {memmove(s->bytes,s->bytes+phase,s->length-(size_t)phase);s->length-=(size_t)phase;ADD(s,discarded_bytes,phase);}
            s->locked=true; ADD(s,acquisitions,1);
        }
        if (s->length) {
            if (s->length<4) {
                size_t take=4-s->length;
                if (take>bytes-r.consumed) take=bytes-r.consumed;
                if (take) {memcpy(s->bytes+s->length,input+r.consumed,take);s->length+=take;r.consumed+=take;}
                if (s->length<4) break;
            }
            size_t offset=0;
            while (offset+4<=s->length && r.produced<capacity) {
                uint32_t w=word_at(s->bytes+offset);
                if (!valid(w)) {ADD(s,invalid_frames,1);ADD(s,discarded_bytes,4);offset+=4;s->locked=false;r.error=-3;break;}
                emit(w,iq,metadata,r.produced++,hif);offset+=4;
            }
            if (offset) {memmove(s->bytes,s->bytes+offset,s->length-offset);s->length-=offset;}
            if (r.error) break;
            if (r.produced==capacity) break;
            if (s->length) {if (r.consumed==bytes) break;continue;}
        }
        while (r.consumed+4<=bytes && r.produced<capacity) {
            uint32_t w=word_at(input+r.consumed);r.consumed+=4;
            if (!valid(w)) {ADD(s,invalid_frames,1);ADD(s,discarded_bytes,4);s->locked=false;r.error=-3;break;}
            emit(w,iq,metadata,r.produced++,hif);
        }
        if (r.error || r.produced==capacity) break;
        size_t tail=bytes-r.consumed;
        if (tail) {memcpy(s->bytes+s->length,input+r.consumed,tail);s->length+=tail;r.consumed+=tail;}
        break;
    }
    ADD(s,emitted_samples,r.produced);
    return r;
}
