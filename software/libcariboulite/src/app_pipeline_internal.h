#pragma once
// Internal transport shared by the application and its demodulator thread.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pthread.h>
#include "nbfm_mod.h"

typedef struct {
    int16_t pcm[480];   // mono, 48 kHz, 10 ms
} aud10_frame_t;

typedef struct aud10_fifo_s {
    aud10_frame_t* q;
    size_t cap, r, w, count;
    pthread_mutex_t m;
    pthread_cond_t  can_put, can_get;
    bool stop;
    size_t drops;
} aud10_fifo_t;

typedef struct rf10_fifo_s rf10_fifo_t;
typedef struct rf10_frame_s rf10_frame_t;

struct rf10_frame_s {
    // One 10 ms RF frame @ 4 MS/s = 40,000 IQ16 pairs
    // Reuse your iq16_t type: struct { int16_t i, q; };
    iq16_t data[40000];
};

struct rf10_fifo_s {
    rf10_frame_t*   q;
    size_t          cap;         // number of frames (e.g., 8)
    size_t          r;           // read index (consumer)
    size_t          w;           // write index (producer)
    size_t          count;       // how many frames ready
    pthread_mutex_t m;
    pthread_cond_t  can_put;
    pthread_cond_t  can_get;
    bool            drop_oldest_on_full;  // if true, overwrite oldest when full
    bool            stop;

	// diagnostics
	size_t max_depth, min_depth;
	size_t drops, puts, gets, timeouts_put, timeouts_get;
};


bool rf10_fifo_get(rf10_fifo_t* f, rf10_frame_t* out, int timeout_ms);
bool aud10_fifo_put(aud10_fifo_t* f, const aud10_frame_t* frm, int timeout_ms);
void aud10_fifo_peek_depth(aud10_fifo_t* f, size_t* count, size_t* cap);
int set_rt_and_affinity_prio(int prio, int cpu_req);
