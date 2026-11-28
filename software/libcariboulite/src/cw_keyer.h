#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Sample-clock keyer. One owner; contact changes are applied before the next
 * tick. No text encoding, paddle-tap memory, or automatic character spacing. */
typedef enum { CW_STRAIGHT, CW_IAMBIC, CW_ULTIMATIC, CW_COOTIE } cw_key_type_t;
typedef struct {
    cw_key_type_t type;
    unsigned wpm;
    bool swap;
    unsigned contacts, priority;
    unsigned element, previous;
    bool mark, gap;
    uint64_t clock;
    double boundary;
} cw_keyer_t;

bool cw_keyer_configure(cw_keyer_t*, cw_key_type_t, unsigned wpm, bool swap);
void cw_keyer_contacts(cw_keyer_t*, unsigned contacts); /* bit 0 dit, bit 1 dah */
bool cw_keyer_tick(cw_keyer_t*); /* one sample at 48 kHz */
bool cw_keyer_busy(const cw_keyer_t*);
void cw_keyer_release(cw_keyer_t*);
static inline const char* cw_key_type_name(cw_key_type_t type)
{
    static const char* names[]={"straight","iambic","ultimatic","cootie"};
    return type>=CW_STRAIGHT && type<=CW_COOTIE ? names[type] : "unknown";
}
static inline bool cw_key_type_parse(const char* text,cw_key_type_t* out)
{
    if(!text||!out)return false;
    for(int i=CW_STRAIGHT;i<=CW_COOTIE;++i)
        if(!strcmp(text,cw_key_type_name(i))){*out=i;return true;}
    return false;
}


typedef struct { double position; } cw_envelope_t;
float cw_envelope_tick(cw_envelope_t*, bool down, float ramp_ms);
