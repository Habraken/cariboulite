#include "sinad.h"
#include <assert.h>
#include <stdio.h>
#include <time.h>

static double run(double fundamental, double harmonic, double hz)
{
    sinad_t s; sinad_reset(&s);
    unsigned updates = 0;
    for (unsigned n=0; n<144000; ++n) {
        double t = 6.283185307179586*n/48000;
        updates += sinad_process(&s, fundamental*sin(hz*t)+harmonic*sin(2000*t));
    }
    assert(updates == 5);
    assert(s.valid);
    return s.db;
}
int main(void)
{
    /* Independent analog Butterworth expectation, with bilinear-warped
     * frequencies. 2 kHz remains in the distortion residual. */
    double f1=tan(3.141592653589793*1000/48000);
    double f2=tan(3.141592653589793*2000/48000);
    double hi=tan(3.141592653589793*3000/48000);
    double lo=tan(3.141592653589793*300/48000);
    double g1=1/((1+pow(lo/f1,4))*(1+pow(f1/hi,4)));
    double g2=1/((1+pow(lo/f2,4))*(1+pow(f2/hi,4)));
    double expected=10*log10((g1+.01*g2)/(.01*g2));
    double actual=run(1,.1,1000);
    assert(fabs(actual-expected)<.1);
    assert(fabs(run(.01,.001,1000)-actual)<.001);
    assert(run(1,0,1000)>79);
    double offset=run(1,0,1001);
    assert(offset>23 && offset<26);
    sinad_t s; sinad_reset(&s);
    for (unsigned n=0;n<48000;++n) sinad_process(&s,0);
    assert(!s.valid);
    uint32_t random=12345;
    for (unsigned n=0;n<96000;++n) {
        random=1664525u*random+1013904223u;
        sinad_process(&s, (float)((int32_t)random/2147483648.0));
    }
    assert(!s.valid);
    assert(sinad_process(&s,NAN)); assert(!s.valid && s.warmup==9600);
    assert(sinad_process(&s,INFINITY)); assert(!s.valid);
    sinad_reset(&s);
    for (unsigned n=0;n<33599;++n) assert(!sinad_process(&s,sin(n*.13)));
    assert(sinad_process(&s,0));
    printf("SINAD tests passed: distortion %.3f dB (expected %.3f), +1 Hz %.3f dB\n",actual,expected,offset);
}
