#pragma once
/* Integer command labels for native 4 MHz / divider rates. Fractional native
 * rates retain exact divider arithmetic in DSP/pacing; labels are nearest Hz. */
static inline unsigned radio_rate_divider(unsigned hz)
{
    switch(hz){
    case 4000000:return 1;case 2000000:return 2;case 1333333:return 3;
    case 1000000:return 4;case 800000:return 5;case 666666:case 666667:return 6;
    case 500000:return 8;case 400000:return 10;default:return 0;
    }
}
static inline unsigned radio_rate_label(unsigned divider)
{
    return divider?(4000000u+divider/2)/divider:0;
}
static inline double radio_rate_exact(unsigned hz)
{
    unsigned d=radio_rate_divider(hz);return d?4000000.0/d:0;
}
static inline int radio_tx_rate_supported(unsigned hz)
{
    /* RX is extended first; TX is enabled only after its pacing validation. */
    return hz==1000000||hz==2000000||hz==4000000;
}

static inline unsigned radio_rate_for_key(char key)
{
    unsigned d=key>='1'&&key<='9'?(unsigned)(key-'0'):(key=='a'||key=='A'?10:0);
    unsigned hz=radio_rate_label(d);return radio_rate_divider(hz)==d?hz:0;
}
