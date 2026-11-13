#include "caribou_smi/smi_rx_stream.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t word(const unsigned char *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void run(const unsigned char *data,size_t bytes,unsigned phase,size_t chunk,size_t cap,bool hif)
{
    smi_rx_stream_t s={0};size_t pos=0,emitted=0;int16_t out[2048];unsigned char meta[1025];
    while(pos<bytes || (s.locked&&s.length>=4)){
        size_t n=bytes-pos;if(n>chunk)n=chunk;
        memset(out,0x55,sizeof(out));memset(meta,0x55,sizeof(meta));
        smi_rx_result_t r=smi_rx_stream_feed(&s,data+pos,n,out,meta,cap,hif);
        assert(!r.error&&r.consumed<=n&&r.produced<=cap);
        for(size_t k=0;k<r.produced;++k){
            uint32_t w=word(data+phase+4*emitted++);int low=(w>>1)&8191,high=(w>>17)&8191;
            if(low>=4096)low-=8192;
            if(high>=4096)high-=8192;
            assert(out[2*k]==(hif?low:high)&&out[2*k+1]==(hif?high:low));assert(meta[k]==(w&1));
        }
        assert(meta[cap]==0x55);
        assert(r.consumed||r.produced);pos+=r.consumed;
    }
    assert(emitted==(bytes-phase)/4);
    smi_rx_stats_t st;smi_rx_stream_stats(&s,&st);
    assert(st.acquisitions==1&&st.discarded_bytes==phase&&st.invalid_frames==0&&st.alignment_failures==0&&st.emitted_samples==emitted);
}
int main(int argc,char **argv)
{
    assert(argc==3);unsigned char data[32768];
    const size_t chunks[]={1,2,3,4,5,7,15,64,511,4096,32768},caps[]={1,3,97,1024};
    for(unsigned phase=0;phase<4;++phase){
        memset(data,0xaa,phase);
        for(unsigned k=0;k<4096;++k){uint32_t lo=(k*131+17)&8191,hi=(k*319+4096)&8191;uint32_t w=0x80004000|(hi<<17)|(lo<<1)|(k&1);for(unsigned j=0;j<4;++j)data[phase+4*k+j]=(unsigned char)(w>>(8*j));}
        for(unsigned a=0;a<sizeof(chunks)/sizeof(*chunks);++a)for(unsigned b=0;b<sizeof(caps)/sizeof(*caps);++b)for(unsigned hif=0;hif<2;++hif)run(data,phase+16384,phase,chunks[a],caps[b],hif);
    }
    // Real tone payloads whose first four headers caused the captured false locks.
    for(unsigned f=0;f<2;++f){FILE *p=fopen(argv[1+f],"rb");assert(p);assert(fread(data,1,512,p)==512);fclose(p);for(unsigned n=1;n<32;++n)memcpy(data+512*n,data,512);for(unsigned a=0;a<sizeof(chunks)/sizeof(*chunks);++a)for(unsigned b=0;b<sizeof(caps)/sizeof(*caps);++b)run(data,16384,f+1,chunks[a],caps[b],true);}
    smi_rx_stream_t s={0};int16_t out[4096];
    // Two phases remain valid: refusing to guess is required.
    const unsigned char ambiguous[]={0x80,0x40,0x40,0x80};for(unsigned n=0;n<sizeof(data);++n)data[n]=ambiguous[n%4];
    smi_rx_result_t r=smi_rx_stream_feed(&s,data,sizeof(data),out,NULL,2048,true);assert(r.error==-3&&r.produced==0&&r.consumed==SMI_RX_BUFFER_BYTES);assert(s.stats.ambiguous_windows==1&&s.stats.alignment_failures==1);
    // Acquire clean framing, then reject a damaged frame without inventing IQ.
    memset(&s,0,sizeof(s));for(unsigned n=0;n<sizeof(data);n+=4){uint32_t w=0x80004000;memcpy(data+n,&w,4);}
    r=smi_rx_stream_feed(&s,data,4096,out,NULL,2048,true);assert(!r.error&&r.produced==1024);
    data[4*5+3]^=0x40;
    r=smi_rx_stream_feed(&s,data,1024,out,NULL,2048,true);assert(r.error==-3&&r.produced==5&&r.consumed==24&&s.stats.invalid_frames==1);
    r=smi_rx_stream_feed(&s,data+24,4096-24,out,NULL,2048,true);assert(!r.error&&r.produced==1018&&s.stats.acquisitions==2);
    // A stale startup prefix must be accounted for, not decoded as samples.
    memset(&s,0,sizeof(s));memset(data,0,37);
    for(unsigned k=37;k+4<=sizeof(data);k+=4){uint32_t w=0x80004000;memcpy(data+k,&w,4);}
    r=smi_rx_stream_feed(&s,data,8192,out,NULL,2048,true);
    assert(!r.error&&r.produced==(8192-37)/4&&s.stats.discarded_bytes==37);
    smi_rx_stream_reset(&s);assert(!s.locked&&!s.length&&s.stats.stream_resets==1);
    puts("PASS: RX byte phases, real false locks, fragmented input/output, genuine counts, channel mapping, ambiguity, corruption and recovery");
}
