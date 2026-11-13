#include "caribou_smi/smi_rx_stream.h"
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
int main(int argc,char **argv){assert(argc==3);FILE *in=fopen(argv[1],"rb"),*out=fopen(argv[2],"wb");assert(in&&out);smi_rx_stream_t s={0};unsigned char bytes[32768];int16_t iq[2048];size_t n;
while((n=fread(bytes,1,sizeof(bytes),in))){size_t off=0;while(off<n){smi_rx_result_t r=smi_rx_stream_feed(&s,bytes+off,n-off,iq,NULL,1024,true);assert(!r.error&&(r.consumed||r.produced));off+=r.consumed;assert(fwrite(iq,4,r.produced,out)==r.produced);}}
for(;;){smi_rx_result_t r=smi_rx_stream_feed(&s,NULL,0,iq,NULL,1024,true);assert(!r.error);assert(fwrite(iq,4,r.produced,out)==r.produced);if(!r.produced)break;}
fclose(in);fclose(out);printf("samples=%llu acquisitions=%llu discarded_bytes=%llu failures=%llu invalid_frames=%llu pending_bytes=%zu\n",(unsigned long long)s.stats.emitted_samples,(unsigned long long)s.stats.acquisitions,(unsigned long long)s.stats.discarded_bytes,(unsigned long long)s.stats.alignment_failures,(unsigned long long)s.stats.invalid_frames,s.length);}
