#include "caribou_smi_candidate.c"
#include <assert.h>
typedef struct {uint64_t before,after,offset,requested;int32_t returned;uint32_t copied;} capture_call;
int main(int argc,char **argv)
{
    assert(argc==4);FILE *f=fopen(argv[1],"rb");assert(f);fseek(f,0,SEEK_END);long len=ftell(f);rewind(f);
    uint8_t *data=malloc((size_t)len);assert(data&&fread(data,1,(size_t)len,f)==(size_t)len);fclose(f);
    f=fopen(argv[2],"rb");assert(f);unsigned true_phase=(unsigned)atoi(argv[3]);capture_call call;size_t n=0;
    caribou_smi_st dev={0};dev.debug_mode=caribou_smi_none;
    while(fread(&call,sizeof(call),1,f)){
        assert(call.offset+call.copied<=(uint64_t)len);
        if(call.copied<20)continue;
        int expected=(int)((true_phase+4-call.offset%4)%4);
        int actual=caribou_smi_find_buffer_offset(&dev,data+call.offset,call.copied);
        if(actual!=expected){fprintf(stderr,"call %zu offset=%d expected=%d bytes=%u\n",n,actual,expected,call.copied);return 1;}
        ++n;
    }
    fclose(f);free(data);fprintf(stderr,"Consensus regression passed on %zu captured device reads\n",n);return 0;
}
