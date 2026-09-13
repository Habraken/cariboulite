#!/usr/bin/env python3
"""Test actual streaming writer with scripted short I/O, without hardware."""
from pathlib import Path
import subprocess,tempfile
s=(Path(__file__).resolve().parents[1]/'src/caribou_smi/caribou_smi.c').read_text()
a=s.index('\nint caribou_smi_write_samples(caribou_smi_st *dev,')+1;b=s.index('// Optionally keep',a)
pre=r'''
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CARIBOU_SMI_BYTES_PER_SAMPLE 4
typedef int caribou_smi_channel_en;
typedef struct { short i,q; } caribou_smi_sample_complex_int16;
typedef struct { int filedesc; size_t native_batch_len,write_partial_bytes; void* write_temp_buffer; } caribou_smi_st;
static unsigned char output[256];
static size_t output_len;
static int script[32], script_len, script_pos;
static void caribou_smi_generate_data(caribou_smi_st* d,uint8_t* out,size_t n,const caribou_smi_sample_complex_int16* s) {
    memcpy(out,s,n); /* Distinct source bytes expose repeats and omissions. */
}
static int caribou_smi_timeout_write(caribou_smi_st* d,uint8_t* p,size_t n,unsigned timeout) {
    int w=script_pos<script_len?script[script_pos++]:(int)n;
    assert(w<0 || (size_t)w<=n);
    if(w>0) { memcpy(output+output_len,p,w);output_len+=w; }
    return w;
}
'''
post=r'''
static void run(int prefix,int terminal) {
    unsigned char temp[16];
    caribou_smi_st d={.filedesc=1,.native_batch_len=16,.write_temp_buffer=temp};
    caribou_smi_sample_complex_int16 input[8];
    for(int i=0;i<8;++i) input[i]=(caribou_smi_sample_complex_int16){i+10,i+30};
    output_len=0;script_pos=0;script_len=5;
    script[0]=prefix;for(int i=1;i<5;++i)script[i]=terminal;
    int ret=caribou_smi_write_samples(&d,0,input,8);
    assert(ret==(prefix>=4?prefix/4:(terminal<0?terminal:0)));
    assert(d.write_partial_bytes==(size_t)(prefix%4));
    int consumed=ret>0?ret:0;
    script_len=script_pos=0;
    assert(caribou_smi_write_samples(&d,0,input+consumed,8-consumed)==8-consumed);
    assert(output_len==sizeof(input) && memcmp(output,input,sizeof(input))==0);
    assert(d.write_partial_bytes==0);
}
int main(void) {
    for(int n=0;n<16;++n) { run(n,0);run(n,-EIO); }
    unsigned char temp[8];caribou_smi_sample_complex_int16 input[8]={0};
    caribou_smi_st d={.filedesc=1,.native_batch_len=8,.write_temp_buffer=temp};
    output_len=script_pos=0;script_len=6;
    script[0]=8;script[1]=4;for(int i=2;i<6;++i)script[i]=0;
    assert(caribou_smi_write_samples(&d,0,input,8)==3);
    assert(output_len==12);
    puts("PASS: partial progress before timeouts/errors, all byte offsets, exact retry stream, multiple chunks");
}
'''
with tempfile.TemporaryDirectory(prefix='tx-progress-') as directory:
 c=Path(directory)/'test.c';exe=Path(directory)/'test';c.write_text(pre+s[a:b]+post)
 subprocess.run(['cc','-Wall','-Wextra','-Werror','-Wno-unused-parameter',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True,timeout=5)
