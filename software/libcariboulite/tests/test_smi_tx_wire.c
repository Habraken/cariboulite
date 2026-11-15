// Check actual bytes passed to the 8-bit SMI bus, not host uint32_t values.
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "caribou_smi/caribou_smi.h"
static unsigned char wire[16];static size_t written;
ssize_t __wrap_write(int fd,const void*data,size_t n){assert(fd==42&&n<=sizeof(wire));memcpy(wire,data,n);written=n;return n;}
int main(void){
 unsigned char scratch[16];caribou_smi_st d={0};d.filedesc=42;d.native_batch_len=sizeof(scratch);d.write_temp_buffer=scratch;
 for(int code=-4096;code<4096;code++){
  caribou_smi_sample_complex_int16 sample={code,-1-code};written=0;
  assert(caribou_smi_write_timed(&d,caribou_smi_channel_2400,&sample,1,0)==1&&written==4);
  // The collector consumes SOF/I-high, I-low, I-bit0/Q-high, Q-low in order.
  assert((wire[0]&0xe0)==0xe0&&!(wire[1]&128)&&!(wire[2]&128)&&!(wire[3]&128));
  unsigned i=((wire[0]&31)<<8)|((wire[1]&127)<<1)|((wire[2]>>6)&1);
  unsigned q=((wire[2]&63)<<7)|(wire[3]&127);
  assert(i==((unsigned)sample.i&8191)&&q==((unsigned)sample.q&8191));
 }
 puts("PASS: actual TX wire bytes preserve all 8192 signed I/Q codes, SOF first");return 0;
}
