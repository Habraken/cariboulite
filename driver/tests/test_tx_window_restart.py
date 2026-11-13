#!/usr/bin/env python3
"""Compile real finite-window restart against prefetched FIFO/concurrency mocks."""
from pathlib import Path
import subprocess,tempfile
s=(Path(__file__).resolve().parents[1]/'smi_stream_dev.c').read_text()
a=s.index('static inline u32 smi_transfer_window_bytes(void)\n{')
b=s.index('// static inline void smi_kick_if_idle',a)
code=s[a:b]
pre=r'''
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef uint32_t u32;
#define DMA_BOUNCE_BUFFER_SIZE (512u*1024)
#define SMI_REFRESH_CHUNKS 1024
#define SMIL 1
#define SMICS 2
#define SMICS_CLEAR 1u
#define SMICS_ENABLE 2u
#define SMICS_WRITE 4u
#define SMICS_START 8u
#define READ_ONCE(x) (x)
#define min_t(t,a,b) ((t)(a)<(t)(b)?(t)(a):(t)(b))
#define mb() ((void)0)
#define spin_lock_irqsave(m,f) do{(f)=0;assert(!pthread_mutex_lock(m));}while(0)
#define spin_unlock_irqrestore(m,f) do{(void)(f);assert(!pthread_mutex_unlock(m));}while(0)
enum {smi_stream_idle,smi_stream_rx_channel_0,smi_stream_tx_channel};
struct bcm2835_smi_instance {pthread_mutex_t transaction_lock;u32 control,length;bool active;unsigned fifo_bytes,starts,writes;};
struct bcm2835_smi_dev_instance {struct bcm2835_smi_instance* smi_inst;int state;bool transfer_thread_running;};
static bool smi_is_active(struct bcm2835_smi_instance* s){return s->active;}
static u32 read_smi_reg(struct bcm2835_smi_instance* s,unsigned reg){assert(reg==SMICS);return s->control;}
static void write_smi_reg(struct bcm2835_smi_instance* s,u32 v,unsigned reg){
 ++s->writes;
 if(reg==SMIL){s->length=v;return;}
 assert(reg==SMICS);
 if(v&SMICS_CLEAR)s->fifo_bytes=0;
 assert(s->fifo_bytes==64); // Prefetched words must survive the restart.
 assert((v&(SMICS_ENABLE|SMICS_WRITE|SMICS_START))==(SMICS_ENABLE|SMICS_WRITE|SMICS_START));
 assert(v&0x80); // Preserve unrelated register bits.
 s->control=v;s->active=true;++s->starts;
}
'''
checks=r'''
static void* restart(void* p){smi_rearm_if_idle(p);return NULL;}
int main(void){
 struct bcm2835_smi_instance hw={.transaction_lock=PTHREAD_MUTEX_INITIALIZER,.control=0x80,.fifo_bytes=64};
 struct bcm2835_smi_dev_instance i={.smi_inst=&hw,.state=smi_stream_tx_channel,.transfer_thread_running=true};
 u32 q=DMA_BOUNCE_BUFFER_SIZE/4,len=smi_transfer_window_bytes();
 assert(len && len<=0xffffff && len%q==0 && len%4==0 && len==0xfe0000);
 for(unsigned cycle=0;cycle<500;++cycle){
  hw.active=false;unsigned before=hw.starts;
  pthread_t t[4];for(unsigned k=0;k<4;++k)assert(!pthread_create(t+k,NULL,restart,&i));
  for(unsigned k=0;k<4;++k)assert(!pthread_join(t[k],NULL));
  assert(hw.starts==before+1 && hw.length==len && hw.fifo_bytes==64);
  unsigned writes=hw.writes;smi_rearm_if_idle(&i);assert(hw.writes==writes);
 }
 hw.active=false;unsigned before=hw.writes;
 i.state=smi_stream_idle;smi_rearm_if_idle(&i);assert(hw.writes==before);
 i.state=smi_stream_rx_channel_0;smi_rearm_if_idle(&i);assert(hw.writes==before);
 i.state=smi_stream_tx_channel;i.transfer_thread_running=false;smi_rearm_if_idle(&i);assert(hw.writes==before);
 puts("PASS: whole-period/IQ windows, preserved prefetched TX FIFO, serialized competing restarts, inactive/RX guards");
}
'''
with tempfile.TemporaryDirectory(prefix='tx-window-') as d:
 p=Path(d);(p/'test.c').write_text(pre+code+checks)
 subprocess.run(['cc','-std=gnu11','-O2','-Wall','-Wextra','-Werror',str(p/'test.c'),'-pthread','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True,timeout=15)
