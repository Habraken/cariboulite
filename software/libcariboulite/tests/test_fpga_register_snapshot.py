#!/usr/bin/env python3
"""Prove the diagnostic performs only its whitelisted reads and keeps partial data."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'src/caribou_fpga/caribou_fpga.c').read_text();a=s.index('int caribou_fpga_read_registers(');function=s[a:s.index('\n}',a)+2]
code=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "caribou_fpga/caribou_fpga_regs.h"
typedef struct {int initialized; int status; uint8_t cached_version;} caribou_fpga_st;
#define CARIBOU_FPGA_CHECK_DEV(d,f) do {if (!(d) || !(d)->initialized) return -1;} while(0)
static unsigned calls, fail;
static int caribou_fpga_spi_transfer(caribou_fpga_st *dev,uint8_t *opcode,uint8_t *value) {
 const uint8_t expected[]={0,1,2,6,32,33,34,37,64,65};
 assert(dev && calls<10 && *opcode==expected[calls] && !(*opcode&128));
 ++calls;
 if(calls==fail)return -1;
 *value=(uint8_t)(0x80|*opcode);return 0;
}
'''+function+r'''
int main(void) {
 caribou_fpga_st dev={1,77,99};caribou_fpga_regs_st r;
 for(fail=0;fail<=10;++fail) {
  calls=0;memset(&r,255,sizeof(r));
  assert(caribou_fpga_read_registers(&dev,&r)==(fail ? -1 : 0));
  assert(calls==10 && dev.status==77 && dev.cached_version==99);
  assert(r.valid_mask==(CARIBOU_FPGA_REG_ALL & ~(fail ? 1U<<(fail-1) : 0U)));
  for(unsigned i=0;i<10;++i) {
   unsigned opcode=(caribou_fpga_reg_desc[i].module<<5)|caribou_fpga_reg_desc[i].ioc;
   assert(r.value[i]==(fail==i+1 ? 0 : (0x80|opcode)));
  }
 }
 calls=0;assert(caribou_fpga_read_registers(&dev,0)==-1 && !calls);
 assert(caribou_fpga_read_registers(0,&r)==-1 && !calls && !r.valid_mask);
 assert(caribou_fpga_reg_desc[5].defined_mask==0x1f);
 assert(caribou_fpga_reg_desc[6].defined_mask==0xfb);
 assert(caribou_fpga_reg_desc[7].defined_mask==0xff);
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='fpga-reg-snapshot-') as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('PASS FPGA snapshot: ten read-only transactions, partial failures, unchanged cached state and masks')
