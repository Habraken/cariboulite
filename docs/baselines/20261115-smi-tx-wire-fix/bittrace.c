#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>
#include <signal.h>
#include <sys/ioctl.h>
#include "cariboulite_setup.h"
#include "at86rf215.h"
#include "monitor_loopback.h"
CARIBOULITE_CONFIG_DEFAULT(board);
static volatile sig_atomic_t abort_test;
static atomic_bool writer_stop;
static unsigned long writer_errors;
static caribou_smi_sample_complex_int16 pattern;
static void interrupted(int x){(void)x;abort_test=1;}
static int reg_read(unsigned char a,unsigned char*value){unsigned char t[2]={a,0},r[2]={0};int e=io_utils_spi_transmit(board.fpga.io_spi,board.fpga.io_spi_handle,t,r,2,io_utils_spi_read_write);*value=r[1];return e;}
static int peek(uint32_t*tx,uint32_t*rx){unsigned char b=0;uint32_t a=0,c=0;if(reg_read(8,&b)||b!=0xa5)return -1;for(int j=0;j<8;j++){if(reg_read(9+j,&b))return -1;if(j<4)a|=(uint32_t)b<<(8*j);else c|=(uint32_t)b<<(8*(j-4));}*rx=0x80004000u|(((a>>13)&8191)<<17)|((a&8191)<<1);*tx=0x80014000u|(((c>>13)&8191)<<17)|((c&8191)<<1);return 0;}
static void*write_samples(void*x){(void)x;caribou_smi_sample_complex_int16 v[4096];for(int j=0;j<4096;j++)v[j]=pattern;while(!abort_test&&!atomic_load(&writer_stop)){int n=caribou_smi_write_timed(&board.smi,caribou_smi_channel_2400,v,4096,20000);if(n<0){writer_errors++;break;}}return NULL;}
static int check(int16_t i,int16_t q){
 uint8_t iq=0,state=0;uint32_t tx=0,rx=0;
 if(at86rf215_read_buffer(&board.modem,REG_RF_IQIFC0,&iq,1)||!(iq&0x80)||(iq&1)||at86rf215_read_buffer(&board.modem,REG_RF24_STATE,&state,1)||(state&7)!=5)return -1;
 pattern=(caribou_smi_sample_complex_int16){i,q};writer_errors=0;atomic_store(&writer_stop,false);
 if(caribou_fpga_set_smi_ctrl_data_direction(&board.fpga,0)||caribou_smi_set_driver_streaming_state(&board.smi,smi_stream_tx_channel))return -1;
 pthread_t thread;if(pthread_create(&thread,NULL,write_samples,NULL)){caribou_smi_set_driver_streaming_state(&board.smi,smi_stream_idle);return -1;}
 uint32_t expected=0x80014000u|(((uint32_t)i&0x1fff)<<17)|(((uint32_t)q&0x1fff)<<1);
 usleep(100000);int observed=0,matched=0;
 for(int n=0;n<20&&!abort_test;n++){if(peek(&tx,&rx)){observed=-1;break;}observed=1;if(tx==expected&&(rx&0xfffefffeu)==(expected&0xfffefffeu)){matched=1;break;}usleep(50000);}
 atomic_store(&writer_stop,true);pthread_join(thread,NULL);
 struct smi_stream_tx_status status={0};ioctl(board.smi.filedesc,SMI_STREAM_IOC_GET_TX_STATUS,&status);
 if(!matched)fprintf(stderr,"DRIVER_FAILURE accepted=%llu dequeued=%llu periods=%llu underflows=%llu queued=%u\n",(unsigned long long)status.accepted_bytes,(unsigned long long)status.fifo_dequeued_bytes,(unsigned long long)status.completed_periods,(unsigned long long)status.underrun_periods,status.fifo_queued_bytes);
 if(!matched){caribou_fpga_smi_fifo_status_st fs={0};caribou_fpga_get_smi_ctrl_fifo_status(&board.fpga,&fs);fprintf(stderr,"FPGA dir=%u tx_full=%u\n",fs.smi_direction,fs.tx_fifo_full);}
 int stop=caribou_smi_set_driver_streaming_state(&board.smi,smi_stream_idle);
 printf("{\"i\":%d,\"q\":%d,\"expected\":\"%08x\",\"collected\":\"%08x\",\"echo\":\"%08x\",\"match\":%s,\"writer_errors\":%lu}\n",i,q,expected,tx,rx,matched?"true":"false",writer_errors);fflush(stdout);
 if(observed<0||stop||writer_errors||abort_test)return -1;
 return matched?0:1;
}
int main(void){
 monitor_loopback_t lb={0};struct smi_settings saved={0};int restore=0,result=1,checked=0,failed=0;
 board.force_fpga_reprogramming=false;cariboulite_set_log_level(cariboulite_log_level_none);
 if(cariboulite_init_driver(&board,NULL))return 2;
 signal(SIGINT,interrupted);signal(SIGTERM,interrupted);signal(SIGHUP,interrupted);
 printf("PROGRAM_DIAGNOSTIC_AFTER_INIT\n");fflush(stdout);
 if(caribou_prog_configure(&board.fpga.prog_dev,"/tmp/am-bittrace-fpga/bittrace.bin")||caribou_fpga_soft_reset(&board.fpga))goto cleanup;
 caribou_smi_setup_ios(&board.smi);
 uint8_t marker=0;if(reg_read(8,&marker)||marker!=0xa5){fprintf(stderr,"Diagnostic FPGA marker missing: %02x\n",marker);for(int a=0;a<17;a++){uint8_t v=0;reg_read(a,&v);fprintf(stderr,"IOC %02x = %02x\n",a,v);}goto cleanup;}
 at86rf215_iq_interface_config_st cfg={.loopback_enable=0,.drv_strength=at86rf215_iq_drive_current_4ma,.common_mode_voltage=at86rf215_iq_common_mode_v_ieee1596_1v2,.tx_control_with_iq_if=0,.radio09_mode=at86rf215_iq_if_mode,.radio24_mode=at86rf215_iq_if_mode,.clock_skew=at86rf215_iq_clock_data_skew_4_906ns};
 at86rf215_setup_iq_if(&board.modem,&cfg);
 if(monitor_loopback_start(&board,&lb))goto cleanup;
 usleep(100000);uint32_t control_tx=0,control_rx=0;
 if(peek(&control_tx,&control_rx))goto cleanup;
 printf("FIXED_PATTERN_CONTROL=%08x\n",control_rx);fflush(stdout);
 if(control_rx!=0x84027048u){fprintf(stderr,"Fixed-pattern control failed\n");goto cleanup;}
 if(caribou_smi_set_driver_streaming_state(&board.smi,smi_stream_idle)||caribou_fpga_set_debug_loopback(&board.fpga,false)||caribou_fpga_set_sys_ctrl_tx_sample_gap(&board.fpga,3))goto cleanup;
 if(caribou_smi_get_current_settings(&board.smi,&saved))goto cleanup;
 struct smi_settings desired=saved;desired.write_setup_time=1;desired.write_strobe_time=4;desired.write_hold_time=2;restore=1;
 if(ioctl(board.smi.filedesc,BCM2835_SMI_IOC_WRITE_SETTINGS,&desired))goto cleanup;
 struct smi_stream_tx_status status={0};
 if(ioctl(board.smi.filedesc,SMI_STREAM_IOC_GET_TX_STATUS,&status))goto cleanup;
 struct smi_stream_tx_buffer_limit limit={.abi_version=SMI_STREAM_TX_ABI_VERSION,.struct_size=sizeof(limit),.fifo_limit_bytes=2*status.period_bytes};
 if(ioctl(board.smi.filedesc,SMI_STREAM_IOC_SET_TX_BUFFER_LIMIT,&limit))goto cleanup;
 printf("DRIVER native=%zu period=%u ring=%u limit=%u\n",board.smi.native_batch_len,status.period_bytes,status.ring_bytes,limit.fifo_limit_bytes);fflush(stdout);
 if(check(1024,512)!=0){fprintf(stderr,"SMI positive control failed\n");goto cleanup;}
 for(int n=0;n<78&&!abort_test;n++){
  int16_t i=0,q=0;
  if(n<26){int16_t bit=(n/2==12)?-4096:(1<<(n/2));if(n&1)q=bit;else i=bit;}
  else if(n==26){i=q=0;}else if(n==27){i=q=-1;}else if(n==28){i=0x555;q=-0x555;}else if(n==29){i=-0x555;q=0x555;}else {i=992+n-30;q=0;}
  int e=check(i,q);if(e<0)goto cleanup;checked++;failed+=e;
 }
 printf("SUMMARY checked=%d failed=%d RF_TX_COMMANDS=0\n",checked,failed);fflush(stdout);
 result=checked==78&&!failed&&!abort_test?0:3;
cleanup:
 if(caribou_smi_set_driver_streaming_state(&board.smi,smi_stream_idle))result=4;
 if(restore&&ioctl(board.smi.filedesc,BCM2835_SMI_IOC_WRITE_SETTINGS,&saved))result=4;
 if(monitor_loopback_stop(&board,&lb))result=4;
 cariboulite_release_driver(&board);return result;
}
