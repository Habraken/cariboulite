#define _GNU_SOURCE
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
static int reg_value;
static void mock_set(void *dev,int drive,int clock){(void)dev;reg_value=((drive&3)<<3)|(clock&7);}
static int mock_read(void *dev,uint16_t address){(void)dev;return address==7?reg_value:-1;}
static void *mock_dlsym(void *,const char *);
#define dlsym mock_dlsym
#include "drive_probe.c"
#undef dlsym
static void *mock_dlsym(void *handle,const char *name){
 (void)handle;
 if(!strcmp(name,"at86rf215_set_clock_output"))return (void *)mock_set;
 if(!strcmp(name,"at86rf215_read_byte"))return (void *)mock_read;
 return 0;
}
int main(int argc,char **argv){
 assert(argc==2);int drive=atoi(argv[1]);
 for(int clock=0;clock<8;clock++){
  at86rf215_set_clock_output(0,3,clock);
  assert(reg_value==((drive<<3)|clock));
 }
 return 0;
}
