#define _GNU_SOURCE
#include <dlfcn.h>
#include <signal.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>
/* Temporary RX diagnostic interposer. It calls the unmodified library read,
 * copies a bounded number of returned IQ/meta samples into prefaulted RAM,
 * and writes files ONLY at normal process exit after RX workers have joined.
 * SIGUSR1 arms one capture. It does not access radio registers or alter data.
 * Only the one RX reader thread calls this API in the backend under test. */
typedef int (*read_fn)(void*,void*,void*,size_t);
typedef struct { uint64_t before_ns,after_ns,offset,requested; int32_t returned; uint32_t copied; } call_t;
static read_fn original;
static ssize_t (*original_read)(int,void*,size_t);
static dev_t smi_device;
static int smi_fd=-1;
static unsigned char *raw_bytes;
static call_t *raw_calls;
static size_t raw_used,raw_ncalls;
static unsigned char *iq,*meta;
static call_t *calls;
static size_t capacity,used,ncalls;
static int active,ever;
static volatile sig_atomic_t requested;
static char prefix[PATH_MAX];
#define MAX_CALLS 65536
static uint64_t ns(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000+t.tv_nsec;}
static void arm(int number) {(void)number;requested=1;}
static void *allocate(size_t bytes)
{
    void *p=mmap(NULL,bytes,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
    if(p==MAP_FAILED){perror("capture mmap");_exit(98);}
    for(size_t k=0;k<bytes;k+=4096)((volatile unsigned char*)p)[k]=0;
    return p;
}
__attribute__((constructor)) static void initialize(void)
{
    original_read=dlsym(RTLD_NEXT,"read");
    const char *device=getenv("CARIBOULITE_CAPTURE_DEVICE");
    struct stat st;if(!stat(device?device:"/dev/smi",&st))smi_device=st.st_rdev;
    original=(read_fn)dlsym(RTLD_NEXT,"cariboulite_radio_read_samples");
    const char *p=getenv("CARIBOULITE_CAPTURE_PREFIX");
    if(!original||!p||!*p){fprintf(stderr,"capture: missing library symbol or output prefix\n");_exit(98);}
    if(snprintf(prefix,sizeof(prefix),"%s",p)>=(int)sizeof(prefix)-20)_exit(98);
    const char *n=getenv("CARIBOULITE_CAPTURE_SAMPLES");
    capacity=n?strtoull(n,NULL,10):8000000;
    if(!capacity||capacity>32000000)_exit(98);
    iq=allocate(capacity*4);meta=allocate(capacity);calls=allocate(MAX_CALLS*sizeof(*calls));
    raw_bytes=allocate(capacity*4);raw_calls=allocate(MAX_CALLS*sizeof(*raw_calls));
    struct sigaction a={0};a.sa_handler=arm;sigemptyset(&a.sa_mask);a.sa_flags=SA_RESTART;
    if(sigaction(SIGUSR1,&a,NULL))_exit(98);
    fprintf(stderr,"capture: prefaulted %zu IQ samples; SIGUSR1 arms one bounded capture\n",capacity);
}
ssize_t read(int fd,void *buffer,size_t length)
{
    uint64_t start=active?ns():0;
    ssize_t result=original_read?original_read(fd,buffer,length):syscall(SYS_read,fd,buffer,length);
    if(active&&result>0&&smi_device){
        if(smi_fd<0){struct stat st;if(!fstat(fd,&st)&&S_ISCHR(st.st_mode)&&st.st_rdev==smi_device)smi_fd=fd;}
        if(fd==smi_fd){
            uint64_t end=ns();size_t take=(size_t)result;
            if(take>capacity*4-raw_used)take=capacity*4-raw_used;
            if(raw_ncalls<MAX_CALLS)raw_calls[raw_ncalls++]=(call_t){start,end,raw_used,length,(int32_t)result,(uint32_t)take};
            if(take){memcpy(raw_bytes+raw_used,buffer,take);raw_used+=take;}
        }
    }
    return result;
}
int cariboulite_radio_read_samples(void *radio,void *buffer,void *metadata,size_t length)
{
    if(requested&&!ever){active=1;ever=1;requested=0;}
    uint64_t start=active?ns():0;
    int result=original(radio,buffer,metadata,length);
    if(active){
        uint64_t end=ns();size_t take=result>0?(size_t)result:0;
        if(take>capacity-used)take=capacity-used;
        if(ncalls<MAX_CALLS)calls[ncalls++]=(call_t){start,end,used,length,result,(uint32_t)take};
        if(take){memcpy(iq+4*used,buffer,4*take);if(metadata)memcpy(meta+used,metadata,take);else memset(meta+used,0,take);used+=take;}
        if(used==capacity)active=0;
    }
    return result;
}
static void save(const char *suffix,const void *data,size_t count)
{
    char path[PATH_MAX];snprintf(path,sizeof(path),"%s%s",prefix,suffix);
    FILE *f=fopen(path,"wb");if(!f){perror(path);return;}
    if(fwrite(data,1,count,f)!=count)perror("capture write");
    fclose(f);
}
__attribute__((destructor)) static void finish(void)
{
    if(!ever)return;
    save(".iq",iq,4*used);save(".meta",meta,used);save(".calls",calls,ncalls*sizeof(*calls));
    save(".smi",raw_bytes,raw_used);save(".smi_calls",raw_calls,raw_ncalls*sizeof(*raw_calls));
    char info[512];int n=snprintf(info,sizeof(info),"{\"samples\":%zu,\"calls\":%zu,\"capacity\":%zu,\"sample_layout\":\"little-endian int16 I,Q\",\"call_record_bytes\":%zu,\"raw_smi_bytes\":%zu,\"raw_read_calls\":%zu}\n",used,ncalls,capacity,sizeof(call_t),raw_used,raw_ncalls);
    if(n>0)save(".json",info,(size_t)n);
    fprintf(stderr,"capture: saved %zu samples, %zu read records at shutdown\n",used,ncalls);
}
