#ifndef SMI_RAW_CAPTURE_H
#define SMI_RAW_CAPTURE_H
/* Private, process-local diagnostic. One SMI owner/reader per process.
 * No public ABI changes. Caller parks reader before close, as for SMI buffers.
 * Capture first 1 MiB of normal RX reads; never writes disk in receive path. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define SMI_RAW_CAPTURE_LIMIT (1024u * 1024u)
static struct {
    const void *owner;
    unsigned char *bytes;
    size_t used;
    int fd;
} smi_raw_capture;
static void smi_raw_capture_open(const void *owner)
{
    const char *path = getenv("CARIBOULITE_SMI_RAW_CAPTURE");
    if (!path || !*path) return;
    if (smi_raw_capture.owner) {
        fprintf(stderr,"SMI raw capture: another owner already active\n");
        return;
    }
    unsigned char *bytes = malloc(SMI_RAW_CAPTURE_LIMIT);
    if (!bytes) { fprintf(stderr,"SMI raw capture: allocation failed\n"); return; }
    int fd = open(path, O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW, 0600);
    if (fd < 0) {
        fprintf(stderr,"SMI raw capture: cannot create %s: %s\n",path,strerror(errno));
        free(bytes); return;
    }
    smi_raw_capture.owner=owner; smi_raw_capture.bytes=bytes;
    smi_raw_capture.used=0; smi_raw_capture.fd=fd;
    fprintf(stderr,"SMI raw capture armed: %s (first %u bytes, saved on clean close)\n",
        path,SMI_RAW_CAPTURE_LIMIT);
}
static void smi_raw_capture_append(const void *owner, const void *data, size_t size)
{
    if (smi_raw_capture.owner != owner || !smi_raw_capture.bytes) return;
    size_t room=SMI_RAW_CAPTURE_LIMIT-smi_raw_capture.used;
    if (size>room) size=room;
    if (size) memcpy(smi_raw_capture.bytes+smi_raw_capture.used,data,size);
    smi_raw_capture.used+=size;
}
static void smi_raw_capture_close(const void *owner)
{
    if (smi_raw_capture.owner != owner || !smi_raw_capture.bytes) return;
    size_t saved=0;
    while (saved<smi_raw_capture.used) {
        ssize_t n=write(smi_raw_capture.fd,smi_raw_capture.bytes+saved,smi_raw_capture.used-saved);
        if (n<0 && errno==EINTR) continue;
        if (n<=0) { fprintf(stderr,"SMI raw capture: write failed: %s\n",strerror(errno)); break; }
        saved+=(size_t)n;
    }
    if (close(smi_raw_capture.fd)) fprintf(stderr,"SMI raw capture: close failed: %s\n",strerror(errno));
    fprintf(stderr,"SMI raw capture: saved %zu/%zu bytes\n",saved,smi_raw_capture.used);
    free(smi_raw_capture.bytes);
    memset(&smi_raw_capture,0,sizeof(smi_raw_capture));
}
#endif
