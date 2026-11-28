/* Exercise the production controller handler against a real dry-run backend. */
#define main controller_program_main
#include "../tools/radio_backend/radio_controller.c"
#undef main
#include <assert.h>

static void wait_requested(controller_t* c, bool requested)
{
    double end = monotonic_seconds() + 3;
    for (;;) {
        assert(!refresh_state(c));
        if (c->state.tx_requested == requested && !c->state.transition) return;
        assert(monotonic_seconds() < end);
        struct timespec delay = {.tv_nsec = 10000000}; nanosleep(&delay, NULL);
    }
}
static void initialize(headset_ptt_t* h,const char* path,unsigned vendor,unsigned product)
{
    FILE* f=fopen(path,"r");assert(f);uint8_t data[HEADSET_REPORT_MAX];size_t n=0;unsigned v;
    while(fscanf(f,"%2x",&v)==1) { assert(n<sizeof(data));data[n++]=(uint8_t)v; }
    fclose(f);assert(!headset_ptt_init(h,data,n,vendor,product));
}
static int report(controller_t* c,headset_ptt_t* h,const uint8_t* r,size_t n,double now,bool observe)
{
    headset_ptt_feed(h,r,n,now);
    return headset_action(c,headset_ptt_poll(h,now+.020),observe);
}
static void jabra(controller_t* c,headset_ptt_t* h,bool green,double now)
{
    uint8_t first[]={3,green?0x22:3,0},last[]={3,2,0};
    headset_ptt_feed(h,first,3,now);
    assert(headset_ptt_poll(h,now+.0005)==HEADSET_NONE);
    headset_ptt_feed(h,last,3,now+.001);
    assert(!headset_action(c,headset_ptt_poll(h,now+.020),false));
}
int main(int argc, char** argv)
{
    assert(argc == 4);
    controller_t c = {.socket=-1, .path=argv[1], .timeout=2};
    headset_ptt_t h; initialize(&h,argv[2],0x046d,0x0a46);
    const uint8_t call[]={8,1}, idle[]={8,3}, mute[]={8,2}, both[]={8,0}, volume[]={1,1};
    assert(!refresh_state(&c) && !c.state.tx_requested);
    uint64_t generation=c.state.config_generation;
    assert(!report(&c,&h,call,2,0,true));
    assert(!report(&c,&h,mute,2,1,false));
    assert(!report(&c,&h,idle,2,1,false));
    assert(!report(&c,&h,both,2,1,false));
    assert(!report(&c,&h,volume,2,1,false));
    assert(!report(&c,&h,call,1,1,false));
    assert(!refresh_state(&c) && c.state.config_generation==generation);
    assert(!report(&c,&h,call,2,2,false));
    assert(c.state.tx_requested);
    assert(!report(&c,&h,idle,2,2.001,false));
    assert(!report(&c,&h,call,2,2.05,false)); /* duplicate cannot unkey */
    assert(!refresh_state(&c) && c.state.tx_requested);
    assert(!report(&c,&h,call,2,2.4,false)); /* second click stops even during start */
    wait_requested(&c,false);
    assert(!strcmp(c.state.state,"RX"));
    assert(!report(&c,&h,call,2,3,false));
    wait_requested(&c,true);
    disconnect_controller(&c); /* same cleanup used on physical unplug/exit */
    struct timespec delay={.tv_nsec=100000000};nanosleep(&delay,NULL);
    wait_requested(&c,false);
    assert(!strcmp(c.state.state,"RX"));
    control_request_t req={.command=CONTROL_TX_MODE,.tx_mode=AUDIO_MOD_CW};
    assert(!exchange(&c,&req));
    assert(!report(&c,&h,call,2,4,false));
    assert(!c.state.ok && !strcmp(c.state.code,"CW_DISABLED") && !c.state.tx_requested);
    req=(control_request_t){.command=CONTROL_TX_MODE,.tx_mode=AUDIO_MOD_NBFM};assert(!exchange(&c,&req));
    req=(control_request_t){.command=CONTROL_CW_SETUP,.cw_type=CW_IAMBIC,.cw_wpm=12,.cw_practice=true,.cw_hang=250,.cw_ramp=5};assert(!exchange(&c,&req));
    req=(control_request_t){.command=CONTROL_CW_ENABLE,.cw_enabled=true};assert(!exchange(&c,&req));
    assert(!report(&c,&h,call,2,5,false));
    assert(!c.state.ok && !strcmp(c.state.code,"CW_OWNS_TX") && !c.state.tx_requested);
    req=(control_request_t){.command=CONTROL_CW_ENABLE,.cw_enabled=false};assert(!exchange(&c,&req));
    req=(control_request_t){.command=CONTROL_STOP};assert(!exchange(&c,&req));wait_requested(&c,false);
    assert(!strcmp(c.state.state,"IDLE"));
    initialize(&h,argv[3],0x0b0e,0x0420);
    uint8_t baseline[]={3,3,0};headset_ptt_prime(&h,baseline,3);
    generation=c.state.config_generation;
    jabra(&c,&h,false,10); /* red from idle never starts TX */
    assert(!c.state.tx_requested&&c.state.config_generation==generation);
    jabra(&c,&h,false,11);assert(!c.state.tx_requested);
    jabra(&c,&h,true,12);wait_requested(&c,true);
    jabra(&c,&h,false,13);wait_requested(&c,false);
    assert(!strcmp(c.state.state,"IDLE")); /* preserve previous RX-off */
    req=(control_request_t){.command=CONTROL_START};assert(!exchange(&c,&req));wait_requested(&c,false);
    jabra(&c,&h,true,14);wait_requested(&c,true);
    jabra(&c,&h,true,15);wait_requested(&c,false);
    assert(!strcmp(c.state.state,"RX")); /* green also toggles TX off */
    generation=c.state.config_generation;jabra(&c,&h,false,16);
    assert(!c.state.tx_requested&&c.state.config_generation==generation);
    jabra(&c,&h,true,17);wait_requested(&c,true);
    jabra(&c,&h,false,18);wait_requested(&c,false);
    assert(!strcmp(c.state.state,"RX"));
    disconnect_controller(&c);
    puts("Headset PTT: H650e/Jabra uniform toggles, red never starts TX, RX/idle restoration, disconnect cleanup and CW interlocks PASS");
    return 0;
}
