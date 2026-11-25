// The Python harness appends unmodified production set/get/list rate methods.
#include "cariboulite_radio.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
#define SOAPY_SDR_RX 1
#define SOAPY_SDR_TX 0
#define SOAPY_SDR_WARNING 4
static unsigned warnings, writes;
static void SoapySDR_logf(int, const char *, ...) { ++warnings; }
extern "C" int cariboulite_radio_set_rx_samp_cutoff(cariboulite_radio_state_st *r, cariboulite_radio_sample_rate_en fs, cariboulite_radio_f_cut_en cut) {
    ++writes; r->rx_fs=fs; r->rx_fcut=cut; return 0;
}
extern "C" int cariboulite_radio_set_tx_samp_cutoff(cariboulite_radio_state_st *r, cariboulite_radio_sample_rate_en fs, cariboulite_radio_f_cut_en cut) {
    ++writes; r->tx_fs=fs; r->tx_fcut=cut; return 0;
}
extern "C" int cariboulite_radio_get_rx_samp_cutoff(cariboulite_radio_state_st *r, cariboulite_radio_sample_rate_en *fs, cariboulite_radio_f_cut_en *) { *fs=r->rx_fs; return 0; }
extern "C" int cariboulite_radio_get_tx_samp_cutoff(cariboulite_radio_state_st *r, cariboulite_radio_sample_rate_en *fs, cariboulite_radio_f_cut_en *) { *fs=r->tx_fs; return 0; }
class Cariboulite {
public:
    cariboulite_radio_state_st state = {};
    cariboulite_radio_state_st *radio = &state;
    void setSampleRate(int, size_t, double);
    double getSampleRate(int, size_t) const;
    std::vector<double> listSampleRates(int, size_t) const;
};
int main() {
    Cariboulite dev;
    dev.state.rx_fcut=cariboulite_radio_rx_f_cut_0_25_half_fs;
    dev.state.tx_fcut=cariboulite_radio_rx_f_cut_0_75_half_fs;
    // Regression: a 1 MS/s request must not program the 1 1/3 MS/s divider.
    dev.setSampleRate(SOAPY_SDR_RX,0,1000000.);
    assert(dev.state.rx_fs==cariboulite_radio_rx_sample_rate_1000khz);
    assert(dev.getSampleRate(SOAPY_SDR_RX,0)==1000000.);
    const cariboulite_radio_sample_rate_en expected[] = {
        cariboulite_radio_rx_sample_rate_4000khz, cariboulite_radio_rx_sample_rate_2000khz,
        cariboulite_radio_rx_sample_rate_1333khz, cariboulite_radio_rx_sample_rate_1000khz,
        cariboulite_radio_rx_sample_rate_800khz, cariboulite_radio_rx_sample_rate_666khz,
        cariboulite_radio_rx_sample_rate_500khz, cariboulite_radio_rx_sample_rate_400khz};
    for (int dir : {SOAPY_SDR_RX, SOAPY_SDR_TX}) {
        const auto rates=dev.listSampleRates(dir,0);
        assert(rates.size()==8);
        // Repeated transitions in both directions exercise every advertised rate.
        for (int pass=0; pass<2; ++pass) for (size_t j=0; j<rates.size(); ++j) {
            size_t i=pass ? rates.size()-1-j : j;
            auto other=dir==SOAPY_SDR_RX ? dev.state.tx_fs : dev.state.rx_fs;
            unsigned before=writes;
            dev.setSampleRate(dir,0,rates[i]);
            assert(writes==before+1);
            assert((dir==SOAPY_SDR_RX ? dev.state.rx_fs : dev.state.tx_fs)==expected[i]);
            assert((dir==SOAPY_SDR_RX ? dev.state.tx_fs : dev.state.rx_fs)==other);
            assert(std::fabs(dev.getSampleRate(dir,0)-rates[i])<1e-6);
            assert(warnings==0);
            assert(dev.state.rx_fcut==cariboulite_radio_rx_f_cut_0_25_half_fs);
            assert(dev.state.tx_fcut==cariboulite_radio_rx_f_cut_0_75_half_fs);
        }
    }
    for (int dir : {SOAPY_SDR_RX, SOAPY_SDR_TX}) for (double alias : {666000.,1333000.}) {
        unsigned before=warnings;
        dev.setSampleRate(dir,0,alias);
        assert(warnings==before+1);
        assert(std::fabs(dev.getSampleRate(dir,0)-(alias==666000. ? 2e6/3 : 4e6/3))<1e-6);
    }
    std::puts("Soapy rate selection: all native rates, transitions and legacy aliases passed");
}
