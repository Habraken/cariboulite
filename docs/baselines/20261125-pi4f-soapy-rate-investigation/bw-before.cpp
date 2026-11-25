#include <vector>
#include <algorithm>
#include <cassert>
#include <cstdio>
namespace flog { template<class... T> void info(T...) {} }
struct Probe { std::vector<float> bandwidthList={-1,20000,50000,100000,160000,200000,250000,320000,400000,500000,630000,800000,1000000,1250000,1600000,2000000};
    float selectBwBySr(double samplerate) {
        float cur = bandwidthList[1];
        std::vector<float> bwListReversed = bandwidthList;
        std::reverse(bwListReversed.begin(), bwListReversed.end());
        for (auto bw : bwListReversed) {
            if (bw >= samplerate) {
                cur = bw;
            }
            else {
                break;
            }
        }
        flog::info("Bandwidth for samplerate {0} is {1}", samplerate, cur);
        return cur;
    }

};
int main(){Probe p;assert(p.selectBwBySr(2000000)==2000000);assert(p.selectBwBySr(1000000)==1000000);assert(p.selectBwBySr(4e6/3)==1600000);assert(p.selectBwBySr(4000000)==20000); puts("Original Auto-bandwidth behavior verified");}
