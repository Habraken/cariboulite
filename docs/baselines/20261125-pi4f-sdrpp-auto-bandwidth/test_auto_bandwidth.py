#!/usr/bin/env python3
"""Compile SDR++'s actual selection method; no hardware or SDR++ dependencies."""
from pathlib import Path
import argparse
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, required=True)
args = parser.parse_args()
source = args.source.read_text()
method = source[source.index('    float selectBwBySr('):source.index('    void selectSampleRate(')]
unit = r'''
#include <algorithm>
#include <cstdio>
#include <vector>
namespace flog { template<class... T> void info(T...) {} }
struct Probe {
    // Real HiF bandwidth list, including SDR++'s Auto sentinel.
    std::vector<float> bandwidthList = {-1,20000,50000,100000,160000,200000,
        250000,320000,400000,500000,630000,800000,1000000,1250000,1600000,2000000};
'''+method+r'''
};
int main() {
    Probe p;
    struct Case { double rate; float expected; };
    const Case cases[] = {
        {0,20000}, {19999,20000}, {20000,20000}, {20001,50000},
        {400000,400000}, {500000,500000}, {2000000./3,800000},
        {800000,800000}, {1000000,1000000}, {4000000./3,1600000},
        {1999999,2000000}, {2000000,2000000}, {2000001,2000000},
        {4000000,2000000}, {8000000,2000000}
    };
    for (int pass=0;pass<2;++pass) for (unsigned j=0;j<sizeof(cases)/sizeof(cases[0]);++j) {
        unsigned i=pass ? sizeof(cases)/sizeof(cases[0])-1-j : j;
        const auto &c=cases[i]; float got=p.selectBwBySr(c.rate);
        if (got!=c.expected) {
            std::fprintf(stderr,"rate %.9g: expected bandwidth %.9g, got %.9g\n",c.rate,c.expected,got);
            return 1;
        }
    }
    // Different device bandwidth limits, with the same sorted-list contract.
    p.bandwidthList={-1,100000,300000};
    if(p.selectBwBySr(200000)!=300000 || p.selectBwBySr(4000000)!=300000) return 2;
    std::puts("PASS: native rates, boundary cases, reverse transitions and alternate device limit");
}
'''
with tempfile.TemporaryDirectory(prefix='sdrpp-auto-bw-') as directory:
    path=Path(directory)
    (path/'test.cpp').write_text(unit)
    subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror',str(path/'test.cpp'),'-o',str(path/'test')],check=True)
    subprocess.run([str(path/'test')],check=True,timeout=10)
