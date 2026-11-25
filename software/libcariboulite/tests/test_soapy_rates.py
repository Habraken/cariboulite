#!/usr/bin/env python3
"""Compile production rate methods with mock radio calls; no Soapy/hardware needed."""
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
src = here.parent / 'src'
source = (src / 'soapy_api/Cariboulite.cpp').read_text()
methods = source[source.index('void Cariboulite::setSampleRate('):source.index('static cariboulite_radio_rx_bw_en convertRxBandwidth(')]
with tempfile.TemporaryDirectory(prefix='soapy-rates-') as directory:
    unit = Path(directory) / 'rates.cpp'
    unit.write_text((here / 'test_soapy_rates.cpp').read_text() + '\n' + methods)
    exe = Path(directory) / 'rates'
    subprocess.run(['c++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-I' + str(src), str(unit), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=10)
