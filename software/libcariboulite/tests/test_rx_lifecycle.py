#!/usr/bin/env python3
"""Compile actual app pipeline code; mock radio/thread lifecycle, use ALSA null.
FIFO cancellation checks use real pthreads. No radio hardware is accessed.
"""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[3]
src = root / 'software/libcariboulite/src'
with tempfile.TemporaryDirectory(prefix='rx-lifecycle-') as directory:
    binary = Path(directory) / 'test'
    wraps = ['cariboulite_radio_set_rx_sample_rate_flt','cariboulite_radio_set_tx_samp_cutoff_flt','cariboulite_radio_get_tx_samp_cutoff_flt','caribou_fpga_get_sys_ctrl_tx_sample_gap','cariboulite_radio_set_tx_power','malloc','calloc','free','cariboulite_radio_read_samples','pthread_create','pthread_cancel','pthread_join',
             'cariboulite_radio_set_frequency','cariboulite_radio_activate_channel',
             'caribou_smi_set_driver_streaming_state','caribou_fpga_set_io_ctrl_mode']
    includes = [src, src/'at86rf215', src/'rffc507x']
    command = ['cc', '-O0', '-g', '-ffunction-sections', '-fdata-sections',
               '-Wno-unused-parameter', *['-I'+str(p) for p in includes],
               str(Path(__file__).with_suffix('.c')), str(src/'nbfm4m_mod.c'), str(src/'alsa48k_source.c'), '-Wl,--gc-sections',
               *['-Wl,--wrap='+s for s in wraps], '-L'+str(root/'build'),
               '-Wl,-rpath,'+str(root/'build'), '-lcariboulite', '-lasound',
               '-lm', '-pthread', '-o', str(binary)]
    subprocess.run(command, check=True)
    subprocess.run([str(binary)], check=True, timeout=30)
