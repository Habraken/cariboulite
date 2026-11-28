#!/usr/bin/env python3
"""Replay captured little-endian IQ16 through the production CW demodulator.

Requires numpy and a shared library built from nbfm_demod.c,
nbfm_demod_dsp.c, wbfm_demod.c and am_demod.c. This does not access a radio.
Select complete keyed 100 ms windows by level above noise and sine-like crest,
excluding the initial 100 ms of filter settling. Silent gaps are not averaged
into the level reference. The digital shift emulates centring the RF receiver.
"""
import argparse
import ctypes as C
import hashlib
import json
from pathlib import Path
import numpy as np

class Config(C.Structure):
    _fields_ = [('rf_rate', C.c_uint), ('audio_rate', C.c_uint),
                ('deemph_tau', C.c_float), ('pcm_gain', C.c_float)]

class Result(C.Structure):
    _fields_ = [('consumed', C.c_size_t), ('produced', C.c_size_t), ('error', C.c_int)]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('iq', type=Path)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--shift', type=float, default=812)
    p.add_argument('--trim', type=float, default=35.5)
    args = p.parse_args()
    lib = C.CDLL(str(args.library.resolve()))
    lib.audio_demod_create.argtypes = [C.c_int, C.POINTER(Config)]
    lib.audio_demod_create.restype = C.c_void_p
    lib.audio_demod_set_cw.argtypes = [C.c_void_p, C.c_uint, C.c_float, C.c_int, C.c_bool]
    lib.audio_demod_process_with_taps.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t,
        C.c_void_p, C.c_void_p, C.c_void_p, C.c_size_t, C.c_double]
    lib.audio_demod_process_with_taps.restype = Result
    lib.audio_demod_destroy.argtypes = [C.c_void_p]
    dsp = lib.audio_demod_create(5, C.byref(Config(1000000, 48000, 0, 8000)))
    assert dsp and lib.audio_demod_set_cw(dsp, 500, 600, 12500, False) == 0
    iq = np.memmap(args.iq, dtype='<i2').reshape(-1, 2)
    pcm = np.empty(50000, np.int16)
    filtered = np.empty(50000, np.float32)
    pending = np.empty(0)
    windows = []
    total = 0
    digest = hashlib.sha256()
    try:
        for start in range(0, len(iq), 1000000):
            chunk = iq[start:start+1000000]
            digest.update(chunk.tobytes())
            z = chunk[:, 0] + 1j*chunk[:, 1]
            z *= np.exp(-2j*np.pi*args.shift*np.arange(start, start+len(chunk))/1e6)
            shifted = np.rint(np.column_stack((z.real, z.imag))).astype(np.int16)
            r = lib.audio_demod_process_with_taps(dsp, shifted.ctypes.data, len(shifted),
                pcm.ctypes.data, None, filtered.ctypes.data, len(filtered), 0)
            assert not r.error and r.consumed == len(shifted)
            audio = np.concatenate((pending, filtered[:r.produced].astype(float)*11642/32768))
            n = len(audio)//4800
            for a in audio[:n*4800].reshape(-1, 4800):
                total += 1
                rms = float(20*np.log10(max(1e-12, np.sqrt(np.mean(a*a)))))
                peak = float(20*np.log10(max(1e-12, max(abs(a)))))
                if total > 1 and rms > -74 and peak-rms < 3.8:
                    out = np.rint(a*32768*10**(args.trim/20)).clip(-32768, 32767)/32768
                    windows.append(dict(window=total, neutral_rms_dbfs=rms, crest_db=peak-rms,
                        matched_rms_dbfs=float(20*np.log10(np.sqrt(np.mean(out*out))))))
            pending = audio[n*4800:]
    finally:
        lib.audio_demod_destroy(dsp)
    levels = [w['matched_rms_dbfs'] for w in windows]
    print(json.dumps(dict(iq_file=args.iq.name, sha256=digest.hexdigest(),
        iq_samples=len(iq), rf_rate_hz=1000000, cw_bandwidth_hz=500, pitch_hz=600,
        digital_recentering_hz=args.shift, matched_trim_db=args.trim, volume_db=0,
        selection=dict(min_neutral_rms_dbfs=-74, max_crest_db=3.8, window_ms=100,
                       initial_settling_windows=1),
        total_windows=total, keyed_windows=len(windows),
        median_matched_rms_dbfs=float(np.median(levels)) if levels else None,
        matched_rms_percentiles_5_95_dbfs=list(np.percentile(levels,[5,95])) if levels else None,
        windows=windows), indent=2))

if __name__ == '__main__':
    main()
