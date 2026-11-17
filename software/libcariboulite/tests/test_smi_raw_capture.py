#!/usr/bin/env python3
"""Hardware-free capture lifecycle and known framing fault regressions."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('analyzer', ROOT/'software/libcariboulite/tools/analyze_smi_raw.py')
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


class Tests(unittest.TestCase):
    def test_framing_and_shift(self):
        good = (0x80004000).to_bytes(4, 'little')*520
        result = analyzer.analyze(b'X'+good)
        self.assertEqual(result['phases'][1]['longest_valid_run'], 520)
        self.assertEqual(result['phases'][1]['framing_bit_errors'], {str(b):0 for b in analyzer.BITS})
        bad = analyzer.analyze((0x80004000 ^ (1<<30)).to_bytes(4, 'little')*520)
        self.assertEqual(bad['phases'][0]['valid_words'], 0)
        self.assertEqual(bad['phases'][0]['framing_bit_errors']['30'], 520)
        self.assertEqual(analyzer.analyze(b'')['phases'][0]['words'], 0)

    def test_byte_order_and_repetition(self):
        original = (0x80004000).to_bytes(4, 'little')
        swapped = bytes(original[i] for i in (2,3,0,1))*520
        result = analyzer.analyze(swapped)
        self.assertEqual(result['byte_order_hypotheses']['swap_halfwords'][0]['longest_valid_run'], 520)
        zeros = analyzer.analyze(bytes(64))['phases'][0]
        self.assertEqual(zeros['zero_words'], 16)
        self.assertEqual(zeros['adjacent_repeated_words'], 15)
        self.assertEqual(zeros['stuck_zero_bits'], list(range(32)))

    def test_capture(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            source = d/'test.c'
            source.write_text('''#include "smi_raw_capture.h"
#include <assert.h>
int main(int argc, char **argv) {
 (void)argc; int owner=0, other=0; unsigned char data[4096];
 memset(data,0xa5,sizeof(data));
 unsetenv("CARIBOULITE_SMI_RAW_CAPTURE"); smi_raw_capture_open(&owner);
 smi_raw_capture_append(&owner,data,sizeof(data)); assert(!smi_raw_capture.owner);
 setenv("CARIBOULITE_SMI_RAW_CAPTURE",argv[1],1); smi_raw_capture_open(&owner);
 assert(smi_raw_capture.owner==&owner);
 smi_raw_capture_append(&other,data,4); assert(!smi_raw_capture.used);
 for(int i=0;i<300;i++) smi_raw_capture_append(&owner,data,sizeof(data));
 assert(smi_raw_capture.used==SMI_RAW_CAPTURE_LIMIT);
 smi_raw_capture_close(&other); assert(smi_raw_capture.owner==&owner);
 smi_raw_capture_close(&owner); smi_raw_capture_close(&owner);
 smi_raw_capture_open(&owner); assert(!smi_raw_capture.owner);
 return 0;
}
''')
            subprocess.run(['cc','-D_GNU_SOURCE','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined',
                            '-I'+str(ROOT/'software/libcariboulite/src/caribou_smi'),str(source),'-o',str(d/'test')],check=True)
            subprocess.run([str(d/'test'),str(d/'raw.bin')],check=True)
            self.assertEqual((d/'raw.bin').read_bytes(), b'\xa5'*(1024*1024))


if __name__ == '__main__':
    unittest.main()
