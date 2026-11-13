from pathlib import Path
base=Path('build/radio-backend/stutter-diagnosis-20261113')
src=Path('software/libcariboulite/src/caribou_smi/caribou_smi.c').read_text()
start=src.index('    if (dev->debug_mode == caribou_smi_none)',src.index('static int caribou_smi_find_buffer_offset'))
end=src.index('    else if (dev->debug_mode == caribou_smi_push',start)
branch='''    if (dev->debug_mode == caribou_smi_none)
    {
        /* DIAGNOSTIC CANDIDATE: select the byte phase with greatest whole-read
         * framing agreement, rather than accepting four coincidental headers.
         * A previous phase breaks equal-score ties on short partial reads.
         * This intentionally leaves the legacy sample-padding contract alone.
         * It is not a complete streaming-fragment preservation fix. */
        int best = -1;
        size_t best_valid = 0, best_count = 1;
        for (unsigned phase = 0; phase < 4; ++phase) {
            size_t count = (len - phase) / 4, valid = 0;
            for (size_t j = 0; j < count; ++j) {
                uint32_t word;
                memcpy(&word, buffer + phase + 4*j, sizeof(word));
                valid += (word & 0xC001C000u) == 0x80004000u;
            }
            uint64_t lhs = (uint64_t)valid * best_count;
            uint64_t rhs = (uint64_t)best_valid * count;
            if (count && (best < 0 || lhs > rhs ||
                (lhs == rhs && (int)phase == diag_last_phase))) {
                best = (int)phase; best_valid = valid; best_count = count;
            }
        }
        if (best < 0 || best_valid * 100 < best_count * 95) return -1;
        int legacy = -1;
        for (unsigned phase = 0; phase < 4 && phase + 16 <= len && legacy < 0; ++phase) {
            bool match = true;
            for (unsigned j = 0; j < 4; ++j) {
                uint32_t word; memcpy(&word, buffer + phase + 4*j, sizeof(word));
                if ((word & 0xC001C000u) != 0x80004000u) match = false;
            }
            if (match) legacy = (int)phase;
        }
        ++diag_phase_counts[best];
        if (legacy != best) ++diag_false_locks_avoided;
        diag_last_phase = best;
        return best;
    }
'''
src=src[:start]+branch+src[end:]
marker='static int caribou_smi_find_buffer_offset'
i=src.index(marker)
src=src[:i]+'''static int diag_last_phase = -1;
static unsigned long long diag_phase_counts[4], diag_false_locks_avoided;
__attribute__((destructor)) static void diag_report_alignment(void)
{
    fprintf(stderr, "DIAG_ALIGNMENT phase_counts=%llu,%llu,%llu,%llu legacy_false_locks_avoided=%llu\\n",
        diag_phase_counts[0], diag_phase_counts[1], diag_phase_counts[2], diag_phase_counts[3], diag_false_locks_avoided);
}
'''+src[i:]
(base/'caribou_smi_candidate.c').write_text(src)
import difflib
(base/'consensus-candidate.patch').write_text(''.join(difflib.unified_diff(Path('software/libcariboulite/src/caribou_smi/caribou_smi.c').read_text().splitlines(True),src.splitlines(True),fromfile='caribou_smi.c',tofile='caribou_smi_candidate.c')))
