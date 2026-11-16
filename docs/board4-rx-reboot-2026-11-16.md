# Board #4 — interrupted post-reboot test — 16 November 2026

Stopped at the user's request after 33 completed points to prioritize device #5.
Both radios were stopped and released; diagnosis of #4 is deferred.

The preceding [full retest](board4-rx-retest-2026-11-16.md), after the user's
manual test-app/LED/RX session, completed all 120 points with zero recorded
transport errors. Its 2,877 retained measurement snapshots reconcile independently.

Pi4f was then rebooted, with boot ID changing from
`96b58652-1b49-448b-9ec7-8197381434c7` to
`871ce5c1-f034-405a-a151-dab13401a1bd`. No manual test-app or LED initialization
was performed after reboot. The first 4 MS/s startup failed; a subsequent
collector attempt reproduced that failure and also failed at 2 MS/s. Lower-rate
measurements encountered invalid frames at 4/3 MS/s and 1 MS/s. Only three
800 kS/s points completed before interruption; the other rates were not measured
in this post-reboot run. No full post-reboot qualification is claimed.

These observations suggest startup/history-dependent behavior but do not establish
its cause. No production software or EEPROM changes were made. The original
failed runs and later full pass remain separate evidence. No TX or oscillator
qualification was performed on #4.

[Partial data, startup failures, reboot and release evidence](baselines/20261116-board4-rx-reboot/README.md)
