# Board #5 EEPROM and failed RX startup evidence

Receiver UUID: `b43713d5-7ebf-45ff-b327-aa2e0962c9e2` on pi4f.
Original pi4g TX reference: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`.

EEPROM programming/readback and post-reboot recognition passed. All eight native
RX rates failed startup with SMI synchronization errors. No power-sweep points
or RSSI/SINAD results exist. Both radios were released. See the
[report](../../board5-rx-qualification-2026-11-16.md).

- `eeprom-preflight.json`, `manifest.json`, `programming-receipt.json`: blank reads,
  unique image, CRC/tool validation, page verification and complete readbacks.
- `boot-identity.json`: post-reboot HAT identity, idle radio and driver provenance.
- `setup.json`, `metadata.json`: user-confirmed90dB link and collector settings.
- `startup-failures.json`, `startup-verification.json`: eight failures and independent checks.
- `release-check.json`: no remaining radio processes or SMI descriptors on either Pi.
- `source.sha256`, `raw-artifacts.sha256`: source and archive hashes.

Complete backups/images remain on pi4f in `build/board5-20261116/` and in pi4g's
`build/board5-20261116/eeprom-evidence.tar.gz`. Raw snapshots, role metadata,
commands and backend logs from both hosts are archived in
`build/board5-20261116/rx-startup-failures.tar.gz` on pi4g; its extracted directory
is `cariboulite-board5-rx-20261116T183417Z/` beside the archive. Transient IPC
sockets are excluded. No production software changes were made.

Offline verification from the repository root:

```sh
python3 docs/baselines/20261116-board5-rx/verify_startup.py \
  build/board5-20261116/cariboulite-board5-rx-20261116T183417Z
```

The collector/helper scripts acquire hardware and briefly transmit on the
original pi4g reference during setup. They guard board UUIDs/hosts, record startup
failures and restrict #5 to RX. They are not needed to inspect existing evidence.
