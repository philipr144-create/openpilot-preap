# NAP Release and Recovery Status — 2026-10-20

## Proven by the original comma/Mac terminal transcript

- GitHub tags and SHAs recorded in the README were reached and verified.
- NAP v1.0.1 fresh clone retrieved all 251 LFS assets; `git lfs fsck OK`.
- Custom opendbc has zero LFS-tracked assets.
- NAP-Dash Tailscale manager source now reads a private configuration file, but previously published history may remain cached or available elsewhere.
- Private archive 20261020T032742Z contained 243 payload files and passed the toolkit's internal manifest verification.
- Runtime ARM64 archive 20261020T040848Z contained the exact three FFmpeg/Tailscale binaries and was checksum-verified.
- Both archives transferred to user's Mac `~/NAP-Recovery/` and both external SHA-256 checks passed.
- Original comma's active startup chain was observed: `comma.service` -> `/usr/comma/comma.sh` -> `/data/continue.sh` -> `/data/openpilot/launch_openpilot.sh` -> openpilot manager. Custom manager includes `server.server_v21` and `server.tailscaled_daemon`.

## Outstanding for a replacement comma (NOT verified)

- Exact comma OS image/ABI compatibility, Python dependencies and build tools.
- Other submodules, generated/native build artifacts, MCU/panda/pedal/EPAS/harness firmware safety interfaces.
- Platform paths and permission semantics (`/usr/comma/comma.sh`, `ffmpeg` in PATH, startup symlink and system service).
- Remote access reauthentication: do not duplicate existing Tailscale identity. A new hostname may require updating `/data/nap_settings/nap_tailscale_public_url`.
- Tailscale's separate system and user-space daemon arrangement must be reviewed; do not disable either without understanding active function.
- Safe restoration of vehicle-specific `/data/params`, calibration, navigation, battery data and private settings.
- Post-install safety validation while parked, independent hardware verification and rollback procedure. No control activation is performed by this package.
- A second off-device encrypted backup is recommended.

## No live openpilot upgrade required

The running `/data/openpilot` was reported at `7c51a9d...` while the latest public release was `2043306...`. The v1.0.1 diff changes only `.lfsconfig` for future GitHub LFS recovery; it does not require an update to active driving controls.
