# NAP Recovery Installer v1.0.1 — guarded rehearsal package

**Fix in installer v1.0.1:** accepts a correct SHA-256 sidecar produced on the comma even when its filename includes the original absolute `/data/nap_recovery/` directory; it still checks the archive digest and refuses mismatched filenames. No changes to NAP GitHub release pins.

**Purpose:** Package and independently verify the three pinned NAP GitHub releases, all 249 Openpilot LFS files, the private configuration backup, and the ARM64 runtime bundle, producing a fresh **isolated installation preview**. Designed for a Mac workstation first. **This is not an unattended installer, road-ready build, or vehicle-control activation script.**

## Required files already copied to your Mac

Existing files in `~/NAP-Recovery/`:

- `nap-private-recovery-20261020T032742Z.tar.gz` and matching `.sha256`
- `nap-runtime-aarch64-20261020T040848Z.tar.gz` and matching `.sha256`

These **private archives are not included in this ZIP**. Keep them encrypted and do not share or commit them.

## Quick start on your Mac

1. Unzip `NAP-Recovery-Installer-v1.0.1.zip` into `~/NAP-Recovery/`. That creates `~/NAP-Recovery/NAP-Recovery-Installer-v1.0.1/`.
2. Confirm dependencies: `python3 --version` (>=3.10), `git --version`, and `git lfs version`. If Git LFS is missing, install it via your Mac's package manager (for example Homebrew: `brew install git-lfs && git lfs install`).
3. Open Terminal and run:

```sh
sh ~/NAP-Recovery/NAP-Recovery-Installer-v1.0.1/run_rehearsal.sh check
```

This performs archive and manifest checks **without extracting**.

4. If it passes and you have adequate disk space/network access, run:

```sh
sh ~/NAP-Recovery/NAP-Recovery-Installer-v1.0.1/run_rehearsal.sh prepare
```

This clones and verifies pinned release commits and all 251 LFS files, copies runtime executables into an isolated, non-running target layout, and extracts private settings **only into `PRIVATE_REVIEW/`**. The command refuses to overwrite an existing destination and never installs into active `/data`. Allow space for Git objects, LFS assets and binary copies.

Output will be in `~/NAP-Recovery/NAP-restoration-preview/`:

```text
NAP-restoration-preview/
  data/
    openpilot/                     GitHub NAP-v1.0.2 source
      opendbc_repo/                Custom pinned opendbc repository
    server/                        NAP-Dash-v1.0.1 source
    tailscale/bin/tailscale        Restored executable copy
    tailscale/bin/tailscaled       Restored executable copy
    ffmpeg-7.0.2-arm64-static/ffmpeg
  PRIVATE_REVIEW/                  PRIVATE configuration and parameters; NOT applied
  REHEARSAL_REPORT.json            Public version/verification metadata
  READ_BEFORE_ANY_INSTALL.txt     Deployment limitations and steps still to verify
```

The output is **not safe to publish**, because `PRIVATE_REVIEW/` contains sensitive settings and possibly credentials.

## Exact release pins

| Component | Tag | Commit |
|---|---|---|
| openpilot-preap | `NAP-v1.0.2` | `fb41fc71b4266e0a92a4550c51c2bfb1ef2a7e91` |
| opendbc-preap | `NAP-opendbc-v1.0.0` | `31da4137f2ce06702bc595a05689d537f046bb70` |
| NAP-Dash | `NAP-Dash-v1.0.1` | `a9eb509e96a2555ecf979149d8121ad72430695e` |

Openpilot is pinned to custom opendbc via gitlink `opendbc_repo`. NAP-Dash goes to `/data/server` on the target, **not `/data/NAP-Dash`**. The Git LFS endpoint in openpilot v1.0.1 points to GitHub; all 249 objects were previously verified in a fresh clone on the original comma.

## What this intentionally does NOT do

- Never overwrites `/data/openpilot`, `/data/server`, `/data/params` or any live installation.
- Does not start, stop, enable or modify `comma.service`, Tailscale, ngrok, or the openpilot manager.
- Does not restore Tailscale node state or automatically authenticate a replacement device.
- Does not inject old calibration or vehicle parameters into a live car.
- Does not build platform dependencies, initialize other submodules, reflash firmware, validate safety devices, or test real driving/ADAS behavior.
- Does not automatically install system paths such as `/usr/local/venv/bin/ffmpeg` or `/usr/comma/comma.sh`.

**A later deployment procedure must include a parked/offline preflight, hardware and firmware compatibility, build/bootstrap validation, safety testing, explicit rollback, and a deliberate activation decision.** Do not use an isolated preview for driving.

## Included toolkit for the original comma

`/scripts/nap_recovery.py` is an updated copy of the recovery script, with the latest three pins; it supports `audit`, `startup`, `backup`, `verify`, `stage`. On the original comma, use the existing installed script in `/data/nap_tools` unless intentionally updating. Note: an original car at openpilot v1.0.0 will show `release=NO` after pinning v1.0.1; that is expected because v1.0.1 changes only `.lfsconfig`.

## Tests

```sh
cd ~/NAP-Recovery/NAP-Recovery-Installer-v1.0.1/scripts
python3 -m unittest -v test_nap_recovery test_nap_package
```

Tests use temporary fake Git repositories and synthetic archives. They do not connect to GitHub or require a car.

## Security / troubleshooting

- Workstation destination is created with restrictive permissions. Keep FileVault or other at-rest encryption enabled; UNIX permissions alone are not encryption.
- Backups and checksum sidecars must match; the script validates archive membership and extracted-file SHA-256 checksums and rejects unsafe archive paths and links.
- If you get a Git LFS failure, confirm `git lfs version`; do not use `GIT_LFS_SKIP_SMUDGE=1` for the full package verification.
- If cloning fails, the temporary staging directory is removed; nothing is installed or activated.
- Run `check` again after transferring/restoring archives to another computer.
- Do not add NAP endpoint hostnames or Tailscale state to public GitHub. Historical references may still exist in caches/clones.
