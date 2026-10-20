# NAP v1.0.0 — installation, verification, and recovery guide

**Applies to:** the custom `philipr144-create` NAP production snapshot published October 2026, initially developed for a 2013 Tesla Model S Pre-AP with a custom comma/pedal/radar/MCU2 integration.

> **Current status: source-reproducible, not one-click-installable.** All three versioned source repositories are published. The exact device service setup, ignored networking helpers, runtime settings, CAN hardware/firmware, and locally recorded data are **not** wholly preserved in Git. Treat a new install as a staged integration project, not a direct substitute for tested vehicle configuration. Never use these commands to rewrite an actively running `/data/openpilot` tree.

## 1. Exact v1.0.0 source manifest

| Item | Repository | Immutable reference |
| --- | --- | --- |
| NAP openpilot | https://github.com/philipr144-create/openpilot-preap | tag `NAP-v1.0.0` → `7c51a9d2c6a384cc0713601639ba4dfc64585ad2` |
| Custom openDBC | https://github.com/philipr144-create/opendbc-preap | submodule commit `31da4137f2ce06702bc595a05689d537f046bb70` |
| MCU2/dashboard server | https://github.com/philipr144-create/NAP-Dash | tag `NAP-Dash-v1.0.0` → `5aeb8de9d87f50a025da93b259ff25c9eddd518b` |

**`main` is the intended production branch in each repository.** For repeatable installs, use the tags and hash above rather than following moving `main`. An opendbc v1.0.0 tag was **not** confirmed; its submodule SHA is the authoritative pin.

Older versions remain available at `archive/main-before-NAP-74fbabb8` (openpilot) and `archive/legacy-main-896fc85` (dashboard). Do not install the upstream `NotAutopilot/openpilot/nap-release` URL assuming it contains this custom v1.0.0 build.

## 2. Confirm hardware before doing anything

A compatible installation needs careful validation of each element; availability varies by car:

- Early Tesla Model S **Pre-AP** electrical architecture; the release was developed for a **2013** car.
- Supported comma device with access to the appropriate openpilot runtime; correctly connected Tesla harness/CAN and Panda safety firmware.
- Compatible steering/EPAS integration. EPAS firmware changes are a separate hazardous procedure, *not* an automatic install step.
- **Comma Pedal** and correct CAN bus, calibration, and actuator behavior for NAP pedal-based longitudinal control.
- Compatible Bosch radar installation, calibration, alignment/offset, and appropriate signals for radar-based lead following, if used.
- Network connectivity among comma, MCU2 browser and optional phone navigation client.
- Sufficient storage for source, camera segments, offline road context, logs, and local SQLite history.

**Never engage controls, issue CAN test commands, flash EPAS, or run manual Air Ride commands as part of a blind software-install procedure.** A capable installer must verify wiring, firmware, calibration, hazards, and disengagement behavior separately.

## 3. Fetch the exact source without touching a running car

Recommended: perform this step on a laptop or an unused device, or in a new, separate staging directory. This does not start services or change a car.

```bash
mkdir -p "$HOME/nap-v1-restore-check"
cd "$HOME/nap-v1-restore-check"

git clone --branch NAP-v1.0.0 --recurse-submodules \
  https://github.com/philipr144-create/openpilot-preap.git openpilot

git clone --branch NAP-Dash-v1.0.0 \
  https://github.com/philipr144-create/NAP-Dash.git server

# Fetch remaining submodules if any were missed:
git -C openpilot submodule update --init --recursive

# Verify pinned references:
git -C openpilot rev-parse HEAD
git -C openpilot rev-parse HEAD:opendbc_repo
git -C openpilot submodule status opendbc_repo
git -C server rev-parse HEAD
```

Expect the three SHAs in §1; the submodule should be checked out at the pinned commit without a leading `+` (mismatch) or `-` (not initialized). A release tag checkout often shows **detached HEAD**: that is expected and *not* an error.

**Do not overwrite `/data/openpilot`, `/data/server`, or the on-device service manager with these staging copies.** Git checkout does not necessarily compile openpilot, provision native dependencies, update safety firmware, or install daemon definitions.

## 4. Inventory a currently working comma (read-only)

On a *parked* installed vehicle, the following identifies rather than changes the current release:

```bash
cd /data/openpilot

git status -sb
git rev-parse HEAD
git submodule status opendbc_repo

git -C /data/server status -sb
git -C /data/server rev-parse HEAD

readlink -f /data/openpilot/server 2>/dev/null || true
ps -eo pid,comm,args | grep -E 'server\.server_v21|mpp_writer|phone_nav' | grep -v grep || true

# Local-only browser check (GET; no actuation):
curl -fsS --max-time 5 http://127.0.0.1:7070/ >/dev/null \
  && echo 'Dashboard HTTP responds' || echo 'Dashboard unavailable'
```

For v1.0.0, expected Git commits are the manifest in §1. Untracked preview screenshots are not part of the release. These checks do not guarantee the control stack is operating safely.

### Important directory layout

| Path | Meaning | Git status |
| --- | --- | --- |
| `/data/openpilot` | main software/controller | main repo |
| `/data/openpilot/opendbc_repo` | Tesla Pre-AP integration | Git submodule (pin required) |
| `/data/server` | MCU2 web backend and phone navigation | separate Git repo |
| `/data/openpilot/server` | optional link to `/data/server` | **verify** if your deployment uses it |
| `/data/nap_settings/` | V9e and diagnostics JSON settings | runtime state; separate backup |
| `/data/bms/nap_dash/` | SQLite battery/charging/performance/efficiency history | runtime state; **not Git** |
| `/data/params` and other parameter stores | per-vehicle settings/calibration | private runtime state; **not Git** |
| `/data/continue.sh` and launch/service definitions | installation-specific startup | must be separately inventoried |
| `/dev/shm/mpp_speed_limit` | volatile MPP writer data | recreated at runtime, not a backup source |

The server imports modules such as `phone_navigation.route_tracking` from the **`/data/server`** Python package. Running its tests without that path can falsely report `ModuleNotFoundError`.

## 5. Preserve data before service migration or replacement

**Use a private storage target**, not a public GitHub repository. The critical items are:

1. Versioned source / Git history and submodule pointers (already published, but verify exact hashes).
2. Service definitions and startup wrappers (`launch_openpilot.sh`, existing MPP writer launch integration, `/data/continue.sh`, network helpers, system service units).
3. Runtime feature settings: `Params`, `/data/nap_settings`, NAP bridge JSON, radar offsets, pedal calibration, tuning profiles, and any navigation settings.
4. Phone pairing credentials, network credentials, and tunnels, **encrypted/offline only**. Do not paste these into issues, README files, or commit them.
5. `/data/bms/nap_dash/nap_history.sqlite3` and runtime JSONL files; local map caches and desired destinations if you want them restored.
6. EPAS original firmware backup and hardware calibration information, stored securely. Do not distribute vehicle- or module-specific firmware.

### Safe SQLite backup while writers may be active

A plain copy of a live `.sqlite3` file can omit committed transactions held in its `-wal` file. Use the SQLite backup API; select a **private storage location** with available space:

```bash
# Example only: change the destination to your private mounted backup volume.
export NAP_BACKUP_DIR=/path/to/private/backup
mkdir -p "$NAP_BACKUP_DIR"
chmod 700 "$NAP_BACKUP_DIR"

python3 - <<'PY'
import os, sqlite3
from pathlib import Path
source = Path('/data/bms/nap_dash/nap_history.sqlite3')
backup = Path(os.environ['NAP_BACKUP_DIR']) / 'nap_history.sqlite3'
if not source.is_file():
    raise SystemExit('No BMS history database present')
if backup.exists():
    raise SystemExit(f'Refusing to overwrite backup: {backup}')
with sqlite3.connect(f'file:{source}?mode=ro', uri=True) as src:
    with sqlite3.connect(str(backup)) as dst:
        src.backup(dst)
        integrity = dst.execute('PRAGMA integrity_check').fetchone()[0]
        if integrity != 'ok':
            raise SystemExit(f'Backup failed integrity check: {integrity}')
backup.chmod(0o600)
print('Verified private SQLite backup:', backup)
PY
```

Copy JSONL, calibrations, and needed configuration into similarly private backup storage **after auditing for credentials and personal location history**. Verify backup integrity and restore permissions. The SQLite snapshot is data, not code; it should not be Git-tracked. A private archive of `/data/params` may contain sensitive identifiers and device keys.

## 6. Carefully stage a new device deployment

A fully automated production installer for this custom three-repository release is **not yet verified**. Use this order with a competent integrator on a safely parked/inactive vehicle:

1. **Inventory and back up the existing device** (if any) using §§4–5. Preserve hardware firmware and calibration records separately.
2. **Fetch and verify all pinned repositories** as in §3 on a non-live path. Confirm the openDBC submodule URL points to `philipr144-create/opendbc-preap` and its exact SHA.
3. **Review hardware, Panda safety, EPAS, pedal, radar, and build prerequisites.** Do not assume the source clone provides a ready-to-run image or firmware. Complete documented platform build/install instructions for the actual comma OS, device and harness.
4. **Review service startup**, particularly the device's existing `launch_openpilot.sh` wrapper and MPP writer. The older README documents that starting `launch_chffrplus.sh` alone bypasses the writer and can make automatic MPP speed-limit updates stale.
5. **Install/configure NAP-Dash independently** at the appropriate service path (historically `/data/server`). Preserve required module import paths, any `/data/openpilot/server` link, and existing service supervision. The service historically runs `server.server_v21` on port **7070**. Do not simply start a second copy over a working server.
6. **Restore only verified runtime settings** and check that expected schema/parameter keys are available. Restore a compatible subset rather than copying an older, mismatched entire Params tree blindly.
7. **Validate offroad**: steering/pedal/radar health, CAN safety mode, dashboard read-only pages, network access, navigation pairing, diagnostics, MPP file freshness, and ability to disengage. Actuating tests require a separate approved procedure.
8. **Run targeted software tests** (§7); inspect every failure. Perform any necessary static/replay, parked, and controlled-road validation under qualified supervision before considering the install operational.

No universal `curl | bash` install command is offered because it would imply compatibility and startup automation that have not been verified. For current users, the safest path is to **preserve a known-good physical installation** and compare with the tagged source rather than reinstalling for the sake of release documentation.

## 7. Validation commands (review tests before running)

For the October 2026 source state, the following focused software tests were previously run on the installed comma. They are *not* hardware safety certification. Review test side effects before execution.

```bash
cd /data/openpilot

# Explicitly include NAP-Dash's phone_navigation modules; disable pytest-xdist:
PYTHONPATH="/data/server${PYTHONPATH:+:$PYTHONPATH}" \
python3 -m pytest -n 0 -q \
  selfdrive/controls/lib/tests/test_predictive_grade_assist.py \
  selfdrive/controls/lib/tests/test_map_stop_v4.py

# Separate dashboard suite, which references installed /data/openpilot:
cd /data/server
python3 -m pytest -n 0 -q tests/
```

Reported release-preparation outcomes: **41 passed plus 19 subtests** for the two openpilot tests above; **38 passed** for the dashboard suite. An earlier parallel run had a `pytest-xdist` serialization problem involving a `SimpleNamespace`; a separate collection error came from missing `/data/server` on Python's import path. The corrected command sequence passed. Repeat results may vary by environment.

Further low-risk read-only checks:

```bash
# Check source/working tree, without modifying the device:
git -C /data/openpilot diff --check
git -C /data/server diff --check

# Check the predictive setting; reading does not enable it:
cat /data/nap_settings/v9e_predictive_grade.json 2>/dev/null || true

# Inspect the runtime MPP bridge freshness if installed:
ls -l --full-time /dev/shm/mpp_speed_limit 2>/dev/null || true
```

## 8. Feature configuration and known safety boundaries

**Predictive versus legacy pitch:** `NAPPitchAssistMode` is the legacy **OFF / MONITOR / ON** setting exposed in the MCU2 dashboard. The separately labeled **Predictive Hill Assist** is controlled by `/data/nap_settings/v9e_predictive_grade.json`. Its UI adapter refuses unsafe enabling while onroad or when driving processes are present and does not create a missing config file; disabling is permitted. The configuration observed during release preparation was `{"enabled": true}`, but **restoring a new device must not silently enable it**. Explicitly inspect and validate the setting for the new hardware/vehicle.

**Driving behavior:** even with radar, detected model stops, map entries and grade forecasts, the driver remains responsible for braking and steering. Experimental and city-turn logic should be treated as alpha-level assistance, not automatic intersection navigation.

**Air Ride:** the web UI has guarded lowering/restore routines, a speed gate below 40 mph, and fault information. It does not provide tested suspension inflation or precision height setting. Don't use it during software bring-up.

**EPAS and safety:** firmware backup/flash/restore controls exist, but routine software installation must not flash steering firmware. Hardware/panda safety validation is separate from Python test results.

**Phone navigation:** pairing, route context and synthetic intent are implementation components, but displayed route intent does not prove an actual blinker actuation. Verify on a parked CAN test fixture before relying on the path.

## 9. Updating documentation in the live repo (no force push needed)

On a machine with a local checkout of `openpilot-preap/main`, copy the supplied documentation files to:

```text
README.md
 docs/INSTALLATION_AND_RECOVERY.md
 docs/FEATURE_MATRIX.md
```

Keep existing unrelated files intact. **Before overwriting the original README, compare its legal/indemnification, safety, and attribution notices with this draft and preserve any required wording.** Review the changes and keep NAP's original license/attribution:

```bash
git status -sb
git diff -- README.md docs/INSTALLATION_AND_RECOVERY.md docs/FEATURE_MATRIX.md
git diff --check
# When reviewed:
git add README.md docs/INSTALLATION_AND_RECOVERY.md docs/FEATURE_MATRIX.md
git commit -m "Document NAP v1.0.0 features, installation and recovery"
git push origin main
```

This is a **documentation-only commit after v1.0.0**. It must not move or rewrite the original `NAP-v1.0.0` release tag; changing `main` docs does not change the pinned release. If repository branch protection is enabled, submit a pull request instead of pushing directly.

## 10. Recovery and rollback

- **Source recovery:** use the exact release tags and opendbc submodule pin in §1. Fetch to an isolated directory; do not reset a running checkout or overwrite untracked files in place.
- **Runtime recovery:** restore compatible, audited device configuration and SQLite data *separately* from Git history, keeping a copy of both pre- and post-recovery states.
- **Old server history:** `archive/legacy-main-896fc85`; **old openpilot main:** `archive/main-before-NAP-74fbabb8`. These are historical rollback references, not a promise they will operate with the current vehicle settings.
- **Failure to start / missing nav modules:** verify `/data/server/phone_navigation`, its import path, server supervision, and the correct linked directory, rather than copying isolated files into openpilot.
- **Unexpected longitudinal/signal behavior:** disengage, disable the optional feature while safely stopped if its UI permits, record versions/logs privately, and do not treat unit tests as grounds to resume driving.
- **Air Ride restoration failure or steering firmware issue:** stop testing; use the vehicle's qualified diagnostic/service process. Do not repeatedly attempt risky CAN commands from generic scripts.

## 11. Remaining work toward a genuine one-command installer

The next release should explicitly package or describe supported platform build dependencies, firmware prerequisites, server service definitions and restart policy, MPP writer supervision, optional networking helpers and ffmpeg, navigation route/cache provisioning, data-backup migrations, device-specific settings defaults, privacy handling, and clean-room install verification. Until those items are tested, the **release is a pinned source snapshot, not a turnkey reflash image**.

For current features and candid status labels, read [FEATURE_MATRIX.md](FEATURE_MATRIX.md).
