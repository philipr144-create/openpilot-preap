<div align="center">

# NAP — NotAutopilot Pre-AP Edition

### Custom openpilot driver assistance for early Tesla Model S

**Production source: v1.0.0 · October 2026**

[Features](#feature-highlights) · [Feature matrix](docs/FEATURE_MATRIX.md) · [Install / recovery guide](docs/INSTALLATION_AND_RECOVERY.md) · [NAP-Dash](https://github.com/philipr144-create/NAP-Dash)

</div>

> **Independent custom fork, not an official comma.ai or NotAutopilot release.** NAP provides *supervised driver assistance*, not autonomous driving. A driver must continuously monitor the roadway and remain ready to steer and brake. Features can be hardware-, mode-, and configuration-dependent; source availability and passing tests do not establish road validation.

## Overview

This repository is a customized openpilot-based system for a **2013 Tesla Model S Pre-Autopilot** project, built upon the earlier NotAutopilot, Tinkla, xnor-tech, and openpilot work. It combines tailored steering and longitudinal control with on-device customization, optional radar/pedal integrations, map and phone-navigation assistance, and a separate web dashboard for the Tesla MCU2 browser.

Unlike the older experimental branch naming, **`main` is the official source branch** for this custom production snapshot. The matching release is **`NAP-v1.0.0`**. Do not confuse it with another organization's `nap-release` installer URL, which does **not** install this pinned custom release.

## Release components

| Repository | Purpose | v1.0.0 reference |
| --- | --- | --- |
| [openpilot-preap](https://github.com/philipr144-create/openpilot-preap) | Control logic, comma UI, map support, integration | `7c51a9d2c6a384cc0713601639ba4dfc64585ad2` · `NAP-v1.0.0` |
| [opendbc-preap](https://github.com/philipr144-create/opendbc-preap) | Tesla Pre-AP interface, CAN, safety/vehicle handling | `31da4137f2ce06702bc595a05689d537f046bb70` · submodule pin |
| [NAP-Dash](https://github.com/philipr144-create/NAP-Dash) | MCU2/phone web UI, navigation backend, telemetry, Air Ride tools | `5aeb8de9d87f50a025da93b259ff25c9eddd518b` · `NAP-Dash-v1.0.0` |

The repositories are **versioned together**, but the dashboard is *not* embedded in the openpilot Git repository. A clone of openpilot alone does not reproduce the running dashboard, persistent configuration, service startup, or vehicle calibration. Follow the [installation and recovery guide](docs/INSTALLATION_AND_RECOVERY.md).

## Feature highlights

### Driving and longitudinal control

- **Tesla Pre-AP steering integration:** EPAS-based lateral assistance with Pre-AP-specific car interface and Panda safety integration; driver steering override remains important.
- **Comma Pedal longitudinal integration:** throttle/regen control when the correct hardware is installed, configured, and calibrated; pedal CAN bus selection and emergency disable in NAP settings.
- **Personality profiles:** **Aggressive**, **Standard**, **Relaxed** (the web dashboard may label relaxed driving **Chill**). Following distance has settings **1–7**.
- **Unified longitudinal refinements:** cruise-speed smoothing, acceleration handoff, coasting/regen refinements, speed hold and prediction-aware control paths, with lead/stop/braking arbitration.
- **Adaptive acceleration limits:** reduce acceleration authority near a lead and permit more appropriate open-road acceleration when enabled.
- **Normal / Experimental operation:** distinct speed-control behaviors; Experimental includes model-driven longitudinal requests. **Model detection of traffic lights or signs is not a guarantee of safe stopping.**
- **Map-based speed assistance:** fresh map context can lower the cruise target near mapped curves and road features; it does not replace the driver's speed decisions or authorize acceleration above the selected ceiling.
- **Corner Assist and curve-comfort profiles:** **More / Standard / Mild** slowing when enabled, plus curve-speed reference refinements.
- **Speed controls:** optional +5 mph offset, bounded speed trim, selected follow gap, and vehicle MPP speed-limit bridge when its companion startup writer is running.

### Hill assistance — two separate features

- **Legacy Pitch Sensor Assist:** **OFF / MONITOR / ON** via `NAPPitchAssistMode`; ON is gated by pitch-sign validation and operating conditions. MONITOR is observational.
- **Predictive Hill Assist (V9e/V10 UI):** separate **file-backed** switch at `/data/nap_settings/v9e_predictive_grade.json`; uses model-relative hill preview and bounded, lead-aware acceleration support. Current code describes operation over **20–80 mph**, tapering above **65 mph**, with a maximum programmed addition of **+0.35 m/s²**. Stops, lead braking, curve braking, invalid forecasts, and speed/acceleration limits retain priority. Enable only offroad; disabling is permitted onroad.
- **Grade diagnostics:** separate optional recording/status, with blocking reasons and reported applied assistance.

**These two controls are not interchangeable.** The MCU2 selector labeled *Legacy Pitch Sensor Assist* changes `NAPPitchAssistMode`; it is **not** the predictive-grade switch.

### Lateral, turn, and lane features

- **Tap Lane Change:** request a lane change with a brief turn-signal tap when its prerequisites are met; not a hands-off lane-change guarantee.
- **City Turns:** turn-signal-assisted low-speed corner logic, with navigation and intent arbitration.
- **Wider Low-Speed Turns:** experimental wider turn-entry behavior to reduce curb cutting on tight, signaled turns.
- **Low-Speed Steering Assist:** increased steering rate allowance in supported low-speed turn situations.
- **Optional lane centering correction (experimental):** model/lane-line correction on clearly marked roads; **Low / Medium / High** strength and adjustable lateral preference (**Left 12/9/6 in, Center, Right 6/9/12 in**). It yields to signaling/driver intervention; the visualized model path can remain unchanged by the correction.

### Navigation and route context

- **Phone navigation remote** hosted by the companion NAP-Dash server (including browser pairing and a six-digit setup-code flow).
- **Route tracking and maneuver context**, a navigation desire bridge, offline/local road/map lookup components, mapped stops and road features, and navigation-to-openpilot integration.
- **Navigation and map-driving toggles** in NAP settings, with staleness/validity checks and driver/vehicle-control priority.
- **Important limit:** displayed navigation intent does **not** prove a turn signal was transmitted or a maneuver executed. Synthetic signaling and connectivity resilience require vehicle-specific verification; this is **not** unsupervised Navigate on Autopilot.

### Comma UI and diagnostics

- **NAP settings panels** for driving features, pedal/radar setup and calibration, radar offset/nosecone setting, EPAS backup/flash/restore tools, advanced settings, and reset/emergency disable.
- **Onroad visualization options:** Classic, Blue, Rainbow, and Dynamic predicted-path styles; optional **11% wider visual path** and fresh supported lead markers. Color/animation reflects model prediction cues, *not* actual accelerator/brake commands.
- **Engagement statistics:** current drive vs tracked totals, steering-only/speed-only/both/manual categories, optional onroad strip; display-only.
- **Driver monitoring and general openpilot camera/alerts** inherited from the underlying fork where supported.

### MCU2 browser dashboard (separate NAP-Dash repository)

Access on the same network with `http://<comma-ip>:7070/` once the server is correctly deployed and running. It includes:

- **Driving dashboard:** speed, selected speed, steering, lead/vehicle observations, and live telemetry.
- **Battery / BMS:** pack voltage, current, power, SOC, nominal capacity, rated range, power limits, temperatures, cell spread, **96 brick-voltage display**, and history/trend views when vehicle signals are available.
- **Energy and charging:** current power flow, trip efficiency, regen and charging-session logging.
- **Performance logger:** recorded acceleration/rolling/braking milestones and pack context.
- **History explorer:** local SQLite-backed records with filters and export controls.
- **Camera / route viewer:** video playback, route segments, timeline, optional telemetry HUD and clip export where the recording/decode dependencies are available.
- **Dashboard settings:** driving personality, follow distance, Experimental and Adaptive Accel, speed offset/trim, and legacy pitch selector.
- **Phone navigation panel** embedded in the dashboard, plus **Air Ride** controls/diagnostics.

### Air suspension tools (specialized, not general-purpose control)

- Continental EAS **read-only DTC inspection** and reporting, with guarded vehicle/CAN integration.
- **Manual lowering** controls for front/rear/all four/individual corners, hold-to-pulse, stop-on-release, and attempted return to factory control.
- Fresh-speed gate for manual starts **strictly below 40 mph**; loss of speed freshness or the hold signal blocks further venting and requests restoration.
- **Limitations:** lowering only; inflation and precise target-height control are **not established**. Parked diagnostic acknowledgments do not guarantee successful physical restoration. Moving-vehicle behavior and reservoir isolation remain unverified. Do not use these tools as a substitute for service procedures.

See the [full feature matrix](docs/FEATURE_MATRIX.md) for availability, required components, and known limitations.

## Hardware and scope

The v1.0.0 snapshot is tied to an **early Model S Pre-AP custom build**. Components may include a supported comma device, suitable Tesla harness and EPAS interface, Comma Pedal for longitudinal control, and a compatible Bosch radar for radar-based lead following. Exact CAN wiring, EPAS firmware, pedal calibration, and safety compatibility **must be assessed for each car**. This repository is not a plug-and-play installer for every pre-AP Tesla.

**Never flash EPAS firmware as a routine installation step.** Firmware changes risk loss of power steering; consult the installed hardware's validated procedure and create a verified backup first. Likewise, do not exercise EAS manual commands or driver-assistance controls on public roads as an installation test.

## Quick start for inspection (does not install or launch)

From a *separate development/recovery environment*, not over an active `/data/openpilot` tree:

```bash
git clone --branch NAP-v1.0.0 --recurse-submodules \
  https://github.com/philipr144-create/openpilot-preap.git nap-openpilot

git clone --branch NAP-Dash-v1.0.0 \
  https://github.com/philipr144-create/NAP-Dash.git nap-dashboard

# Verify exact source versions:
git -C nap-openpilot rev-parse HEAD
git -C nap-openpilot rev-parse HEAD:opendbc_repo
git -C nap-dashboard rev-parse HEAD
```

These steps **only fetch code**. Before attempting hardware deployment or recovery, read [Installation and Recovery](docs/INSTALLATION_AND_RECOVERY.md). Do not use the older upstream NotAutopilot installer address to install this custom v1.0.0 snapshot.

## Test status and limitations

During v1.0.0 preparation, **38 NAP-Dash tests passed**, and a focused grade/navigation suite passed **41 tests plus 19 subtests** with `-n 0` and `/data/server` on `PYTHONPATH`. These are targeted software checks, **not** a complete system validation, a hardware certification, or proof that all driving scenarios are safe. Some features are experimental or only partially road-validated. See the feature matrix for specific caveats.

## Repository branches and release preservation

- **`main`:** official custom production source for this project.
- **`NAP-v1.0.0`:** immutable intended release pointer to the pinned openpilot commit.
- **`release/NAP-v1.0.0`:** archived candidate branch.
- **`archive/main-before-NAP-74fbabb8`:** original `main` retained for recovery.
- Historical `main-dev`, `repair/*`, and other branches are **not** this release.

The NAP-Dash repository similarly preserves its older source at `archive/legacy-main-896fc85`. The custom opendbc commit is pinned through the openpilot submodule, even though its own default branch may need setting to `main` in GitHub settings.

## Credits and lineage

NAP's ancestry includes **comma.ai/openpilot**, **xnor-tech**, the **NotAutopilot** ecosystem, and the **Tinkla project / Boggyver**, with contributions and inspiration from **Lukas Loetkolben, Johnmr1, SeriouslySerious, Pod042, and 1FrostlySlime**. These projects made Pre-AP Tesla integration possible. This independently maintained custom branch does not claim affiliation, approval, or support from those projects or Tesla.

## License, privacy, and responsibility

See the repository [LICENSE](LICENSE) and dependency licenses. **When replacing the older README, retain any required existing upstream indemnification, warranty, or safety notices; this feature document does not supersede those notices.** Preserve upstream notices and attribution. Camera, driving, location, and battery records may contain sensitive personal information; **do not publish `/data/params`, navigation history, BMS SQLite, setup codes, tokens, logs, or recorded routes**. Driver-facing recording/upload settings and telemetry behavior should be verified on the installed build.

**Research-grade driver-assistance software. No warranty; the driver is responsible for safe operation and compliance with local law.**
