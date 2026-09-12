# NAP Pre-AP Feature Guide

This document describes the custom NotAutopilot (NAP) build developed and tested on the
`milestone/2026-09-11` line, including the navigation/signal-arbitration repairs through
openpilot commit `d5c7845b0` and opendbc commit `75884597`.

> **Development status:** This is experimental driver-assistance software, not autonomous
> driving. The milestone has primarily been road-tested on one 2013 Tesla Model S P85
> Pre-AP car with an MCU2 upgrade, comma 3X, red panda, Comma Pedal, and a Bosch radar.
> Behavior on another Pre-AP Model S must be validated before driving.

## What this build is

NAP adapts modern openpilot to early Model S cars that have no factory Autopilot computer,
camera, harness relay, or AP steering controller. It combines:

- a dedicated Tesla Pre-AP car port and panda safety mode;
- openpilot camera-based lateral control and driver monitoring;
- optional Comma Pedal longitudinal control;
- optional Bosch radar integration;
- Pre-AP-specific engagement, steering, and cruise behavior;
- a custom on-device NAP settings panel;
- a local NAP Dash web application;
- experimental phone-fed navigation desires and synthetic turn signaling.

This build currently requires a **comma 3X or newer**. Its launcher deliberately blocks the
original comma 3 (`tici`).

## Additions at a glance

| Area | Added in this NAP build | Status |
|---|---|---|
| Pre-AP foundation | Dedicated vehicle interface, DBC, direct-CAN panda safety mode, EPAS control, engagement FSM, driver overrides, and Pre-AP startup selection | Core, but vehicle compatibility remains limited |
| Lateral | Low-speed entrance widening, smoother turn acquisition/unwind, low-speed steering-rate profile, HSO/takeover latches, city-turn desires, and curve-related limits | Experimental tuning |
| Lane changes | Half-stalk Tap Lane Change with held synthetic indicator, completion cleanup, replay protection, and physical-stalk priority | Experimental |
| Navigation | Phone/web route bridge, model desire selection, automatic bounded synthetic signals, navigation/tap arbitration, and live diagnostics | Prototype |
| Longitudinal | Comma Pedal control, VirtualDAS, zero-torque learning, grade/road-load feedforward, personality profiles, independent follow distance, lead accel cap, and Corner Assist | Experimental; pedal hardware required for full control |
| Radar | Bosch radar parsing, gateway emulation, offset/location configuration, tests, calibration, and replay tools | Optional hardware |
| On-device UI | NAP settings sections, follow-distance buttons, pedal/radar tools, EPAS tools, emergency disable, custom HUD status and controls | Mixed core/experimental |
| Web UI | Live cluster, controls, phone navigation, dashcam viewer, HUD export, BMS, charging, efficiency, performance, history, and diagnostics | Experimental companion application |
| Connectivity | Local port 7070 service and current Tailscale daemon integration; earlier Cloudflare quick-tunnel experiments | Experimental; keep private |
| Not implemented | Navigation-aware longitudinal target, dependable lateral auto-resume, voice control, YOLO/radar fusion, iBooster actuation, and cruise-cluster spoofing | Planned or unsupported |

## Tested hardware

| Component | Current development vehicle | Portability concern |
|---|---|---|
| Vehicle | 2013 Tesla Model S P85, Pre-AP | Other years, trims, EPAS firmware, and CAN variants are not yet broadly validated. |
| Infotainment | MCU2 retrofit | MCU1 browser performance and compatibility may differ. |
| Compute | comma 3X | Original comma 3 is blocked by this build. |
| CAN | Red panda, direct Pre-AP connection | Pre-AP has no AP relay; bus topology and wiring must match the port. |
| Longitudinal | Comma Pedal | Calibration and bus selection are car-specific. Never copy another car's calibration. |
| Radar | Bosch radar | Optional; mounting position, gateway emulation, bus traffic, and lateral offset must be verified. |

## Feature status legend

- **Core:** part of the Pre-AP port and expected for supported installations.
- **Optional:** requires matching hardware or configuration.
- **Experimental:** implemented and tested on the development vehicle, but still being tuned.
- **Prototype:** useful for development, not ready to advertise as generally supported.
- **Planned:** discussed or designed, but not implemented in the current build.

## Driving features and toggles

| User-facing setting | Params key | Default | Status | What it does |
|---|---|---:|---|---|
| City Turns | `NAPCityTurns` | On | Experimental | Below lane-change speed, one physical blinker can request `turnLeft` or `turnRight` instead of a highway lane change. Steering takeover cancels the active manual turn until the signal is released. |
| Wider Low-Speed Turns | `NAPWideLowSpeedTurns` | On | Experimental | On a sharp, signaled turn below 25 mph, reduces curvature during the entrance to avoid cutting the curb, then restores full model curvature near the apex so the car can finish the turn. |
| Low-Speed Steering Assist | `NAPLowSpeedSteeringRate` | On | Experimental | Uses a higher, speed-dependent steering-angle rate during signaled turns below 25 mph while retaining the vehicle model and panda limits. |
| Tap Lane Change | `NapTapLaneChange` | Off | Experimental | A brief half-stalk tap above 40 mph requests one lane change, holds the Tesla indicator through the maneuver, and cancels it when complete. A held/full stalk retains normal driver behavior. |
| Navigation Maneuvers | `NAPNavigationManeuvers` | On | Prototype | Allows fresh phone/web navigation instructions to select supported model desires and request synthetic turn signaling. Navigation is fully gated off when this toggle is off. |
| Corner Assist | `NAPCornerAssist` | On | Experimental | Applies a mild, curvature-based acceleration cap when approaching a supported corner. It does not yet use route distance or guarantee a safe turn speed. |
| Adaptive Accel Limits | `NAPAdaptiveAccel` | On | Experimental | When a radar lead is present, caps positive acceleration independently of personality to reduce surge/regen oscillation while closing a gap. |
| Pedal Interceptor | `NAPPedalEnabled` | Off | Optional | Enables direct longitudinal control through a calibrated Comma Pedal. Requires a reboot. |
| Radar Enabled | `NAPRadarEnabled` | Off | Optional | Enables Bosch radar parsing/emulation for lead tracking. Requires a reboot. |
| Radar Behind Nosecone | `NAPRadarBehindNosecone` | Off | Optional | Enables the configured radar-position/installation adjustment. Requires a reboot. |
| Parked Signal Testing | `NAPParkedSignalTest` | Off | Developer only | Permits bounded synthetic stalk/indicator testing only while safely parked. Never enable for normal driving. |

`NAPForcePreAP` is currently forced on by the NAP settings panel. The iBooster and Brake
Factor controls are displayed but disabled because electronic brake actuation is **not
implemented**.

## Navigation, Tap Lane Change, and signal ownership

Navigation and Tap Lane Change can be enabled at the same time. They share the Tesla
`STW_ACTN_RQ` synthetic-indicator path, so arbitration is explicit:

1. **Physical driver stalk** — always wins immediately.
2. **Active navigation maneuver** — owns the model desire and synthetic announcement in
   its bounded window.
3. **Active tap-generated lane change** — owns the synthetic indicator when navigation is
   not claiming the maneuver.
4. **None** — no synthetic signal is sent.

Feature availability is not treated as signal ownership. Enabling Tap Lane Change does not
reserve the stalk, and disabling Navigation Maneuvers prevents navigation from reserving a
tap window.

### Supported navigation behavior

The route instruction itself authorizes navigation; a physical blinker confirmation is not
required. Synthetic signaling only announces a maneuver that navigation has already
requested. It cannot create a second lane change or feed back through the tap detector.

| Maneuver | Desire window | Synthetic signal window | Other gates |
|---|---|---|---|
| `fork` / `off ramp` | 10–300 m | Speed-based, clamped to approximately 60–120 m | Direction must be left/right or slight left/right. |
| `turn` / `end of road` | 80 m ahead to 8 m past | Approximately 12–30 m, based on speed | Vehicle must be below 25 mph. |

Unsupported U-turns, sharp-turn modifiers, missing maneuver IDs, stale navigation, invalid
vehicle state, or unsupported directions produce no model desire and no synthetic signal.

A real stalk input temporarily takes priority without permanently poisoning the route.
Steering, braking, or a lateral-control interruption pauses navigation output and synthetic
signaling; it may resume when the gate clears. The old permanent “driver cancelled; no
automatic retry” behavior has been removed.

### What navigation does not do yet

This is not full Navigate on openpilot:

- It does not choose a route; a phone/web client must supply a fresh route and maneuver.
- It does not change lanes merely because the synthetic blinker is active.
- It does not yet send route-distance speed targets to the longitudinal planner.
- It does not guarantee that the driving model will execute every requested maneuver.
- The driver must supervise every turn, exit, and lane change.

### Experimental-mode expectations

In Experimental mode, supported navigation desires are supplied to the driving model and
the indicator is announced near the maneuver. Longitudinal behavior remains the existing
experimental model/MPC blend plus Corner Assist; it does not yet slow for a turn solely
because navigation reports one.

A future nav-aware longitudinal layer should publish a bounded maneuver-speed target based
on maneuver type, remaining distance, current speed, model curvature, and comfortable
deceleration. The longitudinal planner—not the raw phone instruction—should blend that
target with cruise, lead, stop, and model constraints.

### Navigation diagnostics

These volatile JSON files make arbitration visible:

| Path | Contents |
|---|---|
| `/dev/shm/nap_navigation_desire.json` | Fresh route/maneuver snapshot consumed by modeld. |
| `/dev/shm/nap_navigation_decision.json` | Route, maneuver, distance, request/desire, selected model desire, decision reason, signal owner, tap state, and synthetic request. |
| `/dev/shm/nap_navigation_signal_request.json` | Modeld's bounded synthetic-indicator request. |
| `/dev/shm/nap_navigation_signal_status.json` | Tesla controller status, direction, cleanup state, request freshness, and physical override. |
| `/dev/shm/nap_tap_lane_change_request.json` | Tap controller request, active phase, filtered physical stalk, and signal ownership. |
| `/dev/shm/nap_tap_lane_change_ack.json` | Model-side acknowledgement and lane-change state. |

All `/dev/shm` files disappear on reboot and are recreated by their owning processes.

## Lateral control and steering changes

### Tesla Pre-AP EPAS control

- Direct steering-angle control through the Pre-AP EPAS messages.
- Vehicle-model-aware Python and panda angle/rate limits.
- Dedicated Pre-AP steering safety checks.
- HSO (human steering override) behavior tuned for early Model S steering feel.
- Low-speed takeover latching prevents openpilot from immediately fighting the driver after
  a takeover during a tight turn.
- A short reacquisition condition requires the requested and actual steering to return near
  center before special low-speed behavior resumes.

### Low-speed turn shaping

The development vehicle tended to cut the inside curb during 90-degree turns. The current
strategy changes only signaled, sufficiently curved, low-speed turns:

- full widening effect at roughly 5 mph;
- progressive fade back to stock curvature by 25 mph;
- a smooth 0.70-second entrance ramp;
- widening confined to approximately the first 0.35–0.75 seconds;
- full model curvature restored for the apex and exit;
- controlled catch-up and unwind rates to avoid steering snaps.

This is vehicle-behavior tuning, not a universal geometric correction. Tire size, alignment,
steering ratio, suspension, EPAS firmware, camera calibration, and model behavior can all
change the result on another car.

### Curve handling

The build includes custom total-acceleration/curve caps and smoother cap release so the car
does not immediately accelerate as steering unwinds. These are separate from navigation and
must be validated on real road logs before being generalized.

## Longitudinal control

### Two operating modes

**Comma Pedal mode**

- openpilot controls acceleration and regenerative deceleration through `GAS_COMMAND`;
- the first 0.5 seconds after engagement are ramp-limited;
- gas-pedal input passes through to the driver and is tracked for smooth resume;
- brake input drops longitudinal output while preserving the build's configured steering
  engagement behavior;
- pedal timeouts or invalid calibration disable pedal output.

**No-pedal mode**

- openpilot supplies lateral assistance;
- stock Tesla cruise controls speed;
- NAP uses bounded stalk spoofing to engage or cancel stock cruise;
- openpilot longitudinal planning is not used for vehicle acceleration.

### VirtualDAS and pedal behavior

- Jerk-limited feedforward mapping from requested acceleration to pedal command.
- Learned zero-torque point to reduce engagement regen spikes.
- Speed/acceleration lookup table with optional `/data/vdas_ff_table.json` override.
- Road-load feedforward to reduce repeated drive/regen crossing at cruising speed.
- Grade estimation and transient pitch compensation.
- Slow integral correction for hills, drag, and steady-speed error.
- Maximum regenerative request currently bounded around `-2.5 m/s²` in the Pre-AP layer.
- Driver gas override sends disabled pedal commands so the physical pedal remains authoritative.

### Personalities and follow distance

- Aggressive, Standard, and Relaxed personalities have separate acceleration/jerk behavior.
- Relaxed acceleration tapers at higher speed.
- NAP follow distance is independently selectable from 1–7; personality no longer silently
  changes the chosen gap.
- The current mapping is approximately 0.80 seconds at level 1, increasing by 0.15 seconds
  per level to approximately 1.70 seconds at level 7.
- The Tesla distance stalk can update the stored NAP follow-distance selection when present.

### Planner changes in this milestone

- Pre-AP lead-follow acceleration cap behind `NAPAdaptiveAccel`.
- Reduced acceleration/regen oscillation safeguards.
- Phantom-braking step buffer for abrupt target changes.
- Smooth release of turn-related acceleration caps.
- Custom stop-distance and personality tuning.
- Corner Assist is the sole current NAP turn-related longitudinal gate; the older unconditional
  blinker-only maximum-regen override is disabled.

These values were tuned around a P85 with a specific battery, pedal calibration, tires, and
road environment. They are not validated acceleration limits for every Pre-AP trim.

## Engagement and driver override

- Shared Pre-AP stalk finite-state machine.
- Configurable single/double-pull engagement behavior.
- Pedal and no-pedal modes publish different openpilot longitudinal capabilities.
- Real stalk cancel is separated from transmitted stalk echoes.
- Steering disengage and EPAS rejection tear down engagement state.
- Driver gas always overrides commanded pedal output.
- Physical stalk input always outranks navigation and tap-generated synthetic signaling.
- Synthetic `0x45` messages are remembered and filtered so they cannot replay as driver input.

See [engagement.md](engagement.md) for the state flow and [safety-model.md](safety-model.md)
for panda-enforced behavior.

## Radar support

- Optional Bosch radar parser with lead points and radar state.
- Optional Pre-AP gateway/radar emulation.
- Configurable behind-nosecone mode.
- Configurable lateral `yRel` offset.
- On-device connectivity test, diagnostics, replay helpers, and calibration scripts.
- Lead data feeds openpilot's radar/MPC path and NAP Dash telemetry.

Radar mounting, firmware, message rates, bus routing, and offset can differ between cars.
Do not enable radar emulation until CAN traffic and mounting are verified.

## Panda safety layer

NAP uses a standalone `SAFETY_TESLA_PREAP` mode rather than the later Tesla safety mode.
Important properties include:

- no assumed Autopilot harness relay;
- explicit TX whitelist for steering, EPAS control, longitudinal/pedal, and stalk messages;
- steering angle/rate and vehicle-model checks;
- controls-allowed gating;
- hands-on, EPAS error, door, gear, and real-stalk cancellation handling;
- AEB command blocking;
- conditional pedal RX/TX checks based on the configured hardware;
- stalk-spoof echo filtering;
- optional radar forwarding/emulation flags.

EPAS checksum/counter validation is currently relaxed for the known Pre-AP firmware problem
described in [safety-model.md](safety-model.md). That is a known port limitation, not evidence
that safety checks are unnecessary.

## On-device NAP settings and tools

The NAP panel adds:

- driving-feature toggles listed above;
- follow distance 1–7;
- Pedal CAN bus 0/2 selection;
- pedal calibration status and guided calibration;
- radar enable/location/offset controls;
- radar calibration and connectivity tests;
- stock EPAS firmware extraction/backup;
- EPAS firmware flashing and restoration tools;
- emergency pedal disable and calibration clearing;
- reset-to-default actions;
- NAP acknowledgments and contributor credits.

The on-road UI also contains custom lower-left follow-distance/personality controls and NAP
status additions. Exact placement differs between the standard and mici UI implementations.

Hardware-changing actions are restricted to off-road operation where practical. EPAS
flashing and pedal/radar configuration are expert-only operations and should not be treated
as ordinary preferences.

## NAP Dash web application

The current integrated dashboard is a standalone Python server (`server_v21.py`, currently
identifying itself as NAP Telemetry v24) launched by manager as `server_v21`. It normally
listens on port `7070` and renders most graphics in the browser to limit comma CPU usage.

### Dashboard capabilities

- live speed, steering, engagement, planner, pedal, lead, and radar telemetry;
- browser-rendered road/lead visualization;
- driving personality, follow-distance, Experimental Mode, adaptive acceleration, and speed
  offset/trim controls;
- navigation status and a phone-navigation remote page;
- navigation update/status/clear HTTP endpoints;
- route list and dashcam playback for road, wide, and driver cameras;
- telemetry-aligned HUD playback;
- MP4 export with a burned-in HUD when the installed ffmpeg supports the required filters;
- battery/BMS, regen, charging, efficiency, and performance views;
- bounded SQLite history with export support;
- settings-change audit history;
- Tailscale process integration for persistent remote access in the current device build.

Earlier dashboard versions focused on a lightweight live cluster, remote settings, and an
iOS-friendly dashcam viewer. The current server retains those goals but is substantially more
coupled to this milestone because it also owns history, BMS/performance processing, and the
phone-navigation API.

### Dashboard files and storage

| Path | Purpose |
|---|---|
| `/data/NAP-Dash-Release/` | Typical device-side dashboard checkout. |
| `/data/nap_settings.json` | Legacy/live web settings such as speed offset, speed trim, and auto-resume. |
| `/data/bms/nap_dash/nap_history.sqlite3` | Current bounded history database. |
| `/data/media/0/realdata/` | openpilot route segments used by the dashcam viewer. |
| `/dev/shm/` | Temporary remux/export and live bridge files. |

### Dashboard limitations

- The web server is tightly coupled to the services, schemas, Params keys, and file paths in
  this fork. “Universal” fallbacks improve resilience but do not guarantee compatibility.
- BMS decoding and derived health/capacity values are still being validated against the 2013
  P85 Rev-D pack. Do not use the dashboard alone for repair or pack-safety decisions.
- Performance capture and historical statistics are informational, not calibrated test gear.
- Video HUD export depends on the comma's ffmpeg build. Builds missing the `ass`/subtitle
  filter cannot complete the current export path.
- Remote exposure requires authentication and network hardening. Prefer a private Tailscale
  path; do not expose an unauthenticated control dashboard through a public quick tunnel.
- The dashboard must never be operated by the driver while the car is moving.

## Speed offset, trim, and remote controls

The dashboard can store a fixed +5 mph speed offset and a numeric speed trim in
`/data/nap_settings.json`. The Pre-AP CarState bridge applies these to the reported cruise
target. Personality, follow distance, adaptive acceleration, and Experimental Mode use the
openpilot Params system.

Because these settings affect vehicle behavior, an older dashboard must not be copied into a
newer build unless its parameter names, accepted ranges, and CarState consumer are verified.

The existing `auto_resume` setting is experimental and has not demonstrated reliable lateral
resume on the development vehicle. It should not be presented as supported.

The build also contains legacy/custom speed-limit plumbing developed for MCU/phone inputs.
That path is not yet documented or validated well enough to call portable; older users should
treat it as development code rather than a supported traffic-speed-limit source.

## Compatibility with older NAP builds

The dashboard by itself does not add vehicle-control features. Compatibility has three levels:

| Level | Likely result |
|---|---|
| Telemetry-only dashboard on a nearby NAP version | Basic speed/lead/state pages may work through schema introspection. Missing services should degrade to unavailable fields. |
| Dashboard controls on an older build | Risky. A key may be absent, renamed, have a different type, or have no consumer. Verify every setting before enabling writes. |
| Navigation/tap/synthetic signaling on an older build | **Not drop-in.** Requires the paired modeld, DesireHelper, Tesla CarState/CarController, synthetic-stalk, echo-filter, Params, and panda whitelist changes. |

For early web-UI users, start in telemetry-only mode. Disable or hide control-writing endpoints
until the target fork's exact keys and consumers are documented.

## Bringing this build to another Pre-AP Model S

Do not assume that “Pre-AP” means electrically or dynamically identical. Before road testing:

1. Record the model year, trim, drive unit, wheel/tire size, suspension, MCU version, EPAS
   firmware, comma hardware, panda type, pedal hardware, radar hardware, and radar mounting.
2. Confirm the exact openpilot commit and the exact `opendbc_repo` commit. Update the opendbc
   repository first, then commit the parent submodule pointer.
3. Begin with pedal, radar, navigation maneuvers, tap lane change, parked signal testing, and
   all web control writes disabled.
4. Validate passive CAN parsing, gear, door, brake, gas, steering angle, steering effort,
   indicator state, cruise stalk, and counters/checksums from a parked log.
5. Verify physical stalk input is distinguishable from every transmitted `0x45` echo.
6. Verify panda safety tests and the Python/panda vehicle-model limits agree.
7. If using a Comma Pedal, select the correct bus and perform a fresh calibration on that car.
   Never copy `NAPPedalCalib*` values from another vehicle.
8. If using radar, validate message rates and tracks before enabling control decisions.
9. Test engagement, disengagement, brake, gas, steering takeover, stalk cancellation, and CAN
   fault behavior while stationary or on closed property.
10. Enable one experimental feature at a time and capture a route/log for review.

### Do not copy blindly

- `/data/params/d/` from another comma;
- pedal calibration values;
- radar offset or behind-nosecone setting;
- EPAS firmware images or assumptions;
- vehicle-specific feedforward tables;
- BMS capacity/brick interpretations;
- `NAPForcePreAP` onto a non-Pre-AP Tesla;
- the web server's write-enabled controls onto an unknown fork.

## Known limitations and current work

- Most custom dynamics tuning has one primary development vehicle behind it.
- Navigation is a local desire/signaling bridge, not full autonomy.
- Navigation-aware longitudinal planning is planned, not implemented.
- Automatic lateral resume remains unreliable and should be considered unsupported.
- Some BMS values and video export paths still require validation/fixes.
- MCU2 and remote-browser performance can constrain dashboard refresh rates.
- Tailscale persistence should be verified after device and network reboots.
- Synthetic signaling and tap arbitration have focused unit coverage, but still need broader
  real-car regression logs across stalk firmware and vehicle variants.
- Voice commands through a Pixel, YOLO object/radar fusion, navigation-aware longitudinal
  planning, and Tesla instrument-cluster cruise indication are ideas under development, not
  features in this milestone.

## Developer verification

At minimum, changes should receive:

```bash
python3 -m py_compile <changed Python files>
git diff --check
```

For navigation and tap arbitration:

```bash
python -m pytest selfdrive/controls/lib/tests/test_tap_lane_change.py
```

For Pre-AP panda safety:

```bash
python -m pytest opendbc_repo/opendbc/safety/tests/test_tesla_preap.py -v
```

Then validate with parked CAN tests and a supervised low-risk road test. A passing syntax or
unit test is not proof that a CAN modification is safe on another vehicle.

## Repository layout

| Area | Location |
|---|---|
| Main openpilot fork | `philipr144-create/openpilot-preap` |
| Pre-AP car port and safety | `opendbc_repo` → `philipr144-create/opendbc-preap` |
| NAP-specific developer docs | `docs-nap/` |
| NAP settings UI | `selfdrive/ui/layouts/settings/nap.py` and mici equivalent |
| Navigation desire/arbitration | `selfdrive/modeld/navigation_desire.py`, `selfdrive/modeld/modeld.py` |
| Tap model integration | `selfdrive/controls/lib/tap_lane_change.py` |
| Tesla synthetic signaling | `opendbc_repo/opendbc/car/tesla/preap/tap_lane_change.py` |
| Local dashboard source | `philipr144-create/NAP-Dash` |
| Device dashboard checkout | `/data/NAP-Dash-Release` |

## Release recommendation

Do not merge this milestone directly into a general release yet. A reasonable path is:

1. keep `milestone/2026-09-11` as the known baseline;
2. test `repair/nav-signal-arbitration` on the development car;
3. collect navigation, stalk, model desire, CAN, and low-speed turn logs;
4. separate vehicle-specific tuning from generally safe Pre-AP infrastructure;
5. publish a telemetry-only dashboard option for older NAP builds;
6. promote features individually after multi-car validation.

## Acknowledgments

This work builds on comma.ai openpilot, NotAutopilot, the Tinkla/Boggyver Pre-AP work,
xnor-tech's Tesla support, and the testers and contributors already listed in the main NAP
README.
