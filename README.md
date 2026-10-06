<div align="center">

<h1>NotAutopilot</h1>

<p>
  <b>Self driving for pre-Autopilot Tesla Model S.</b>
  <br>
  Lane keeping, adaptive cruise, and driver monitoring for the 25,000+ pre-AP vehicles Tesla left behind.
</p>

<h3>
  <a href="https://notautopilot.com">Website</a>
  <span> · </span>
  <a href="https://notautopilot.com/wiki">Wiki</a>
  <span> · </span>
  <a href="https://discord.gg/notautopilot">Discord</a>
  <span> · </span>
  <a href="https://notautopilot.com/how-it-works">How It Works</a>
</h3>

Install on your comma 3X: `https://installer.comma.ai/NotAutopilot/openpilot/nap-release`

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

</div>

---

## What is NotAutopilot?

NotAutopilot (NAP) is a fork of [comma.ai's openpilot](https://github.com/commaai/openpilot) that brings driver assistance to 2012–2014 pre-Autopilot Tesla Model S vehicles. These cars shipped without forward-facing cameras, radar, or any driver assistance — NAP adds all of it.

Built on the foundation of the [Tinkla project](https://tinkla.us/t/index) (which first proved openpilot could work on pre-AP Teslas) and [xnor-tech's openpilot](https://github.com/xnor-tech/openpilot) (which added working AP1 Model S support), NAP extends support to pre-Autopilot vehicles with a standalone safety model, Bosch radar integration, and Comma Pedal longitudinal control.

### Features

- **Lateral control** — Steering via EPAS with full panda safety validation
- **Longitudinal control** — Comma Pedal interceptor with zero-torque learning, smooth engagement, and three driving profiles (Aggressive, Standard, Chill)
- **Radar** — Bosch radar support with GTW emulation for lead car detection and adaptive following
- **Driver monitoring** — Stock openpilot driver monitoring camera
- **Air Ride web controls** — Front, rear, all-four, and individual-corner lowering with hold-to-pulse controls, factory restoration, error reset, and a 40 mph safety gate
- **On-device settings** — Full NAP settings panel for pedal calibration, radar configuration, EPAS firmware management, and tuning

### Hardware Required

1. **2012–2014 Tesla Model S** (pre-Autopilot, no AP ECU)
2. **[comma 3X](https://comma.ai/shop)** (or comma 3)
3. **[Tesla Model S HW1 harness](https://comma.ai/shop/car-harness)** 
4. **[Comma Pedal](https://github.com/commaai/openpilot/wiki/Comma-Pedal)** — for longitudinal (throttle/regen) control
5. **Bosch radar** *(optional)* — for adaptive cruise with lead car following

## Installation

On your comma device, enter this URL during setup or in Software settings under Custom Fork:

```
https://installer.comma.ai/NotAutopilot/openpilot/nap-release
```

## NAP Web Dashboard and Air Ride Control

On the same network as the comma, open `http://<comma-address>:7070/` and select **Air Ride**. The Air Ride page is also available at `/airride`. The comma address can change when switching between Wi-Fi and a phone hotspot.

### Lowering the suspension

- Hold **Front Down** to pulse the two front corners, or **Rear Down** for the rear corners.
- Hold **Pancake · All Four** to pulse all four corners.
- Expand **Individual corners** for Driver Front, Passenger Front, Driver Rear, and Passenger Rear.
- Selected corners vent in sequence, with the pulse cycle repeated approximately every 0.8 seconds. Release the button to stop venting.

The controller uses the recovered Tesla Model S Continental EAS service and vent routines. Service entry sends `10 03`, starts `0310`, waits 0.4 seconds, and finishes `0310` before venting. Explicit factory restoration reopens the extended session, stops venting when its stop has not already been acknowledged, starts `0309`, waits one second, finishes `0309`, queries its result, and returns to `10 01`. Negative responses and timeouts remain faults; routine acknowledgements do not independently prove that the physical suspension warning cleared. Unconfirmed restoration state is preserved across software reloads. Letting go requests vent stop and factory restoration. Select **Restore Factory Control** to run the recovered restoration sequence and return leveling to the factory controller. This restores control rather than selecting a particular suspension height. The page currently provides lowering only; inflation and a target-height command have not been established.

### Safety, recovery, and connection handling

Manual lowering requires fresh, valid vehicle-speed data and a speed strictly **below 40 mph**. At 40 mph or above, or when speed data becomes unavailable, manual commands are blocked and the controller attempts to stop venting and restore factory control. Restoration errors remain visible; an attempted restoration is not treated as confirmed success.

The page maintains a short hold heartbeat. Pointer or touch release/cancel, leaving the button, losing page focus, and connection loss end the hold and request factory restoration. Controls suppress text selection and browser long-press menus; a new hold is blocked while the release request is pending. An expired backend hold lease triggers factory restoration. Opening the page or checking status does not move the suspension.

**Reset Error** becomes available once fault cleanup has finished, speed data is valid, and the car is below the speed limit. It clears the error without sending a suspension command, so a hold button can be used to retry. Resetting an error does not confirm factory restoration or remove the speed gate. The status API retains recent requests, responses, and timeouts for diagnosis.

Air Ride shares the existing openpilot `sendcan` publisher. The Tesla Pre-AP Panda safety changes allow only the recovered diagnostic address, exact payloads, and bus; manual starts also require fresh speed data below 40 mph. Stop and restoration commands remain available for recovery above the speed limit. Off-road Panda `noOutput` mode blocks transmission, and Camp Mode does not override it. Existing dashboard navigation and watchdog behavior are retained.

Moving-vehicle operation is still under investigation: parked commands have received positive responses, but reported errors/timeouts while moving below 40 mph have not yet been diagnosed. Reservoir isolation during corner dumping has not been verified.

### MPP speed-limit updates and startup

The existing `mpp_writer.py` reads the vehicle's raw MPP speed-limit signal and updates `/dev/shm/mpp_speed_limit`. In pedal longitudinal mode, the existing engagement logic uses changes in this file to update the cruise target when longitudinal control is enabled.

Preserve the device's `launch_openpilot.sh` startup wrapper: it starts the MPP writer before launching openpilot. Starting `launch_chffrplus.sh` directly bypasses that writer and can leave automatic speed-limit updates reading a stale value. After a restart, verify that one MPP writer is running and that the shared file refreshes from CAN. The recent recovery confirmed a refreshing file matching the received 55 mph limit; automatic cruise-target changes still require driving confirmation.

### Continental EAS fault reading

The backend supports a stationary, read-only DTC request: POST `{"action":"read_dtc"}` to `/api/airride`, then poll `/api/airride/status` for `dtc`. It opens the known extended diagnostic session, sends UDS `19 02 FF` (DTCs by status mask), handles ISO-TP multi-frame replies through card's existing CAN publisher, and returns to the default session. Results retain the raw DTC bytes and status masks. No fault-clearing request is permitted. This reader uses the installed openDBC UDS implementation; DTC descriptions for the Continental controller must be established separately. It requires valid stationary speed, active Tesla Pre-AP safety mode, and no active manual suspension operation.

### Source and verification

The web server is a separate repository at `/data/server`, linked into openpilot as `server`. Air Ride lives in `airride.py` and `airride_page.py`, with routes in `server_v21.py`; openpilot integration is in `selfdrive/car/card.py`. The Panda safety changes and tests live in the `opendbc_repo` submodule. Preserve all three repositories when backing up or reproducing this installation.

The 38 Air Ride tests pass, covering controller behavior, the local endpoint, HTTP/navigation integration, and shared CAN publication. Run them from `/data/openpilot` using the `server` symlink path:

```sh
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python -m unittest discover -s /data/openpilot/server/tests -q
```

## Branches

| Branch | Install URL | Description |
|--------|------------|-------------|
| `nap-release` | `NotAutopilot/openpilot/nap-release` | **Recommended.** Stable release for daily driving. |
| `nap-staging` | — | Pre-release testing. Coming soon. |
| `nap-alpha` | `NotAutopilot/openpilot/nap-alpha` | Alpha builds for testers. May have rough edges. |
| `nap-dev` | — | Active development. Expect things to break. |

## Lineage

```
comma.ai/openpilot
  └── sunnypilot/sunnypilot
        └── xnor-tech/openpilot (AP1 Model S support)
              └── NotAutopilot/openpilot (Pre-AP Model S support)

boggyver/openpilot (Tinkla — the original Pre-AP Tesla fork)
  └── inspiration, radar emulation, pedal architecture → NotAutopilot
```

## Acknowledgments

NotAutopilot wouldn't exist without:

- **[Boggyver](https://github.com/boggyver) and the [Tinkla Project](https://tinkla.us/t/index)** — First proved openpilot could work on pre-AP Teslas. The foundation everything is built on.
- **[Lukas Loetkolben](https://github.com/lukasloetkolben)** — Clean isolated safety model approach that inspired the SAFETY_TESLA_PREAP refactor.
- **[xnor-tech](https://github.com/xnor-tech)** — Working AP1/HW1-4 Tesla support in openpilot.
- **[comma.ai](https://comma.ai)** — openpilot itself. The hard parts were already solved.
- **Johnmr1, SeriouslySerious, Pod042, 1FrostlySlime** — Early testers who drove broken builds so others wouldn't have to.

## Contributing

Pull requests are welcome against the `nap-dev` branch. See [`docs-nap/`](docs-nap/) for branch structure, build instructions, pre-push checklist, safety invariants, and the engagement flow.

## User Data

By default, openpilot uploads driving data to comma servers. You can access your data through [comma connect](https://connect.comma.ai/). Data collection can be disabled in settings.

## Licensing

NotAutopilot is released under the [MIT License](LICENSE). This repository includes original work as well as code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot) and the [Tinkla project by boggyver](https://github.com/boggyver/openpilot), both released under the MIT license.

> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys' fees and costs) which arise out of, relate to or result from any use of this software by user.

Any user of this software shall also indemnify and hold harmless the NotAutopilot project, its contributors, maintainers, and affiliates from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys' fees and costs) which arise out of, relate to or result from any use of this software by user.
