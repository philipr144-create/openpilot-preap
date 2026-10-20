# NAP-Dash — MCU2 browser dashboard and navigation backend

**Companion to [NAP openpilot-preap](https://github.com/philipr144-create/openpilot-preap)** · Production `main` · `NAP-Dash-v1.0.0` (`5aeb8de9d87f50a025da93b259ff25c9eddd518b`).

NAP-Dash is the separate Python/web service used by the customized Tesla Model S Pre-AP NAP system. It historically runs from `/data/server`, with an MCU2/local-browser interface at `http://<comma-ip>:7070/`. The service is not included automatically in a checkout of openpilot. This README is an optional replacement for the **NAP-Dash** repository's own README; the main NAP README and [installation/recovery guide](https://github.com/philipr144-create/openpilot-preap) describe the full system.

## What is included

| Area | Features | Caveats |
| --- | --- | --- |
| Driving | Live speed, steering, selected speed, lead data, mode/profile, settings and current telemetry | Depends on fresh openpilot/CAN data |
| BMS | Pack voltage/current/kW/SOC, nominal full energy/range, temperatures, limits, spread, 96 brick display, battery trend | Only as complete as accessible BMS messages |
| Energy | Trip efficiency, regen, power graphs and charge-session charts/history | Requires recorder and storage |
| Performance | 0–speed, rolling and braking run markers, pack/thermal context | Diagnostic data, not certified timing equipment |
| History | SQLite-backed searchable records, event detail, CSV/JSON export | Runtime DB stored outside Git |
| Cameras | Road/driver segments, timeline, optional telemetry HUD, clip export | Recordings, codec/ffmpeg, device permissions required |
| Navigation | Browser phone remote, setup-code pairing, route tracking/desire bridge, mapped road/stop context | Do not assume automated turns or blinkers |
| Settings | Driving personality, follow distance, adaptive accel, experimental, speed offset/trim, **legacy** pitch sensor selector | V9e predictive-grade switch is separate |
| Air Ride | Continental read-only DTC, safeguarded hold-to-pulse lowering, factory restoration request | No proven inflation/target-height; service-only investigation |

### Runtime source layout

```text
/data/server/
  server_v21.py
  phone_navigation/
  airride.py
  airride_page.py
  airride_diagnostics.py
  tests/
```

The historical server has no independent `pip install` release and relies on the associated openpilot installation, Panda/CAN integration, correct launch/supervision and runtime paths. External, intentionally ignored scripts may also be involved in the specific vehicle deployment.

### Test status

The release-preparation check reported **38 passing tests** using:

```bash
cd /data/server
python3 -m pytest -n 0 -q tests/
```

The Air Ride integration tests reference `/data/openpilot/selfdrive/car/card.py`; they are not standalone tests that run without the matching openpilot tree.

### Safety and privacy

This dashboard is a research tool, **not** a driving authority or general-purpose Tesla service system. Do not publish `/data/bms/nap_dash` history, phone pairing codes, recorded routes, secrets, or local service configuration. Air Ride manual venting and diagnostic CAN actions carry physical risks; read-only dashboard access does not actuate the suspension. Published tests do not prove safe operation under every vehicle condition.

### License and credits

Preserve the repository's existing license and notices. NAP-Dash is part of an independently customized Pre-AP project built upon NotAutopilot, openpilot and the Tesla/Tinkla community.
