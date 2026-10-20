# NAP v1.0.0 — feature inventory and verification matrix

This matrix documents the October 2026 custom release. Labels mean:

- **Implemented / configurable:** code and UI paths identified in the release; effectiveness remains vehicle-dependent.
- **Conditional:** hardware, settings, companion services, fresh telemetry, and/or specific driving mode required.
- **Experimental / unverified:** present in code or UI but not established as fully validated behavior.
- **Not implemented:** a visible placeholder or requested feature that must not be advertised as ready.

| Feature | Component / control | Status | Critical conditions or limitations |
| --- | --- | --- | --- |
| EPAS lateral steering | openpilot + opendbc | Conditional | Vehicle-specific hardware, calibration, safety; driver override |
| Comma Pedal throttle/regen | openpilot + opendbc | Conditional | Pedal and calibration required |
| Bosch radar lead following | opendbc + openpilot | Conditional | Correct radar mounting, bus, offsets and signal health |
| Driving personalities | comma settings and dash | Implemented / configurable | Aggressive / Standard / Relaxed; dashboard may say Chill |
| Follow distance | comma settings and dash | Implemented / configurable | 1–7; may be overridden by stalk |
| Adaptive acceleration | openpilot | Implemented / configurable | Lead/headroom restrictions remain authoritative |
| Normal/Experimental longitudinal paths | openpilot | Conditional | Experimental model behavior is alpha-level; never assume stops are guaranteed |
| Unified cruise smoothing / coasting / regen | openpilot controls | Implemented; road tuning ongoing | Preserve braking authority and valid speed ceilings |
| Brake + throttle resume | NAP driving setting | Conditional | Brake release plus low-speed accelerator action, according to UI |
| MPP speed-limit writer | startup + `/dev/shm/mpp_speed_limit` | Conditional | Requires custom startup wrapper; validate freshness |
| +5 mph offset / speed trim | NAP settings / MCU2 | Implemented / configurable | Driver responsible for speed; not a speed-limit permission |
| Map driving assist | openpilot map service | Conditional | Fresh map data; lower-target assist, not an acceleration authority |
| Corner assist / curve comfort | openpilot | Conditional | More / Standard / Mild profile; map/model/lead constraints |
| City turns | openpilot + opendbc | Experimental / conditional | Turn signal, speed, intent/state and steering constraints |
| Wider low-speed turns | openpilot | Experimental / conditional | Not a curb-strike prevention guarantee |
| Low-speed steering assist | openpilot/opendbc | Conditional | Designed for slow signaled turns; physical steering limits apply |
| Tap lane change | openpilot/opendbc | Experimental / conditional | Driver must supervise; indicator arbitration matters |
| Lane centering assist | openpilot | Experimental / unverified | Clear lane lines, >20 mph; bounded corrections; driver and signals take priority |
| Lane centering strength/position | comma settings | Implemented / configurable | Low/Medium/High; lateral offset 12/9/6 in or center |
| Phone navigation pairing/remote | NAP-Dash | Implemented / conditional | Companion server and phone/browser network needed |
| Route tracking/desire bridge | NAP-Dash + openpilot | Conditional | Fresh inputs and source availability; not autonomous turns |
| Local/offline road context and mapped stops | openpilot + NAP-Dash | Conditional | Map coverage and input freshness vary |
| Navigation synthetic signals | openpilot/opendbc | Experimental / unverified | Visible turn intent does not establish reliable blinkers |
| Legacy Pitch Sensor Assist | opendbc + both UIs | Conditional | OFF / MONITOR / ON; sign verification gate |
| Predictive Hill Assist | openpilot | Experimental / conditional | Separate file switch; 20–80 mph, taper >65, bounded +0.35 m/s²; braking/lead/curves priority |
| Pitch/speed diagnostic status | openpilot | Implemented / conditional | Fresh status and enabled diagnostic recording |
| Visual path themes and width | comma UI | Implemented / display only | Classic/Blue/Rainbow/Dynamic; wider path **visual only** |
| Lead markers | comma UI | Implemented / display only | Supported fresh lead estimates, **not** blind-spot detection |
| Engagement stats | comma UI | Implemented / display only | Time totals by steering/speed/mode; onroad strip optional |
| MCU2 driving dashboard | NAP-Dash `server_v21.py` | Conditional | Server running, local network, data sources healthy |
| Battery/BMS health and 96 brick display | NAP-Dash | Conditional | Available CAN/BMS signals; chart history needs recorder data |
| Charging/energy/efficiency logs | NAP-Dash | Conditional | Data recording and persistent disk space |
| Performance runs | NAP-Dash | Conditional | Valid speed/power samples and history recorder |
| History SQLite/explorer/export | NAP-Dash + `/data/bms/nap_dash` | Conditional | Runtime DB **not** in Git |
| Camera viewer / telemetry HUD / export | NAP-Dash | Conditional | Recording files, codec, optional ffmpeg, storage and permissions |
| Continental EAS read-only DTC | NAP-Dash + openpilot/opendbc | Conditional | Parked, valid CAN/speed, safety prerequisites |
| EAS manual lowering | NAP-Dash + openpilot/opendbc | Experimental / conditional | Below 40 mph and fresh data, hold-to-run, restore required; use only with appropriate service precautions |
| EAS inflation and fixed height | Not provided | **Not implemented** | No confirmed target-height/inflate command |
| iBooster braking | NAP settings placeholder | **Not implemented** | UI explicitly disabled |
| EPAS firmware backup/flash/restore | NAP settings actions | Conditional / high risk | Trained operator, compatibility, power, backups; not standard install |
| Automatic hands-off intersection stopping or fully autonomous route driving | — | **Not established** | Driver must be prepared to brake and steer |

## Source locations

- `selfdrive/ui/layouts/settings/nap.py`, `nap_content.py`, `toggles.py`: visible feature selectors and warnings.
- `selfdrive/controls/lib/longitudinal_planner.py`, `predictive_grade_assist.py`, `navigation_turn_commit.py`: custom speed/grade/turn arbitration.
- `selfdrive/nap/grade_toggle_ui.py`: V9e file-backed enable and diagnostics with offroad safeguards.
- `selfdrive/ui/onroad/visual_theme.py`, `selfdrive/ui/engagement_stats.py`: presentation/statistics features.
- `opendbc_repo/opendbc/car/tesla/preap/`: specialized Tesla Pre-AP integration.
- NAP-Dash: `server_v21.py`, `phone_navigation/`, `airride.py`, `airride_diagnostics.py`, `airride_page.py`, `tests/`.

## Features and deployment items still requiring a check

1. Full installed hardware and car compatibility outside the existing 2013 Model S build.
2. Service startup mechanism for `/data/server`, `/data/continue.sh`, MPP writer, networking processes and phone navigation.
3. Secret-free packaging or explicit instructions for ignored `tailscaled_daemon.py`, `cloudflared_tunnel.py`, and optional `ffmpeg_full`.
4. Restore of settings, radar offsets, Comma Pedal calibration, local road cache, and battery/charging history without exposing private data.
5. On-road validation of experimental lane/turn/map/grade systems, and blinkers generated from navigation intent.
6. Safe factory-restoration confirmation for EAS manual controls and reliable interpretation of Continental DTCs.

When documentation and source disagree, **source and verified hardware tests win**. New features should be added here only after checking their implementation, required inputs, and limitations.
