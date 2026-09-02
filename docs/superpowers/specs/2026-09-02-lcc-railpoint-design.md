# lcc-railpoint — MP1 turnout node (WiFi transport) — Design

- **Date:** 2026-09-02
- **Status:** approved, pre-implementation
- **Fleet:** LCC/OpenLCB node #4 — node ID `03.04.00.1E.48.04` (MERG member 7752)
- **Repo:** Forgejo `john/lcc-railpoint`
- **Supersedes (transport only):** the MQTT `john/railpoint` firmware, same XIAO-S3 board

## Purpose

Drive **MTB MP1v2** motorised point machines (via **DRV8871** H-bridges) as native
JMRI OpenLCB **Turnouts**, replacing the MQTT control path — on the **same physical
XIAO ESP32-S3 board**, with **no CAN hardware added** (the board is a finished WiFi
unit). Gains over MQTT: native JMRI turnout + CDI config, boot-position restore, no
broker. Cost: a JMRI hub dependency + WiFi/TCP transport.

## Approved decisions (2026-09-02)

- **Transport = WiFi** (GridConnect-TCP uplink to the JMRI hub `192.168.1.46:12021`),
  the approach validated end-to-end on a spike this session — see
  `[[lcc-node-fleet]]` §"WiFi-transport OpenLCB node + JMRI hub".
- **WiFi provisioning = tzapu WiFiManager captive portal** (SSID/password entered by
  the user, NOT hardcoded). RISK: tzapu + `Esp32WiFiManager` coexistence is unproven
  — proven FIRST as task 1 before building the rest.
- **Scope now = FY1 + FY2** (two points); array left expandable.
- **Node ID** `03.04.00.1E.48.04`.
- **MCP23017 GPIO expander = later** (to scale point count beyond the XIAO's GPIOs);
  out of scope for this build.

## Architecture

### Transport (from the validated spike)
- OpenMRNLite v2.0.0 `Esp32WiFiManager` in **uplink mode** (`#define USE_WIFI`, no
  `USE_CAN`/`USE_TWAI` → no CAN/transceiver/`Esp32HardwareTwai` compiled). Full node:
  SPIFFS + `ConfigDef` (with a `WiFiConfiguration` group so `cfg.seg().wifi()`
  resolves) + SNIP.
- `factory_reset` defaults the uplink to **Manual Only + `192.168.1.46:12021`** so it
  connects with zero JMRI-side CDI editing.
- **WiFi creds via tzapu WiFiManager:** global mutable `char` buffers for SSID/pass;
  `Esp32WiFiManager` constructed pointing at them. In `setup()` BEFORE
  `openmrn.begin()`, tzapu obtains the creds (captive portal `lcc-railpoint-Setup` on
  first boot / saved creds thereafter) into those buffers; then `Esp32WiFiManager`
  owns the connection after `begin()`. (Task 1 proves this doesn't fight.)
- **`Esp32Gpio.hxx:97` octal-PSRAM `static_assert` typo** (stray `)`) breaks any S3
  build — fixed by a **committed build-time `extra_scripts`** step that patches the
  vendored header in `.pio/libdeps` after install (a real repo can't rely on a manual
  edit).

### Turnout channel model (adapt fleet node #3)
- A `RepeatedGroup<PointConfig, N>` — presented to JMRI as OpenLCB **Turnouts**
  (thrown/closed consume). Consume event → set commanded state + `restart_`; the
  loop-thread updater drives the actuator. NVS boot-restore + 3-pass producer-query
  recovery (JMRI-authoritative), ported from node #3.
- **Per-point CDI:** `event_closed`, `event_thrown`, `throwMs` (default 2000, range
  200–10000), `invert` (swap which drive direction = closed/thrown), `description`.
- **IN1/IN2 pins are a compile-time array** (physically wired to each DRV8871, not
  runtime-selectable): **FY1 = D1/D3**, **FY2 = D4/D5** (XIAO-S3 clean pairs; avoid
  D2/GPIO3 strapping, D6/D7 UART). A point with no events assigned is inert.

### Actuator — DRV8871 H-bridge driver (new, mirrors the MQTT drive)
- New thin driver: `set_direction(in1, in2, dir)` and `coast(in1, in2)`.
- On a command: set IN1/IN2 for the direction (`in1high = closed ^ invert`), pulse
  for `throwMs`, then **coast** (both inputs low → outputs off; the MP1's internal
  end-stop has already cut motor current). A per-point loop-thread state machine
  (energise → coast after `throwMs`), analogous to node #3's servo sweep tick. All
  actuator GPIO writes happen on the loop thread.
- **No frog logic** — the MP1's single aux SPDT switches the frog mechanically.
  Position is dead-reckoned (the one SPDT is the frog, so no sensing), same as the
  MQTT version.

## Hardware

- **XIAO ESP32-S3** (existing board), **u.FL antenna required**. One DRV8871 per
  point (IN1/IN2 → 2 GPIOs; OUT1/OUT2 → MP1 POZ1/POZ2; COM → +12 V = driver VM).
  MP1 aux SPDT → frog (external PPTC in the frog feed). ~9–14 V, ~150 mA moving, 0 at
  rest. See `[[railpoint-mp1-driver]]` for the full MP1 wiring.
- Genuine DRV8871 = 6 pins (VM/GND/IN1/IN2/OUT1/OUT2); cheap multipacks ship DOA —
  swap the board early, don't deep-debug.

## Success criteria

- First boot: captive portal provisions WiFi; node connects to the JMRI hub and
  appears as node `…48.04` alongside the CAN nodes.
- A JMRI Turnout thrown/closed → the MP1 throws (DRV8871 pulses `throwMs` then
  coasts), frog follows via the aux contact, JMRI shows the commanded state.
- Config (events, throwMs, invert, name) applies live (`trigger_update`).
- After a power-cut: each point restores its last commanded position (NVS), then
  producer-query lets JMRI's state win.
- `invert` fixes a reversed motor without rewiring.

## Build phases (input to the plan)

1. **WiFi-provisioning + uplink spike** — tzapu WiFiManager → `Esp32WiFiManager`
   uplink → JMRI hub, appears in JMRI. **De-risks the unproven coexistence FIRST.**
   (Includes the `extra_scripts` `Esp32Gpio.hxx` patch + repo scaffold.)
2. **Turnout channel + CDI** — `RepeatedGroup<PointConfig, 2>`, consume thrown/closed,
   live-apply; no actuator yet (log intent).
3. **DRV8871 driver + drive logic** — pulse `throwMs` then coast; FY1/FY2 pin array;
   `invert`.
4. **Persistence + boot restore + producer-query recovery** (NVS, from node #3).
5. *(Later)* MCP23017 expander for more points.
