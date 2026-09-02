# lcc-railpoint Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development to implement
> this plan task-by-task.

**Goal:** A WiFi-transport OpenLCB node on the XIAO ESP32-S3 that drives MP1v2 point
machines (DRV8871) as JMRI Turnouts, replacing the MQTT railpoint control path.

**Architecture:** OpenMRNLite v2.0.0 `Esp32WiFiManager` uplink → JMRI hub; node #3's
turnout channel pattern (consume thrown/closed → drive → NVS restore + producer-query)
with a DRV8871 H-bridge actuator (pulse `throwMs` then coast); frog via the MP1 aux
contact (no GPIO). tzapu WiFiManager for WiFi provisioning.

**Tech Stack:** PlatformIO `espressif32@6.9.0` (arduino-esp32 2.x), OpenMRNLite v2.0.0,
`gnu++17`, `min_spiffs`, board `seeed_xiao_esp32s3`.

**Spec:** `docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md`

## Global Constraints
- Node ID `0x0304001E4804`. Toolchain pinned exactly as above; OpenMRNLite `#v2.0.0`.
- `#define USE_WIFI` only (no CAN/TWAI). Uplink → `192.168.1.46:12021` (Manual).
- `Esp32Gpio.hxx:97` octal-PSRAM typo patched via a committed `extra_scripts` step
  (patches `.pio/libdeps` after install) — never a manual edit.
- `Esp32WiFiManager::CONN_MODE_UPLINK_BIT` is private → omit the last ctor arg.
- WiFi creds provisioned by tzapu WiFiManager into buffers before `openmrn.begin()`.
- Actuator/GPIO writes only on the loop thread. NO `Co-Authored-By: Claude` /
  `Claude-Session` commit trailers. Reference the spec for exact values.
- First flash = manual download mode (BOOT held, tap RESET); u.FL antenna attached.

---

### Task 1: Scaffold + WiFi provisioning + uplink (DE-RISK FIRST)

**Files:** `platformio.ini`, `patch_openmrnlite.py` (extra_script), `src/main.cpp`,
`src/config.h`.

**Goal / acceptance:** the node provisions WiFi via a tzapu captive portal
(`lcc-railpoint-Setup`), then connects as an `Esp32WiFiManager` uplink to the JMRI hub
and appears in JMRI as node `03.04.00.1E.48.04`. **This proves the tzapu +
Esp32WiFiManager coexistence — the plan's biggest risk.** No turnout logic yet.

- Reuse the validated spike at `~/…/scratchpad/s3-wifi-node/` (platformio.ini,
  main.cpp, config.h) as the starting point for the WiFi-only node + the uplink
  factory-reset defaults + the `Esp32Gpio.hxx` fix — but move the header fix into a
  committed `extra_scripts = pre:patch_openmrnlite.py` that seds/patches the vendored
  `Esp32Gpio.hxx` (line ~97, the stray `)`), since `.pio/libdeps` is git-ignored.
- Replace the hardcoded `secrets.h` creds with **tzapu WiFiManager provisioning**:
  global `char g_ssid[33]; char g_pass[65];`; construct `Esp32WiFiManager
  wifi_mgr(g_ssid, g_pass, openmrn.stack(), cfg.seg().wifi());`. In `setup()` BEFORE
  `openmrn.begin()`: run tzapu `WiFiManager` (`autoConnect("lcc-railpoint-Setup")`,
  `setConfigPortalTimeout(180)`); on success copy the creds into `g_ssid`/`g_pass`
  (from `WiFi.SSID()`/`WiFi.psk()`), then hand off to `Esp32WiFiManager` (which starts
  the WiFi/uplink after `begin()`). Handle the two libraries not fighting over the
  WiFi driver — if a clean handoff needs `WiFi.disconnect(false)` or similar, do it
  and document why. `lib_deps` adds `tzapu/WiFiManager@^2.0.17`.
- Build (`pio run`), report Flash%. Do NOT flash from the subagent — the controller
  flashes with the user's manual download-mode assist and confirms in JMRI.

### Task 2: Turnout channel + CDI (RepeatedGroup, 2 points)

**Files:** `src/config.h`, `src/main.cpp`.

**Goal / acceptance:** JMRI shows 2 Turnout tabs; throwing/closing sets the commanded
state and applies live; no actuator yet (log the intent, e.g. `LOG(INFO,"[FY%d] ->
%s")`).

- Adapt node #3's `TurnoutChannel` (consume `event_thrown`/`event_closed` via
  `GPIOBit`+`BitEventConsumer`+a `CmdGpio` setting `commanded_thrown_`+`restart_`).
- `RepeatedGroup<PointConfig, 2>`; per-point CDI: `event_closed`, `event_thrown`,
  `throwMs` (Uint16, Default 2000, Min 200 Max 10000), `invert` (Uint8 Normal/Invert),
  `description` (String<16>). `CANONICAL_VERSION` starts at e.g. `0x1900`.
- Reuse node #3's `SyncedFileMemorySpace` + debounced-fsync + `trigger_update`
  live-apply block verbatim. `factory_reset` seeds blanks.

### Task 3: DRV8871 driver + drive logic

**Files:** `src/drv8871.h` (+ `.cpp` if needed), `src/main.cpp`.

**Goal / acceptance:** on hardware, a JMRI throw/close pulses the correct DRV8871
IN1/IN2 for `throwMs` then coasts; `invert` flips direction; FY1=D1/D3, FY2=D4/D5.

- Thin driver: `set_direction(in1,in2,dir)` / `coast(in1,in2)` (pinMode outputs at
  init). Compile-time `POINT_PINS[2][2] = {{D1,D3},{D4,D5}}`.
- Per-point loop-thread state machine: on `restart_`, set IN1/IN2 (`in1high =
  commanded_thrown_ ? !closedIsIn1 : ... ` — use `commanded_thrown_ ^ invert_` to pick
  direction, mirroring the MQTT `closed ^ invert`), mark pulsing + timestamp; when
  `millis()-start >= throwMs_`, coast. Reset settle for restore. All writes in the
  loop tick (like node #3's `update`).

### Task 4: Persistence + boot restore + producer-query recovery

**Files:** `src/main.cpp`.

**Goal / acceptance:** after a power-cut each point restores its last commanded
position (re-pulses to it), then producer-query lets JMRI's state win. No config-layout
change (NVS).

- Port node #3's NVS scheme verbatim: `Preferences` namespace `railpoint`, key
  `t<idx>`, persist `commanded_thrown_` on settle only-on-change; restore in
  `apply_configuration(initial_load)` (set `commanded_thrown_` + `restart_`; no GPIO in
  apply). Port node #3's 3-pass `query_state`/producer-query `loop()` sweep.

## Self-review
Spec coverage: transport (T1), provisioning (T1), turnout+CDI+live-apply (T2), DRV8871
+ throwMs/coast/invert + FY1/FY2 (T3), NVS restore + producer-query (T4), frog = aux
(no task, hardware). MCP23017 deferred. Types: `commanded_thrown_`, `throwMs_`,
`invert_`, `POINT_PINS` consistent across T2–T4.
