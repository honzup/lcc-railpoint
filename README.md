# lcc-railpoint — Fleet #4 (WiFi-transport)

LCC/OpenLCB node driving **MTB MP1v2 motorised point machines** (via **DRV8871**
H-bridges) as JMRI turnouts — the OpenLCB successor to the MQTT `john/railpoint`
firmware, on the same **XIAO ESP32-S3** hardware.

- Node ID: `03.04.00.1E.48.04` (MERG member 7752)
- **WiFi transport** (no CAN hardware): connects over WiFi as a GridConnect-TCP
  uplink to a **JMRI hub** (`192.168.1.46:12021`). Same LCC network as the CAN nodes
  behind the ESP32-P4 bridge.
- Each point = a JMRI OpenLCB **Turnout**; a command pulses the DRV8871 for `throwMs`
  then coasts (the MP1 end-stop cuts current). **Frog polarity is switched by the
  MP1's own aux SPDT contact** — no GPIO/relay.
- **3 points, native GPIO** (`main`). Each point is one DRV8871 — IN1/IN2 below,
  plus COM→+12 V, OUT1/OUT2→POZ1/POZ2, aux→frog:

  | Point | IN1 | IN2 | GPIO |
  |-------|-----|-----|------|
  | FY1   | D1  | D3  | 2 / 4 |
  | FY2   | D8  | D9  | 7 / 8 |
  | FY3   | D10 | D0  | 9 / 1 |

  D4/D5 are left free (I²C, if the MCP path is ever revived). JMRI OpenLCB turnout
  systemName is `MT<thrown-event>;<closed-event>`; events are node-derived — FY1
  `…00/…01`, FY2 `…02/…03`, FY3 `…04/…05`.
- MCP23017 expansion (points 4–8 over I²C) lives on the **`mcp-8point`** branch. It
  destabilises the node (frequent resets — cause not yet found), so it's parked;
  native-only is the stable line.

Design spec: [`docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md`](docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md)

Toolchain: PlatformIO `espressif32@6.9.0` (arduino-esp32 2.x), **OpenMRNLite v2.0.0**,
`gnu++17`, `min_spiffs`, board `seeed_xiao_esp32s3`. First flash needs manual
download mode (hold BOOT, tap RESET, release BOOT); OTA after. XIAO-S3 needs its
**u.FL antenna** attached.

Status: **live** — 3 native points (FY1/FY2/FY3) running stably on layout node `.86`.
`CANONICAL_VERSION 0x1C00`; DRV8871 inputs are driven low at boot so points don't
twitch during the WiFi-provisioning window.
