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
- Scope: **FY1 (D1/D3) + FY2** now; array expandable. MCP23017 GPIO expander is a
  later addition.

Design spec: [`docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md`](docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md)

Toolchain: PlatformIO `espressif32@6.9.0` (arduino-esp32 2.x), **OpenMRNLite v2.0.0**,
`gnu++17`, `min_spiffs`, board `seeed_xiao_esp32s3`. First flash needs manual
download mode (hold BOOT, tap RESET, release BOOT); OTA after. XIAO-S3 needs its
**u.FL antenna** attached.

Status: **in development** (adapts fleet node #3's turnout pattern onto the validated
WiFi-transport path).
