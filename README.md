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
- Points: **4 built** (`NUM_POINTS = 4`) — FY1–FY3 on native GPIO + **FY4 on an
  MCP23017** (`0x20`), all four verified on hardware and stable on the WiFi uplink.
  The [Pin map](#pin-map) also lists FY5–FY8 (MCP), but growing to them enlarges the
  CDI, which **destabilises the WiFi uplink** — so dense expansion waits for CAN
  transport (parked on the `mcp-8point` branch). Native pins max out at ~4–5.

## Pin map

MP1 point → DRV8871 inputs (`src/main.cpp` `POINT_PINS`). Per point: one DRV8871 —
COM→+12 V, OUT1/OUT2→POZ1/POZ2, aux→frog. Event pairs are derived from the node ID
(closed = even, thrown = odd); JMRI systemName is `M2T03.04.00.1E.48.04.00.0X;…0Y`.
**FY1–FY4 are built and running; FY5–FY8 are the planned MCP expansion (need CAN).**

| Point | IN1 | IN2 | On | Event pair (`…48.04.00._`) |
|-------|-----|-----|----|----------------------------|
| FY1 | D1 (GPIO2)  | D3 (GPIO4) | native      | `00 ; 01` |
| FY2 | D8 (GPIO7)  | D9 (GPIO8) | native      | `02 ; 03` |
| FY3 | D10 (GPIO9) | D0 (GPIO1) | native      | `04 ; 05` |
| FY4 | GPA0 (DIP 21) | GPA1 (DIP 22) | MCP23017 | `06 ; 07` |
| FY5 | GPA2 (DIP 23) | GPA3 (DIP 24) | MCP23017 | `08 ; 09` |
| FY6 | GPA4 (DIP 25) | GPA5 (DIP 26) | MCP23017 | `0A ; 0B` |
| FY7 | GPA6 (DIP 27) | GPA7 (DIP 28) | MCP23017 | `0C ; 0D` |
| FY8 | GPB0 (DIP 1)  | GPB1 (DIP 2)  | MCP23017 | `0E ; 0F` |

**I²C bus:** SDA = D4 (GPIO5), SCL = D5 (GPIO6). MCP23017 module: VCC→3.3 V, GND→GND,
A0–A2→GND (address `0x20`), /RESET pulled high on-board. The module's on-board 10 kΩ
SDA/SCL pull-ups suffice — don't add more. GPB2–GPB7 are spare (3 more points if
`NUM_POINTS` is raised). Spare native pins: D2 (GPIO3, strapping — avoid), D6/D7
(GPIO43/44, UART).

Design spec: [`docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md`](docs/superpowers/specs/2026-09-02-lcc-railpoint-design.md)

Toolchain: PlatformIO `espressif32@6.9.0` (arduino-esp32 2.x), **OpenMRNLite v2.0.0**,
`gnu++17`, `min_spiffs`, board `seeed_xiao_esp32s3`. First flash needs manual
download mode (hold BOOT, tap RESET, release BOOT); OTA after. XIAO-S3 needs its
**u.FL antenna** attached.

Status: **live** — 4 points running on layout node `.86` (FY1–FY3 native + FY4 on the
MCP23017), stable on the WiFi uplink. `CANONICAL_VERSION 0x1E00`. Full 8-point / dense
MCP expansion is parked on the `mcp-8point` branch, pending CAN transport (WiFi can't
carry the bigger CDI reliably).
