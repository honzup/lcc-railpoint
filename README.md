# lcc-railpoint

LCC / OpenLCB firmware that drives **MTB MP1** motorised point machines from a
**Seeed XIAO ESP32-S3** through **DRV8871** H-bridges, as ordinary JMRI turnouts.

It is a **WiFi node**: there is no CAN hardware. The node joins your WiFi and
connects to JMRI's OpenLCB hub over TCP (GridConnect), so it is on the same LCC
network as your CAN nodes, with JMRI forwarding between the two.

- One DRV8871 per point. A thrown/closed event drives the motor for `throwMs`
  (default 2 s), then lets it coast – the MP1's own end-stop cuts the current.
- **Frog polarity** comes from the MP1's own changeover contact – no GPIO or relay.
- **5 points**: 3 on native GPIO plus 2 on an MCP23017 I²C expander.
- Per point, set from JMRI's node configuration (CDI): closed/thrown events,
  `throwMs`, invert, and a description.
- Remembers each point's position across power cycles, and asks the network for
  the current state at start-up so it stays in step with JMRI.
- WiFi set-up through a captive portal (no credentials in the code); WiFi
  firmware updates (OTA).

## Hardware

- Seeed **XIAO ESP32-S3** with its **u.FL WiFi antenna fitted** (it barely
  connects without it).
- One **DRV8871** module per point (the genuine ones have exactly six pins:
  VM, GND, IN1, IN2, OUT1, OUT2). Cheap multipacks often include dead modules –
  if a point won't move, swap the module before debugging the wiring.
- **MCP23017** module for points 4 and 5 (address `0x20`: A0–A2 to GND).
- 12 V supply for the point motors, and 5 V (USB-C, or a buck converter from the
  12 V) for the XIAO. **Common ground** between everything.

### Wiring per point

The MP1 is a three-wire machine: `COM`, `POZ1`, `POZ2`, plus one changeover
contact for the frog.

| From | To |
|------|----|
| +12 V | DRV8871 VM **and** MP1 `COM` |
| 0 V | DRV8871 GND (and XIAO GND) |
| DRV8871 OUT1 / OUT2 | MP1 `POZ1` / `POZ2` |
| XIAO pin (table below) | DRV8871 IN1 / IN2 |
| MP1 changeover contact | frog and the two rail feeds (fit a resettable fuse in the frog feed – the MP1 v2 contact has no current limit) |

If a point moves the wrong way, either swap OUT1/OUT2 or set **Invert** for it in
the CDI.

### Pin map (`POINT_PINS` in `src/main.cpp`)

| Point | IN1 | IN2 | On |
|-------|-----|-----|----|
| 1 | D1 (GPIO2) | D3 (GPIO4) | XIAO |
| 2 | D8 (GPIO7) | D9 (GPIO8) | XIAO |
| 3 | D10 (GPIO9) | D0 (GPIO1) | XIAO |
| 4 | GPA0 | GPA1 | MCP23017 |
| 5 | GPA2 | GPA3 | MCP23017 |

MCP23017: SDA = D4 (GPIO5), SCL = D5 (GPIO6), VCC 3.3 V. The pull-ups on a
typical MCP23017 module are enough. Spare XIAO pins: D6/D7 (UART) and D2
(a strapping pin – avoid).

All DRV8871 inputs are driven low at the very start of `setup()`, so points
don't twitch while the node boots.

## Build and install

Uses PlatformIO with `espressif32@6.9.0` (arduino-esp32 2.0.x) and
**OpenMRNLite v2.0.0**.

1. **Set two values** at the top of `src/main.cpp` (the "SET THESE FOR YOUR
   LAYOUT" block):
   - `NODE_ID` – a node ID from **your own** range. The one in the file is
     mine; two nodes with the same ID break the network. (MERG members:
     `03.04.00.<membership number in hex>.<nn>`; otherwise get a free range
     from OpenLCB.)
   - `HUB_HOST` / `HUB_PORT` – the address of the computer running JMRI, port
     `12021` by default.
2. **In JMRI**, start the hub: **OpenLCB menu → Start Hub**.
3. **First flash over USB.** The XIAO-S3 needs download mode: hold **BOOT**, tap
   **RESET**, release **BOOT**, then:
   ```bash
   pio run -e xiao_s3_wifi -t upload
   ```
4. **Join your WiFi.** On first boot the node opens a WiFi access point called
   **`lcc-railpoint-Setup`** for 3 minutes. Connect from a phone, pick your
   network and enter the password. (No credentials saved and the portal timed
   out? Power-cycle to try again.)
5. The node connects to the JMRI hub and appears in **OpenLCB → Open LccPro**
   (or **Configure Nodes**) as "LCC RailPoint".
6. **Create the turnouts** in JMRI (Turnout table → Add, OpenLCB connection),
   one per point, using that point's closed/thrown events from the CDI.
   Defaults are derived from the node ID: point 1 = `…00` / `…01`, point 2 =
   `…02` / `…03`, and so on.
7. Adjust `throwMs`, Invert and descriptions per point in the CDI.

**Later updates go over WiFi:**

```bash
pio run -e xiao_s3_wifi_ota -t upload --upload-port <node-ip>
```

Use the IP address, not a `.local` name. ArduinoOTA has **no password**
in this build – add `ArduinoOTA.setPassword()` in `main.cpp` if your WiFi isn't
trusted.

### OpenMRNLite 2.0.0 patch for the ESP32-S3

OpenMRNLite 2.0.0 has a stray bracket in `Esp32Gpio.hxx` that breaks every
ESP32-S3 build. `patch_openmrnlite.py` runs automatically before each build and
fixes the downloaded library in place, so nothing manual is needed – but it's
worth knowing about if you start your own S3 project.

## Limits and findings

- **Five points is the tested maximum on WiFi.** With 4 points (8 minutes) and
  5 points (54 minutes) the node held one hub connection throughout. An 8-point build (more
  MCP23017 points, and so a larger configuration) made the node reconnect every
  few moves. I haven't proven why. That attempt is kept under the tag
  `mcp-8point`; on a CAN connection it may well be fine.
- No position feedback: the MP1 has one changeover contact, used here for the
  frog, so the node reports the *commanded* position.
- A build with only the three native points and no MCP23017 is under the tag
  `native-3point`.

## Licence

MIT – see `LICENSE`.
