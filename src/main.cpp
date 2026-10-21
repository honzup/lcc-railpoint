/// Copyright (c) 2026 John Holmes -- MIT licence (see LICENSE).
/// Parts based on the ESP32IOBoard example from OpenMRNLite,
/// Copyright (c) 2019 Mike Dunston -- BSD 2-Clause (full notice in THIRD_PARTY_NOTICES).
///
/// LCC RailPoint -- WiFi-transport OpenLCB node for the Seeed XIAO ESP32-S3.
/// Connects as a GridConnect-TCP uplink client to a JMRI OpenLCB hub (no CAN
/// hardware) and drives MTB MP1 point motors through DRV8871 H-bridges: each
/// thrown/closed event pulses the motor for throwMs, then coasts.
///
/// WiFi credentials are NOT hardcoded: a tzapu WiFiManager captive portal
/// ("lcc-railpoint-Setup") provisions them on first boot / when unset, and
/// OpenMRNLite's Esp32WiFiManager owns the connection from then on. See the
/// "[prov] handing off" block in setup() for how the two libraries hand off
/// the WiFi driver without fighting over it.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <Wire.h> // I2C for the MCP23017 GPIO expander (extra point pins)
#include <WiFi.h>
#include <WiFiManager.h> // tzapu/WiFiManager -- captive-portal provisioning
#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <unistd.h> // fsync -- config persistence to flash
#include <Preferences.h> // NVS -- last commanded position, survives power-cut

#include <OpenMRNLite.h>
#include "openlcb/ConfiguredConsumer.hxx"

#define USE_WIFI

#include "config.h"
#include "drv8871.h"

/// MCP23017 GPIO expander (I2C @0x20): lets a point's DRV8871 IN1/IN2 live on
/// its 16 outputs instead of a native GPIO, so more points fit on this one
/// board once the XIAO-S3's native pins run out. Declared here, begin()'d in
/// setup() before any point drives. drv8871.h references it as `extern`.
Mcp23017 g_mcp;

/// Per-point DRV8871 IN1/IN2 pins. A value < 100 is a native XIAO-S3 GPIO
/// (Arduino pin macro); MCP(n) is MCP23017 output n (0-15) -- see drv8871.h.
/// FY1 on D1/D3 matches the proven MQTT railpoint's wiring (see
/// ~/railpoint/railpoint/railpoint.ino). FY2 moved off D4/D5 (now the I2C
/// SDA/SCL bus for the MCP23017) to the still-free D8/D9 pair -- update it
/// here if FY2's DRV8871 is actually wired to different pins.
/// Pre-sized to 8 points: FY1-FY3 native, FY4-FY8 on the MCP23017. Wiring
/// FY4-FY8's DRV8871s later needs no firmware change beyond swapping their
/// MCP output here -- the CDI already holds all 8 (no reformat on hardware add).
static constexpr uint8_t POINT_PINS[openlcb::NUM_POINTS][2] = {
    {D1, D3},         // FY1 native
    {D8, D9},         // FY2 native
    {D10, D0},        // FY3 native
    {MCP(0), MCP(1)}, // FY4 MCP GPA0/GPA1
    {MCP(2), MCP(3)}, // FY5 MCP GPA2/GPA3
};

// ===================== SET THESE FOR YOUR LAYOUT =====================
/// Node ID -- must be unique on your LCC network. Use an ID from your own
/// range (MERG members: 03.04.00.<membership number in hex>.<nn>; otherwise
/// request a free range from OpenLCB). This one is the author's.
static constexpr uint64_t NODE_ID = UINT64_C(0x0304001E4804);

/// JMRI OpenLCB hub the node connects to (JMRI: OpenLCB menu > Start Hub).
/// Written as the factory default on first boot; afterwards it can be changed
/// in the node's CDI (WiFi > Uplink) from JMRI.
static constexpr const char *HUB_HOST = "192.168.1.46";
static constexpr uint16_t HUB_PORT = 12021;
// ======================================================================

/// Primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

/// ConfigDef comes from config.h; defines the CDI/config memory layout.
static constexpr openlcb::ConfigDef cfg(0);

/// WiFi credentials, provisioned at runtime by the tzapu captive portal in
/// setup() (never hardcoded). Sized to match WiFiManager's own limits
/// (32-char SSID + NUL, 64-char WPA2 password + NUL).
char g_ssid[33] = {0};
char g_pass[65] = {0};

/// Esp32WiFiManager is constructed *inside* setup(), after the tzapu portal
/// has populated g_ssid/g_pass -- see the comment at its construction site
/// for why a file-scope global (built before setup() runs) would not work
/// here. Esp32WiFiManager::CONN_MODE_UPLINK_BIT is private; the connection
/// mode arg is simply omitted since its default is already uplink-only.
Esp32WiFiManager *wifi_mgr = nullptr;

/// Last-commanded-position store (NVS), keyed per point ("t%d"). Opened in
/// setup() before openmrn.begin() so it's ready when apply_configuration()
/// restores on initial_load. Ported VERBATIM (pattern) from my LCC servo node.
Preferences prefs;

// ============ DURABLE config persistence: fsync-after-write (debounced) =======
// Ported VERBATIM (pattern) from my LCC servo node --
// see that file's header comment for the full root-cause writeup. Short
// version: ESP32 SPIFFS keeps file writes in cache and only commits to flash
// on fsync/close, so we fsync ONLY after a config WRITE (never a read),
// debounced, on the loop() thread -- and once flushed, trigger_update()
// re-applies config live so a JMRI edit (event/throwMs/invert/description)
// takes effect without a reboot.
static constexpr uint32_t CFG_SYNC_DEBOUNCE_MS = 350;
static int                s_cfg_fd       = -1;
static volatile bool      s_cfg_dirty    = false;
static volatile uint32_t  s_cfg_dirty_ms = 0;

/// One turnout/point: a config-driven consumer whose thrown/closed events
/// set a commanded state, which drives a DRV8871 H-bridge (see drv8871.h)
/// for throw_ms_ then coasts -- the MP1's own end-stop cuts motor current
/// at travel end. Adapted from my LCC servo node's TurnoutChannel, minus the
/// PCA9685/MCP23017 servo+frog drive (frog here is mechanical, via the MP1
/// aux SPDT -- no GPIO).
class PointChannel : public DefaultConfigUpdateListener
{
public:
    PointChannel(openlcb::Node *node, const openlcb::PointConfig &cfg, int idx)
        : node_(node)
        , gpio_(this)
        , impl_(node, 0, 0, &gpio_)
        , consumer_(&impl_)
        , cfg_(cfg)
        , idx_(idx)
    {
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
        BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        fd_ = fd; // retained for parity with the servo node (future save/calibration API)
        throw_ms_ = cfg_.throwMs().read(fd);
        invert_ = cfg_.invert().read(fd) != 0;
        snprintf(name_, sizeof(name_), "%s", cfg_.description().read(fd).c_str());

        if (initial_load)
        {
            // Restore last commanded position from NVS (power-cut recovery).
            // Reading NVS here (executor thread) is fine -- it's not GPIO;
            // the DRV8871 itself is only ever driven from update() below, via
            // restart_ (same rule as everything else in this class). Ported
            // VERBATIM (pattern) from my LCC servo node.
            char key[8];
            snprintf(key, sizeof(key), "t%d", idx_);
            commanded_thrown_ = prefs.getUChar(key, 0) != 0;
            persisted_thrown_ = commanded_thrown_; // don't immediately re-write it
            restart_ = true; // drive to the restored position on the next tick
        }

        openlcb::EventId thrown = cfg_.event_thrown().read(fd);
        openlcb::EventId closed = cfg_.event_closed().read(fd);
        if (thrown == impl_.event_on() && closed == impl_.event_off())
            return UPDATED; // only throwMs/invert/description (or nothing) changed
        // (Re)register the consumer for the new events, keeping the same gpio_.
        consumer_.~BitEventConsumer();
        impl_.openlcb::GPIOBit::~GPIOBit();
        new (&impl_) openlcb::GPIOBit(node_, thrown, closed, &gpio_);
        new (&consumer_) openlcb::BitEventConsumer(&impl_);
        return REINIT_NEEDED;
    }

    void factory_reset(int fd) override
    {
        CDI_FACTORY_RESET(cfg_.throwMs);
        CDI_FACTORY_RESET(cfg_.invert);
        cfg_.description().write(fd, "");
    }

    /// Loop-thread tick (called every ~30ms, same cadence as the servo node's servo
    /// sweep). Drives the DRV8871 pulse for this point; all GPIO writes
    /// happen here, never on the executor/event-handler thread.
    void update(uint32_t now)
    {
        if (!pins_ready_)
        {
            drv8871_init(POINT_PINS[idx_][0], POINT_PINS[idx_][1]);
            pins_ready_ = true;
        }

        if (restart_)
        {
            restart_ = false;
            // Mirror the proven MQTT railpoint's `bool in1high = closed ^
            // invert;` (closed (uninverted) -> IN1 high). Here "closed" is
            // !commanded_thrown_, so in1high = !commanded_thrown_ ^ invert_.
            bool in1high = (!commanded_thrown_) ^ invert_;
            drv8871_drive(POINT_PINS[idx_][0], POINT_PINS[idx_][1], in1high);
            pulsing_ = true;
            pulse_start_ = now;
            LOG(INFO, "[FY%d] -> %s (throwMs=%u invert=%d)", idx_,
                commanded_thrown_ ? "THROWN" : "CLOSED", throw_ms_, invert_);
        }

        if (pulsing_ && (uint32_t)(now - pulse_start_) >= throw_ms_)
        {
            drv8871_coast(POINT_PINS[idx_][0], POINT_PINS[idx_][1]);
            pulsing_ = false;
            LOG(INFO, "[FY%d] settled/coast", idx_);
            // Persist the commanded position to NVS, only on real change --
            // stops a re-apply (trigger_update() after a config fsync) from
            // looping, and limits flash writes to actual position changes.
            if (commanded_thrown_ != persisted_thrown_)
            {
                char k[8];
                snprintf(k, sizeof(k), "t%d", idx_);
                prefs.putUChar(k, commanded_thrown_ ? 1 : 0);
                persisted_thrown_ = commanded_thrown_;
            }
        }
    }

    /// Recover this point's state from the bus after boot: emit an Identify
    /// Producers query so the ProducerIdentified(Valid/Invalid) reply flows
    /// back through consumer_ -> GPIOBit::set_state -> CmdGpio ->
    /// commanded_thrown_, setting the state deterministically -- overriding
    /// the NVS-restored fallback with JMRI's definite state. Asynchronous:
    /// `done` fires once the query is enqueued. Ported VERBATIM from my LCC servo
    /// node's TurnoutChannel::query_state.
    void query_state(openlcb::WriteHelper *w, BarrierNotifiable *done)
    {
        if (impl_.event_on() == 0 && impl_.event_off() == 0) { done->notify(); return; }
        consumer_.SendQuery(w, done); // emits MTI_PRODUCER_IDENTIFY for event_on
    }

private:
    /// A Gpio that routes writes into the owning channel's commanded state
    /// (instead of a physical pin). SET = commanded THROWN, CLR = commanded
    /// CLOSED.
    class CmdGpio : public Gpio
    {
    public:
        explicit CmdGpio(PointChannel *o) : owner_(o) {}
        void write(Value v) const override
        {
            if (v == SET) owner_->commanded_thrown_ = true;
            else          owner_->commanded_thrown_ = false;
            owner_->restart_ = true;
        }
        Value read() const override { return owner_->commanded_thrown_ ? SET : CLR; }
        void set() const override { write(SET); }
        void clr() const override { write(CLR); }
        void set_direction(Direction) const override {}
        Direction direction() const override { return Direction::DOUTPUT; }

    private:
        PointChannel *owner_;
    };

    openlcb::Node *node_;
    CmdGpio gpio_;
    openlcb::GPIOBit impl_;
    openlcb::BitEventConsumer consumer_;
    const openlcb::PointConfig cfg_;
    const int idx_;

    uint16_t throw_ms_ = 2000;
    bool     invert_ = false;
    char     name_[17] = ""; // per-channel Description (StringConfigEntry<16>)

    volatile bool commanded_thrown_ = false; // written from executor (event handler)
    bool     persisted_thrown_ = false;      // last value written to NVS (loop-only)
    volatile bool restart_          = false; // intent changed, (re)start the drive pulse
    int      fd_ = -1;                       // config fd, retained (parity w/ the servo node)

    // Loop-thread-only drive state (never touched off update()).
    bool     pins_ready_  = false; // drv8871_init() done (lazy -- global ctor runs pre-Arduino-init)
    bool     pulsing_     = false;
    uint32_t pulse_start_ = 0;
};

#define PCH(i) PointChannel(openmrn.stack()->node(), cfg.seg().turnouts().entry<i>(), i)
PointChannel point_channels[] = {
    PCH(0), PCH(1), PCH(2), PCH(3), PCH(4) };
#undef PCH

class FactoryResetHelper : public DefaultConfigUpdateListener
{
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
        BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
        cfg.userinfo().description().write(
            fd, "LCC RailPoint -- WiFi uplink to JMRI hub");

        // Pre-point the uplink at the JMRI hub so it connects with zero
        // JMRI-side CDI editing: Manual Only, HUB_HOST:HUB_PORT.
        cfg.seg().wifi().uplink().search_mode().write(
            fd, 3 /* SocketClientParams::SearchMode::MANUAL_ONLY */);
        cfg.seg().wifi().uplink().manual_address().ip_address().write(
            fd, HUB_HOST);
        cfg.seg().wifi().uplink().manual_address().port().write(fd, HUB_PORT);
    }
} factory_reset_helper;

namespace openlcb
{
// Name of CDI.xml to generate dynamically.
const char CDI_FILENAME[] = "/spiffs/cdi.xml";

// This will stop openlcb from exporting the CDI memory space upon start.
extern const char CDI_DATA[] = "";

// Path to where OpenMRN should persist general configuration data.
extern const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";

// The size of the memory space to export over the above device.
extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

// Default to store the dynamic SNIP data in the same persistent data file
// as general configuration data.
extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
} // namespace openlcb

/// Overrides the default SPACE_CONFIG memory space so every config WRITE
/// (never a read) arms the debounced fsync above. Ported VERBATIM (pattern)
/// from my LCC servo node.
class SyncedFileMemorySpace : public openlcb::FileMemorySpace
{
public:
    SyncedFileMemorySpace(int fd, address_t len)
        : openlcb::FileMemorySpace(fd, len) {}

    size_t write(address_t dest, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) override
    {
        size_t w = openlcb::FileMemorySpace::write(dest, data, len, error, again);
        s_cfg_dirty_ms = millis(); // arm/re-arm the debounce (write path only)
        s_cfg_dirty    = true;
        return w;
    }
};

void setup()
{
    Serial.begin(115200L);
    Esp32SocInfo::print_soc_info();
    LOG(INFO, "[Node] ID: %s", uint64_to_string_hex(NODE_ID).c_str());

    // ---- I2C + MCP23017 GPIO expander ----------------------------------
    // Brought up before any point channel drives (points drive from loop(),
    // never setup()). SDA/SCL are the XIAO-S3 defaults (D4/D5), which is why
    // FY2 was moved off that pair. g_mcp.begin() sets all 16 pins as outputs
    // latched low, so an MCP-backed point starts coasting like a native one.
    // This node has no web server, so the serial scan below is the only
    // "is the MCP alive?" diagnostic -- expect "found 0x20".
    Wire.begin(SDA, SCL);
    g_mcp.begin(0x20);
    for (uint8_t a = 1; a < 128; a++)
    {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0)
            LOG(INFO, "[i2c] found 0x%02x", a);
    }

    // ---- Point pins LOW *now*, before the multi-second WiFi provisioning ---
    // The ESP32-S3 powers up with these as floating high-Z inputs, and a
    // floating DRV8871 input can drive the H-bridge -- twitching/moving the
    // point for the whole boot window. update()'s lazy drv8871_init only runs
    // once loop() starts (after all of setup()), which is far too late: any
    // reset or wire reconnect moves the point unexpectedly until then. MCP is
    // already begin()'d above, so MCP-backed points are safe to init here too.
    for (uint8_t i = 0; i < openlcb::NUM_POINTS; ++i)
        drv8871_init(POINT_PINS[i][0], POINT_PINS[i][1]);

    // ---- WiFi provisioning: tzapu captive portal ----------------------
    // Runs before Esp32WiFiManager touches the WiFi driver at all. tzapu's
    // autoConnect() uses arduino-esp32's own WiFi stack to either join
    // saved credentials, or (first boot / no saved creds) start an AP +
    // captive portal ("lcc-railpoint-Setup") for up to 180s so the user
    // can enter the home WiFi SSID/password from a phone/laptop.
    LOG(INFO, "[prov] portal (join saved WiFi, or SSID "
        "'lcc-railpoint-Setup' if unprovisioned)");
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    bool provisioned = wm.autoConnect("lcc-railpoint-Setup");

    if (provisioned)
    {
        // Copy the now-connected creds out of arduino-esp32's WiFi state
        // into the buffers Esp32WiFiManager will read from. This MUST
        // happen before WiFi.disconnect() below -- WiFiSTAClass::SSID()/
        // psk() both return "" once the driver is torn down.
        strlcpy(g_ssid, WiFi.SSID().c_str(), sizeof(g_ssid));
        strlcpy(g_pass, WiFi.psk().c_str(), sizeof(g_pass));
        LOG(INFO, "[prov] got SSID len=%d pass len=%d (values not logged)",
            (int)strlen(g_ssid), (int)strlen(g_pass));
    }
    else
    {
        LOG(WARNING, "[prov] portal timed out with no credentials -- node "
            "will not join WiFi until provisioned (power-cycle to retry "
            "the portal)");
    }

    // ---- Hand off the WiFi driver to Esp32WiFiManager ------------------
    // tzapu's autoConnect() leaves arduino-esp32's WiFi stack initialised
    // (and, if provisioned, connected). Esp32WiFiManager's own bring-up
    // unconditionally calls esp_wifi_init() -- which ESP_ERROR_CHECK-aborts
    // if the driver is already initialised -- and unconditionally calls
    // esp_netif_create_default_wifi_sta()/_ap(), which HARD-ASSERTS
    // (esp_netif_new: "duplicate key") if a netif with the same if_key
    // already exists. Confirmed on hardware: tzapu's autoConnect() already
    // created the default "WIFI_STA_DEF"/"WIFI_AP_DEF" netifs (via
    // arduino-esp32's WiFiGenericClass::mode() -> wifiLowLevelInit()), so
    // Esp32WiFiManager's own create call crashed with that exact assert
    // and boot-looped. WiFi.disconnect(true, false) deinits the esp_wifi
    // DRIVER (mode -> WIFI_OFF -> esp_wifi_deinit()), clearing the
    // esp_wifi_init() guard; `false` (eraseap) keeps the creds tzapu saved
    // to NVS untouched. It does NOT touch the netif objects -- arduino-esp32
    // never destroys those on deinit -- so they must be destroyed
    // explicitly here, before Esp32WiFiManager re-creates them.
    LOG(INFO, "[prov] handing off WiFi driver to Esp32WiFiManager");
    WiFi.disconnect(true, false);
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif)
    {
        esp_netif_destroy_default_wifi(sta_netif);
    }
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif)
    {
        esp_netif_destroy_default_wifi(ap_netif);
    }

    // Esp32WiFiManager's constructor copies station_ssid/station_password
    // into its own std::string members immediately (by value, at
    // construction time), so it must be built *after* g_ssid/g_pass are
    // populated above. A file-scope global (built during static init,
    // before setup() ever runs) would forever capture the empty initial
    // buffers and never see the provisioned credentials.
    wifi_mgr = new Esp32WiFiManager(
        g_ssid, g_pass, openmrn.stack(), cfg.seg().wifi());

    // Initialize the SPIFFS filesystem as our persistence layer.
    if (!SPIFFS.begin())
    {
        printf("SPIFFS failed to mount, attempting to format and remount\n");
        if (!SPIFFS.begin(true))
        {
            printf("SPIFFS mount failed even with format, giving up!\n");
            while (1)
            {
                // Unable to start SPIFFS successfully, give up and wait
                // for WDT to kick in.
            }
        }
    }

    // Open the NVS store BEFORE openmrn.begin() -- PointChannel::
    // apply_configuration(initial_load) restores from it, and begin() is
    // where that first fires.
    prefs.begin("railpoint", false);

    // Create the CDI.xml dynamically.
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file.
    s_cfg_fd = openmrn.stack()->create_config_file_if_needed(
        cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Start the OpenMRN stack; Esp32WiFiManager brings up WiFi + the
    // uplink connection from here.
    openmrn.begin();

    // Override the default SPACE_CONFIG memory space with one that fsyncs
    // after writes (see SyncedFileMemorySpace above), enabling live re-apply
    // of point config edits (event/throwMs/invert/description) via
    // ConfigUpdateService::instance()->trigger_update() in loop().
    if (s_cfg_fd >= 0)
    {
        static SyncedFileMemorySpace synced_cfg(
            s_cfg_fd, openlcb::CONFIG_FILE_SIZE);
        openmrn.stack()->memory_config_handler()->registry()->insert(
            openmrn.stack()->node(),
            openlcb::MemoryConfigDefs::SPACE_CONFIG, &synced_cfg);
    }

    openmrn.start_executor_thread();
}

// ---- Producer-state recovery sweep (bounded retry) ----------------------------
// After the node initialises, query each configured point's producer so the
// ProducerIdentified reply sets commanded_thrown_ deterministically (see
// PointChannel::query_state). SendQuery is async and a single WriteHelper can't
// be reused concurrently, so the points are serialised: fire point i+1 only
// after point i's query is enqueued. s_query_done (executor ctx) clears the
// busy flag; loop() advances the index. Ported VERBATIM from my LCC servo node.
//
// ProducerIdentified replies are best-effort -- on any given boot JMRI may not
// answer one point in time, leaving that point at its NVS-restored position
// until toggled. So the whole sweep runs QUERY_PASSES times, each starting
// QUERY_RETRY_GAP_MS after the previous one finishes; a reply missed on one
// pass is picked up on a later one. Re-querying a point that already
// recovered is idempotent, so a full re-sweep is safe.
// ponytail: bounded boot-time retry, NOT a periodic poll -- parks permanently
// after QUERY_PASSES so it never floods the bus. Linear index + busy flag +
// pass counter in loop(), one shared WriteHelper -- no StateFlow. Ceiling:
// strictly one query in flight, passes never overlap; upgrade to a StateFlow
// only if the sweep ever needs to overlap other outbound traffic.
static constexpr int      QUERY_PASSES       = 3;      // total recovery sweeps per boot
static constexpr uint32_t QUERY_RETRY_GAP_MS = 3000;   // gap after a sweep before the next
static openlcb::WriteHelper s_query_wh;
static BarrierNotifiable     s_query_bn;              // wraps s_query_done per query
static volatile bool         s_query_busy = false;    // query in flight (set/cleared cross-thread)
static int                   s_query_idx  = 0;        // next point; -1 between/after passes
static int                   s_query_pass = 0;        // sweeps completed; stops at QUERY_PASSES
static uint32_t              s_query_next_pass_ms = 0; // earliest start of the next sweep
static struct QueryDone : public Notifiable {
    void notify() override { s_query_busy = false; }
} s_query_done;

void loop()
{
    // ---- ArduinoOTA (WiFi flashing) ------------------------------------
    // Esp32WiFiManager already owns WiFi + its own mDNS (see boot-log
    // "[mDNS] Initializing mDNS system" from OpenMRNLite). ArduinoOTA's
    // begin() would normally call MDNS.begin() itself too -- a second
    // mDNS init that risks fighting Esp32WiFiManager's. Fix:
    // setMdnsEnabled(false) so ArduinoOTA.begin() never touches mDNS at
    // all (verified in arduino-esp32's ArduinoOTA.cpp: MDNS.begin()/.end()
    // are both gated behind `if (_mdnsEnabled)`). OTA is then reached by
    // IP:3232 instead of mDNS hostname discovery -- espota supports a raw
    // --host-ip target, which is how the controller invokes it (see
    // platformio.ini env:xiao_s3_wifi_ota).
    //
    // WiFi comes up asynchronously (Esp32WiFiManager connects after
    // openmrn.begin() above), so ArduinoOTA.begin() can't run in setup();
    // start it here, once, as soon as the connection is up.
    static bool ota_started = false;
    if (!ota_started && WiFi.status() == WL_CONNECTED)
    {
        ArduinoOTA.setHostname("lcc-railpoint");
        ArduinoOTA.setMdnsEnabled(false);
        // ponytail: no OTA password -- bench-only node on a trusted LAN.
        // Add ArduinoOTA.setPassword()/setPasswordHash() before this node
        // (or its network) is any less trusted.
        ArduinoOTA.onStart([]() {
            LOG(INFO, "[OTA] start");
        });
        ArduinoOTA.onEnd([]() {
            LOG(INFO, "[OTA] end, rebooting");
        });
        ArduinoOTA.onError([](ota_error_t error) {
            LOG(WARNING, "[OTA] error %u", (unsigned)error);
        });
        ArduinoOTA.begin();
        ota_started = true;
        LOG(INFO, "[OTA] ready at %s:3232", WiFi.localIP().toString().c_str());
    }
    if (ota_started)
    {
        ArduinoOTA.handle();
    }

    // Call the OpenMRN executor as often as possible from loop().
    openmrn.loop();

    // Producer-state recovery: one query per configured point, serialised,
    // repeated for QUERY_PASSES bounded passes (see block comment above).
    if (s_query_pass < QUERY_PASSES && !s_query_busy &&
        openmrn.stack()->node()->is_initialized())
    {
        if (s_query_idx < 0)   // between passes -- wait out the retry gap, then re-arm
        {
            if ((int32_t)(millis() - s_query_next_pass_ms) >= 0) s_query_idx = 0;
        }
        else if (s_query_idx >= (int)ARRAYSIZE(point_channels))
        {
            // one full sweep done -- schedule the next pass, or park after the last.
            s_query_idx = -1;
            s_query_next_pass_ms = millis() + QUERY_RETRY_GAP_MS;
            ++s_query_pass;
        }
        else
        {
            s_query_busy = true;   // cleared by s_query_done when the query lands
            point_channels[s_query_idx++].query_state(
                &s_query_wh, s_query_bn.reset(&s_query_done));
        }
    }

    // Point-channel tick: every ~30ms (same cadence as the servo node's servo
    // sweep). Drives each point's DRV8871 pulse and coasts it after throwMs.
    static uint32_t s_last_tick = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - s_last_tick) >= 30)
    {
        s_last_tick = now;
        for (auto &p : point_channels) p.update(now);
    }

    // Debounced config fsync: flush to flash once writes have been idle for
    // CFG_SYNC_DEBOUNCE_MS. Armed only by SyncedFileMemorySpace::write(), so
    // a read never triggers it. Runs here (loop thread), not on the executor.
    if (s_cfg_dirty &&
        (uint32_t)(millis() - s_cfg_dirty_ms) >= CFG_SYNC_DEBOUNCE_MS)
    {
        uint32_t armed = s_cfg_dirty_ms;
        if (s_cfg_fd >= 0) fsync(s_cfg_fd);
        // Clear only if no write landed during the fsync; else re-sync next pass.
        if (s_cfg_dirty_ms == armed)
        {
            s_cfg_dirty = false;
            // Config now committed to flash -> re-apply it LIVE so point
            // edits take effect immediately instead of only at next boot.
            ConfigUpdateService::instance()->trigger_update();
        }
    }
}
