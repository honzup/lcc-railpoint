/// LCC RailPoint (Fleet #4) -- WiFi-transport OpenLCB node for the
/// ESP32-S3 (XIAO S3). Connects as a GridConnect-TCP uplink client to the
/// JMRI hub. No CAN/TWAI. Task 1 scope: WiFi provisioning + uplink join
/// only. Task 2 scope: PointChannel consumers (2 points) -- thrown/closed
/// events set commanded state and are LOGGED; no actuator hardware yet
/// (that's Task 3).
///
/// WiFi credentials are NOT hardcoded: a tzapu WiFiManager captive portal
/// ("lcc-railpoint-Setup") provisions them on first boot / when unset, and
/// OpenMRNLite's Esp32WiFiManager owns the connection from then on. See the
/// "[prov] handing off" block in setup() for how the two libraries hand off
/// the WiFi driver without fighting over it -- that handoff is the risk
/// this task exists to de-risk.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiManager.h> // tzapu/WiFiManager -- captive-portal provisioning
#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <unistd.h> // fsync -- config persistence to flash

#include <OpenMRNLite.h>
#include "openlcb/ConfiguredConsumer.hxx"

#define USE_WIFI

#include "config.h"

/// Node id for this node, unique on the network (Fleet #4).
static constexpr uint64_t NODE_ID = UINT64_C(0x0304001E4804);

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

// ============ DURABLE config persistence: fsync-after-write (debounced) =======
// Ported VERBATIM (pattern) from fleet node #3 (lcc-servo-node/src/main.cpp) --
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
/// set a commanded state. Adapted from fleet node #3's TurnoutChannel, minus
/// the PCA9685/MCP23017 servo+frog drive -- Task 2 scope is consume + LOG
/// only, no actuator hardware yet (Task 3).
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
        fd_ = fd; // retained for parity with node #3 (future save/calibration API)
        throw_ms_ = cfg_.throwMs().read(fd);
        invert_ = cfg_.invert().read(fd) != 0;
        snprintf(name_, sizeof(name_), "%s", cfg_.description().read(fd).c_str());

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

    /// Loop-thread tick (called every ~30ms, same cadence as node #3's servo
    /// sweep). Task 2: no actuator yet -- just log the commanded intent.
    void update(uint32_t /*now*/)
    {
        if (restart_)
        {
            restart_ = false;
            LOG(INFO, "[FY%d] -> %s", idx_, commanded_thrown_ ? "THROWN" : "CLOSED");
        }
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
    volatile bool restart_          = false; // intent changed, log it on next tick
    int      fd_ = -1;                       // config fd, retained (parity w/ node #3)
};

#define PCH(i) PointChannel(openmrn.stack()->node(), cfg.seg().turnouts().entry<i>(), i)
PointChannel point_channels[] = { PCH(0), PCH(1) };
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
            fd, "LCC RailPoint (Fleet #4) -- WiFi uplink to JMRI hub");

        // Pre-point the uplink at the JMRI hub so it connects with zero
        // JMRI-side CDI editing: Manual Only, 192.168.1.46:12021.
        cfg.seg().wifi().uplink().search_mode().write(
            fd, 3 /* SocketClientParams::SearchMode::MANUAL_ONLY */);
        cfg.seg().wifi().uplink().manual_address().ip_address().write(
            fd, "192.168.1.46");
        cfg.seg().wifi().uplink().manual_address().port().write(fd, 12021);
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
/// from fleet node #3.
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

    // Point-channel tick: every ~30ms (same cadence as node #3's servo
    // sweep). Task 2: logs a commanded thrown/closed intent; no actuator yet.
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
