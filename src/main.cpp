/// LCC RailPoint (Fleet #4) -- WiFi-transport OpenLCB node for the
/// ESP32-S3 (XIAO S3). Connects as a GridConnect-TCP uplink client to the
/// JMRI hub. No CAN/TWAI. Task 1 scope: WiFi provisioning + uplink join
/// only -- no turnout/producer/consumer logic yet.
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

#include <OpenMRNLite.h>

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
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Start the OpenMRN stack; Esp32WiFiManager brings up WiFi + the
    // uplink connection from here.
    openmrn.begin();
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
}
