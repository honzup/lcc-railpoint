#ifndef _LCC_RAILPOINT_CONFIG_H_
#define _LCC_RAILPOINT_CONFIG_H_

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

#include "freertos_drivers/esp32/Esp32WiFiConfiguration.hxx"

namespace openlcb
{

/// Node identification, mirrors ESP32IOBoard's SNIP_STATIC_DATA layout.
extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,
    "OpenMRN",
    "LCC RailPoint (Fleet #4)",
    ARDUINO_VARIANT,
    "1.00"};

/// Modify this value every time the EEPROM needs to be cleared on the node
/// after an update.
/// 0x1900 = Task 2: added PointConfig (2-entry RepeatedGroup) -- layout
///          change (new entries appended after the wifi group) -> bump.
/// 0x1A00 = added 3rd point (FY3) -- grows the turnouts RepeatedGroup -> reformat.
/// 0x1B00 = pre-sized to 8 points (3 native + 5 MCP23017). Planned final size,
///          so wiring points 4-8 later needs NO version bump and NO reformat --
///          the CDI already holds all 8, they just start blank/at defaults.
/// 0x1E00 = 4-point test: FY1/FY2/FY3 native + FY4 on MCP23017.
/// 0x1F00 = 5 points: FY1-3 native + FY4/FY5 on MCP23017 (GPA0/1, GPA2/3).
static constexpr uint16_t CANONICAL_VERSION = 0x1F00;

constexpr uint8_t NUM_POINTS = 5;

/// One turnout/point: the OpenLCB event pair that commands it, plus the
/// timing/orientation config the (not-yet-built) actuator will use. Task 2
/// scope: consuming event_thrown/event_closed only sets commanded state and
/// logs it -- no GPIO/actuator driven yet (that's Task 3).
CDI_GROUP(PointConfig);
CDI_GROUP_ENTRY(event_closed, EventConfigEntry, Name("Event Closed"),
    Description("Command this point CLOSED."));
CDI_GROUP_ENTRY(event_thrown, EventConfigEntry, Name("Event Thrown"),
    Description("Command this point THROWN."));
CDI_GROUP_ENTRY(throwMs, Uint16ConfigEntry, Default(2000), Min(200), Max(10000),
    Name("Throw time (ms)"),
    Description("How long the actuator takes to move end-to-end."));
CDI_GROUP_ENTRY(invert, Uint8ConfigEntry, Default(0), Name("Invert"),
    Description("0 = normal, 1 = swap which physical direction is Closed/Thrown."),
    MapValues(
        "<relation><property>0</property><value>Normal</value></relation>"
        "<relation><property>1</property><value>Invert</value></relation>"));
CDI_GROUP_ENTRY(description, StringConfigEntry<16>, Name("Description"),
    Description("User name."));
CDI_GROUP_END();

using AllPoints = RepeatedGroup<PointConfig, NUM_POINTS>;

/// Main segment, laid out at origin 128 to leave space for ACDI user data.
/// internal_config/wifi stay first and in order so the Esp32WiFiManager
/// uplink's config offsets are unaffected -- turnouts is appended after.
CDI_GROUP(WifiNodeSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
CDI_GROUP_ENTRY(internal_config, InternalConfigData);
CDI_GROUP_ENTRY(wifi, WiFiConfiguration, Name("WiFi Configuration"));
CDI_GROUP_ENTRY(turnouts, AllPoints, Name("Points"), RepName("Point"));
CDI_GROUP_END();

/// The main structure of the CDI, referenced from main.cpp as ConfigDef.
CDI_GROUP(ConfigDef, MainCdi());
CDI_GROUP_ENTRY(ident, Identification);
CDI_GROUP_ENTRY(acdi, Acdi);
CDI_GROUP_ENTRY(userinfo, UserInfoSegment, Name("User Info"));
CDI_GROUP_ENTRY(seg, WifiNodeSegment, Name("Settings"));
CDI_GROUP_END();

} // namespace openlcb

#endif // _LCC_RAILPOINT_CONFIG_H_
