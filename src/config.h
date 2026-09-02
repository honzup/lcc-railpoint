#ifndef _LCC_RAILPOINT_CONFIG_H_
#define _LCC_RAILPOINT_CONFIG_H_

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
static constexpr uint16_t CANONICAL_VERSION = 0x1000;

/// Main segment, laid out at origin 128 to leave space for ACDI user data.
CDI_GROUP(WifiNodeSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
CDI_GROUP_ENTRY(internal_config, InternalConfigData);
CDI_GROUP_ENTRY(wifi, WiFiConfiguration, Name("WiFi Configuration"));
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
