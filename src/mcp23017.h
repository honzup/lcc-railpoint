/// Minimal MCP23017 16-ch output driver — just enough to switch frog relays.
#pragma once
#include <cstdint>
class Mcp23017 {
public:
    void begin(uint8_t addr = 0x20);      // all 16 pins -> outputs, latches low
    void set_pin(uint8_t pin, bool on);   // pin 0..15 (0-7=PortA, 8-15=PortB)
private:
    void write8(uint8_t reg, uint8_t val);
    uint8_t addr_ = 0x20;
    uint8_t olat_[2] = {0, 0};            // shadow of OLATA/OLATB
};
