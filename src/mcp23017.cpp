#include "mcp23017.h"
#include <Arduino.h>
#include <Wire.h>
// MCP23017 registers (IOCON.BANK=0 default)
static constexpr uint8_t IODIRA = 0x00, IODIRB = 0x01, OLATA = 0x14, OLATB = 0x15;
void Mcp23017::write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void Mcp23017::begin(uint8_t addr) {
    addr_ = addr;
    write8(IODIRA, 0x00);   // all outputs
    write8(IODIRB, 0x00);
    olat_[0] = olat_[1] = 0;
    write8(OLATA, 0);
    write8(OLATB, 0);
}
void Mcp23017::set_pin(uint8_t pin, bool on) {
    if (pin > 15) return;
    uint8_t port = pin >> 3, bit = pin & 7;
    if (on) olat_[port] |= (1 << bit);
    else    olat_[port] &= ~(1 << bit);
    write8(port ? OLATB : OLATA, olat_[port]);
}
