/// Tiny stateless DRV8871 H-bridge driver (IN1/IN2 pins). Per-point pulse
/// timing (throwMs, coast-after) lives in PointChannel, not here.
///
/// A pin number can name EITHER a native ESP32 GPIO or an MCP23017 output:
/// a value >= MCP_BASE means MCP pin (p - MCP_BASE); otherwise native GPIO.
/// This lets more points fit on one board (native pins exhausted). MCP pins
/// go through g_mcp (I2C @0x20) -- so drv8871 calls, like everything else in
/// this driver, must only run on the loop thread (see PointChannel::update).
#ifndef _LCC_RAILPOINT_DRV8871_H_
#define _LCC_RAILPOINT_DRV8871_H_

#include <Arduino.h>
#include "mcp23017.h"

/// Pins >= MCP_BASE live on the MCP23017. MCP(n) declares MCP output n (0-15).
#define MCP_BASE 100
#define MCP(n)   (MCP_BASE + (n))

extern Mcp23017 g_mcp; // defined in main.cpp; already begin()'d before drive

/// Configure one IN pin as an output, driven low. Native pins need pinMode;
/// MCP pins are already outputs (g_mcp.begin() set IODIR + latched low).
static inline void railpoint_pin_init(uint8_t p)
{
    if (p >= MCP_BASE)
        g_mcp.set_pin(p - MCP_BASE, false);
    else
    {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }
}

/// Set one IN pin's level, native or MCP-backed.
static inline void railpoint_pin_write(uint8_t p, bool level)
{
    if (p >= MCP_BASE)
        g_mcp.set_pin(p - MCP_BASE, level);
    else
        digitalWrite(p, level ? HIGH : LOW);
}

/// Configure both DRV8871 inputs as outputs and coast (both LOW = outputs
/// off/high-Z).
static inline void drv8871_init(uint8_t in1, uint8_t in2)
{
    railpoint_pin_init(in1);
    railpoint_pin_init(in2);
}

/// Drive the bridge: in1high true -> IN1 high/IN2 low, false -> IN1 low/IN2
/// high. Never both high.
static inline void drv8871_drive(uint8_t in1, uint8_t in2, bool in1high)
{
    railpoint_pin_write(in1, in1high);
    railpoint_pin_write(in2, !in1high);
}

/// Both inputs LOW: outputs off/high-Z (MP1's end-stop already cut motor
/// current; this just stops driving it).
static inline void drv8871_coast(uint8_t in1, uint8_t in2)
{
    railpoint_pin_write(in1, false);
    railpoint_pin_write(in2, false);
}

#endif // _LCC_RAILPOINT_DRV8871_H_
