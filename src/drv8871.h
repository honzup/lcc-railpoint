/// Tiny stateless DRV8871 H-bridge driver (IN1/IN2 pins). No OpenMRN/CAN
/// coupling -- just pinMode/digitalWrite via Arduino.h. Per-point pulse
/// timing (throwMs, coast-after) lives in PointChannel, not here.
#ifndef _LCC_RAILPOINT_DRV8871_H_
#define _LCC_RAILPOINT_DRV8871_H_

#include <Arduino.h>

/// Configure both DRV8871 inputs as outputs and coast (both LOW = outputs
/// off/high-Z).
static inline void drv8871_init(uint8_t in1, uint8_t in2)
{
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

/// Drive the bridge: in1high true -> IN1 high/IN2 low, false -> IN1 low/IN2
/// high. Never both high.
static inline void drv8871_drive(uint8_t in1, uint8_t in2, bool in1high)
{
    digitalWrite(in1, in1high ? HIGH : LOW);
    digitalWrite(in2, in1high ? LOW : HIGH);
}

/// Both inputs LOW: outputs off/high-Z (MP1's end-stop already cut motor
/// current; this just stops driving it).
static inline void drv8871_coast(uint8_t in1, uint8_t in2)
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

#endif // _LCC_RAILPOINT_DRV8871_H_
