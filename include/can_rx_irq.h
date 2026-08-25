#pragma once

#include <Arduino.h>

// MCP2515 INT pin (active low). While recording, an ISR sets a pending flag so
// the main loop drains RX before the chip's two-frame buffer overflows.

void canRxIrqInitPin();
void canRxIrqEnable();
void canRxIrqDisable();
bool canRxIrqPending();

// Call after draining MCP2515 RX; clears the ISR flag unless INT is still low.
void canRxIrqAckDrain();
