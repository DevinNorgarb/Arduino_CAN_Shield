#pragma once

#include <Arduino.h>

// Thin wrappers around the MCP2515 (defined in main.cpp). All CAN traffic flows
// through these so the recorder sees every frame and other modules (e.g. the
// UDS client) can share one choke point instead of touching the driver directly.

bool canAvailable();
uint8_t canReadFrame(unsigned long *id, uint8_t *len, uint8_t *data);
uint8_t canSendFrame(unsigned long id, uint8_t ext, uint8_t len, const uint8_t *data);
