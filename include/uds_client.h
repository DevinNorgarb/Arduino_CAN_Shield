#pragma once

#include <Arduino.h>

// Minimal UDS (ISO 14229 over ISO-TP) client for a single ECU, layered on the
// shared CAN I/O wrappers. Used to reach non-OBD modules that generic OBD mode
// 03/04 can't touch - e.g. the VW ABS/ESP controller for chassis "C" codes.
//
// Sends a short (single-frame) UDS request to `reqId` and collects the full
// ISO-TP response payload from `respId`, following flow control for multi-frame
// replies and transparently waiting out "response pending" (NRC 0x78).
//
// Returns the number of response payload bytes (>= 0), or -1 on timeout.
int udsRequest(unsigned long reqId, unsigned long respId, const uint8_t *req,
               uint8_t reqLen, uint8_t *resp, size_t respSize, uint32_t timeoutMs);
