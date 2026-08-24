#pragma once

#include <Arduino.h>

// Read-only UDS pass: probe each VAG module, collect identity + DTCs, then dump
// identification DIDs from the engine ECU (0x7E0). Never writes, never enters
// a programming session, never requests security access.
void performEcuScan();
void performUdsRead(uint8_t moduleId);
void performUdsClear(uint8_t moduleId);
