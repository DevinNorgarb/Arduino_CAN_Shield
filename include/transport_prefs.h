#pragma once

#include <Arduino.h>
#include <stdint.h>

// How the ESP32 picks WiFi vs Bluetooth at boot (one radio — never both).
enum class TransportMode : uint8_t {
  Auto = 0,       // Hotspot within wait window → WiFi; else Bluetooth
  Wifi = 1,       // Always WiFi (dashboard + TCP 35000), never Bluetooth
  Bluetooth = 2,  // Always Bluetooth SPP, WiFi off (no dashboard)
};

TransportMode loadTransportMode();
void saveTransportMode(TransportMode mode);
const char *transportModeName(TransportMode mode);

// Parse "transport auto|wifi|bluetooth" from serial or WebSocket admin.
bool applyTransportCommand(const String &cmd);
