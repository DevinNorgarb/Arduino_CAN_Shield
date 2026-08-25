#include "transport_prefs.h"

#include <Preferences.h>
#include <ESP.h>

namespace {

constexpr char kNs[] = "obd";
constexpr char kKey[] = "transport";

Preferences prefs;

}  // namespace

TransportMode loadTransportMode() {
  if (!prefs.begin(kNs, true)) {
    return TransportMode::Auto;
  }
  const uint8_t raw = prefs.getUChar(kKey, static_cast<uint8_t>(TransportMode::Auto));
  prefs.end();
  if (raw > static_cast<uint8_t>(TransportMode::Bluetooth)) {
    return TransportMode::Auto;
  }
  return static_cast<TransportMode>(raw);
}

void saveTransportMode(TransportMode mode) {
  if (!prefs.begin(kNs, false)) {
    return;
  }
  prefs.putUChar(kKey, static_cast<uint8_t>(mode));
  prefs.end();
}

const char *transportModeName(TransportMode mode) {
  switch (mode) {
    case TransportMode::Wifi:
      return "wifi";
    case TransportMode::Bluetooth:
      return "bluetooth";
    case TransportMode::Auto:
    default:
      return "auto";
  }
}

bool applyTransportCommand(const String &cmd) {
  TransportMode mode = TransportMode::Auto;
  if (cmd == "transport_wifi") {
    mode = TransportMode::Wifi;
  } else if (cmd == "transport_bluetooth") {
    mode = TransportMode::Bluetooth;
  } else if (cmd == "transport_auto") {
    mode = TransportMode::Auto;
  } else {
    return false;
  }
  if (loadTransportMode() == mode) {
    Serial.printf("Transport already %s\n", transportModeName(mode));
    return true;
  }
  saveTransportMode(mode);
  Serial.printf("Transport saved as %s — rebooting\n", transportModeName(mode));
  delay(300);
  ESP.restart();
  return true;
}
