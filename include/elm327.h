#pragma once

// ELM327 emulator. Phone OBD apps talk AT / OBD hex as if this were a dongle.
// At boot: hotspot up → WiFi TCP :35000. No hotspot → Bluetooth SPP (OBDII).
// Never both (ESP32 has one 2.4 GHz radio).

void initElm327Bluetooth();
void handleElm327();
void elm327StartTcp();
void elm327StopTcp();
void elm327OnWifiUp();
bool elm327ClientConnected();
bool elm327UsingBluetooth();
