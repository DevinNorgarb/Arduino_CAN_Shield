#pragma once

void initWebDashboard();
void handleWebDashboard();
// Push live OBD JSON to dashboards. Throttled to ~5 Hz unless force=true
// (commands, scan milestones, heartbeats) so WiFi/WebSocket stay healthy.
void broadcastObdState(bool force = false);

// Blocks until the hotspot is associated, or timeoutMs elapses. Pumps the
// WiFi/dashboard loop so mDNS and the ELM TCP path come up.
bool waitForHotspot(uint32_t timeoutMs);

// Release the 2.4 GHz radio so Classic Bluetooth can start (no coexistence).
void stopWifiRadio();

// Pushes a single raw NMEA sentence to any connected dashboards, where it's
// shown in a collapsible debug console. Used when ENABLE_GPS is 1.
void broadcastNmea(const String &line);

// Pushes a single candump-format CAN frame line to any connected dashboards,
// where it's collected by the CAN recorder for download.
void broadcastCanFrame(const String &line);
