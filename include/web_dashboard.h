#pragma once

void initWebDashboard();
void handleWebDashboard();
void broadcastObdState();

// Pushes a single raw NMEA sentence to any connected dashboards, where it's
// shown in a collapsible debug console.
void broadcastNmea(const String &line);

// Pushes a single candump-format CAN frame line to any connected dashboards,
// where it's collected by the CAN recorder for download.
void broadcastCanFrame(const String &line);
