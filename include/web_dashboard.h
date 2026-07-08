#pragma once

void initWebDashboard();
void handleWebDashboard();
void broadcastObdState();

// Pushes a single raw NMEA sentence to any connected dashboards, where it's
// shown in a collapsible debug console.
void broadcastNmea(const String &line);
