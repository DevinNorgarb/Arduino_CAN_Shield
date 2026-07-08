#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

#include "config.h"
#include "obd_state.h"
#include "web_dashboard.h"
#include "gps.h"

namespace {

TinyGPSPlus gps;
HardwareSerial gpsSerial(GPS_UART_NUM);

// Push a fresh position to the dashboard at most this often to avoid flooding
// the WebSocket (the module emits several NMEA sentences per second).
constexpr uint32_t kGpsBroadcastIntervalMs = 1000;
uint32_t lastGpsBroadcastMs = 0;

// A fix is considered stale (car in a tunnel, antenna unplugged) after this.
constexpr uint32_t kGpsFixTimeoutMs = 5000;

// Buffer for reassembling one NMEA sentence (longest standard sentence is well
// under 82 chars) before pushing it to the dashboard console.
char nmeaLine[128];
size_t nmeaLen = 0;

}  // namespace

void initGps() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("GPS UART%d up (RX=GPIO%d TX=GPIO%d @ %d baud)\n", GPS_UART_NUM,
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void handleGps() {
  bool gotNewFix = false;

  while (gpsSerial.available() > 0) {
    const char c = gpsSerial.read();
    // Echo raw NMEA to the serial console so it's visible even with no OBD/CAN.
    Serial.write(c);

    // Reassemble complete sentences and forward them to the dashboard console.
    if (c == '\r' || c == '\n') {
      if (nmeaLen > 0) {
        nmeaLine[nmeaLen] = '\0';
        broadcastNmea(nmeaLine);
        nmeaLen = 0;
      }
    } else if (nmeaLen < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaLen++] = c;
    } else {
      // Overlong/garbled line: drop it and resync on the next terminator.
      nmeaLen = 0;
    }

    if (!gps.encode(c)) {
      continue;
    }

    if (gps.satellites.isValid()) {
      gObdState.satellites = gps.satellites.value();
    }

    if (gps.location.isValid() && gps.location.isUpdated()) {
      gObdState.latitude = gps.location.lat();
      gObdState.longitude = gps.location.lng();
      gObdState.gpsValid = true;
      gObdState.gpsLastFixMs = millis();
      gotNewFix = true;

      if (gps.speed.isValid()) {
        gObdState.gpsSpeedKmh = gps.speed.kmph();
      }
      if (gps.altitude.isValid()) {
        gObdState.altitudeM = gps.altitude.meters();
      }
      if (gps.course.isValid()) {
        gObdState.headingDeg = gps.course.deg();
      }
    }
  }

  // Mark the fix stale if we stop hearing valid positions.
  if (gObdState.gpsValid &&
      (millis() - gObdState.gpsLastFixMs) > kGpsFixTimeoutMs) {
    gObdState.gpsValid = false;
  }

  const uint32_t now = millis();
  if (gotNewFix && (now - lastGpsBroadcastMs) >= kGpsBroadcastIntervalMs) {
    lastGpsBroadcastMs = now;
    broadcastObdState();
  }
}
