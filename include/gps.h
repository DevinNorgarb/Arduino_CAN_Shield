#pragma once

// u-blox NEO GPS over hardware UART. Feeds NMEA into TinyGPS++ and updates
// the GPS fields on gObdState so the web dashboard can render position/track.
// No-ops while ENABLE_GPS is 0 (see config.h).
void initGps();
void handleGps();
