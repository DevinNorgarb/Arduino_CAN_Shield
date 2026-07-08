#pragma once

// u-blox NEO GPS over hardware UART. Feeds NMEA into TinyGPS++ and updates
// the GPS fields on gObdState so the web dashboard can render position/track.
void initGps();
void handleGps();
