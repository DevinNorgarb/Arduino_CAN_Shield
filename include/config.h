#pragma once

// Phone hotspot credentials (Settings → Personal Hotspot / Mobile Hotspot on your phone)
#define WIFI_SSID "devin"
#define WIFI_PASSWORD "devin900"

// Browser shortcut when connected to the same hotspot: http://obd.local
#define MDNS_HOSTNAME "obd"

// MCP2515 SPI — same GPIO numbers on NodeMCU-32S and ESP32-DOIT-DevKit-V1.
// NodeMCU silkscreen: D5 / D4 / D18 / D19 / D23, VCC → 5V/VIN
// DOIT silkscreen:    5  / 4  / 18  / 19  / 23,  VCC → 5V (not 3V3)
#define CAN_CS_PIN 5     // NodeMCU D5  | DOIT 5
#define CAN_INT_PIN 4    // NodeMCU D4  | DOIT 4
#define CAN_SPI_SCK 18   // NodeMCU D18 | DOIT 18
#define CAN_SPI_MISO 19  // NodeMCU D19 | DOIT 19
#define CAN_SPI_MOSI 23  // NodeMCU D23 | DOIT 23

// Most cheap MCP2515 OBD modules use an 8 MHz crystal. If sends fail with ignition ON, try MCP_16MHZ.
#define CAN_CLOCK MCP_8MHZ

// u-blox NEO-6M / NEO-M8 (GY-GPS6MV2) on hardware UART2. Free of the CAN SPI pins.
// Wiring: GPS VCC->3V3, GND->GND, GPS TX->GPIO16 (RX2), GPS RX->GPIO17 (TX2).
// Off for now — set to 1 to init UART and feed TinyGPS++ / the dashboard.
#define ENABLE_GPS 0
#define GPS_UART_NUM 2
#define GPS_RX_PIN 16  // ESP32 receives on this pin; wire to GPS TX
#define GPS_TX_PIN 17  // ESP32 transmits on this pin; wire to GPS RX
#define GPS_BAUD 9600  // NMEA default for these u-blox modules

// ELM327: WiFi TCP 35000 when the phone hotspot is up. If the hotspot is
// missing at boot, WiFi is powered off and Classic Bluetooth SPP takes the
// radio instead (OBDII / PIN 1234). Never both — one 2.4 GHz radio.
#define ELM327_TCP_PORT 35000
#define ELM327_ID_STRING "ELM327 v1.5"
#define ELM327_HOTSPOT_WAIT_MS 15000
#define ELM327_BT_NAME "OBDII"
#define ELM327_BT_PIN "1234"

// OBD-II uses ISO 15765-4 on CAN at 500 kbps (11-bit IDs on most vehicles)
#define OBD_REQUEST_ID 0x7DF
#define OBD_RESPONSE_ID 0x7E8

// Set true for vehicles that require 29-bit CAN IDs (e.g. some Honda)
#define OBD_USE_EXTENDED_ID false

// Live CAN uplink. The ESP32 is a WebSocket *client* that pushes every captured
// frame (OBD polls and passive bus traffic) to a remote processor through the
// phone hotspot's cellular path. Empty host disables the uplink.
//
// Production: ws://monitor.f1y.ing:8765  (monitor VPS, not NPM)
#define CAN_STREAM_HOST "monitor.f1y.ing"
#define CAN_STREAM_PORT 8765
#define CAN_STREAM_PATH "/"
#define CAN_STREAM_SSL 0

// VW/VAG module UDS addressing for codes that generic OBD mode 03/04 cannot
// reach. These are manufacturer-specific; verify for your vehicle. Defaults are
// the common VAG addresses (physical request / response pairs, offset +0x6A).
//   ABS/ESP (address 03): chassis "C" codes - traction control / ESC
//   Airbag/SRS (address 15): "B" codes - supplemental restraints
#define ABS_UDS_REQUEST_ID 0x713
#define ABS_UDS_RESPONSE_ID 0x77D
#define AIRBAG_UDS_REQUEST_ID 0x715
#define AIRBAG_UDS_RESPONSE_ID 0x77F

#if OBD_USE_EXTENDED_ID
#define OBD_REQUEST_ID_EXT 0x18DB33F1UL
#define OBD_RESPONSE_ID_EXT 0x18DAF110UL
#endif
