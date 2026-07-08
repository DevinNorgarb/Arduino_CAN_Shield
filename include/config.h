#pragma once

// Phone hotspot credentials (Settings → Personal Hotspot / Mobile Hotspot on your phone)
#define WIFI_SSID "devin"
#define WIFI_PASSWORD "devin900"

// Browser shortcut when connected to the same hotspot: http://obd.local
#define MDNS_HOSTNAME "obd"

// MCP2515 SPI pins for NodeMCU-32S (D-labels on the silkscreen)
#define CAN_CS_PIN 5   // D5
#define CAN_INT_PIN 4  // D4
#define CAN_SPI_SCK 18   // D18
#define CAN_SPI_MISO 19  // D19
#define CAN_SPI_MOSI 23  // D23

// Most cheap MCP2515 OBD modules use an 8 MHz crystal. If sends fail with ignition ON, try MCP_16MHZ.
#define CAN_CLOCK MCP_8MHZ

// u-blox NEO-6M / NEO-M8 (GY-GPS6MV2) on hardware UART2. Free of the CAN SPI pins.
// Wiring: GPS VCC->3V3, GND->GND, GPS TX->GPIO16 (RX2), GPS RX->GPIO17 (TX2).
#define GPS_UART_NUM 2
#define GPS_RX_PIN 16  // ESP32 receives on this pin; wire to GPS TX
#define GPS_TX_PIN 17  // ESP32 transmits on this pin; wire to GPS RX
#define GPS_BAUD 9600  // NMEA default for these u-blox modules

// OBD-II uses ISO 15765-4 on CAN at 500 kbps (11-bit IDs on most vehicles)
#define OBD_REQUEST_ID 0x7DF
#define OBD_RESPONSE_ID 0x7E8

// Set true for vehicles that require 29-bit CAN IDs (e.g. some Honda)
#define OBD_USE_EXTENDED_ID false

// VW/VAG ABS-ESP controller UDS addressing for chassis "C" codes (traction
// control / ESC), which generic OBD mode 03/04 cannot read or clear. These are
// manufacturer-specific; verify for your vehicle. Default is the common VAG
// ABS module address (physical request 0x713, response 0x77D).
#define ABS_UDS_REQUEST_ID 0x713
#define ABS_UDS_RESPONSE_ID 0x77D

#if OBD_USE_EXTENDED_ID
#define OBD_REQUEST_ID_EXT 0x18DB33F1UL
#define OBD_RESPONSE_ID_EXT 0x18DAF110UL
#endif
