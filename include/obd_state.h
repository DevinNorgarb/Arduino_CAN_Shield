#pragma once

#include <Arduino.h>

#include "uds_modules.h"

constexpr uint8_t kMaxDtcs = 16;
constexpr uint8_t kDtcTextLen = 6;
constexpr uint8_t kUdsDtcTextLen = 10;  // UDS "C1234-08" + null
constexpr uint8_t kMaxSupportedPids = 224;
constexpr uint8_t kMaxDidDump = 64;
constexpr uint8_t kDidValueLen = 40;
constexpr uint8_t kIdentLen = 24;

enum DtcStatus : uint8_t {
  DTC_IDLE = 0,
  DTC_READING = 1,
  DTC_DONE = 2,
  DTC_CLEARED = 3,
  DTC_ERROR = 4,
};

enum ScanStatus : uint8_t {
  SCAN_IDLE = 0,
  SCAN_RUNNING = 1,
  SCAN_DONE = 2,
  SCAN_ERROR = 3,
};

// Read/clear state for one UDS module (ABS, airbag, engine, ...).
struct UdsModuleState {
  char dtcCodes[kMaxDtcs][kUdsDtcTextLen] = {};
  uint8_t dtcCount = 0;
  uint8_t status = DTC_IDLE;
  volatile bool cmdRead = false;
  volatile bool cmdClear = false;
  bool probed = false;
  bool present = false;
  char partNumber[kIdentLen] = {};
  char swVersion[kIdentLen] = {};
  char hwNumber[kIdentLen] = {};
  char sysName[kIdentLen] = {};
};

struct DidDumpEntry {
  uint16_t id = 0;
  char value[kDidValueLen] = {};
};

struct ObdState {
  // Live values
  float rpm = 0;
  uint8_t speedKmh = 0;
  int16_t coolantC = 0;
  float engineLoadPct = 0;
  float batteryV = 0;
  int16_t intakeAirC = 0;
  float throttlePct = 0;
  float fuelPct = 0;
  float timingAdv = 0;
  float mafGs = 0;
  uint16_t mapKpa = 0;
  uint16_t baroKpa = 101;
  int16_t boostKpa = 0;

  // Validity flags
  bool rpmValid = false;
  bool speedValid = false;
  bool coolantValid = false;
  bool engineLoadValid = false;
  bool batteryValid = false;
  bool intakeAirValid = false;
  bool throttleValid = false;
  bool fuelValid = false;
  bool timingValid = false;
  bool mafValid = false;
  bool mapValid = false;
  bool boostValid = false;

  // Session peaks
  float maxRpm = 0;
  uint8_t maxSpeedKmh = 0;
  int16_t maxCoolantC = 0;
  int16_t maxBoostKpa = 0;

  // GPS (u-blox NEO via UART/NMEA). Unused while ENABLE_GPS is 0.
  double latitude = 0;
  double longitude = 0;
  float gpsSpeedKmh = 0;
  float altitudeM = 0;
  float headingDeg = 0;
  uint8_t satellites = 0;
  bool gpsValid = false;  // true once a position fix is available
  uint32_t gpsLastFixMs = 0;

  // CAN status
  bool canReady = false;
  bool busActive = false;
  uint8_t lastCanError = 0;
  uint32_t sendFailures = 0;
  uint32_t timeouts = 0;

  uint32_t lastUpdateMs = 0;
  uint32_t lastStatusBroadcastMs = 0;

  // Diagnostic trouble codes (emissions / powertrain, via OBD mode 03/04)
  char dtcCodes[kMaxDtcs][kDtcTextLen] = {};
  uint8_t dtcCount = 0;
  uint8_t dtcStatus = DTC_IDLE;

  // Non-OBD module codes (ABS, airbag, ...) read/cleared over UDS
  UdsModuleState udsModules[UDS_MODULE_COUNT];

  // Supported-PID scan results (Mode 01 PID 0x00/0x20/...)
  uint8_t supportedPids[kMaxSupportedPids] = {};
  uint8_t supportedCount = 0;
  uint8_t pidScanStatus = SCAN_IDLE;

  // Read-only UDS ECU scan + engine DID dump
  uint8_t ecuScanStatus = SCAN_IDLE;
  uint8_t ecuScanFound = 0;
  char ecuScanLabel[32] = {};
  char vin[18] = {};
  DidDumpEntry engineDids[kMaxDidDump] = {};
  uint8_t engineDidCount = 0;

  // Commands set by the web layer, consumed by the CAN loop
  volatile bool cmdResetPeaks = false;
  volatile bool cmdReadDtc = false;
  volatile bool cmdClearDtc = false;
  volatile bool cmdScanPids = false;
  volatile bool cmdScanEcus = false;
};

extern ObdState gObdState;

void updateObdState(uint8_t pid, const uint8_t *data, uint8_t length);
void recordCanSendError(uint8_t errorCode);
void recordCanTimeout();
void resetSessionPeaks();
const char *canErrorName(uint8_t errorCode);
const char *canStatusMessage();
const char *dtcStatusName();
const char *dtcStatusText(uint8_t status);
const char *scanStatusName();
const char *ecuScanStatusName();
void decodeDtcPair(uint8_t a, uint8_t b, char *out);
void decodeUdsDtc(uint8_t a, uint8_t b, uint8_t c, char *out);
void applyUdsDtcResponse(UdsModuleState &mod, const uint8_t *resp, int n);
void sanitizeIdent(const uint8_t *data, uint8_t n, char *out, size_t outSize);
