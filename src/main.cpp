#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#include "config.h"
#include "obd_state.h"
#include "web_dashboard.h"
#include "gps.h"
#include "can_recorder.h"

MCP_CAN canBus(CAN_CS_PIN);

// Physical request ID for the engine ECU (used for ISO-TP flow control).
constexpr unsigned long kEcuPhysicalRequestId = 0x7E0;

// All CAN traffic flows through these two wrappers so the recorder sees every
// frame (both our requests and the ECU's responses) from a single choke point.
uint8_t canReadFrame(unsigned long *id, uint8_t *len, uint8_t *data) {
  const uint8_t status = canBus.readMsgBuf(id, len, data);
  if (status == CAN_OK) {
    canRecordFrame(false, *id, *len, data);
  }
  return status;
}

uint8_t canSendFrame(unsigned long id, uint8_t ext, uint8_t len, const uint8_t *data) {
  const uint8_t status = canBus.sendMsgBuf(id, ext, len, const_cast<uint8_t *>(data));
  canRecordFrame(true, id, len, data);
  return status;
}

struct ObdPid {
  uint8_t pid;
  const char *name;
  uint32_t intervalMs;
};

// Mode 01 PIDs to poll. Slow-changing values poll less often.
const ObdPid kPidsToPoll[] = {
    {0x0C, "rpm", 400},
    {0x0D, "speed_kmh", 400},
    {0x11, "throttle_pct", 500},
    {0x0B, "map_kpa", 700},
    {0x05, "coolant_c", 2000},
    {0x04, "engine_load_pct", 1000},
    {0x0F, "intake_air_c", 2000},
    {0x10, "maf_gs", 1000},
    {0x0E, "timing_adv", 2000},
    {0x2F, "fuel_pct", 5000},
    {0x42, "battery_v", 2000},
    {0x33, "baro_kpa", 10000},
};

constexpr size_t kPidCount = sizeof(kPidsToPoll) / sizeof(kPidsToPoll[0]);
constexpr uint8_t kMaxConsecutiveTimeouts = 5;
// How long a PID stays in "slow retry" mode after repeated timeouts. This is a
// backoff, NOT a permanent disable: a PID that looks unsupported at boot (bus
// asleep / ignition warming up) is retried again later instead of latched off.
constexpr uint32_t kUnsupportedRetryMs = 15000;
constexpr uint32_t kPollResponseTimeoutMs = 220;

uint32_t lastPollMs[kPidCount] = {};
uint8_t consecutiveTimeouts[kPidCount] = {};
uint32_t pidRetryAtMs[kPidCount] = {};

uint8_t lastLoggedCanError = 0xFF;
uint32_t lastErrorLogMs = 0;

bool isObdResponseId(unsigned long rxId) {
#if OBD_USE_EXTENDED_ID
  return rxId == OBD_RESPONSE_ID_EXT;
#else
  return rxId >= 0x7E8 && rxId <= 0x7EF;
#endif
}

bool detectCanBusActivity(uint32_t listenMs) {
  const uint32_t deadline = millis() + listenMs;

  while ((int32_t)(deadline - millis()) > 0) {
    if (canBus.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId = 0;
      uint8_t rxLen = 0;
      uint8_t rxData[8] = {};

      if (canReadFrame(&rxId, &rxLen, rxData) == CAN_OK) {
        Serial.printf("CAN bus active - saw frame id=0x%lX len=%u\n", rxId, rxLen);
        return true;
      }
    }
  }

  return false;
}

bool initCan() {
  SPI.begin(CAN_SPI_SCK, CAN_SPI_MISO, CAN_SPI_MOSI, CAN_CS_PIN);
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);

  if (canBus.begin(MCP_ANY, CAN_500KBPS, CAN_CLOCK) != CAN_OK) {
    gObdState.canReady = false;
    recordCanSendError(1);
    return false;
  }

  canBus.setMode(MCP_NORMAL);
  gObdState.canReady = true;

  Serial.println("Listening for CAN bus activity (2s)...");
  gObdState.busActive = detectCanBusActivity(2000);
  if (!gObdState.busActive) {
    Serial.println("No CAN frames seen - check OBD plug and ignition");
  }

  return true;
}

bool sendServiceRequest(uint8_t mode, uint8_t pid, bool hasPid, uint8_t &errorCode) {
  const uint8_t numBytes = hasPid ? 0x02 : 0x01;
  uint8_t request[8] = {numBytes, mode, pid, 0, 0, 0, 0, 0};
#if OBD_USE_EXTENDED_ID
  errorCode = canSendFrame(OBD_REQUEST_ID_EXT, 1, 8, request);
#else
  errorCode = canSendFrame(OBD_REQUEST_ID, 0, 8, request);
#endif
  return errorCode == CAN_OK;
}

bool waitForObdResponse(uint8_t expectedPid, uint8_t *response, uint8_t &length,
                        uint32_t timeoutMs) {
  const uint32_t deadline = millis() + timeoutMs;

  while ((int32_t)(deadline - millis()) > 0) {
    if (canBus.checkReceive() != CAN_MSGAVAIL) {
      continue;
    }

    unsigned long rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxData[8] = {};

    if (canReadFrame(&rxId, &rxLen, rxData) != CAN_OK) {
      continue;
    }

    gObdState.busActive = true;

    if (!isObdResponseId(rxId)) {
      continue;
    }

    if (rxLen < 4 || rxData[1] != 0x41 || rxData[2] != expectedPid) {
      continue;
    }

    length = rxLen;
    memcpy(response, rxData, rxLen);
    return true;
  }

  return false;
}

void logCanErrorThrottled(uint8_t errorCode, uint8_t pid) {
  const uint32_t now = millis();
  if (errorCode == lastLoggedCanError && (now - lastErrorLogMs) < 5000) {
    return;
  }

  lastLoggedCanError = errorCode;
  lastErrorLogMs = now;
  Serial.printf("CAN error %s (pid=0x%02X) - %s\n", canErrorName(errorCode), pid,
                canStatusMessage());
}

void pollObdPid(const ObdPid &pidDef, size_t index) {
  const uint32_t now = millis();

  // In backoff? Skip until the retry time, then try once more.
  if (pidRetryAtMs[index] != 0 && (int32_t)(pidRetryAtMs[index] - now) > 0) {
    return;
  }

  if ((now - lastPollMs[index]) < pidDef.intervalMs) {
    return;
  }

  lastPollMs[index] = now;

  uint8_t sendError = 0;
  if (!sendServiceRequest(0x01, pidDef.pid, true, sendError)) {
    recordCanSendError(sendError);
    logCanErrorThrottled(sendError, pidDef.pid);
    broadcastObdState();
    return;
  }

  gObdState.lastCanError = 0;

  uint8_t response[8] = {};
  uint8_t responseLen = 0;

  if (!waitForObdResponse(pidDef.pid, response, responseLen, kPollResponseTimeoutMs)) {
    recordCanTimeout();
    if (++consecutiveTimeouts[index] >= kMaxConsecutiveTimeouts) {
      pidRetryAtMs[index] = now + kUnsupportedRetryMs;
      consecutiveTimeouts[index] = 0;
      Serial.printf("PID 0x%02X (%s) not answering - backing off %us\n", pidDef.pid,
                    pidDef.name, kUnsupportedRetryMs / 1000);
    }
    return;
  }

  consecutiveTimeouts[index] = 0;
  pidRetryAtMs[index] = 0;
  updateObdState(pidDef.pid, response, responseLen);
  broadcastObdState();
}

// ---- Diagnostic trouble codes (Mode 03 / Mode 04) ----

void decodeDtcPair(uint8_t a, uint8_t b, char *out) {
  static const char kTypes[] = {'P', 'C', 'B', 'U'};
  out[0] = kTypes[(a & 0xC0) >> 6];
  out[1] = '0' + ((a & 0x30) >> 4);
  const char *hex = "0123456789ABCDEF";
  out[2] = hex[a & 0x0F];
  out[3] = hex[(b & 0xF0) >> 4];
  out[4] = hex[b & 0x0F];
  out[5] = '\0';
}

// Reads a Mode 03 response, following ISO-TP for multi-frame replies.
// Returns the number of DTC data bytes copied into buf, or -1 on failure.
int readDtcResponse(uint8_t *buf, size_t bufSize) {
  const uint32_t deadline = millis() + 800;
  int total = 0;
  int expected = -1;
  uint8_t nextSeq = 1;

  while ((int32_t)(deadline - millis()) > 0) {
    if (canBus.checkReceive() != CAN_MSGAVAIL) {
      continue;
    }

    unsigned long rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxData[8] = {};
    if (canReadFrame(&rxId, &rxLen, rxData) != CAN_OK || !isObdResponseId(rxId)) {
      continue;
    }

    const uint8_t pci = rxData[0] & 0xF0;

    if (pci == 0x00) {  // Single frame
      const uint8_t sfLen = rxData[0] & 0x0F;
      if (sfLen < 1 || rxData[1] != 0x43) {
        continue;
      }
      for (uint8_t i = 2; i < 1 + sfLen && i < rxLen && total < (int)bufSize; i++) {
        buf[total++] = rxData[i];
      }
      return total;
    }

    if (pci == 0x10) {  // First frame
      expected = ((rxData[0] & 0x0F) << 8) | rxData[1];
      if (rxData[2] != 0x43) {
        continue;
      }
      for (uint8_t i = 3; i < 8 && total < (int)bufSize; i++) {
        buf[total++] = rxData[i];
      }

      // Send flow control: clear to send, no block size, no separation time.
      uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
      canSendFrame(kEcuPhysicalRequestId, 0, 8, fc);
      continue;
    }

    if (pci == 0x20) {  // Consecutive frame
      if ((rxData[0] & 0x0F) != (nextSeq & 0x0F)) {
        continue;
      }
      nextSeq++;
      for (uint8_t i = 1; i < 8 && total < (int)bufSize; i++) {
        buf[total++] = rxData[i];
      }
      // total counts header 0x43 + count byte too; stop when we have the payload.
      if (expected > 0 && total >= expected - 1) {
        return total;
      }
    }
  }

  return total > 0 ? total : -1;
}

void performDtcRead() {
  gObdState.dtcStatus = DTC_READING;
  gObdState.dtcCount = 0;
  broadcastObdState();

  uint8_t sendError = 0;
  if (!sendServiceRequest(0x03, 0x00, false, sendError)) {
    gObdState.dtcStatus = DTC_ERROR;
    broadcastObdState();
    return;
  }

  uint8_t raw[64] = {};
  const int n = readDtcResponse(raw, sizeof(raw));
  if (n < 0) {
    gObdState.dtcStatus = DTC_ERROR;
    broadcastObdState();
    return;
  }

  // raw may start with a DTC-count byte; if the byte count is odd, the first
  // byte is the count and we skip it.
  int start = (n % 2 == 1) ? 1 : 0;
  uint8_t count = 0;
  for (int i = start; i + 1 < n && count < kMaxDtcs; i += 2) {
    if (raw[i] == 0 && raw[i + 1] == 0) {
      continue;  // padding
    }
    decodeDtcPair(raw[i], raw[i + 1], gObdState.dtcCodes[count]);
    count++;
  }

  gObdState.dtcCount = count;
  gObdState.dtcStatus = DTC_DONE;
  Serial.printf("Read %u DTC(s)\n", count);
  broadcastObdState();
}

void performDtcClear() {
  gObdState.dtcStatus = DTC_READING;
  broadcastObdState();

  uint8_t sendError = 0;
  if (!sendServiceRequest(0x04, 0x00, false, sendError)) {
    gObdState.dtcStatus = DTC_ERROR;
    broadcastObdState();
    return;
  }

  // Wait briefly for the 0x44 positive response.
  const uint32_t deadline = millis() + 500;
  while ((int32_t)(deadline - millis()) > 0) {
    if (canBus.checkReceive() != CAN_MSGAVAIL) {
      continue;
    }
    unsigned long rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxData[8] = {};
    if (canReadFrame(&rxId, &rxLen, rxData) == CAN_OK && isObdResponseId(rxId) &&
        rxData[1] == 0x44) {
      gObdState.dtcCount = 0;
      gObdState.dtcStatus = DTC_CLEARED;
      Serial.println("DTCs cleared");
      broadcastObdState();
      return;
    }
  }

  gObdState.dtcStatus = DTC_ERROR;
  broadcastObdState();
}

// ---- Supported-PID scan (Mode 01 PIDs 0x00 / 0x20 / ... / 0xC0) ----

void performPidScan() {
  gObdState.pidScanStatus = SCAN_RUNNING;
  gObdState.supportedCount = 0;
  broadcastObdState();

  const uint8_t bases[] = {0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0};

  for (uint8_t bi = 0; bi < sizeof(bases); bi++) {
    const uint8_t base = bases[bi];

    uint8_t sendError = 0;
    if (!sendServiceRequest(0x01, base, true, sendError)) {
      break;
    }

    uint8_t resp[8] = {};
    uint8_t len = 0;
    if (!waitForObdResponse(base, resp, len, 250) || len < 7) {
      break;  // range not supported / no response
    }

    const uint32_t mask = (static_cast<uint32_t>(resp[3]) << 24) |
                          (static_cast<uint32_t>(resp[4]) << 16) |
                          (static_cast<uint32_t>(resp[5]) << 8) |
                          static_cast<uint32_t>(resp[6]);

    for (uint8_t i = 0; i < 32; i++) {
      if (mask & (0x80000000UL >> i)) {
        const uint8_t pid = base + i + 1;
        if (gObdState.supportedCount < kMaxSupportedPids) {
          gObdState.supportedPids[gObdState.supportedCount++] = pid;
        }
      }
    }

    // The last bit (base + 0x20) indicates whether the next range exists.
    if (!(mask & 0x00000001UL)) {
      break;
    }
  }

  gObdState.pidScanStatus = SCAN_DONE;
  Serial.printf("PID scan complete: %u supported PIDs\n", gObdState.supportedCount);

  // The scan just proved the bus is awake, so clear any poll backoff and let
  // every live PID retry immediately instead of waiting out its backoff window.
  for (size_t i = 0; i < kPidCount; i++) {
    consecutiveTimeouts[i] = 0;
    pidRetryAtMs[i] = 0;
  }

  broadcastObdState();
}

void handlePendingCommands() {
  if (gObdState.cmdResetPeaks) {
    gObdState.cmdResetPeaks = false;
    resetSessionPeaks();
    Serial.println("Session peaks reset");
    broadcastObdState();
  }

  if (gObdState.cmdScanPids) {
    gObdState.cmdScanPids = false;
    performPidScan();
  }

  if (gObdState.cmdReadDtc) {
    gObdState.cmdReadDtc = false;
    performDtcRead();
  }

  if (gObdState.cmdClearDtc) {
    gObdState.cmdClearDtc = false;
    performDtcClear();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("NodeMCU-32S OBD CAN reader");
  Serial.println("Plug MCP2515 OBD adapter into vehicle OBD-II port");

  if (initCan()) {
    Serial.println("CAN ready at 500 kbps");
  } else {
    // Keep running so GPS NMEA still prints even with no OBD/CAN connection.
    Serial.println("CAN init failed - check wiring and CAN_CLOCK (8 vs 16 MHz)");
    Serial.println("Continuing without OBD - GPS NMEA will still stream");
  }

  initGps();
  initWebDashboard();
}

void loop() {
  handleWebDashboard();
  handleGps();

  if (gObdState.canReady) {
    handlePendingCommands();

    for (size_t i = 0; i < kPidCount; i++) {
      pollObdPid(kPidsToPoll[i], i);
    }
  }

  const uint32_t now = millis();
  if ((now - gObdState.lastStatusBroadcastMs) >= 2000) {
    gObdState.lastStatusBroadcastMs = now;
    broadcastObdState();
  }
}
