#include "obd_state.h"

ObdState gObdState;

namespace {

void trackPeaks() {
  if (gObdState.rpmValid && gObdState.rpm > gObdState.maxRpm) {
    gObdState.maxRpm = gObdState.rpm;
  }
  if (gObdState.speedValid && gObdState.speedKmh > gObdState.maxSpeedKmh) {
    gObdState.maxSpeedKmh = gObdState.speedKmh;
  }
  if (gObdState.coolantValid && gObdState.coolantC > gObdState.maxCoolantC) {
    gObdState.maxCoolantC = gObdState.coolantC;
  }
  if (gObdState.boostValid && gObdState.boostKpa > gObdState.maxBoostKpa) {
    gObdState.maxBoostKpa = gObdState.boostKpa;
  }
}

void recomputeBoost() {
  if (gObdState.mapValid) {
    gObdState.boostKpa =
        static_cast<int16_t>(gObdState.mapKpa) - static_cast<int16_t>(gObdState.baroKpa);
    gObdState.boostValid = true;
  }
}

}  // namespace

void updateObdState(uint8_t pid, const uint8_t *data, uint8_t length) {
  if (length < 4) {
    return;
  }

  const uint8_t a = data[3];
  const uint8_t b = length > 4 ? data[4] : 0;

  switch (pid) {
    case 0x04:
      gObdState.engineLoadPct = (a * 100.0f) / 255.0f;
      gObdState.engineLoadValid = true;
      break;
    case 0x05:
      gObdState.coolantC = static_cast<int16_t>(a - 40);
      gObdState.coolantValid = true;
      break;
    case 0x0B:
      gObdState.mapKpa = a;
      gObdState.mapValid = true;
      recomputeBoost();
      break;
    case 0x0C:
      gObdState.rpm = ((static_cast<uint16_t>(a) << 8) | b) / 4.0f;
      gObdState.rpmValid = true;
      break;
    case 0x0D:
      gObdState.speedKmh = a;
      gObdState.speedValid = true;
      break;
    case 0x0E:
      gObdState.timingAdv = (a / 2.0f) - 64.0f;
      gObdState.timingValid = true;
      break;
    case 0x0F:
      gObdState.intakeAirC = static_cast<int16_t>(a - 40);
      gObdState.intakeAirValid = true;
      break;
    case 0x10:
      gObdState.mafGs = ((static_cast<uint16_t>(a) << 8) | b) / 100.0f;
      gObdState.mafValid = true;
      break;
    case 0x11:
      gObdState.throttlePct = (a * 100.0f) / 255.0f;
      gObdState.throttleValid = true;
      break;
    case 0x2F:
      gObdState.fuelPct = (a * 100.0f) / 255.0f;
      gObdState.fuelValid = true;
      break;
    case 0x33:
      gObdState.baroKpa = a;
      recomputeBoost();
      break;
    case 0x42:
      gObdState.batteryV = ((static_cast<uint16_t>(a) << 8) | b) / 1000.0f;
      gObdState.batteryValid = true;
      break;
    default:
      return;
  }

  trackPeaks();
  gObdState.lastUpdateMs = millis();
}

void recordCanSendError(uint8_t errorCode) {
  gObdState.lastCanError = errorCode;
  gObdState.sendFailures++;
}

void recordCanTimeout() {
  gObdState.timeouts++;
}

void resetSessionPeaks() {
  gObdState.maxRpm = 0;
  gObdState.maxSpeedKmh = 0;
  gObdState.maxCoolantC = 0;
  gObdState.maxBoostKpa = 0;
}

const char *canErrorName(uint8_t errorCode) {
  switch (errorCode) {
    case 0:
      return "ok";
    case 1:
      return "init_failed";
    case 6:
      return "tx_buffer_busy";
    case 7:
      return "send_timeout_no_ack";
    default:
      return "unknown";
  }
}

const char *canStatusMessage() {
  if (!gObdState.canReady) {
    return "CAN controller failed to initialize";
  }

  if (!gObdState.busActive) {
    return "No CAN bus activity - plug into OBD port and turn ignition ON";
  }

  if (gObdState.lastCanError == 7) {
    return "CAN send failed (no ACK) - ignition ON? Engine running helps";
  }

  if (gObdState.lastCanError == 6) {
    return "CAN transmit buffer busy - retrying";
  }

  if (gObdState.lastUpdateMs == 0 && gObdState.timeouts > 0) {
    return "OBD requests sent but no ECU response yet";
  }

  if (gObdState.lastUpdateMs == 0) {
    return "Waiting for first OBD response";
  }

  return "Receiving OBD data";
}

const char *dtcStatusText(uint8_t status) {
  switch (status) {
    case DTC_READING:
      return "reading";
    case DTC_DONE:
      return "done";
    case DTC_CLEARED:
      return "cleared";
    case DTC_ERROR:
      return "error";
    default:
      return "idle";
  }
}

const char *dtcStatusName() {
  return dtcStatusText(gObdState.dtcStatus);
}

const char *scanStatusName() {
  switch (gObdState.pidScanStatus) {
    case SCAN_RUNNING:
      return "scanning";
    case SCAN_DONE:
      return "done";
    case SCAN_ERROR:
      return "error";
    default:
      return "idle";
  }
}

const char *ecuScanStatusName() {
  switch (gObdState.ecuScanStatus) {
    case SCAN_RUNNING:
      return "scanning";
    case SCAN_DONE:
      return "done";
    case SCAN_ERROR:
      return "error";
    default:
      return "idle";
  }
}

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

void decodeUdsDtc(uint8_t a, uint8_t b, uint8_t c, char *out) {
  decodeDtcPair(a, b, out);
  const char *hex = "0123456789ABCDEF";
  out[5] = '-';
  out[6] = hex[(c & 0xF0) >> 4];
  out[7] = hex[c & 0x0F];
  out[8] = '\0';
}

void applyUdsDtcResponse(UdsModuleState &mod, const uint8_t *resp, int n) {
  if (n < 3 || resp[0] != 0x59) {
    mod.status = DTC_ERROR;
    return;
  }

  uint8_t count = 0;
  for (int i = 3; i + 3 < n && count < kMaxDtcs; i += 4) {
    if (resp[i] == 0 && resp[i + 1] == 0 && resp[i + 2] == 0) {
      continue;
    }
    decodeUdsDtc(resp[i], resp[i + 1], resp[i + 2], mod.dtcCodes[count]);
    count++;
  }

  mod.dtcCount = count;
  mod.status = DTC_DONE;
}

void sanitizeIdent(const uint8_t *data, uint8_t n, char *out, size_t outSize) {
  if (outSize == 0) {
    return;
  }

  uint8_t printable = 0;
  uint8_t content = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (data[i] == 0x00 || data[i] == 0x20 || data[i] == 0xFF) {
      continue;
    }
    content++;
    if (data[i] >= 0x20 && data[i] <= 0x7E && data[i] != '"' && data[i] != '\\') {
      printable++;
    }
  }

  size_t o = 0;
  if (content > 0 && printable * 2 >= content) {
    for (uint8_t i = 0; i < n && o + 1 < outSize; i++) {
      const char c = static_cast<char>(data[i]);
      if (c == 0x00 || c == 0xFF) {
        continue;
      }
      if (c < 0x20 || c > 0x7E || c == '"' || c == '\\') {
        continue;
      }
      out[o++] = c;
    }
    while (o > 0 && out[o - 1] == ' ') {
      o--;
    }
  } else {
    const char *hex = "0123456789ABCDEF";
    for (uint8_t i = 0; i < n && o + 3 < outSize; i++) {
      if (i > 0) {
        out[o++] = ' ';
      }
      out[o++] = hex[(data[i] >> 4) & 0x0F];
      out[o++] = hex[data[i] & 0x0F];
    }
  }
  out[o] = '\0';
}
