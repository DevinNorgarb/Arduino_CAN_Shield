#include "elm327.h"
#include "config.h"

#include <string.h>
#include <ctype.h>
#include <WiFi.h>
#include <mcp_can.h>
#include <BluetoothSerial.h>
#include <esp_gap_bt_api.h>

#include "can_io.h"
#include "obd_state.h"
#include "web_dashboard.h"

namespace {

constexpr uint32_t kIdMask = 0x1FFFFFFF;
constexpr uint32_t kDefaultTimeoutMs = 200;
constexpr size_t kCmdMax = 96;
constexpr size_t kPayloadMax = 256;

BluetoothSerial SerialBT;
WiFiServer elmTcpServer(ELM327_TCP_PORT);
WiFiClient elmTcpClient;
bool elmHadClient = false;
bool elmBluetoothActive = false;

struct ElmSession {
  bool echo = true;
  bool headers = false;
  bool linefeeds = true;
  bool spaces = true;
  bool caf = true;
  uint32_t timeoutMs = kDefaultTimeoutMs;
  unsigned long headerId = OBD_REQUEST_ID;
  char lastCmd[kCmdMax] = {};
};

ElmSession session;

char cmdBuf[kCmdMax];
size_t cmdLen = 0;

bool isObdEcuId(unsigned long rxId) {
  rxId &= kIdMask;
#if OBD_USE_EXTENDED_ID
  return rxId == OBD_RESPONSE_ID_EXT;
#else
  if (session.headerId != OBD_REQUEST_ID) {
    // Physical header 0x7E0 → expect 0x7E8, etc.
    return rxId == (session.headerId + 8);
  }
  return rxId >= 0x7E8 && rxId <= 0x7EF;
#endif
}

unsigned long flowControlId(unsigned long rxId) {
  rxId &= kIdMask;
#if OBD_USE_EXTENDED_ID
  return OBD_REQUEST_ID_EXT;
#else
  if (rxId >= 0x7E8 && rxId <= 0x7EF) {
    return rxId - 8;
  }
  return session.headerId;
#endif
}

void elmWrite(const char *s) {
  if (s == nullptr) {
    return;
  }
  if (elmBluetoothActive && SerialBT.hasClient()) {
    SerialBT.print(s);
  }
  if (elmTcpClient.connected()) {
    elmTcpClient.print(s);
  }
}

void elmCrlf() {
  elmWrite("\r");
  if (session.linefeeds) {
    elmWrite("\n");
  }
}

void elmPrompt() { elmWrite(">"); }

void resetSession() {
  session = ElmSession();
  cmdLen = 0;
}

void sendFlowControl(unsigned long reqId) {
  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
  canSendFrame(reqId, 0, 8, fc);
}

void stripInPlace(char *s) {
  char *w = s;
  for (char *r = s; *r; r++) {
    if (*r != ' ' && *r != '\t') {
      *w++ = static_cast<char>(toupper(static_cast<unsigned char>(*r)));
    }
  }
  *w = '\0';
}

int parseHexBytes(const char *s, uint8_t *out, size_t outMax) {
  int n = 0;
  int nibble = -1;
  for (; *s; s++) {
    char c = *s;
    int v;
    if (c >= '0' && c <= '9') {
      v = c - '0';
    } else if (c >= 'A' && c <= 'F') {
      v = 10 + (c - 'A');
    } else if (c >= 'a' && c <= 'f') {
      v = 10 + (c - 'a');
    } else {
      continue;
    }
    if (nibble < 0) {
      nibble = v;
    } else {
      if (n >= (int)outMax) {
        return -1;
      }
      out[n++] = static_cast<uint8_t>((nibble << 4) | v);
      nibble = -1;
    }
  }
  // Odd trailing nibble: treat as a low nibble of a byte (ELM327 does this).
  if (nibble >= 0) {
    if (n >= (int)outMax) {
      return -1;
    }
    out[n++] = static_cast<uint8_t>(nibble);
  }
  return n;
}

void appendHexByte(char *buf, size_t &n, size_t cap, uint8_t b, bool leadingSpace) {
  if (n + 3 >= cap) {
    return;
  }
  if (leadingSpace && session.spaces) {
    buf[n++] = ' ';
  }
  static const char kHex[] = "0123456789ABCDEF";
  buf[n++] = kHex[b >> 4];
  buf[n++] = kHex[b & 0x0F];
}

void appendCanId(char *buf, size_t &n, size_t cap, unsigned long id) {
  id &= kIdMask;
  char tmp[12];
  snprintf(tmp, sizeof(tmp), "%lX", id);
  const size_t len = strlen(tmp);
  if (n + len + 2 >= cap) {
    return;
  }
  memcpy(buf + n, tmp, len);
  n += len;
  if (session.spaces) {
    buf[n++] = ' ';
  }
}

// Format one assembled ISO-TP payload the way an ELM327 would.
void formatPayload(char *buf, size_t cap, unsigned long rxId, const uint8_t *payload,
                   int payloadLen) {
  size_t n = 0;
  buf[0] = '\0';
  if (payloadLen <= 0) {
    return;
  }

  if (session.headers) {
    appendCanId(buf, n, cap, rxId);
    if (session.caf) {
      appendHexByte(buf, n, cap, static_cast<uint8_t>(payloadLen), false);
    }
  }

  for (int i = 0; i < payloadLen; i++) {
    const bool space = n > 0;
    appendHexByte(buf, n, cap, payload[i], space);
  }
  buf[n] = '\0';
}

// Send an ISO-TP single-frame request and collect one complete reply.
int elmIsotpRequest(const uint8_t *req, uint8_t reqLen, uint8_t *resp, size_t respSize,
                    unsigned long *respId) {
  if (reqLen == 0 || reqLen > 7) {
    return -1;
  }

  uint8_t frame[8] = {};
  frame[0] = reqLen & 0x0F;
  memcpy(frame + 1, req, reqLen);

#if OBD_USE_EXTENDED_ID
  const uint8_t err = canSendFrame(OBD_REQUEST_ID_EXT, 1, 8, frame);
#else
  const uint8_t err = canSendFrame(session.headerId, 0, 8, frame);
#endif
  if (err != CAN_OK) {
    recordCanSendError(err);
    return -2;
  }
  gObdState.lastCanError = 0;

  const uint32_t deadline = millis() + session.timeoutMs;
  int total = 0;
  int expected = -1;
  uint8_t nextSeq = 1;
  bool receiving = false;

  while ((int32_t)(deadline - millis()) > 0) {
    if (!canAvailable()) {
      continue;
    }

    unsigned long rxId = 0;
    uint8_t rxLen = 0;
    uint8_t d[8] = {};
    if (canReadFrame(&rxId, &rxLen, d) != CAN_OK || !isObdEcuId(rxId)) {
      continue;
    }

    gObdState.busActive = true;
    *respId = rxId;

    const uint8_t pci = d[0] & 0xF0;

    if (pci == 0x00) {
      const uint8_t n = d[0] & 0x0F;
      // NRC 0x78 response-pending: keep waiting.
      if (n >= 3 && d[1] == 0x7F && d[3] == 0x78) {
        continue;
      }
      total = 0;
      for (uint8_t i = 0; i < n && i < 7 && total < (int)respSize; i++) {
        resp[total++] = d[1 + i];
      }
      return total;
    }

    if (pci == 0x10) {
      expected = ((d[0] & 0x0F) << 8) | d[1];
      total = 0;
      for (uint8_t i = 2; i < 8 && total < (int)respSize; i++) {
        resp[total++] = d[i];
      }
      sendFlowControl(flowControlId(rxId));
      receiving = true;
      nextSeq = 1;
      continue;
    }

    if (pci == 0x20 && receiving) {
      if ((d[0] & 0x0F) != (nextSeq & 0x0F)) {
        continue;
      }
      nextSeq++;
      for (uint8_t i = 1; i < 8 && total < (int)respSize && total < expected; i++) {
        resp[total++] = d[i];
      }
      if (expected > 0 && total >= expected) {
        return total;
      }
    }
  }

  return total > 0 ? total : -1;
}

void maybeUpdateDashboard(const uint8_t *payload, int len) {
  if (len < 3 || payload[0] != 0x41) {
    return;
  }
  // updateObdState expects a raw CAN frame: PCI, 41, pid, A, B, …
  uint8_t frame[8] = {};
  const uint8_t copy = len > 7 ? 7 : static_cast<uint8_t>(len);
  frame[0] = copy;
  memcpy(frame + 1, payload, copy);
  updateObdState(payload[1], frame, copy + 1);
  broadcastObdState();
}

void replyBody(const char *body) {
  if (body != nullptr && body[0] != '\0') {
    elmWrite(body);
    elmCrlf();
  }
  elmPrompt();
}

void replyOk() { replyBody("OK"); }

void replyIdentify() { replyBody(ELM327_ID_STRING); }

void handleAtCommand(char *cmd) {
  // cmd is already stripped of spaces and uppercased, still starts with "AT".
  const char *rest = cmd + 2;

  if (rest[0] == '\0' || strcmp(rest, "I") == 0) {
    replyIdentify();
    return;
  }
  if (strcmp(rest, "Z") == 0 || strcmp(rest, "WS") == 0) {
    resetSession();
    elmCrlf();
    elmWrite(ELM327_ID_STRING);
    elmCrlf();
    elmPrompt();
    return;
  }
  if (strcmp(rest, "D") == 0) {
    const unsigned long keepHeader = session.headerId;
    resetSession();
    session.headerId = keepHeader;
    replyOk();
    return;
  }
  if (strcmp(rest, "E0") == 0) {
    session.echo = false;
    replyOk();
    return;
  }
  if (strcmp(rest, "E1") == 0) {
    session.echo = true;
    replyOk();
    return;
  }
  if (strcmp(rest, "H0") == 0) {
    session.headers = false;
    replyOk();
    return;
  }
  if (strcmp(rest, "H1") == 0) {
    session.headers = true;
    replyOk();
    return;
  }
  if (strcmp(rest, "L0") == 0) {
    session.linefeeds = false;
    replyOk();
    return;
  }
  if (strcmp(rest, "L1") == 0) {
    session.linefeeds = true;
    replyOk();
    return;
  }
  if (strcmp(rest, "S0") == 0) {
    session.spaces = false;
    replyOk();
    return;
  }
  if (strcmp(rest, "S1") == 0) {
    session.spaces = true;
    replyOk();
    return;
  }
  if (strcmp(rest, "CAF0") == 0) {
    session.caf = false;
    replyOk();
    return;
  }
  if (strcmp(rest, "CAF1") == 0) {
    session.caf = true;
    replyOk();
    return;
  }
  if (strncmp(rest, "SP", 2) == 0) {
    // This hardware is ISO 15765-4 CAN 11/500. Accept auto (0) and protocol 6.
    replyOk();
    return;
  }
  if (strcmp(rest, "DP") == 0) {
    replyBody("ISO 15765-4 (CAN 11/500)");
    return;
  }
  if (strcmp(rest, "DPN") == 0) {
    replyBody("6");
    return;
  }
  if (strcmp(rest, "RV") == 0) {
    if (gObdState.canReady && !gObdState.batteryValid) {
      const uint8_t req[] = {0x01, 0x42};
      uint8_t resp[16] = {};
      unsigned long rid = 0;
      const int n = elmIsotpRequest(req, 2, resp, sizeof(resp), &rid);
      if (n >= 3 && resp[0] == 0x41) {
        maybeUpdateDashboard(resp, n);
      }
    }
    char volt[16];
    const float v = gObdState.batteryValid ? gObdState.batteryV : 12.6f;
    snprintf(volt, sizeof(volt), "%0.1fV", v);
    replyBody(volt);
    return;
  }
  if (strcmp(rest, "@1") == 0) {
    replyBody("OBDII to RS232 Interpreter");
    return;
  }
  if (strcmp(rest, "@2") == 0) {
    replyBody("ArduinoCANShield");
    return;
  }
  if (strncmp(rest, "ST", 2) == 0) {
    if (rest[2] == '\0' || strcmp(rest + 2, "00") == 0) {
      session.timeoutMs = kDefaultTimeoutMs;
    } else {
      uint8_t raw[2] = {};
      if (parseHexBytes(rest + 2, raw, 1) >= 1) {
        session.timeoutMs = raw[0] * 4;
        if (session.timeoutMs < 50) {
          session.timeoutMs = 50;
        }
      }
    }
    replyOk();
    return;
  }
  if (strncmp(rest, "SH", 2) == 0) {
    const unsigned long id = strtoul(rest + 2, nullptr, 16);
    if (id != 0) {
      session.headerId = id;
    }
    replyOk();
    return;
  }

  // Adaptive timing, flow-control, CAN silent, long messages, etc. — accept.
  if (strncmp(rest, "AT", 2) == 0 || strncmp(rest, "AL", 2) == 0 ||
      strncmp(rest, "FC", 2) == 0 || strncmp(rest, "CS", 2) == 0 ||
      strncmp(rest, "CF", 2) == 0 || strncmp(rest, "CM", 2) == 0 ||
      strncmp(rest, "CRA", 3) == 0 || strncmp(rest, "R", 1) == 0 ||
      strncmp(rest, "M", 1) == 0 || strncmp(rest, "PC", 2) == 0 ||
      strncmp(rest, "PP", 2) == 0 || strncmp(rest, "KW", 2) == 0 ||
      strncmp(rest, "TA", 2) == 0 || strncmp(rest, "CEA", 3) == 0 ||
      strncmp(rest, "V", 1) == 0 || strncmp(rest, "D", 1) == 0 ||
      strncmp(rest, "S", 1) == 0) {
    replyOk();
    return;
  }

  replyOk();
}

void handleObdRequest(const uint8_t *req, uint8_t reqLen) {
  if (!gObdState.canReady) {
    replyBody("UNABLE TO CONNECT");
    return;
  }

  uint8_t resp[kPayloadMax] = {};
  unsigned long rid = 0;
  const int n = elmIsotpRequest(req, reqLen, resp, sizeof(resp), &rid);
  if (n == -2) {
    replyBody("UNABLE TO CONNECT");
    return;
  }
  if (n <= 0) {
    recordCanTimeout();
    replyBody("NO DATA");
    return;
  }

  maybeUpdateDashboard(resp, n);

  char line[512];
  formatPayload(line, sizeof(line), rid, resp, n);
  replyBody(line);
}

void processCommand(const char *raw) {
  char cmd[kCmdMax];
  strncpy(cmd, raw, kCmdMax - 1);
  cmd[kCmdMax - 1] = '\0';

  // Empty line repeats the last command (ELM327 behaviour).
  if (cmd[0] == '\0') {
    if (session.lastCmd[0] == '\0') {
      elmPrompt();
      return;
    }
    strncpy(cmd, session.lastCmd, kCmdMax - 1);
  } else {
    strncpy(session.lastCmd, cmd, kCmdMax - 1);
    session.lastCmd[kCmdMax - 1] = '\0';
  }

  if (session.echo) {
    elmWrite(raw[0] ? raw : session.lastCmd);
    elmCrlf();
  }

  char stripped[kCmdMax];
  strncpy(stripped, cmd, kCmdMax - 1);
  stripped[kCmdMax - 1] = '\0';
  stripInPlace(stripped);

  if (stripped[0] == '\0') {
    elmPrompt();
    return;
  }

  if (stripped[0] == 'A' && stripped[1] == 'T') {
    handleAtCommand(stripped);
    return;
  }

  uint8_t req[8] = {};
  const int n = parseHexBytes(stripped, req, sizeof(req));
  if (n <= 0 || n > 7) {
    replyBody("?");
    return;
  }

  handleObdRequest(req, static_cast<uint8_t>(n));
}

void onClientConnected() {
  resetSession();
  Serial.println("ELM327 client — dashboard polling paused");
  broadcastObdState();
}

void onClientDisconnected() {
  Serial.println("ELM327 client gone — dashboard polling resumed");
  broadcastObdState();
}

void feedElmByte(char c) {
  if (c == '\n') {
    return;
  }
  if (c == '\r') {
    cmdBuf[cmdLen] = '\0';
    processCommand(cmdBuf);
    cmdLen = 0;
    return;
  }
  if (c == '\b' || c == 0x7F) {
    if (cmdLen > 0) {
      cmdLen--;
    }
    return;
  }
  if (cmdLen < kCmdMax - 1) {
    cmdBuf[cmdLen++] = c;
  }
}

void acceptTcpClient() {
  if (elmTcpClient.connected()) {
    return;
  }
  WiFiClient incoming = elmTcpServer.accept();
  if (incoming) {
    elmTcpClient.stop();
    elmTcpClient = incoming;
    elmTcpClient.setNoDelay(true);
    Serial.printf("ELM327 WiFi client %s\n", elmTcpClient.remoteIP().toString().c_str());
  }
}

}  // namespace

void initElm327Bluetooth() {
  // WiFi is already off. Classic-only (no BLE) — no coexistence needed.
  SerialBT.disableSSP();
  if (!SerialBT.begin(ELM327_BT_NAME, false, true)) {
    Serial.println("ELM327 Bluetooth Serial failed to start");
    return;
  }
  SerialBT.setPin(ELM327_BT_PIN, sizeof(ELM327_BT_PIN) - 1);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  elmBluetoothActive = true;
  Serial.printf("ELM327 Bluetooth: \"%s\"  PIN %s  MAC %s\n", ELM327_BT_NAME,
                ELM327_BT_PIN, SerialBT.getBtAddressString().c_str());
  Serial.println("Pair OBDII from Android Settings → Bluetooth (iPhone cannot use SPP).");
}

void elm327StartTcp() {
  static bool started = false;
  if (started) {
    return;
  }
  // Must run after WiFi.mode() — socket() before lwIP/tcpip is up panics
  // with xQueueSemaphoreTake on a NULL queue.
  elmTcpServer.begin();
  elmTcpServer.setNoDelay(true);
  started = true;
  Serial.printf("ELM327 WiFi TCP %u (Car Scanner → WiFi adapter)\n", ELM327_TCP_PORT);
}

void elm327StopTcp() {
  elmTcpClient.stop();
  elmTcpServer.end();
}

void elm327OnWifiUp() {
  Serial.printf("ELM327 WiFi adapter at %s:%u\n", WiFi.localIP().toString().c_str(),
                ELM327_TCP_PORT);
}

void handleElm327() {
  if (!elmBluetoothActive) {
    acceptTcpClient();
  }

  const bool linked = (elmBluetoothActive && SerialBT.hasClient()) ||
                      elmTcpClient.connected();
  if (linked && !elmHadClient) {
    elmHadClient = true;
    onClientConnected();
  } else if (!linked && elmHadClient) {
    elmHadClient = false;
    onClientDisconnected();
  }

  if (elmBluetoothActive) {
    while (SerialBT.available()) {
      feedElmByte(static_cast<char>(SerialBT.read()));
    }
  }
  while (elmTcpClient.connected() && elmTcpClient.available()) {
    feedElmByte(static_cast<char>(elmTcpClient.read()));
  }
}

bool elm327ClientConnected() {
  return (elmBluetoothActive && SerialBT.hasClient()) || elmTcpClient.connected();
}

bool elm327UsingBluetooth() { return elmBluetoothActive; }
