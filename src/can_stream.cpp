#include "can_stream.h"

#include <WiFi.h>
#include <WebSocketsClient.h>

#include "can_recorder.h"
#include "config.h"
#include "web_dashboard.h"

namespace {

constexpr uint8_t kBatchMax = 16;
constexpr uint32_t kFlushMs = 40;
constexpr uint32_t kReconnectMs = 5000;
constexpr size_t kJsonCap = 2048;

struct PendingFrame {
  uint32_t t;
  uint32_t seq;
  uint32_t id;
  uint8_t len;
  uint8_t flags;  // bit0 = tx, bit1 = ext
  uint8_t data[8];
};

WebSocketsClient client;
bool begun = false;
bool paused = false;
bool connected = false;
uint32_t sentCount = 0;
uint32_t dropCount = 0;
uint32_t nextSeq = 1;

PendingFrame batch[kBatchMax];
uint8_t batchCount = 0;
uint32_t batchStartMs = 0;
char jsonBuf[kJsonCap];

bool hostConfigured() {
  return CAN_STREAM_HOST[0] != '\0';
}

void dropBatch() {
  dropCount += batchCount;
  batchCount = 0;
}

size_t appendHex(char *out, size_t cap, size_t n, const uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len && n + 2 < cap; i++) {
    static const char kHex[] = "0123456789ABCDEF";
    out[n++] = kHex[data[i] >> 4];
    out[n++] = kHex[data[i] & 0x0F];
  }
  if (n < cap) {
    out[n] = '\0';
  }
  return n;
}

bool flushBatch() {
  if (batchCount == 0) {
    return true;
  }
  if (!connected) {
    dropBatch();
    return false;
  }

  char *json = jsonBuf;
  size_t n = 0;
  n += snprintf(json + n, kJsonCap - n, "{\"v\":1,\"frames\":[");

  for (uint8_t i = 0; i < batchCount && n + 80 < kJsonCap; i++) {
    const PendingFrame &f = batch[i];
    if (i > 0) {
      json[n++] = ',';
    }
    n += snprintf(json + n, kJsonCap - n,
                  "{\"t\":%lu,\"seq\":%lu,\"tx\":%s,\"id\":%lu,\"ext\":%s,\"data\":\"",
                  (unsigned long)f.t, (unsigned long)f.seq, (f.flags & 0x01) ? "true" : "false",
                  (unsigned long)f.id, (f.flags & 0x02) ? "true" : "false");
    n = appendHex(json, kJsonCap, n, f.data, f.len);
    if (n + 2 < kJsonCap) {
      json[n++] = '"';
      json[n++] = '}';
      json[n] = '\0';
    }
  }

  if (n + 2 < kJsonCap) {
    json[n++] = ']';
    json[n++] = '}';
    json[n] = '\0';
  }

  if (!client.sendTXT(json, n)) {
    dropBatch();
    return false;
  }

  sentCount += batchCount;
  batchCount = 0;
  return true;
}

void onWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      connected = true;
      Serial.printf("CAN stream connected to %s:%u%s\n", CAN_STREAM_HOST, CAN_STREAM_PORT,
                    CAN_STREAM_PATH);
      {
        char hello[160];
        snprintf(hello, sizeof(hello),
                 "{\"v\":1,\"type\":\"hello\",\"device\":\"obd\",\"ip\":\"%s\"}",
                 WiFi.localIP().toString().c_str());
        client.sendTXT(hello);
      }
      break;
    case WStype_DISCONNECTED:
      connected = false;
      dropBatch();
      Serial.println("CAN stream disconnected");
      break;
    case WStype_ERROR:
      Serial.println("CAN stream websocket error");
      break;
    case WStype_TEXT: {
      String cmd;
      cmd.reserve(length);
      for (size_t i = 0; i < length; i++) {
        cmd += static_cast<char>(payload[i]);
      }
      applyDashboardCommand(cmd);
      break;
    }
    case WStype_BIN:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    case WStype_PING:
    case WStype_PONG:
    default:
      (void)payload;
      (void)length;
      break;
  }
}

void startClient() {
  connected = false;
  client.onEvent(onWsEvent);
  client.setReconnectInterval(kReconnectMs);
  client.enableHeartbeat(15000, 3000, 2);

#if CAN_STREAM_SSL
  client.beginSSL(CAN_STREAM_HOST, CAN_STREAM_PORT, CAN_STREAM_PATH);
#else
  client.begin(CAN_STREAM_HOST, CAN_STREAM_PORT, CAN_STREAM_PATH);
#endif

  Serial.printf("CAN stream connecting to %s://%s:%u%s\n", CAN_STREAM_SSL ? "wss" : "ws",
                CAN_STREAM_HOST, CAN_STREAM_PORT, CAN_STREAM_PATH);
}

}  // namespace

void canStreamLoop() {
  if (!hostConfigured()) {
    return;
  }
  canCaptureSyncIrq();

  if (WiFi.status() != WL_CONNECTED) {
    if (begun) {
      canStreamStop();
    }
    return;
  }

  if (!begun) {
    startClient();
    begun = true;
    canCaptureSyncIrq();
  }

  client.loop();

  if (batchCount > 0 && (millis() - batchStartMs) >= kFlushMs) {
    flushBatch();
  }
}

void canStreamStop() {
  if (!begun && !connected) {
    dropBatch();
    return;
  }
  flushBatch();
  client.disconnect();
  begun = false;
  connected = false;
  dropBatch();
  canCaptureSyncIrq();
}

void canStreamPublish(bool tx, unsigned long id, bool extended, uint8_t len,
                      const uint8_t *data) {
  if (!hostConfigured() || paused) {
    return;
  }
  if (!connected) {
    dropCount++;
    return;
  }
  if (batchCount >= kBatchMax) {
    if (!flushBatch()) {
      dropCount++;
      return;
    }
  }
  if (batchCount == 0) {
    batchStartMs = millis();
  }

  PendingFrame &f = batch[batchCount++];
  f.t = millis();
  f.seq = nextSeq++;
  f.id = id;
  f.len = len > 8 ? 8 : len;
  f.flags = (tx ? 0x01 : 0x00) | (extended ? 0x02 : 0x00);
  memcpy(f.data, data, f.len);
}

void canStreamSendText(const String &payload) {
  if (!connected || paused || payload.length() == 0) {
    return;
  }
  client.sendTXT(payload.c_str(), payload.length());
}

bool canStreamConfigured() {
  return hostConfigured();
}

bool canStreamConnected() {
  return connected;
}

bool canStreamWantFrames() {
  return hostConfigured() && !paused;
}

bool canStreamPaused() {
  return paused;
}

void canStreamSetPaused(bool value) {
  if (paused == value) {
    return;
  }
  paused = value;
  if (paused) {
    flushBatch();
  }
  canCaptureSyncIrq();
  Serial.printf("CAN stream %s\n", paused ? "paused" : "resumed");
}

uint32_t canStreamSent() {
  return sentCount;
}

uint32_t canStreamDropped() {
  return dropCount;
}

const char *canStreamHost() {
  return CAN_STREAM_HOST;
}
