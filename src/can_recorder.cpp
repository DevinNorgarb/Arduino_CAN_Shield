#include "can_recorder.h"

#include "web_dashboard.h"

namespace {

volatile bool gRecording = false;
uint32_t gRecordStartMs = 0;
uint32_t gRecordCount = 0;

}  // namespace

void canRecordStart() {
  gRecordStartMs = millis();
  gRecordCount = 0;
  gRecording = true;
}

void canRecordStop() {
  gRecording = false;
}

bool canRecordActive() {
  return gRecording;
}

uint32_t canRecordCount() {
  return gRecordCount;
}

void canRecordFrame(bool tx, unsigned long id, uint8_t len, const uint8_t *data) {
  if (!gRecording) {
    return;
  }

  const uint32_t elapsed = millis() - gRecordStartMs;
  const uint32_t secs = elapsed / 1000;
  const uint32_t usec = (elapsed % 1000) * 1000;

  // mcp_can's readMsgBuf OR's 0x80000000 into the ID to flag extended (29-bit)
  // frames. Strip it so the logged ID is the real CAN ID, and use it to pick
  // candump's width convention (3 hex digits for 11-bit, 8 for 29-bit).
  const bool extended = (id & 0x80000000UL) != 0;
  id &= 0x1FFFFFFFUL;

  char line[72];
  int n = snprintf(line, sizeof(line), "(%lu.%06lu) %s ", (unsigned long)secs,
                   (unsigned long)usec, tx ? "tx" : "rx");

  if (extended || id > 0x7FF) {
    n += snprintf(line + n, sizeof(line) - n, "%08lX#", id);
  } else {
    n += snprintf(line + n, sizeof(line) - n, "%03lX#", id);
  }

  for (uint8_t i = 0; i < len && n < (int)sizeof(line) - 3; i++) {
    n += snprintf(line + n, sizeof(line) - n, "%02X", data[i]);
  }

  gRecordCount++;
  broadcastCanFrame(String(line));
}
