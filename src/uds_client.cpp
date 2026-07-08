#include "uds_client.h"

#include <string.h>

#include <mcp_can.h>

#include "can_io.h"

namespace {

constexpr uint32_t kIdMask = 0x1FFFFFFF;  // strip mcp_can's extended flag bit

void sendFlowControl(unsigned long reqId) {
  // Clear-to-send, no block size, no separation time.
  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
  canSendFrame(reqId, 0, 8, fc);
}

// A single-frame negative response of the form "7F <sid> 78" means the ECU is
// busy and a real answer is still coming, so we must keep waiting.
bool isResponsePending(const uint8_t *d, uint8_t n) {
  return n >= 3 && d[1] == 0x7F && d[3] == 0x78;
}

}  // namespace

int udsRequest(unsigned long reqId, unsigned long respId, const uint8_t *req,
               uint8_t reqLen, uint8_t *resp, size_t respSize, uint32_t timeoutMs) {
  if (reqLen == 0 || reqLen > 7) {
    return -1;
  }

  uint8_t frame[8] = {};
  frame[0] = reqLen & 0x0F;  // single-frame PCI
  memcpy(frame + 1, req, reqLen);
  canSendFrame(reqId, 0, 8, frame);

  const uint32_t deadline = millis() + timeoutMs;
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
    if (canReadFrame(&rxId, &rxLen, d) != CAN_OK || (rxId & kIdMask) != respId) {
      continue;
    }

    const uint8_t pci = d[0] & 0xF0;

    if (pci == 0x00) {  // single frame
      const uint8_t n = d[0] & 0x0F;
      if (isResponsePending(d, n)) {
        continue;
      }
      total = 0;
      for (uint8_t i = 0; i < n && i < 7 && total < (int)respSize; i++) {
        resp[total++] = d[1 + i];
      }
      return total;
    }

    if (pci == 0x10) {  // first frame of a multi-frame reply
      expected = ((d[0] & 0x0F) << 8) | d[1];
      total = 0;
      for (uint8_t i = 2; i < 8 && total < (int)respSize; i++) {
        resp[total++] = d[i];
      }
      sendFlowControl(reqId);
      receiving = true;
      nextSeq = 1;
      continue;
    }

    if (pci == 0x20 && receiving) {  // consecutive frame
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
