#include "uds_scan.h"

#include <string.h>

#include "obd_state.h"
#include "uds_client.h"
#include "uds_modules.h"
#include "web_dashboard.h"

namespace {

constexpr uint32_t kProbeTimeoutMs = 120;
constexpr uint32_t kDidTimeoutMs = 80;
constexpr uint32_t kIdentTimeoutMs = 220;
constexpr uint32_t kDtcTimeoutMs = 1000;
constexpr uint16_t kDidScanStart = 0xF180;
constexpr uint16_t kDidScanEnd = 0xF1FF;

void setScanLabel(const char *text) {
  strncpy(gObdState.ecuScanLabel, text, sizeof(gObdState.ecuScanLabel) - 1);
  gObdState.ecuScanLabel[sizeof(gObdState.ecuScanLabel) - 1] = '\0';
}

void pumpScan() {
  handleWebDashboard();
  broadcastObdState();
  yield();
}

bool anyUdsReply(const uint8_t *req, uint8_t reqLen, unsigned long reqId,
                 unsigned long respId, uint32_t timeoutMs) {
  uint8_t resp[16] = {};
  const int n = udsRequest(reqId, respId, req, reqLen, resp, sizeof(resp), timeoutMs);
  return n > 0;
}

bool probeModule(const UdsModuleConfig &cfg) {
  const uint8_t testerPresent[] = {0x3E, 0x00};
  if (anyUdsReply(testerPresent, sizeof(testerPresent), cfg.reqId, cfg.respId,
                  kProbeTimeoutMs)) {
    return true;
  }
  const uint8_t defaultSession[] = {0x10, 0x01};
  return anyUdsReply(defaultSession, sizeof(defaultSession), cfg.reqId, cfg.respId,
                     kProbeTimeoutMs);
}

bool readDid(const UdsModuleConfig &cfg, uint16_t did, uint8_t *data, uint8_t &dataLen,
             size_t dataMax, uint32_t timeoutMs) {
  const uint8_t req[] = {0x22, static_cast<uint8_t>(did >> 8),
                         static_cast<uint8_t>(did & 0xFF)};
  uint8_t resp[128] = {};
  const int n = udsRequest(cfg.reqId, cfg.respId, req, sizeof(req), resp, sizeof(resp),
                           timeoutMs);
  if (n < 3 || resp[0] != 0x62 || resp[1] != req[1] || resp[2] != req[2]) {
    return false;
  }
  const uint8_t payload = static_cast<uint8_t>(n - 3);
  dataLen = payload > dataMax ? static_cast<uint8_t>(dataMax) : payload;
  memcpy(data, resp + 3, dataLen);
  return true;
}

void storeIdent(char *dest, size_t destSize, const uint8_t *data, uint8_t n) {
  sanitizeIdent(data, n, dest, destSize);
}

void readIdent(uint8_t moduleId) {
  const UdsModuleConfig &cfg = kUdsModules[moduleId];
  UdsModuleState &mod = gObdState.udsModules[moduleId];
  uint8_t buf[48] = {};
  uint8_t n = 0;

  if (readDid(cfg, 0xF190, buf, n, sizeof(buf), kIdentTimeoutMs) &&
      moduleId == UDS_MOD_ENGINE) {
    sanitizeIdent(buf, n, gObdState.vin, sizeof(gObdState.vin));
  }
  if (readDid(cfg, 0xF187, buf, n, sizeof(buf), kIdentTimeoutMs)) {
    storeIdent(mod.partNumber, sizeof(mod.partNumber), buf, n);
  }
  if (readDid(cfg, 0xF189, buf, n, sizeof(buf), kIdentTimeoutMs)) {
    storeIdent(mod.swVersion, sizeof(mod.swVersion), buf, n);
  }
  if (readDid(cfg, 0xF191, buf, n, sizeof(buf), kIdentTimeoutMs)) {
    storeIdent(mod.hwNumber, sizeof(mod.hwNumber), buf, n);
  }
  if (readDid(cfg, 0xF197, buf, n, sizeof(buf), kIdentTimeoutMs)) {
    storeIdent(mod.sysName, sizeof(mod.sysName), buf, n);
  }
}

void readModuleDtcs(uint8_t moduleId) {
  const UdsModuleConfig &cfg = kUdsModules[moduleId];
  UdsModuleState &mod = gObdState.udsModules[moduleId];
  const uint8_t req[] = {0x19, 0x02, 0xFF};
  uint8_t resp[128] = {};
  const int n =
      udsRequest(cfg.reqId, cfg.respId, req, sizeof(req), resp, sizeof(resp), kDtcTimeoutMs);
  applyUdsDtcResponse(mod, resp, n);
}

bool didAlreadyStored(uint16_t did) {
  for (uint8_t i = 0; i < gObdState.engineDidCount; i++) {
    if (gObdState.engineDids[i].id == did) {
      return true;
    }
  }
  return false;
}

void storeDid(uint16_t did, const uint8_t *data, uint8_t n) {
  if (gObdState.engineDidCount >= kMaxDidDump || didAlreadyStored(did)) {
    return;
  }
  DidDumpEntry &entry = gObdState.engineDids[gObdState.engineDidCount++];
  entry.id = did;
  sanitizeIdent(data, n, entry.value, sizeof(entry.value));
}

void dumpEngineDids() {
  const UdsModuleConfig &cfg = kUdsModules[UDS_MOD_ENGINE];
  uint8_t buf[64] = {};
  uint8_t n = 0;

  static const uint16_t kPriorityDids[] = {
      0xF180, 0xF181, 0xF182, 0xF186, 0xF187, 0xF188, 0xF189, 0xF18A, 0xF18C,
      0xF190, 0xF191, 0xF194, 0xF195, 0xF197, 0xF19E, 0xF1A0, 0xF1A2, 0xF1AA,
      0xF1DF,
  };

  for (uint8_t i = 0; i < sizeof(kPriorityDids) / sizeof(kPriorityDids[0]); i++) {
    const uint16_t did = kPriorityDids[i];
    snprintf(gObdState.ecuScanLabel, sizeof(gObdState.ecuScanLabel), "Engine DID %04X",
             did);
    pumpScan();
    if (readDid(cfg, did, buf, n, sizeof(buf), kIdentTimeoutMs)) {
      storeDid(did, buf, n);
      if (did == 0xF190 && gObdState.vin[0] == '\0') {
        sanitizeIdent(buf, n, gObdState.vin, sizeof(gObdState.vin));
      }
    }
  }

  for (uint16_t did = kDidScanStart; did <= kDidScanEnd; did++) {
    if (didAlreadyStored(did) || gObdState.engineDidCount >= kMaxDidDump) {
      continue;
    }
    if ((did & 0x07) == 0) {
      snprintf(gObdState.ecuScanLabel, sizeof(gObdState.ecuScanLabel), "Engine DID %04X",
               did);
      pumpScan();
    }
    if (readDid(cfg, did, buf, n, sizeof(buf), kDidTimeoutMs)) {
      storeDid(did, buf, n);
    }
  }
}

}  // namespace

void performUdsRead(uint8_t moduleId) {
  const UdsModuleConfig &cfg = kUdsModules[moduleId];
  UdsModuleState &mod = gObdState.udsModules[moduleId];

  mod.status = DTC_READING;
  mod.dtcCount = 0;
  broadcastObdState();
  readModuleDtcs(moduleId);
  Serial.printf("%s: read %u DTC(s)\n", cfg.key, mod.dtcCount);
  broadcastObdState();
}

void performUdsClear(uint8_t moduleId) {
  const UdsModuleConfig &cfg = kUdsModules[moduleId];
  UdsModuleState &mod = gObdState.udsModules[moduleId];

  mod.status = DTC_READING;
  broadcastObdState();

  const uint8_t session[] = {0x10, 0x03};
  uint8_t scratch[16] = {};
  udsRequest(cfg.reqId, cfg.respId, session, sizeof(session), scratch, sizeof(scratch),
             500);

  const uint8_t req[] = {0x14, 0xFF, 0xFF, 0xFF};
  uint8_t resp[16] = {};
  const int n = udsRequest(cfg.reqId, cfg.respId, req, sizeof(req), resp, sizeof(resp),
                           1500);

  if (n >= 1 && resp[0] == 0x54) {
    mod.dtcCount = 0;
    mod.status = DTC_CLEARED;
    Serial.printf("%s: DTCs cleared\n", cfg.key);
  } else {
    mod.status = DTC_ERROR;
  }
  broadcastObdState();
}

void performEcuScan() {
  gObdState.ecuScanStatus = SCAN_RUNNING;
  gObdState.ecuScanFound = 0;
  gObdState.engineDidCount = 0;
  gObdState.vin[0] = '\0';
  setScanLabel("Starting…");
  for (uint8_t i = 0; i < UDS_MODULE_COUNT; i++) {
    UdsModuleState &mod = gObdState.udsModules[i];
    mod.probed = false;
    mod.present = false;
    mod.dtcCount = 0;
    mod.status = DTC_IDLE;
    mod.partNumber[0] = '\0';
    mod.swVersion[0] = '\0';
    mod.hwNumber[0] = '\0';
    mod.sysName[0] = '\0';
  }
  pumpScan();

  for (uint8_t i = 0; i < UDS_MODULE_COUNT; i++) {
    const UdsModuleConfig &cfg = kUdsModules[i];
    UdsModuleState &mod = gObdState.udsModules[i];
    snprintf(gObdState.ecuScanLabel, sizeof(gObdState.ecuScanLabel), "%02X %s",
             cfg.vagAddr, cfg.name);
    pumpScan();

    const bool present = probeModule(cfg);
    mod.probed = true;
    mod.present = present;
    if (!present) {
      Serial.printf("UDS %02X %s: absent (0x%lX)\n", cfg.vagAddr, cfg.name, cfg.reqId);
      continue;
    }

    gObdState.ecuScanFound++;
    gObdState.busActive = true;
    readIdent(i);
    readModuleDtcs(i);
    Serial.printf("UDS %02X %s present  %s  sw=%s  dtcs=%u\n", cfg.vagAddr, cfg.name,
                  mod.partNumber[0] ? mod.partNumber : "-",
                  mod.swVersion[0] ? mod.swVersion : "-", mod.dtcCount);
  }

  if (gObdState.udsModules[UDS_MOD_ENGINE].present) {
    setScanLabel("Engine DID dump");
    pumpScan();
    dumpEngineDids();
  }

  gObdState.cmdScanEcus = false;
  gObdState.ecuScanStatus = SCAN_DONE;
  snprintf(gObdState.ecuScanLabel, sizeof(gObdState.ecuScanLabel), "%u module%s",
           gObdState.ecuScanFound, gObdState.ecuScanFound == 1 ? "" : "s");
  Serial.printf("ECU scan complete: %u present, %u engine DIDs\n", gObdState.ecuScanFound,
                gObdState.engineDidCount);
  broadcastObdState();
}
