#pragma once

#include <Arduino.h>

// VAG modules we can probe over UDS on the OBD diagnostic CAN (MQB-A0 Polo).
// Adding a module is one table entry. Dedicated dashboard DTC buttons exist
// for abs / airbag; the ECU scanner lists every entry.

enum UdsModuleId : uint8_t {
  UDS_MOD_ENGINE = 0,
  UDS_MOD_GEARBOX,
  UDS_MOD_ABS,
  UDS_MOD_HVAC,
  UDS_MOD_BCM,
  UDS_MOD_ACC,
  UDS_MOD_AIRBAG,
  UDS_MOD_CLUSTER,
  UDS_MOD_GATEWAY,
  UDS_MOD_EPS,
  UDS_MOD_INFOTAINMENT,
  UDS_MODULE_COUNT,
};

struct UdsModuleConfig {
  const char *key;       // JSON/command prefix, e.g. "abs" -> read_abs / abs_dtcs
  const char *name;      // dashboard label
  uint8_t vagAddr;       // VCDS-style address (01 engine, 03 ABS, ...)
  unsigned long reqId;   // UDS physical request CAN ID
  unsigned long respId;  // UDS response CAN ID
};

extern const UdsModuleConfig kUdsModules[UDS_MODULE_COUNT];
