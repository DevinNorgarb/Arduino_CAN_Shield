#pragma once

#include <Arduino.h>

// Non-OBD ECUs we can read/clear over UDS. Adding a module is one table entry
// (see uds_modules.cpp) plus a dashboard section using the same "<key>" prefix.

enum UdsModuleId : uint8_t {
  UDS_MOD_ABS = 0,
  UDS_MOD_AIRBAG,
  UDS_MODULE_COUNT,
};

struct UdsModuleConfig {
  const char *key;       // JSON/command prefix, e.g. "abs" -> read_abs / abs_dtcs
  unsigned long reqId;   // UDS physical request CAN ID
  unsigned long respId;  // UDS response CAN ID
};

extern const UdsModuleConfig kUdsModules[UDS_MODULE_COUNT];
