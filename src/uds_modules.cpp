#include "uds_modules.h"

#include "config.h"

// Physical request/response pairs used on MQB. Engine/gearbox keep the
// ISO 15765-4 OBD IDs; everything else is VAG UDS (response = request + 0x6A)
// with a few well-known exceptions (BCM 0x770, HVAC 0x73C, MIB 0x773).
const UdsModuleConfig kUdsModules[UDS_MODULE_COUNT] = {
    {"engine", "Engine", 0x01, 0x7E0, 0x7E8},
    {"gearbox", "Gearbox", 0x02, 0x7E1, 0x7E9},
    {"abs", "ABS / ESP", 0x03, ABS_UDS_REQUEST_ID, ABS_UDS_RESPONSE_ID},
    {"hvac", "HVAC", 0x08, 0x73C, 0x7A6},
    {"bcm", "Central electrics", 0x09, 0x770, 0x7DA},
    {"acc", "ACC", 0x13, 0x757, 0x7C1},
    {"airbag", "Airbag / SRS", 0x15, AIRBAG_UDS_REQUEST_ID, AIRBAG_UDS_RESPONSE_ID},
    {"cluster", "Instruments", 0x17, 0x714, 0x77E},
    {"gateway", "Gateway", 0x19, 0x710, 0x77A},
    {"eps", "Steering assist", 0x44, 0x712, 0x77C},
    {"infotainment", "Infotainment", 0x5F, 0x773, 0x7DD},
};
