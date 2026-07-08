#include "uds_modules.h"

#include "config.h"

const UdsModuleConfig kUdsModules[UDS_MODULE_COUNT] = {
    {"abs", ABS_UDS_REQUEST_ID, ABS_UDS_RESPONSE_ID},
    {"airbag", AIRBAG_UDS_REQUEST_ID, AIRBAG_UDS_RESPONSE_ID},
};
