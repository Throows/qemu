#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "exec/memory.h"

typedef struct SsiPsramState {
    SSIPeripheral parent_obj;
    uint32_t size_mbytes;
    uint32_t dummy;
    int command;
    int addr;
    int byte_count;
    bool is_octal;

    uint8_t mr0;
    uint8_t mr1;
    uint8_t mr2;
    uint8_t mr3;
    uint8_t mr4;
    uint8_t mr8;

    MemoryRegion data_mr;
} SsiPsramState;

#define TYPE_SSI_PSRAM "ssi_psram"
OBJECT_DECLARE_SIMPLE_TYPE(SsiPsramState, SSI_PSRAM)

