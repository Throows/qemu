/*
 * ESP-PSRAM basic emulation
 *
 * Copyright (c) 2021-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/qdev-properties.h"
#include "hw/misc/ssi_psram.h"

#define CMD_READ_ID 0x9f
#define PSRAM_ID_MFG 0x0d
#define PSRAM_ID_KGD 0x5d

#define CMD_READ_OCTAL_REG 0x4040
#define CMD_WRITE_OCTAL_REG 0xc0c0

#define MR0_DRIVE_STRENGHT_HALF     ((uint8_t)0b01  << 0)
#define MR0_RD_LATENCY_CODE         ((uint8_t)0b010 << 2)
#define MR0_RD_LT_VARIABLE          ((uint8_t)0b0   << 5)

#define MR1_VENDOR_ID               ((uint8_t)0b01101 << 0)
#define MR1_NO_ULP                  ((uint8_t)0b0     << 5)

#define MR2_DENSITY_MAPPING_64MB    ((uint8_t)0b011   << 0)
#define MR2_DEVICE_ID_3_GEN         ((uint8_t)0b10    << 3)
#define MR2_GOOD_DIE_BIT_PASS       ((uint8_t)0b1     << 7)

#define MR3_SRF_FAST_REFRESH        ((uint8_t)0b1     << 5)
#define MR3_OP_VOLTAGE_1V8          ((uint8_t)0b0     << 6)
#define MR3_RBX_NOT_SUPPORTED       ((uint8_t)0b0     << 7)

#define MR4_PASR_64MB               ((uint8_t)0b000   << 0)
#define MR4_FAST_REFRESH            ((uint8_t)0b0     << 3)
#define MR4_WRITE_LATENCY_5         ((uint8_t)0b010   << 4)

#define MR6_ULP_HALF_SLEEP          ((uint8_t)0xF0    << 0)

#define MR8_32BYTE_BURST            ((uint8_t)0b01    << 0)
#define MR8_HYBRID_BURST            ((uint8_t)0b1     << 2)
#define MR8_RBX_READ_DISABLE        ((uint8_t)0b0     << 3)

#define FAKE_16MB_ID    0x6a
#define FAKE_32MB_ID    0x8e

static int get_eid_by_size(uint32_t size_mbytes) {
    switch (size_mbytes)
    {
    case 2:
        return 0x00;
    case 4:
        return 0x21;
    case 8:
        return 0x40;
    case 16:
        return FAKE_16MB_ID;
    case 32:
        return FAKE_32MB_ID;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: PSRAM size %" PRIu32 "MB not implemented\n",
                      __func__, size_mbytes);
        return 0;
    }
}

static uint32_t psram_quad_read(SsiPsramState *s)
{
    uint32_t result = 0;
    if (s->command == CMD_READ_ID) {
        uint8_t read_id_response[] = {
            0x00, 0x00, 0x00, 0x00,
            0x00, PSRAM_ID_MFG, PSRAM_ID_KGD,
            get_eid_by_size(s->size_mbytes),
            0xaa, 0xbb, 0xcc, 0xdd, 0xee
        };
        int index = s->byte_count - s->dummy;
        if (index < ARRAY_SIZE(read_id_response)) {
            result = read_id_response[index];
        }
    }
    return result;
}

static void psram_quad_write(SsiPsramState *s, uint32_t value)
{
    if (s->byte_count == s->dummy) {
        s->command = value;
    }
    s->byte_count++;
}

static uint32_t psram_octal_read(SsiPsramState *s)
{
    uint32_t result = 0;
    if (s->byte_count < 7) return result;

    if (s->command == CMD_READ_OCTAL_REG) {  
        // Odd read bytes correspond to the next register
        switch (s->addr)
        {
        case 0:
            result = (s->byte_count % 2) ? s->mr1 : s->mr0;
            break;
        case 1:
            result = (s->byte_count % 2) ? s->mr2 : s->mr1;
            break;
        case 2:
            result = (s->byte_count % 2) ? s->mr3 : s->mr2;
            break;
        case 3:
            result = (s->byte_count % 2) ? s->mr4 : s->mr3;
            break;
        case 4:
            result = (s->byte_count % 2) ? s->mr8 : s->mr4;
            break;
        case 8:
            result = (s->byte_count % 2) ? s->mr0 : s->mr8;
            break;
        default:
            // Should not happen
            break;
        }
    }
    return result;
}

static void psram_octal_write(SsiPsramState *s, uint32_t value)
{
    // Save the transfer to the state
    if (s->byte_count == 0) {
        s->command = value << s->byte_count;       
    } else if (s->byte_count == 1) {
        s->command |= value << (s->byte_count * 8);
    } else if (s->byte_count == (s->dummy + 2)) {       // addr
        s->addr = value;                       
    }
    s->byte_count++;

    // save the write command to the corresponding register
    if (s->byte_count == 7 && s->command == CMD_WRITE_OCTAL_REG) {
        switch (s->addr)
        {
        case 0:
            s->mr0 = value;
            break;
        case 4:
            s->mr4 = value;
            break;
        case 8:
            s->mr8 = value;
            break;
        default:
            // Should not happen
            break;
        }
    }
}

static uint32_t psram_transfer(SSIPeripheral *dev, uint32_t value)
{
    SsiPsramState *s = SSI_PSRAM(dev);
    if (s->is_octal) {
        psram_octal_write(s, value);
        return psram_octal_read(s);
    } else {
        psram_quad_write(s, value);
        return psram_quad_read(s);
    }
}

static int psram_cs(SSIPeripheral *ss, bool select)
{
    SsiPsramState *s = SSI_PSRAM(ss);

    if (!select) {
        s->byte_count = 0;
        s->command = -1;
        s->addr = -1;
    }
    return 0;
}

static void psram_realize(SSIPeripheral *ss, Error **errp)
{
    SsiPsramState *s = SSI_PSRAM(ss);
    /* Make sure the given size is supported */
    if (get_eid_by_size(s->size_mbytes) == 0) {
        error_report("[PSRAM] Invalid size %dMB for the PSRAM", s->size_mbytes);
    }

    /* Set the default MR values for the octal psram (ref: Datasheet APS6408L_OBMx) */
    s->mr0 = MR0_RD_LT_VARIABLE | MR0_RD_LATENCY_CODE | MR0_DRIVE_STRENGHT_HALF;
    s->mr1 = MR1_NO_ULP | MR1_VENDOR_ID;
    s->mr2 = MR2_GOOD_DIE_BIT_PASS | MR2_DEVICE_ID_3_GEN | MR2_DENSITY_MAPPING_64MB;   
    s->mr3 = MR3_RBX_NOT_SUPPORTED | MR3_OP_VOLTAGE_1V8 | MR3_SRF_FAST_REFRESH;
    s->mr4 = MR4_WRITE_LATENCY_5 | MR4_FAST_REFRESH | MR4_PASR_64MB;
    s->mr8 = MR8_RBX_READ_DISABLE | MR8_HYBRID_BURST | MR8_32BYTE_BURST;

    if (s->is_octal) {
        s->dummy = 3;
    }

    /* Allocate the actual array that will act as a vritual RAM */
    const uint32_t size_bytes = s->size_mbytes * 1024 * 1024;
    memory_region_init_ram(&s->data_mr, OBJECT(s), "psram.memory_region", size_bytes, &error_fatal);
}

static Property psram_properties[] = {
    DEFINE_PROP_BOOL("is_octal", SsiPsramState, is_octal, false),
    DEFINE_PROP_UINT32("size_mbytes", SsiPsramState, size_mbytes, 4),
    DEFINE_PROP_UINT32("dummy", SsiPsramState, dummy, 1),
    DEFINE_PROP_END_OF_LIST(),
};


static void psram_class_init(ObjectClass *klass, void *data)
{
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->transfer = psram_transfer;
    k->set_cs = psram_cs;
    k->cs_polarity = SSI_CS_LOW;
    k->realize = psram_realize;
    device_class_set_props(dc, psram_properties);
}

static const TypeInfo psram_info = {
    .name          = TYPE_SSI_PSRAM,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SsiPsramState),
    .class_init    = psram_class_init
};

static void psram_register_types(void)
{
    type_register_static(&psram_info);
}

type_init(psram_register_types)
