/*
 * ESP32 SoC and machine
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "target/xtensa/cpu.h"
#include "hw/misc/esp32_reg.h"
#include "hw/char/esp32_uart.h"
#include "hw/misc/esp32_gpio.h"
#include "hw/misc/esp32_dport.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "elf.h"

#define TYPE_ESP32_SOC "xtensa.esp32"
#define ESP32_SOC(obj) OBJECT_CHECK(Esp32SocState, (obj), TYPE_ESP32_SOC)

#define TYPE_ESP32_CPU XTENSA_CPU_TYPE_NAME("esp32")

typedef struct XtensaCPU XtensaCPU;


enum {
    ESP32_MEMREGION_IROM,
    ESP32_MEMREGION_DROM,
    ESP32_MEMREGION_DRAM,
    ESP32_MEMREGION_IRAM,
    ESP32_MEMREGION_ICACHE0,
    ESP32_MEMREGION_ICACHE1,
    ESP32_MEMREGION_RTCSLOW,
    ESP32_MEMREGION_RTCFAST_D,
    ESP32_MEMREGION_RTCFAST_I,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32_memmap[] = {
    [ESP32_MEMREGION_DROM] = { 0x3ff90000, 0x10000 },
    [ESP32_MEMREGION_IROM] = { 0x40000000, 0x70000 },
    [ESP32_MEMREGION_DRAM] = { 0x3ffae000, 0x52000 },
    [ESP32_MEMREGION_IRAM] = { 0x40080000, 0x20000 },
    [ESP32_MEMREGION_ICACHE0] = { 0x40070000, 0x8000 },
    [ESP32_MEMREGION_ICACHE1] = { 0x40078000, 0x8000 },
    [ESP32_MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_I] = { 0x400C0000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_D] = { 0x3ff80000, 0x2000 },
};


#define ESP32_SOC_RESET_PROCPU    0x1
#define ESP32_SOC_RESET_APPCPU    0x2
#define ESP32_SOC_RESET_PERIPH    0x4
#define ESP32_SOC_RESET_DIG       (ESP32_SOC_RESET_PROCPU | ESP32_SOC_RESET_APPCPU | ESP32_SOC_RESET_PERIPH)
#define ESP32_SOC_RESET_RTC       0x8
#define ESP32_SOC_RESET_ALL       (ESP32_SOC_RESET_RTC | ESP32_SOC_RESET_DIG)


typedef struct Esp32SocState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    XtensaCPU cpu[ESP32_CPU_COUNT];
    Esp32DportState dport;
    ESP32UARTState uart[ESP32_UART_COUNT];
    Esp32GpioState gpio;
    Esp32RtcCntlState rtc_cntl;

    uint32_t requested_reset;
} Esp32SocState;



static void esp32_dig_reset(void *opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = ESP32_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_soc_reset(DeviceState *dev)
{
    Esp32SocState *s = ESP32_SOC(dev);

    if (s->requested_reset == 0) {
        s->requested_reset = ESP32_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32_SOC_RESET_RTC) {
        device_reset(DEVICE(&s->rtc_cntl));
    }
    if (s->requested_reset & ESP32_SOC_RESET_PERIPH) {
        device_reset(DEVICE(&s->dport));
        device_reset(DEVICE(&s->gpio));
        for (int i = 0; i < ESP32_UART_COUNT; ++i) {
            device_reset(DEVICE(&s->uart));
        }
    }
    if (s->requested_reset & ESP32_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    if (s->requested_reset & ESP32_SOC_RESET_APPCPU) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    s->requested_reset = 0;
}

static void esp32_cpu_stall(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);

    bool stall;
    if (n == 0) {
        stall = s->rtc_cntl.cpu_stall_state[0];
    } else {
        stall = s->rtc_cntl.cpu_stall_state[1] && s->dport.appcpu_stall_state;
    }

    xtensa_runstall(&s->cpu[n].env, stall);
}

static void esp32_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32SocState *s = ESP32_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    const struct MemmapEntry *memmap = esp32_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *drom = g_new(MemoryRegion, 1);
    MemoryRegion *irom = g_new(MemoryRegion, 1);
    MemoryRegion *icache0 = g_new(MemoryRegion, 1);
    MemoryRegion *icache1 = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_i = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_d = g_new(MemoryRegion, 1);

    memory_region_init_rom(irom, NULL, "esp32.irom",
                           memmap[ESP32_MEMREGION_IROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_IROM].base, irom);

    memory_region_init_alias(drom, NULL, "esp32.drom", irom, 0x60000, memmap[ESP32_MEMREGION_DROM].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_DROM].base, drom);

    memory_region_init_ram(dram, NULL, "esp32.dram",
                           memmap[ESP32_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_DRAM].base, dram);

    memory_region_init_ram(iram, NULL, "esp32.iram",
                           memmap[ESP32_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(icache0, NULL, "esp32.icache0",
                           memmap[ESP32_MEMREGION_ICACHE0].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE0].base, icache0);

    memory_region_init_ram(icache1, NULL, "esp32.icache1",
                           memmap[ESP32_MEMREGION_ICACHE1].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE1].base, icache1);

    memory_region_init_ram(rtcslow, NULL, "esp32.rtcslow",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_RTCSLOW].base, rtcslow);

    memory_region_init_ram(rtcfast_i, NULL, "esp32.rtcfast_i",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_RTCFAST_I].base, rtcfast_i);

    memory_region_init_alias(rtcfast_d, NULL, "esp32.rtcfast_d", rtcfast_i, 0, memmap[ESP32_MEMREGION_RTCFAST_D].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_RTCFAST_D].base, rtcfast_d);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        const uint32_t cpuid[ESP32_CPU_COUNT] = { 0xcdcd, 0xabab };
        object_property_set_bool(OBJECT(&s->cpu[i]), true, "realized", &error_abort);
        s->cpu[i].env.sregs[PRID] = cpuid[i];
    }

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE};
        object_property_set_bool(OBJECT(&s->uart[i]), true, "realized", &error_abort);

        MemoryRegion *uart = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->uart[i]), 0);
        memory_region_add_subregion_overlap(sys_mem, uart_base[i], uart, 0);
    }

    object_property_set_bool(OBJECT(&s->gpio), true, "realized", &error_abort);

    MemoryRegion *gpio = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->gpio), 0);
    memory_region_add_subregion_overlap(sys_mem, DR_REG_GPIO_BASE, gpio, 0);

    object_property_set_bool(OBJECT(&s->dport), true, "realized", &error_abort);

    MemoryRegion *dport = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dport), 0);
    memory_region_add_subregion(sys_mem, DR_REG_DPORT_BASE, dport);

    object_property_set_bool(OBJECT(&s->rtc_cntl), true, "realized", &error_abort);

    MemoryRegion *rtc_cntl = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->rtc_cntl), 0);
    memory_region_add_subregion(sys_mem, DR_REG_RTCCNTL_BASE, rtc_cntl);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_DIG_RESET_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_RESET_GPIO, i,
                                        qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_STALL_GPIO, i,
                                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, i));
    }

    create_unimplemented_device("esp32.frc0", DR_REG_FRC_TIMER_BASE, 0x1000);
    create_unimplemented_device("esp32.rtcio", DR_REG_RTCIO_BASE, 0x400);
    create_unimplemented_device("esp32.uart1", DR_REG_UART1_BASE, 0x1000);
    create_unimplemented_device("esp32.tg0", DR_REG_TIMERGROUP0_BASE, 0x1000);
    create_unimplemented_device("esp32.tg1", DR_REG_TIMERGROUP1_BASE, 0x1000);
    create_unimplemented_device("esp32.efuse", DR_REG_EFUSE_BASE, 0x1000);
    create_unimplemented_device("esp32.iomux", DR_REG_IO_MUX_BASE, 0x2000);
    create_unimplemented_device("esp32.hinf", DR_REG_HINF_BASE, 0x1000);
    create_unimplemented_device("esp32.slc", DR_REG_SLC_BASE, 0x1000);
    create_unimplemented_device("esp32.slchost", DR_REG_SLCHOST_BASE, 0x1000);
    create_unimplemented_device("esp32.spi0", DR_REG_SPI0_BASE, 0x1000);
    create_unimplemented_device("esp32.spi1", DR_REG_SPI1_BASE, 0x1000);
    create_unimplemented_device("esp32.spi2", DR_REG_SPI2_BASE, 0x1000);
    create_unimplemented_device("esp32.spi3", DR_REG_SPI3_BASE, 0x1000);
    create_unimplemented_device("esp32.apbctrl", DR_REG_APB_CTRL_BASE, 0x1000);

    qemu_register_reset((QEMUResetHandler*) esp32_soc_reset, dev);
}

static void esp32_soc_init(Object *obj)
{
    Esp32SocState *s = ESP32_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());

    for (int i = 0; i < ms->smp.cpus; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_initialize_child(obj, name, &s->cpu[i], sizeof(s->cpu[i]), TYPE_ESP32_CPU, &error_abort, NULL);
    }

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], sizeof(s->uart[i]),
                                TYPE_ESP32_UART, &error_abort, NULL);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart), "chardev",
                              &error_abort);

    object_initialize_child(obj, "gpio", &s->gpio, sizeof(s->gpio),
                                TYPE_ESP32_GPIO, &error_abort, NULL);

    object_initialize_child(obj, "dport", &s->dport, sizeof(s->dport),
                            TYPE_ESP32_DPORT, &error_abort, NULL);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, sizeof(s->rtc_cntl),
                            TYPE_ESP32_RTC_CNTL, &error_abort, NULL);

    qdev_init_gpio_in_named(DEVICE(s), esp32_dig_reset, ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_reset, ESP32_RTC_CPU_RESET_GPIO, 2);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_stall, ESP32_RTC_CPU_STALL_GPIO, 2);
}

static Property esp32_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_soc_reset;
    dc->realize = esp32_soc_realize;
    dc->props = esp32_soc_properties;
}

static const TypeInfo esp32_soc_info = {
    .name = TYPE_ESP32_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SocState),
    .instance_init = esp32_soc_init,
    .class_init = esp32_soc_class_init
};

static void esp32_soc_register_types(void)
{
    type_register_static(&esp32_soc_info);
}

type_init(esp32_soc_register_types)


static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

static void esp32_machine_inst_init(MachineState *machine)
{
    Esp32SocState *s = g_new0(Esp32SocState, 1);

    object_initialize_child(OBJECT(machine), "soc", s, sizeof(*s),
                            TYPE_ESP32_SOC, &error_abort, NULL);
    object_property_set_bool(OBJECT(s), true, "realized", &error_abort);


    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&s->cpu[0]));

    if (machine->kernel_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(machine->kernel_filename, NULL,
                               translate_phys_addr, &s->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, 0, EM_XTENSA, 0, 0);
        if (success > 0) {
            s->cpu[0].env.pc = elf_entry;
        }
    }
}

/* Initialize machine type */
static void esp32_machine_init(MachineClass *mc)
{
    mc->desc = "Espressif ESP32 machine";
    mc->init = esp32_machine_inst_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
}

DEFINE_MACHINE("esp32", esp32_machine_init)

