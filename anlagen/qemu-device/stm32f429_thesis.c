#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "qemu/error-report.h"
#include "hw/arm/stm32f429_soc.h"
#include "hw/arm/boot.h"

/*
 * Thesis test implementation for stm32f429 soc
 */

/* Main SYSCLK frequency in Hz (168MHz) */
#define SYSCLK_FRQ 168000000ULL

static void stm32_f429_thesis_init(MachineState *machine)
{
    DeviceState *dev;
    Clock *sysclk;

    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    dev = qdev_new(TYPE_STM32F429_SOC);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       0, FLASH_SIZE);
}

static void stm32_f429_thesis_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };

    mc->desc = "SMT32F429 Thesis test device (Cortex-m4)";
    mc->init = stm32_f429_thesis_init;
    mc->valid_cpu_types = valid_cpu_types;

    /* SRAM pre-allocated as part of the SoC instantiation */
    mc->default_ram_size = 0;
}

DEFINE_MACHINE("stm32f429-thesis-device", stm32_f429_thesis_machine_init)
