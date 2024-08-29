#ifndef HW_ARM_STM32F429_SOC_H
#define HW_ARM_STM32F429_SOC_H

#include "hw/misc/stm32f4xx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/adc/stm32f2xx_adc.h"
#include "hw/misc/stm32f4xx_exti.h"
#include "hw/or-irq.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/gpio/stm32f429_gpio.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "qemu/units.h"

#define TYPE_STM32F429_SOC "stm32f429-soc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F429State, STM32F429_SOC)

#define STM_NUM_USARTS 7
#define STM_NUM_TIMERS 4
#define STM_NUM_ADCS 5
#define STM_NUM_SPIS 6
#define STM_NUM_GPIO 11

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (2 * MiB)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (192 * KiB)
#define CCM_BASE_ADDRESS 0x10000000
#define CCM_SIZE (64 * KiB)

struct STM32F429State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    STM32F4xxSyscfgState syscfg;
    STM32F4xxExtiState exti;
    STM32F2XXUsartState usart[STM_NUM_USARTS];
    STM32F2XXTimerState timer[STM_NUM_TIMERS];
    OrIRQState adc_irqs;
    STM32F2XXADCState adc[STM_NUM_ADCS];
    STM32F2XXSPIState spi[STM_NUM_SPIS];
    STM32F429GpioState gpio[STM_NUM_GPIO];

    MemoryRegion ccm;
    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;

    Clock *sysclk;
    Clock *refclk;
};

#endif // HW_ARM_STM32F429_SOC_H
