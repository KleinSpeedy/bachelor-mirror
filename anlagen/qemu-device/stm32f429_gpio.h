#ifndef STM32F429_GPIO_H
#define STM32F429_GPIO_H

#include "hw/sysbus.h"
#include "qom/object.h"

/* GPIO Memory range */
#define STM32F429_GPIO_MMIO_SIZE 0x400
#define STM32F429_GPIO_NUM_PINS 16

#define TYPE_STM32F429_GPIO "stm32f429_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F429GpioState, STM32F429_GPIO)

/**
 * @brief Locking config registers through LCKR is supported and needs a
 * special locking sequence, see TRM 8.4.8 for details
 */
typedef enum Stm32F429GpioLockSequence {
    LS_WRITE_ONE = 0,   /* first write in locking sequence occured */
    LS_WRITE_TWO,       /* second write in locking sequence occured */
    LS_WRITE_THREE,     /* third write in locking sequence occured */
    LS_READ,            /* read in locking sequence occured, lock succesfull */
    LS_LOCKED           /* lckr lock is active */
} Stm32F429GpioLockSequence;

/**
 * @brief Type represents a GPIO module in STM32F429 microcontroller
 */
struct STM32F429GpioState
{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    struct MemoryRegion mmio;

    /* GPIO registers */
    /* Configuration registers */
    uint32_t moder;
    uint32_t otyper;
    uint32_t pupdr;
    uint32_t ospeedr;
    /* Data registers */
    uint32_t idr;
    uint32_t odr;
    /* Bit set/reset register */
    uint32_t bsrr;
    /* Locking register */
    uint32_t lckr;
    /* Alternate function (high/low) register */
    uint32_t afhr;
    uint32_t aflr;

    /* GPIO ports A and B have different reset values for MODER, OTYPER and
     * OSPEEDR, so we need to configure them for each port individually */
    uint32_t moder_reset_val;
    uint32_t ospeedr_reset_val;
    uint32_t pupdr_reset_val;

    /* current state in locking sequence */
    Stm32F429GpioLockSequence lock_state;

    qemu_irq pin_irqs[STM32F429_GPIO_NUM_PINS];
};

#endif // STM32F429_GPIO_H
