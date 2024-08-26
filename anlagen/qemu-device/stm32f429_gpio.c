/*
 * STM32F429 GPIO Controller - Thesis implementation
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/gpio/stm32f429_gpio.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "hw/resettable.h"
#include "trace.h"

static const char * const type_desc = "STM32F420 Gpio module";

// register reset value unless otherwise specified by ref-manual
#define DEFAULT_RESET_VALUE 0x00000000ul

#define UPPER_WORD_MASK 0xFFFF0000ul
#define LOWER_WORD_MASK 0x0000FFFFul

typedef enum GpioPinMode {
    PINMODE_UNDEFINED = 0,
    PINMODE_IN_FLOAT,       /* Input floating */
    PINMODE_IN_PD,          /* Input pull down */
    PINMODE_IN_PU,          /* Input pull up*/
    PINMODE_OUT_OD_FLOAT,   /* output open-drain floating */
    PINMODE_OUT_OD_PD,      /* output open-drain pull down */
    PINMODE_OUT_OD_PU,      /* output open-drain pull up */
    PINMODE_OUT_PP_FLOAT,   /* output push-pull floating */
    PINMODE_OUT_PP_PD,      /* output push-pull pull down */
    PINMODE_OUT_PP_PU       /* output push-pull pull up */
} GpioPinMode;

/*
 * 32bit registers contain 16 pins represented by two bits each
 */
static inline uint8_t get_pin_val_32(const uint32_t *r, uint8_t pin)
{
    return ((*r) & (1 << (pin * 2 + 1))) | ((*r) & (1 << (pin * 2)));
}

static inline uint8_t get_pin_val_16(const uint32_t *r, uint8_t pin)
{
    return (*r) & (1 << pin);
}

static inline GpioPinMode get_pin_mode(STM32F429GpioState *s, uint8_t pin)
{
    assert(pin < STM32F429_GPIO_NUM_PINS);

    const uint8_t moder_pin = get_pin_val_32(&s->moder, pin);
    const uint8_t pupdr_pin = get_pin_val_32(&s->pupdr, pin);
    const uint8_t otyper_pin = get_pin_val_16(&s->otyper, pin);

    // extracting pin mode as described in table 35 trm
    if(moder_pin == 0)
    {
        if(pupdr_pin > 0)
        {
            return (pupdr_pin == 1) ? PINMODE_IN_PU : PINMODE_IN_PD;
        }
        else
        {
            return PINMODE_IN_FLOAT;
        }
    }
    else if(moder_pin == 1)
    {
        if(otyper_pin == 0)
        {
            if(pupdr_pin > 0)
            {
                return (pupdr_pin == 1) ? PINMODE_OUT_PP_PU : PINMODE_OUT_PP_PD;
            }
            else
            {
                return PINMODE_OUT_PP_FLOAT;
            }
        }
        else
        {
            if(pupdr_pin > 0)
            {
                return (pupdr_pin == 1) ? PINMODE_OUT_OD_PU : PINMODE_OUT_OD_PD;
            }
            else
            {
                return PINMODE_OUT_OD_FLOAT;
            }
        }
    }
    // alternate function and analog mode are not supported
    return PINMODE_UNDEFINED;
}

// container for easier access to LCK register
struct LockRegister {
    uint8_t lckk : 1;   // lock key bit
    uint16_t lckconf;   // pin lock configuration
};

/*
 * Combine LCKK bit and pin lock configuration from LockRegister type
 */
static inline uint32_t extract_u32_from_lockregister(struct LockRegister lr)
{
    // bits 17..31 must be kept at reset value
    return (uint32_t)(0x00000000 | ((lr.lckk << 16) | lr.lckconf));
}

/*
 * Only write control register if they're not freezed by LCKR
 */
static inline void write_control_register(STM32F429GpioState *s, uint32_t *reg,
                                            uint32_t value)
{
    if(!(s->lock_state == LS_LOCKED))
    {
        *reg = value;
    }
}

// Offsets of GPIO Module registers
typedef enum StmGpioRegOffset {
    OFFSET_MODER = 0x00,
    OFFSET_OTYPER = 0x04,
    OFFSET_OSPEEDR = 0x08,
    OFFSET_PUPDR = 0x0C,
    OFFSET_IDR = 0x10,
    OFFSET_ODR = 0x14,
    OFFSET_BSRR = 0x18,
    OFFSET_LCKR = 0x1C,
    OFFSET_AFRL = 0x20,
    OFFSET_AFRH = 0x24
} StmGpioRegOffset;

/* The lock sequence is described in TRM 8.4.8 as follows:
 * 1. write LCKR[16] = 1 + LCKR[0..15]
 * 2. write LCKR[16] = 0 + LCKR[0..15]
 * 3. write LCKR[16] = 1 + LCKR[0..15]
 * 4. read LCKR
 * LCKR[16] is the lock key (LCKK)
 * LCKR[0..15] is the pin configuration which must not change
 */
static void update_lckr(STM32F429GpioState *s, uint32_t value,
                                 bool is_read)
{
    if(!is_read)
    {
        if((s->lock_state != LS_WRITE_ONE) && !(s->lock_state == LS_READ) &&
        ((s->lckr & LOWER_WORD_MASK) != (value & LOWER_WORD_MASK)))
        {
            // abort configuration
            s->lock_state = LS_WRITE_ONE;
            return;
        }
    }

    const struct LockRegister temp_lckr = {
        .lckk = (value & 1 << 16),
        .lckconf = (value & LOWER_WORD_MASK),
    };

    switch(s->lock_state)
    {
        case LS_WRITE_ONE:
        {
            if(temp_lckr.lckk != 1 || is_read)
            {
                break;
            }

            s->lckr = extract_u32_from_lockregister(temp_lckr);
            s->lock_state = LS_WRITE_TWO;
            break;
        }
        case LS_WRITE_TWO:
        {
            if(temp_lckr.lckk != 0 || is_read)
            {
                s->lock_state = LS_WRITE_ONE;
                break;
            }

            s->lckr = extract_u32_from_lockregister(temp_lckr);
            s->lock_state = LS_WRITE_THREE;
            break;
        }
        case LS_WRITE_THREE:
        {
            if(temp_lckr.lckk != 1 || is_read)
            {
                s->lock_state = LS_WRITE_ONE;
                break;
            }

            s->lckr = extract_u32_from_lockregister(temp_lckr);
            s->lock_state = LS_READ;
            break;
        }
        case LS_READ:
        {
            if(!is_read)
            {
                s->lock_state = LS_WRITE_ONE;
                break;
            }
            // locking sequence succesfull, all steps done
            s->lock_state = LS_LOCKED;
            break;
        }
        case LS_LOCKED:
        {
            /* lock state can only be changed by reset after locking sequence
             * is completed */
            break;
        }
    }
}

/* Read from the memory region. @addr is relative to @mr; @size is in bytes */
static uint64_t stm32f429_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    (void) size;

    trace_stm32f429_gpio_read(offset);

    if(offset <= STM32F429_GPIO_MMIO_SIZE)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: offset out of bounds\n", __func__);
        return 0;
    }

    STM32F429GpioState *s = opaque;

    switch((StmGpioRegOffset) offset)
    {
        case OFFSET_MODER:
        {
            return s->moder;
        }
        case OFFSET_OTYPER:
        {
            return s->otyper;
        }
        case OFFSET_OSPEEDR:
        {
            qemu_log_mask(LOG_UNIMP, "%s: OSPEEDR is not supported\n",
                    __func__);
            return s->ospeedr;
        }
        case OFFSET_PUPDR:
        {
            return s->pupdr;
        }
        case OFFSET_IDR:
        {
            return (s->idr & LOWER_WORD_MASK);
        }
        case OFFSET_ODR:
        {
            return (s->odr & LOWER_WORD_MASK);
        }
        case OFFSET_BSRR:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: BSRR is write only\n",
                    __func__);
            return 0;
        }
        case OFFSET_LCKR:
        {
            update_lckr(s, s->lckr, true);
            return s->lckr;
        }
        case OFFSET_AFRL:
        {
            qemu_log_mask(LOG_UNIMP, "%s: AFRL not supported\n", __func__);
            return s->aflr;
        }
        case OFFSET_AFRH:
        {
            qemu_log_mask(LOG_UNIMP, "%s: AFRH not supported\n", __func__);
            return s->aflr;
        }
    }

    return 0;
}

/* Write to the memory region. @addr is relative to @mr; @size is in bytes */
static void stm32f429_gpio_write(void *opaque, hwaddr offset, uint64_t data,
                                 unsigned size)
{
    // unused, we require access as 32 bit (word)
    (void) size;

    trace_stm32f429_gpio_write(offset, data);

    if(offset <= STM32F429_GPIO_MMIO_SIZE)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: offset out of bounds\n", __func__);
        return;
    }

    STM32F429GpioState *s = opaque;
    const uint32_t value_32 = (uint32_t)data;

    switch((StmGpioRegOffset) offset)
    {
        case OFFSET_MODER:
        {
            write_control_register(s, &s->moder, value_32);
            break;
        }
        case OFFSET_OTYPER:
        {
            // upper 16 bits reserved
            write_control_register(s, &s->otyper, (value_32 & LOWER_WORD_MASK));
            break;
        }
        case OFFSET_OSPEEDR:
        {
            qemu_log_mask(LOG_UNIMP, "%s: OSPEEDR is not supported\n",
                    __func__);
            write_control_register(s, &s->ospeedr, value_32);
            break;
        }
        case OFFSET_PUPDR:
        {
            write_control_register(s, &s->pupdr, value_32);
            break;
        }
        case OFFSET_IDR:
        {
            qemu_log_mask(LOG_UNIMP, "%s: IDR is read only\n", __func__);
            break;
        }
        case OFFSET_ODR:
        {
            // upper 16 bits reserved
            s->odr = (value_32 & LOWER_WORD_MASK);
            break;
        }
        case OFFSET_LCKR:
        {
            update_lckr(s, value_32, false);
            break;
        }
        case OFFSET_BSRR:
        {
            const uint16_t to_set = (value_32 & UPPER_WORD_MASK) >> 16;
            const uint16_t to_reset = value_32 & LOWER_WORD_MASK;
            // setting bits has priority (see TRM 8.4.7)
            s->odr |= to_set;
            s->odr &= ~to_reset;
           break;
        }
        case OFFSET_AFRL:
        {
            qemu_log_mask(LOG_UNIMP,
                    "%s: Alternate functions are not supported\n",
                    __func__);
            write_control_register(s, &s->aflr, value_32);
            break;
        }
        case OFFSET_AFRH:
        {
            qemu_log_mask(LOG_UNIMP,
                    "%s: Alternate functions are not supported\n",
                    __func__);
            write_control_register(s, &s->afhr, value_32);
            break;
        }
    }
}

// I/O operations on memory region
static const MemoryRegionOps memops = {
    .read = stm32f429_gpio_read,
    .write = stm32f429_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false
    }
};

static void stm32f429_gpio_reset(Object *obj)
{
    STM32F429GpioState *s = STM32F429_GPIO(obj);

    // reset GPIO port registers
    s->moder = s->moder_reset_val;
    s->otyper = DEFAULT_RESET_VALUE;
    s->pupdr = s->pupdr_reset_val;
    s->ospeedr = s->ospeedr_reset_val;
    s->idr = DEFAULT_RESET_VALUE;
    s->odr = DEFAULT_RESET_VALUE;
    s->bsrr = DEFAULT_RESET_VALUE;
    s->lckr = DEFAULT_RESET_VALUE;
    s->afhr = DEFAULT_RESET_VALUE;
    s->aflr = DEFAULT_RESET_VALUE;
}

static void stm32f429_gpio_init(Object *obj)
{
    STM32F429GpioState *s = STM32F429_GPIO(obj);

    memory_region_init_io(&s->mmio, OBJECT(s), &memops, s, TYPE_STM32F429_GPIO,
                          STM32F429_GPIO_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
}

static const VMStateDescription vmstate_stm32f429_gpio = {
    .name = TYPE_STM32F429_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(moder, STM32F429GpioState),
        VMSTATE_UINT32(otyper, STM32F429GpioState),
        VMSTATE_UINT32(pupdr, STM32F429GpioState),
        VMSTATE_UINT32(ospeedr, STM32F429GpioState),
        VMSTATE_UINT32(idr, STM32F429GpioState),
        VMSTATE_UINT32(odr, STM32F429GpioState),
        VMSTATE_UINT32(bsrr, STM32F429GpioState),
        VMSTATE_UINT32(lckr, STM32F429GpioState),
        VMSTATE_UINT32(afhr, STM32F429GpioState),
        VMSTATE_UINT32(aflr, STM32F429GpioState),
        VMSTATE_END_OF_LIST()
    }
};

// Device properties holding specific reset values
static Property properties_stm32f42_gpio[] = {
    DEFINE_PROP_UINT32("moder_reset_val", STM32F429GpioState,
            moder_reset_val, DEFAULT_RESET_VALUE),
    DEFINE_PROP_UINT32("ospeedr_reset_val", STM32F429GpioState,
            ospeedr_reset_val, DEFAULT_RESET_VALUE),
    DEFINE_PROP_UINT32("pupdr_reset_val", STM32F429GpioState,
            pupdr_reset_val, DEFAULT_RESET_VALUE),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32f429_gpio_class_init(struct ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, properties_stm32f42_gpio);
    dc->desc = type_desc;
    dc->vmsd = &vmstate_stm32f429_gpio;
    rc->phases.hold = &stm32f429_gpio_reset;
}

static const TypeInfo stm32f429_gpio_info = {
    .name          = TYPE_STM32F429_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F429GpioState),
    .instance_init = stm32f429_gpio_init,
    .class_init    = stm32f429_gpio_class_init,
};

static void stm32f429_gpio_register_types(void)
{
    type_register_static(&stm32f429_gpio_info);
}

type_init(stm32f429_gpio_register_types)
