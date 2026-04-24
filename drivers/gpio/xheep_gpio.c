/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP GPIO driver for Zephyr.
 *
 * Supports two hardware instances:
 *    esl-epfl,x-heep-gpio-ao
 *      Always-On GPIO (8 pins). Interrupt lines tied to FIC (6-13).
 *    esl-epfl,x-heep-gpio
 *      User-domain GPIO (24 pins). Interrupt lines tied to PLIC (9-32).
 *
 * Interrupt routing:
 *   Always-On GPIO:   pin p (0 ≤ p ≤  7) → FIC source (6 + p) → hlic (22 + p)
 *                     Managed via xheep_fic_irq_enable/disable/clear
 *   User-domain GPIO: pin p (8 ≤ p ≤ 31) → PLIC source (p + 1) → MEXT (hlic 11)
 *                     Multi-level IRQ: irq_to_level_2(p+1) | XHEEP_PLIC_L1_IRQ
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include "../interrupt_controller/xheep_intc_fic.h"


//! DRIVER COMPAT

// Done in the instantiation macros


//! CONSTANTS

//* Register Offsets
#define GPIO_INFO_OFF              0x000U //   (RO) GPIO_CNT[9:0], VERSION[19:10]
#define GPIO_CFG_OFF               0x004U //   (RW) GLBL_INTRPT_MODE, PIN_LVL_INTRPT_MODE
#define GPIO_MODE_0_OFF            0x008U //   (RW) IO mode for pins  0-15 (2 bits/pin)
#define GPIO_MODE_1_OFF            0x00CU //   (RW) IO mode for pins 16-31 (2 bits/pin)
#define GPIO_EN_OFF                0x080U //   (RW) Input-sampling enable  (1 bit/pin)
#define GPIO_IN_OFF                0x100U //   (RO) Current input values
#define GPIO_OUT_OFF               0x180U //   (RW) Current output values
#define GPIO_SET_OFF               0x200U //   (WO) Atomic set bits in GPIO_OUT
#define GPIO_CLEAR_OFF             0x280U //   (WO) Atomic clear bits in GPIO_OUT
#define GPIO_TOGGLE_OFF            0x300U //   (WO) Atomic toggle bits in GPIO_OUT
#define GPIO_RISE_EN_OFF           0x380U //   (RW) Rising-edge interrupt enable
#define GPIO_FALL_EN_OFF           0x400U //   (RW) Falling-edge interrupt enable
#define GPIO_LVL_HIGH_EN_OFF       0x480U //   (RW) Level-high interrupt enable
#define GPIO_LVL_LOW_EN_OFF        0x500U //   (RW) Level-low interrupt enable
#define GPIO_INTR_STATUS_OFF       0x580U // (RW1C) Combined interrupt status (any type)
#define GPIO_RISE_STATUS_OFF       0x600U // (RW1C) Rising-edge status
#define GPIO_FALL_STATUS_OFF       0x680U // (RW1C) Falling-edge status
#define GPIO_LVL_HIGH_STATUS_OFF   0x700U // (RW1C) Level-high status
#define GPIO_LVL_LOW_STATUS_OFF    0x780U // (RW1C) Level-low status

//* IO mode values (2-bit field per pin in GPIO_MODE_0/1)
/*
  IO modes (2-bit field per pin in GPIO_MODE_0/1):
    0b00  INPUT_ONLY     — input only, output driver disabled
    0b01  OUTPUT_ACTIVE  — push-pull output
    0b10  OPEN_DRAIN0    — open-drain: 0 → High-Z,    1 → Drive High
    0b11  OPEN_DRAIN1    — open-drain: 0 → Drive Low, 1 → High-Z
*/

#define GPIO_MODE_INPUT_ONLY     0U  /* Input only (output driver off) */
#define GPIO_MODE_OUTPUT_ACTIVE  1U  /* Push-pull output */
#define GPIO_MODE_OPEN_DRAIN0    2U  /* Open-drain: 0 → High-Z,    1 → Drive High */
#define GPIO_MODE_OPEN_DRAIN1    3U  /* Open-drain: 0 → Drive Low, 1 → High-Z */

//* Interrupt bindings (user-domain GPIO only)

/*
  Pins 8-31 of the user-domain GPIO controller have PLIC interrupt sources.
  PLIC source for GPIO pin p = p + 1  (pin 8 → source 9, pin 31 → source 32).
*/
#define GPIO_IRQ_FIRST_PIN  8U

/*
  PLIC L1 IRQ: Machine External Interrupt (MEXT) = 11.
  Used at runtime in pin_interrupt_configure to build multi-level IRQ values.
*/
#define XHEEP_PLIC_L1_IRQ   11U


//! DRIVER STRUCTURES

struct xheep_gpio_config {
  /* gpio_driver_config MUST be first — GPIO core casts dev->config to it */
  struct gpio_driver_config common;
  mem_addr_t  base;
  uint8_t     ngpios;
  bool        has_interrupts;  /* false only if no interrupt path at all */
  bool        fic_based;       /* true for GPIO_AO (FIC), false for GPIO0 (PLIC) */
  uint8_t     fic_first_src;   /* first FIC source index (GPIO_AO: XHEEP_FIC_SRC_GPIO_AO0) */
};

struct xheep_gpio_data {
  /* gpio_driver_data MUST be first — GPIO core casts dev->data to it */
  struct gpio_driver_data common;
  sys_slist_t callbacks;
};


//! REGISTER HELPERS

static inline uint32_t gpio_reg_read(const struct xheep_gpio_config *cfg, uint32_t off) {
  return sys_read32(cfg->base + off);
}

static inline void gpio_reg_write(const struct xheep_gpio_config *cfg, uint32_t off, uint32_t val) {
  sys_write32(val, cfg->base + off);
}


//! API FUNCTIONS

static int xheep_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags) {
  /*
    Configure direction, drive mode, and initial state for a single pin.
    Pull-up / pull-down resistors are controlled by the Pad_Control peripheral (another IP of X-HEEP).
  */
  const struct xheep_gpio_config *cfg = dev->config;
  uint32_t hw_mode, mode_reg, bit_shift, cur, en;

  if (pin >= cfg->ngpios) {
    return -EINVAL;
  }

  // Determine hardware IO mode from Zephyr GPIO flags
  if (flags & GPIO_OUTPUT) {
    hw_mode = (flags & GPIO_OPEN_DRAIN) ? GPIO_MODE_OPEN_DRAIN1 : GPIO_MODE_OUTPUT_ACTIVE;
  }
  else {
    // GPIO_INPUT or GPIO_DISCONNECTED - input (sampling controlled below)
    hw_mode = GPIO_MODE_INPUT_ONLY;
  }

  // Write the 2-bit mode field: GPIO_MODE_0 covers pins 0-15, GPIO_MODE_1 pins 16-31
  mode_reg  = (pin < 16u) ? GPIO_MODE_0_OFF : GPIO_MODE_1_OFF;
  bit_shift = (pin % 16u) * 2u;
  cur = gpio_reg_read(cfg, mode_reg);
  cur = (cur & ~(0x3u << bit_shift)) | (hw_mode << bit_shift);
  gpio_reg_write(cfg, mode_reg, cur);

  // Enable input sampling for input pins; disable for outputs / disconnected
  en = gpio_reg_read(cfg, GPIO_EN_OFF);
  if (flags & GPIO_INPUT) {
    en |= BIT(pin);
  }
  else {
    en &= ~BIT(pin);
  }
  gpio_reg_write(cfg, GPIO_EN_OFF, en);

  // Apply initial output level when configuring an output
  if (flags & GPIO_OUTPUT) {
    if (flags & GPIO_OUTPUT_INIT_HIGH) {
      gpio_reg_write(cfg, GPIO_SET_OFF, BIT(pin));
    }
    else if (flags & GPIO_OUTPUT_INIT_LOW) {
      gpio_reg_write(cfg, GPIO_CLEAR_OFF, BIT(pin));
    }
  }

  return 0;
}


static int xheep_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value) {
  /*
    Loads the input register value.
  */
  const struct xheep_gpio_config *cfg = dev->config;

  *value = gpio_reg_read(cfg, GPIO_IN_OFF);
  return 0;
}


static int xheep_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value) {
  /*
    Manages the SET and CLEAR atomic operations.
    Applies a mask to avoid over-write of bits.
  */
  const struct xheep_gpio_config *cfg = dev->config;
  gpio_port_pins_t to_set   = mask &  value;
  gpio_port_pins_t to_clear = mask & ~value;

  if (to_set) {
    gpio_reg_write(cfg, GPIO_SET_OFF, to_set);
  }
  if (to_clear) {
    gpio_reg_write(cfg, GPIO_CLEAR_OFF, to_clear);
  }
  return 0;
}


static int xheep_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins) {
  /*
    Direct SET operation.
  */
  const struct xheep_gpio_config *cfg = dev->config;

  gpio_reg_write(cfg, GPIO_SET_OFF, pins);
  return 0;
}


static int xheep_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins) {
  /*
    Direct CLEAR operation.
  */
  const struct xheep_gpio_config *cfg = dev->config;

  gpio_reg_write(cfg, GPIO_CLEAR_OFF, pins);
  return 0;
}


static int xheep_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins) {
  /*
    Direct TOGGLE operation.
  */
  const struct xheep_gpio_config *cfg = dev->config;

  gpio_reg_write(cfg, GPIO_TOGGLE_OFF, pins);
  return 0;
}


static int xheep_gpio_pin_interrupt_configure(
  const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig) {
  /*
    Configure edge/level interrupt triggers for a single GPIO pin.

    Two different interrupt backends are supported (depending on the peripheral):
      FIC (GPIO_AO):    all pins 0-7 have dedicated FIC sources (6-13).
                        Managed via xheep_fic_irq_enable/disable.
      PLIC (user GPIO): only pins 8-31 have PLIC sources (p+1).
                        Managed via the native Zephyr irq_enable/disable with
                        multi-level encoding.

    Pins with no interrupt connectivity (or unidentified) return -ENOTSUP for any active mode
    and 0 (safe no-op) for GPIO_INT_MODE_DISABLED.
  */
  const struct xheep_gpio_config *cfg = dev->config;
  uint32_t pin_mask = BIT(pin);
  uint32_t rise_en, fall_en, lvl_hi, lvl_lo, en;

  if (pin >= cfg->ngpios) {
    return -EINVAL;
  }

  // Check interrupt capability for this pin
  if (!cfg->has_interrupts || (!cfg->fic_based && pin < GPIO_IRQ_FIRST_PIN)) {
    // No interrupt path for this pin, disable is a safe no-op
    return (mode == GPIO_INT_MODE_DISABLED) ? 0 : -ENOTSUP;
  }

  // Read current interrupt enable state for all four trigger types
  rise_en = gpio_reg_read(cfg, GPIO_RISE_EN_OFF);
  fall_en = gpio_reg_read(cfg, GPIO_FALL_EN_OFF);
  lvl_hi  = gpio_reg_read(cfg, GPIO_LVL_HIGH_EN_OFF);
  lvl_lo  = gpio_reg_read(cfg, GPIO_LVL_LOW_EN_OFF);

  // Clear this pin from all four interrupt enable masks
  rise_en &= ~pin_mask;
  fall_en &= ~pin_mask;
  lvl_hi  &= ~pin_mask;
  lvl_lo  &= ~pin_mask;

  if (mode != GPIO_INT_MODE_DISABLED) {
    // Input sampling must be enabled for interrupt detection to work
    en = gpio_reg_read(cfg, GPIO_EN_OFF);
    gpio_reg_write(cfg, GPIO_EN_OFF, en | pin_mask);

    if (mode == GPIO_INT_MODE_EDGE) {
      if (trig == GPIO_INT_TRIG_HIGH || trig == GPIO_INT_TRIG_BOTH) {
        rise_en |= pin_mask;
      }
      if (trig == GPIO_INT_TRIG_LOW || trig == GPIO_INT_TRIG_BOTH) {
        fall_en |= pin_mask;
      }
    }
    else {  /* GPIO_INT_MODE_LEVEL */
      if (trig == GPIO_INT_TRIG_HIGH) {
        lvl_hi |= pin_mask;
      } else {
        lvl_lo |= pin_mask;
      }
    }

    // Enable interrupt delivery through the appropriate backend
    if (cfg->fic_based) {
      xheep_fic_irq_enable(cfg->fic_first_src + pin);
    }
    else {
      // PLIC path: pin p - PLIC source (p+1) routed through MEXT (L1 IRQ 11)
      irq_enable(irq_to_level_2(pin + 1u) | XHEEP_PLIC_L1_IRQ);
    }
  }
  else {
    // Disable interrupt delivery through the appropriate backend
    if (cfg->fic_based) {
      xheep_fic_irq_disable(cfg->fic_first_src + pin);
    }
    else {
      irq_disable(irq_to_level_2(pin + 1u) | XHEEP_PLIC_L1_IRQ);
    }
  }

  // Update trigger configuration
  gpio_reg_write(cfg, GPIO_RISE_EN_OFF, rise_en);
  gpio_reg_write(cfg, GPIO_FALL_EN_OFF, fall_en);
  gpio_reg_write(cfg, GPIO_LVL_HIGH_EN_OFF, lvl_hi);
  gpio_reg_write(cfg, GPIO_LVL_LOW_EN_OFF, lvl_lo);

  return 0;
}


static int xheep_gpio_manage_callback(
  const struct device *dev, struct gpio_callback *callback, bool set) {
  /*
    Wrapper for the generic Zephyr gpio_manage_callback()
  */
  struct xheep_gpio_data *data = dev->data;

  return gpio_manage_callback(&data->callbacks, callback, set);
}


static uint32_t xheep_gpio_get_pending_int(const struct device *dev) {
  /*
    Direct INTR read operation.
  */
  const struct xheep_gpio_config *cfg = dev->config;

  return gpio_reg_read(cfg, GPIO_INTR_STATUS_OFF);
}


//! ISR — GPIO_AO (FIC-routed, pins 0-7)

static void gpio_ao_xheep_isr(const void *arg) {
  /*
    Shared ISR for all FIC-routed GPIO_AO sources (pins 0-7).

    Each of the 8 FIC sources (6-13, mcause 22-29) points to this handler.
    The handler reads INTRPT_STATUS to find all pins that fired, clears those
    bits, acknowledges each active FIC source, and then fires callbacks.

    ?CAUTION:
    Acknowledging all active FIC sources here means a concurrent ISR
    invocation for a second pin (e.g. mcause 23 pending while mcause 22
    is being handled) will find no pending status and can fire empty callbacks.
  */
  const struct device *dev = arg;
  const struct xheep_gpio_config *cfg = dev->config;
  struct xheep_gpio_data *data = dev->data;
  uint32_t status;
  uint8_t pin;

  // Read which AO GPIO pins asserted any interrupt
  status = gpio_reg_read(cfg, GPIO_INTR_STATUS_OFF);

  // Clear exactly the bits captured above (RW1C) to de-assert FIC source lines
  gpio_reg_write(cfg, GPIO_INTR_STATUS_OFF, status);

  // Acknowledge each active FIC source (W1C FAST_INTR_CLEAR[N])
  for (pin = 0u; pin < cfg->ngpios; pin++) {
    if (status & BIT(pin)) {
      xheep_fic_irq_clear(cfg->fic_first_src + pin);
    }
  }

  // Dispatch to all registered callbacks where pin mask overlaps
  gpio_fire_callbacks(&data->callbacks, dev, status);
}


//! ISR — GPIO_USER (PLIC-routed, pins 8-31)

static void gpio_xheep_isr(const void *arg) {
  /*
    Shared ISR for all PLIC-routed GPIO_USER sources (pins 8-31).

    Each of the 24 PLIC sources (9-32) points to this handler. On entry the
    PLIC has already claimed one source. This handler reads INTRPT_STATUS to
    determine which pins fired, clears those status bits (RW1C), and then
    fires all matching callbacks.

    Clearing only the bits read avoids accidentally suppressing a new
    interrupt that fires between the read and the write.
  */
  const struct device *dev = arg;
  const struct xheep_gpio_config *cfg = dev->config;
  struct xheep_gpio_data *data = dev->data;
  uint32_t status;

  // Read which pins asserted any interrupt
  status = gpio_reg_read(cfg, GPIO_INTR_STATUS_OFF);

  // Clear exactly the bits captured above (RW1C)
  gpio_reg_write(cfg, GPIO_INTR_STATUS_OFF, status);

  // Dispatch to all registered callbacks whose pin mask overlaps
  gpio_fire_callbacks(&data->callbacks, dev, status);
}


//! DRIVER API TABLE

static DEVICE_API(gpio, xheep_gpio_driver_api) = {
  .pin_configure           = xheep_gpio_pin_configure,
  .port_get_raw            = xheep_gpio_port_get_raw,
  .port_set_masked_raw     = xheep_gpio_port_set_masked_raw,
  .port_set_bits_raw       = xheep_gpio_port_set_bits_raw,
  .port_clear_bits_raw     = xheep_gpio_port_clear_bits_raw,
  .port_toggle_bits        = xheep_gpio_port_toggle_bits,
  .pin_interrupt_configure = xheep_gpio_pin_interrupt_configure,
  .manage_callback         = xheep_gpio_manage_callback,
  .get_pending_int         = xheep_gpio_get_pending_int,
};


//! INITIALIZATION FUNCTION (Common fro gpio_oa and gpio_user)

static int xheep_gpio_init(const struct device *dev) {
  const struct xheep_gpio_config *cfg = dev->config;
  struct xheep_gpio_data *data = dev->data;

  // Set all pins to input-only mode and disable input sampling
  gpio_reg_write(cfg, GPIO_MODE_0_OFF, 0u);
  gpio_reg_write(cfg, GPIO_MODE_1_OFF, 0u);
  gpio_reg_write(cfg, GPIO_EN_OFF,     0u);

  // Disable all interrupt trigger enables
  gpio_reg_write(cfg, GPIO_RISE_EN_OFF,     0u);
  gpio_reg_write(cfg, GPIO_FALL_EN_OFF,     0u);
  gpio_reg_write(cfg, GPIO_LVL_HIGH_EN_OFF, 0u);
  gpio_reg_write(cfg, GPIO_LVL_LOW_EN_OFF,  0u);

  // Clear any stale interrupt status bits
  gpio_reg_write(cfg, GPIO_INTR_STATUS_OFF,     0xFFFFFFFFu);
  gpio_reg_write(cfg, GPIO_RISE_STATUS_OFF,     0xFFFFFFFFu);
  gpio_reg_write(cfg, GPIO_FALL_STATUS_OFF,     0xFFFFFFFFu);
  gpio_reg_write(cfg, GPIO_LVL_HIGH_STATUS_OFF, 0xFFFFFFFFu);
  gpio_reg_write(cfg, GPIO_LVL_LOW_STATUS_OFF,  0xFFFFFFFFu);

  // Initialize the callback list
  sys_slist_init(&data->callbacks);

  return 0;
}


//! INSTANTIATION MACROS — ALWAYS-ON GPIO_AO (esl-epfl,x-heep-gpio-ao)

/*
  GPIO_AO has 8 pins, each with a dedicated FIC source (6-13, mcause 22-29).

  ? CAUTION
  hlic has #interrupt-cells = <1> (IRQ number only, no priority cell), so
  IRQ_CONNECT must hardcode priority 0 rather than reading it from DT.

  ? NOTE
  IRQ_CONNECT only populates the ISR table slot. FIC enable/MIE bits are set
  at runtime in pin_interrupt_configure when the application arms a pin.
*/
#define DT_DRV_COMPAT esl_epfl_x_heep_gpio_ao

#define XHEEP_GPIO_AO_IRQ_CONNECT(idx, n)                                      \
  COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, idx), (                                   \
    IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, idx, irq), 0, gpio_ao_xheep_isr,         \
      DEVICE_DT_INST_GET(n), 0)                                                \
  ), ())

#define XHEEP_GPIO_AO_INIT(n)                                                  \
  static struct xheep_gpio_data xheep_gpio_ao_data_##n;                        \
                                                                               \
  static const struct xheep_gpio_config xheep_gpio_ao_cfg_##n = {              \
    .common = {                                                                \
      .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(                         \
                         DT_INST_PROP(n, ngpios)),                             \
    },                                                                         \
    .base           = DT_INST_REG_ADDR(n),                                     \
    .ngpios         = DT_INST_PROP(n, ngpios),                                 \
    .has_interrupts = true,                                                    \
    .fic_based      = true,                                                    \
    .fic_first_src  = XHEEP_FIC_SRC_GPIO_AO0,                                  \
  };                                                                           \
                                                                               \
  static int xheep_gpio_ao_##n##_init(const struct device *dev)                \
  {                                                                            \
    /* Wire ISR table slots for all 8 FIC-routed AO GPIO sources */            \
    LISTIFY(8, XHEEP_GPIO_AO_IRQ_CONNECT, (;), n)                              \
    return xheep_gpio_init(dev);                                               \
  }                                                                            \
                                                                               \
  DEVICE_DT_INST_DEFINE(n,                                                     \
    xheep_gpio_ao_##n##_init,                                                  \
    NULL,                                                                      \
    &xheep_gpio_ao_data_##n,                                                   \
    &xheep_gpio_ao_cfg_##n,                                                    \
    PRE_KERNEL_1,                                                              \
    CONFIG_GPIO_INIT_PRIORITY,                                                 \
    &xheep_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XHEEP_GPIO_AO_INIT)


//! INSTANTIATION MACROS — GPIO_USER (esl-epfl,x-heep-gpio)

/*
  Per-interrupt-index helper used inside the per-instance init wrapper.
  LISTIFY expands this 24 times (idx 0-23). Each expansion that matches a
  DT interrupt entry calls IRQ_CONNECT to populate the ISR table slot for
  the corresponding PLIC source. Indices beyond the DT count are no-ops.

  ? NOTE
  IRQ_CONNECT only populates the ISR table slot. PLIC enable bits are set
  at runtime in pin_interrupt_configure when the application arms a pin.
*/
#define DT_DRV_COMPAT esl_epfl_x_heep_gpio

#define XHEEP_GPIO_IRQ_CONNECT(idx, n)                                         \
  COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, idx), (                                   \
    IRQ_CONNECT(IRQ_TO_L2(DT_INST_IRQ_BY_IDX(n, idx, irq)) +                   \
      DT_IRQN(DT_NODELABEL(plic)), DT_INST_IRQ_BY_IDX(n, idx, priority),       \
      gpio_xheep_isr, DEVICE_DT_INST_GET(n), 0)                                \
  ), ())

/*
  A per-instance init wrapper is generated so that the compile-time
  IRQ_CONNECT calls (which must appear inside a function body) are tied to
  the correct DT instance at expansion time, then the common runtime
  xheep_gpio_init() is called to complete hardware setup.
*/
#define XHEEP_GPIO_INIT(n)                                                     \
  static struct xheep_gpio_data xheep_gpio_data_##n;                           \
                                                                               \
  static const struct xheep_gpio_config xheep_gpio_cfg_##n = {                 \
    .common = {                                                                \
      .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(                         \
                         DT_INST_PROP(n, ngpios)),                             \
    },                                                                         \
    .base           = DT_INST_REG_ADDR(n),                                     \
    .ngpios         = DT_INST_PROP(n, ngpios),                                 \
    .has_interrupts = true,                                                    \
    .fic_based      = false,                                                   \
    .fic_first_src  = 0u,                                                      \
  };                                                                           \
                                                                               \
  static int xheep_gpio_##n##_init(const struct device *dev)                   \
  {                                                                            \
    /* Populate ISR table entries for all PLIC-routed GPIO sources */          \
    LISTIFY(24, XHEEP_GPIO_IRQ_CONNECT, (;), n)                                \
    return xheep_gpio_init(dev);                                               \
  }                                                                            \
                                                                               \
  DEVICE_DT_INST_DEFINE(n,                                                     \
    xheep_gpio_##n##_init,                                                     \
    NULL,                                                                      \
    &xheep_gpio_data_##n,                                                      \
    &xheep_gpio_cfg_##n,                                                       \
    PRE_KERNEL_1,                                                              \
    CONFIG_GPIO_INIT_PRIORITY,                                                 \
    &xheep_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XHEEP_GPIO_INIT)

#undef DT_DRV_COMPAT
