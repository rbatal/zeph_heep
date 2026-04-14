/*
  * X-HEEP Fast Interrupt Controller (FIC) driver dor Zephyr.
  *
  * Each FIC source N is connected to irq_i[16+N] - mcause (16+N).
  * The driver controls both the FIC ENABLE register and the MIE CSR so that
  * the core takes the fast-IRQ trap only in the scenario where the peripheral
  * armed it.
  *
  * Register map offsets:
  *   Offset        Register
  *   ──────        ────────────────────────────────
  *   0x000         FAST_INTR_PENDING  (pending sources)
  *   0x004         FAST_INTR_CLEAR    (acknowledge and clear)
  *   0x008         FAST_INTR_ENABLE   (source enable mask)
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/logging/log.h>
#include "xheep_intc_fic.h"

LOG_MODULE_REGISTER(xheep_fic, CONFIG_INTC_LOG_LEVEL);


//! DRIVER COMPAT MACRO

#define DT_DRV_COMPAT esl_epfl_x_heep_fic


//! CONSTANTS

//* Register offsets (from PLIC base)

#define FIC_OFF_PENDING  0x0u
#define FIC_OFF_CLEAR    0x4u
#define FIC_OFF_ENABLE   0x8u


//! DRIVER STRUCTURES

struct xheep_fic_config {
  mem_addr_t base;
};

struct xheep_fic_data {
};


//! DEVICE ACCESSOR

/*
  fic_cfg() — convenience accessor for the single X-HEEP FIC instance.
*/
static inline const struct xheep_fic_config *fic_cfg(void) {
  return DEVICE_DT_INST_GET(0)->config;
}


//! PUBLIC API

void xheep_fic_irq_enable(uint8_t src) {
  /*
    Enable FIC source N
  */
  const struct xheep_fic_config *cfg = fic_cfg();

  if (src >= XHEEP_FIC_NUM_SRC) {
    return;
  }

  /*
    1. Set FAST_INTR_ENABLE bit N (FIC gates the interrupt signal).
  */
  uint32_t val = sys_read32(cfg->base + FIC_OFF_ENABLE);
  val |= BIT(src);
  sys_write32(val, cfg->base + FIC_OFF_ENABLE);

  /*
    2. Unmask MIE[16+N] (core gates whether it takes the trap).
  */
  csr_set(mie, BIT(16u + src));
}


void xheep_fic_irq_disable(uint8_t src) {
  /*
    Disable FIC source N
    (Clearing MIE first prevents a re-entry window while the FIC bit is
    still set)
  */
  const struct xheep_fic_config *cfg = fic_cfg();

  if (src >= XHEEP_FIC_NUM_SRC) {
    return;
  }

  /*
    1. Clear MIE[16+N] first so the core stops accepting the trap.
  */
  csr_clear(mie, BIT(16u + src));

  /*
    2. Clear FAST_INTR_ENABLE bit N.
  */
  uint32_t val = sys_read32(cfg->base + FIC_OFF_ENABLE);
  val &= ~BIT(src);
  sys_write32(val, cfg->base + FIC_OFF_ENABLE);
}


void xheep_fic_irq_clear(uint8_t src) {
  /*
    Acknowledge a pending FIC interrupt.
    Should be called from within the ISR after handling, before re-enabling or
    returning.
  */
  const struct xheep_fic_config *cfg = fic_cfg();

  if (src >= XHEEP_FIC_NUM_SRC) {
    return;
  }

  sys_write32(BIT(src), cfg->base + FIC_OFF_CLEAR);
}


int xheep_fic_irq_is_pending(uint8_t src) {
  const struct xheep_fic_config *cfg = fic_cfg();

  if (src >= XHEEP_FIC_NUM_SRC) {
    return 0;
  }

  return (sys_read32(cfg->base + FIC_OFF_PENDING) & BIT(src)) ? 1 : 0;
}


//! INITIALIZATION FUNCTION

static int xheep_fic_init(const struct device *dev) {
  const struct xheep_fic_config *cfg = dev->config;

  LOG_INF("X-HEEP FIC @ 0x%08lx", cfg->base);

  // Disable all sources and clear any stale pending bits
  sys_write32(0u, cfg->base + FIC_OFF_ENABLE);
  sys_write32(0xFFFFu, cfg->base + FIC_OFF_CLEAR);

  LOG_INF("X-HEEP FIC initialized");
  return 0;
}


//! INSTANTIATION MACRO

/*
  FIC initialization has to run before PRE_KERNEL_2 so that peripheral drivers
  (rv_timer, DMA, GPIO) can register and enable their interrupts during their
  own init.
*/
#define XHEEP_FIC_INIT(n)                                             \
  static const struct xheep_fic_config xheep_fic_cfg_##n = {          \
    .base = DT_INST_REG_ADDR(n),                                      \
  };                                                                  \
  static struct xheep_fic_data xheep_fic_data_##n;                    \
                                                                      \
  DEVICE_DT_INST_DEFINE(n,                                            \
    xheep_fic_init,                                                   \
    NULL,                                                             \
    &xheep_fic_data_##n,                                              \
    &xheep_fic_cfg_##n,                                               \
    PRE_KERNEL_1,                                                     \
    CONFIG_INTC_INIT_PRIORITY,                                        \
    NULL);

DT_INST_FOREACH_STATUS_OKAY(XHEEP_FIC_INIT)
