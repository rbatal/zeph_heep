/*
  * X-HEEP Platform-Level Interrupt Controller (PLIC) driver for Zephyr.
  *
  * This driver targets the lowRISC/OpenTitan rv_plic IP as instantiated
  * in X-HEEP, which uses a COMPACT register layout that does NOT match
  * the standard RISC-V PLIC memory map (nor the latest OpenTitan version).
  * Zephyr's built-in intc_plic.c cannot be used because it hardcodes the
  * standard offsets.
  *
  * X-HEEP rv_plic register map (64 sources, 1 target, 3-bit priority):
  *
  *   Offset        Register
  *   ──────        ────────────────────────────────
  *   0x000-0x004   IP_0, IP_1       (pending, 2×32-bit, RO)
  *   0x008-0x00C   LE_0, LE_1       (level/edge mode, 2×32-bit, RW)
  *   0x010-0x10C   PRIO0 – PRIO63   (source priority, 64×32-bit, RW)
  *   0x200-0x204   IE0_0, IE0_1     (enable target 0, 2×32-bit, RW)
  *   0x208         THRESHOLD0       (priority threshold target 0, RW)
  *   0x20C         CC0              (claim/complete target 0, RW)
  *   0x210         MSIP0            (software interrupt hart 0, RW)
  *
  * The driver provides the riscv_plic_* symbols required by Zephyr's
  * RISC-V architecture layer (arch/riscv/core/irq_manage.c) when
  * CONFIG_RISCV_HAS_PLIC is selected.
*/

// TODO: Add @brief for proper function documentation


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/logging/log.h>


//! MACROS

LOG_MODULE_REGISTER(xheep_plic, CONFIG_INTC_LOG_LEVEL);


//! CONSTANTS

//* Devicetree compat

#define DT_DRV_COMPAT   esl_epfl_x_heep_plic

//* Register offsets (from PLIC base)

#define PLIC_OFF_IP0        0x000u
#define PLIC_OFF_IP1        0x004u
#define PLIC_OFF_LE0        0x008u
#define PLIC_OFF_LE1        0x00Cu
#define PLIC_OFF_PRIO_BASE  0x010u
#define PLIC_OFF_IE0_0      0x200u
#define PLIC_OFF_IE0_1      0x204u
#define PLIC_OFF_THRESHOLD0 0x208u
#define PLIC_OFF_CC0        0x20Cu

//* Other

#define PRIO_MASK  0x7u


//! DRIVER STRUCTURES

struct xheep_plic_config {
  mem_addr_t base;
  uint32_t   max_prio;
  uint32_t   num_src;
};

struct xheep_plic_data {
  /* No mutable state needed; placeholder for DEVICE_DT_INST_DEFINE */
};


//! DEVICE ACCESSOR

/*
  plic_cfg() — convenience accessor for the single X-HEEP PLIC instance.

  The riscv_plic_* API is called by arch/riscv/core/irq_manage.c without a
  device pointer, so we reach the config through DEVICE_DT_INST_GET(0).
  X-HEEP has exactly one PLIC, so this is always correct.
*/
static inline const struct xheep_plic_config *plic_cfg(void) {
  return DEVICE_DT_INST_GET(0)->config;
}

const struct device *riscv_plic_get_dev(void) {
  /*
    Required by arch/riscv/core/irq_manage.c.

    Used in z_irq_spurious() to log "PLIC interrupt line causing the IRQ: %d (%p)".
    Returning the real device pointer enables that debug path.
  */
  return DEVICE_DT_INST_GET(0);
}


//! HELPER FUNCTIONS

//* Multi-level IRQ decoding

static ALWAYS_INLINE uint32_t plic_source_from_irq(uint32_t irq) {
  /*
    Extract the PLIC source ID from a Zephyr multi-level encoded IRQ.
    2nd-level encoding: ((source + 1) << 8) | parent_irq

    If the upper byte is zero, the value is already a raw source ID
    (used internally by the driver, e.g. in the claim/complete path).
  */

  // Multi-level encoded
  if (irq >> 8) {
    return (irq >> 8) - 1;
  }

  // Raw PLIC source ID
  return irq;
}


//! API FUNCTIONS (OVERRIDE)

/*
  riscv_plic_* API is called by arch/riscv/core/irq_manage.c

  The `irq` parameter in all these functions is the raw PLIC source
  ID (1-63).  The multi-level encoding/decoding is handled by the
  arch layer before calling these.
*/


void riscv_plic_irq_enable(uint32_t irq) {
  /*
    Enable a PLIC interrupt source for target 0.

    Sets the corresponding bit in IE0_0 or IE0_1.
   */
  const struct xheep_plic_config *cfg = plic_cfg();
  uint32_t src = plic_source_from_irq(irq);

  if (src == 0 || src >= cfg->num_src) {
    return;
  }

  uint32_t reg_idx = src / 32u;
  uint32_t bit_pos = src % 32u;
  uint32_t reg_addr = cfg->base + PLIC_OFF_IE0_0 + (reg_idx * 4u);

  uint32_t val = sys_read32(reg_addr);
  val |= BIT(bit_pos);
  sys_write32(val, reg_addr);
}


void riscv_plic_irq_disable(uint32_t irq) {
  /*
    Disable a PLIC interrupt source for target 0.

    Clears the corresponding bit in IE0_0 or IE0_1.
  */
  const struct xheep_plic_config *cfg = plic_cfg();
  uint32_t src = plic_source_from_irq(irq);

  if (src == 0 || src >= cfg->num_src) {
    return;
  }

  uint32_t reg_idx = src / 32u;
  uint32_t bit_pos = src % 32u;
  uint32_t reg_addr = cfg->base + PLIC_OFF_IE0_0 + (reg_idx * 4u);

  uint32_t val = sys_read32(reg_addr);
  val &= ~BIT(bit_pos);
  sys_write32(val, reg_addr);
}


int riscv_plic_irq_is_enabled(uint32_t irq) {
  /*
    Check whether a PLIC interrupt source is enabled.
  */
  const struct xheep_plic_config *cfg = plic_cfg();
  uint32_t src = plic_source_from_irq(irq);

  if (src == 0 || src >= cfg->num_src) {
    return 0;
  }

  uint32_t reg_idx = src / 32u;
  uint32_t bit_pos = src % 32u;
  uint32_t reg_addr = cfg->base + PLIC_OFF_IE0_0 + (reg_idx * 4u);

  return (sys_read32(reg_addr) & BIT(bit_pos)) != 0;
}


void riscv_plic_set_priority(uint32_t irq, uint32_t priority) {
  /*
    Set the priority of a PLIC interrupt source.

    irq:      Source ID (1-63).
    priority: Priority value (0 = never fires, 1-7 active).
  */
  const struct xheep_plic_config *cfg = plic_cfg();
  uint32_t src = plic_source_from_irq(irq);

  if (src >= cfg->num_src) {
    return;
  }

  if (priority > cfg->max_prio) {
    priority = cfg->max_prio;
  }

  sys_write32(priority & PRIO_MASK, cfg->base + PLIC_OFF_PRIO_BASE + (src * 4u));
}


unsigned int riscv_plic_get_irq(void) {
  /*
    Claim the highest-pending interrupt (read CC0).

    Returns source ID of the claimed interrupt (1-63), or 0 if none.
  */
  return sys_read32(plic_cfg()->base + PLIC_OFF_CC0);
}


void riscv_plic_irq_complete(uint32_t irq_val) {
  /*
    Complete handling of a PLIC interrupt (write CC0).

    Writes the source ID previously returned by riscv_plic_get_irq().
  */
  sys_write32(irq_val, plic_cfg()->base + PLIC_OFF_CC0);
}


void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf) {
  /*
    Override the default fatal error handler to print something
    before the system resets.
  */
  sys_write32('F', 0x30080000u + 0x18u);
  sys_write32('A', 0x30080000u + 0x18u);
  sys_write32('T', 0x30080000u + 0x18u);
  sys_write32('L', 0x30080000u + 0x18u);

  // Spin forever so Verilator output is visible
  for (;;) {}
}


//! MAIN IRQ HANDLER

void xheep_plic_irq_handler(const void *arg) {
  /*
    Dispatches the 2nd-level PLIC interrupts.
  */
  const struct device *dev = arg;
  const struct xheep_plic_config *cfg = dev->config;

  //? DEBUG: raw character to UART (bypasses all software layers)
  ////sys_write32('!', 0x30080000u + 0x18u);

  unsigned int claim_id = riscv_plic_get_irq();

  //? DEBUG: write claim ID as hex digit
  ////sys_write32('0' + (claim_id & 0xF), 0x30080000u + 0x18u);

  //? DEBUG: write 'Z' for zero-claim
  ////if (claim_id == 0) {
  ////  sys_write32('Z', 0x30080000u + 0x18u);
  ////  return;
  ////}

  if (claim_id >= cfg->num_src) {
    // Should never happen with 6-bit CC0
    LOG_ERR("PLIC: claim ID %u out of range", claim_id);
    riscv_plic_irq_complete(claim_id);
    return;
  }

  /*
    Dispatch the 2nd-level ISR.

    In Zephyr's multi-level interrupt scheme the PLIC sources
    occupy _sw_isr_table starting at CONFIG_2ND_LVL_ISR_TBL_OFFSET.
    The claim ID is used directly as an index from that base.
  */
  const struct _isr_table_entry *entry = &_sw_isr_table[CONFIG_2ND_LVL_ISR_TBL_OFFSET + claim_id];
  entry->isr(entry->arg);

  // Signal completion to the PLIC gateway
  riscv_plic_irq_complete(claim_id);
}


//! INITIALIZATION FUNCTION

static int xheep_plic_init(const struct device *dev) {
  const struct xheep_plic_config *cfg = dev->config;

  LOG_INF("X-HEEP PLIC @ 0x%08lx: %u sources, max priority %u",
    cfg->base, cfg->num_src, cfg->max_prio);

  /*
    1. Disable all interrupt sources for target 0.
  */
  sys_write32(0u, cfg->base + PLIC_OFF_IE0_0);
  sys_write32(0u, cfg->base + PLIC_OFF_IE0_1);

  /*
    2. Set all source priorities to 0 (disabled).
  */
  for (unsigned int i = 0; i < cfg->num_src; i++) {
    sys_write32(0u, cfg->base + PLIC_OFF_PRIO_BASE + (i * 4u));
  }

  /*
    3. Set all sources to level-triggered (LE = 0).
  */
  sys_write32(0u, cfg->base + PLIC_OFF_LE0);
  sys_write32(0u, cfg->base + PLIC_OFF_LE1);

  /*
    4. Set priority threshold to 0 (allow any priority > 0).
  */
  sys_write32(0u, cfg->base + PLIC_OFF_THRESHOLD0);

  /*
    5. Clear any stale claims by reading then completing.
  */
  unsigned int stale;
  do {
    stale = sys_read32(cfg->base + PLIC_OFF_CC0);
    if (stale != 0) {
      sys_write32(stale, cfg->base + PLIC_OFF_CC0);
    }
  } while (stale != 0);

  /*
    6. Register our handler for Machine External Interrupt (cause 11).

    This is the 1st-level IRQ — when any PLIC source fires, the CPU
    takes the MEI trap. Then the 2nd-level IRQ is decoded.
  */

  // Compile-time macro to populate the IRQ Table
  IRQ_CONNECT(RISCV_IRQ_MEXT, 0, xheep_plic_irq_handler, DEVICE_DT_INST_GET(0), 0);

  // FALLBACK: Directly populate _sw_isr_table[11]
  ////extern struct _isr_table_entry _sw_isr_table[];
  ////_sw_isr_table[11].arg = NULL;
  ////_sw_isr_table[11].isr = xheep_plic_irq_handler;

  /*
    7. Enable MEIE (with the table slot now populated).
  */
  csr_set(mie, BIT(RISCV_IRQ_MEXT));

  LOG_INF("X-HEEP PLIC initialized");
  return 0;
}


//! INSTANTIATION MACRO

/*
  PLIC initialization has to run before PRE_KERNEL_2 so that peripheral drivers
  (UART, GPIO, etc.) can register and enable their interrupts during their own init.
*/

#define XHEEP_PLIC_INIT(n)                                            \
  static const struct xheep_plic_config xheep_plic_cfg_##n = {        \
    .base     = DT_INST_REG_ADDR(n),                                  \
    .max_prio = DT_INST_PROP(n, riscv_max_priority),                  \
    .num_src  = DT_INST_PROP(n, riscv_ndev),                          \
  };                                                                  \
  static struct xheep_plic_data xheep_plic_data_##n;                  \
                                                                      \
  DEVICE_DT_INST_DEFINE(n,                                            \
    xheep_plic_init,                                                  \
    NULL,                                                             \
    &xheep_plic_data_##n,                                             \
    &xheep_plic_cfg_##n,                                              \
    PRE_KERNEL_1,                                                     \
    CONFIG_INTC_INIT_PRIORITY,                                        \
    NULL);  /* no standard interrupt_controller driver API in Zephyr */

DT_INST_FOREACH_STATUS_OKAY(XHEEP_PLIC_INIT)
