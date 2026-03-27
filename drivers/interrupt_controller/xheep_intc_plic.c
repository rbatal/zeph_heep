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
#include <zephyr/init.h>
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

//* Devicetree Parameters

#define DT_DRV_COMPAT   esl_epfl_x_heep_plic
#define PLIC_NODE       DT_INST(0, DT_DRV_COMPAT)
#define PLIC_MAX_PRIO   DT_PROP(PLIC_NODE, riscv_max_priority)   /* 7  */
#define PLIC_NUM_SRC    DT_PROP(PLIC_NODE, riscv_ndev)           /* 64 */

//* Register Map

#define PLIC_BASE_ADDR  DT_REG_ADDR(PLIC_NODE)
#define PLIC_BASE         DT_REG_ADDR(PLIC_NODE)
#define PLIC_IP0          (PLIC_BASE + 0x000u)
#define PLIC_IP1          (PLIC_BASE + 0x004u)
#define PLIC_LE0          (PLIC_BASE + 0x008u)
#define PLIC_LE1          (PLIC_BASE + 0x00Cu)
#define PLIC_PRIO_BASE    (PLIC_BASE + 0x010u)
#define PLIC_IE0_0        (PLIC_BASE + 0x200u)
#define PLIC_IE0_1        (PLIC_BASE + 0x204u)
#define PLIC_THRESHOLD0   (PLIC_BASE + 0x208u)
#define PLIC_CC0          (PLIC_BASE + 0x20Cu)

//* Other

#define PRIO_MASK                  0x7u


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
  uint32_t src = plic_source_from_irq(irq);

  if (src == 0 || src >= PLIC_NUM_SRC) {
    return;
  }

  uint32_t reg_idx = src / 32u;
  uint32_t bit_pos = src % 32u;
  uint32_t reg_addr = PLIC_IE0_0 + (reg_idx * 4u);

  uint32_t val = sys_read32(reg_addr);
  val |= BIT(bit_pos);
  sys_write32(val, reg_addr);
}


void riscv_plic_irq_disable(uint32_t irq) {
  /*
    Disable a PLIC interrupt source for target 0.

    Clears the corresponding bit in IE0_0 or IE0_1.
  */
  uint32_t src = plic_source_from_irq(irq);

  if (src == 0 || src >= PLIC_NUM_SRC) {
    return;
  }

  uint32_t reg_idx = src / 32u;
  uint32_t bit_pos = src % 32u;
  uint32_t reg_addr = PLIC_IE0_0 + (reg_idx * 4u);

  uint32_t val = sys_read32(reg_addr);
  val &= ~BIT(bit_pos);
  sys_write32(val, reg_addr);
}


int riscv_plic_irq_is_enabled(uint32_t irq) {
  /*
    Check whether a PLIC interrupt source is enabled.
  */

  uint32_t src = plic_source_from_irq(irq);

  if (src == 0 || src >= PLIC_NUM_SRC) {
    return 0;
  }

  uint32_t reg_idx = src / 32u;
  uint32_t bit_pos = src % 32u;
  uint32_t reg_addr = PLIC_IE0_0 + (reg_idx * 4u);

  return (sys_read32(reg_addr) & BIT(bit_pos)) != 0;
}


void riscv_plic_set_priority(uint32_t irq, uint32_t priority) {
  /*
    Set the priority of a PLIC interrupt source.

    irq:      Source ID (1-63).
    priority: Priority value (0 = never fires, 1-7 active).
  */

  uint32_t src = plic_source_from_irq(irq);

  if (src >= PLIC_NUM_SRC) {
    return;
  }

  if (priority > PLIC_MAX_PRIO) {
    priority = PLIC_MAX_PRIO;
  }
  
  uint32_t reg_addr = PLIC_PRIO_BASE + (src * 4u);
  sys_write32(priority & PRIO_MASK, reg_addr);
}


unsigned int riscv_plic_get_irq() {
  /*
    Claim the highest-pending interrupt (read CC0).

    Returns source ID of the claimed interrupt (1-63), or 0 if none.
  */
  return sys_read32(PLIC_CC0);
}


void riscv_plic_irq_complete(uint32_t irq_val) {
  /*
    Complete handling of a PLIC interrupt (write CC0).

    Writes the source ID previously returned by riscv_plic_get_irq().
  */

  sys_write32(irq_val, PLIC_CC0);
}


const struct device *riscv_plic_get_dev() {
 /*
  Required by arch/riscv/core/irq_manage.c

  The arch layer calls this in z_irq_spurious() and other places to
  get the PLIC device pointer.  Since we use SYS_INIT (no struct device),
  we return NULL.  The arch layer only uses this for debug logging
  and multi-instance PLIC lookups — neither applies to X-HEEP.
 */
  return NULL;
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
    Dispatches the 2nd-level PLIC interrupts
  */

  ARG_UNUSED(arg);

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

  if (claim_id >= PLIC_NUM_SRC) {
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

static int xheep_plic_init() {
  LOG_INF("X-HEEP PLIC @ 0x%08x: %u sources, max priority %u",
    PLIC_BASE, PLIC_NUM_SRC, PLIC_MAX_PRIO);

  /*
    1. Disable all interrupt sources for target 0.
  */
  sys_write32(0u, PLIC_IE0_0);
  sys_write32(0u, PLIC_IE0_1);

  /*
    2. Set all source priorities to 0 (disabled).
  */
  for (unsigned int i = 0; i < PLIC_NUM_SRC; i++) {
    sys_write32(0u, PLIC_PRIO_BASE + (i * 4u));
  }

  /*
    3. Set all sources to level-triggered (LE = 0).
  */
  sys_write32(0u, PLIC_LE0);
  sys_write32(0u, PLIC_LE1);

  /*
    4. Set priority threshold to 0 (allow any priority > 0).
  */
  sys_write32(0u, PLIC_THRESHOLD0);

  /*
    5. Clear any stale claims by reading then completing.
  */
  unsigned int stale;
  do {
    stale = sys_read32(PLIC_CC0);
    if (stale != 0) {
      sys_write32(stale, PLIC_CC0);
    }
  } while (stale != 0);

  /*
    6. Register our handler for Machine External Interrupt (cause 11).

    This is the 1st-level IRQ — when any PLIC source fires, the CPU
    takes the MEI trap. Then the 2nd-level IRQ is decoded.
  */

  // Compile-time macro to populate the IRQ Table
  IRQ_CONNECT(RISCV_IRQ_MEXT, 0, xheep_plic_irq_handler, NULL, 0);

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


//! INITIALIZATION MACRO

/*
  PLIC initialization has to run before PRE_KERNEL_2 so that peripheral drivers
  (UART, GPIO, etc.) can register and enable their interrupts during their own init.
*/
SYS_INIT(xheep_plic_init, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);
