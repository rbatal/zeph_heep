/*
  * X-HEEP PLIC Verification Test
  * 
  * Sequence 1: Register access (no interrupts)
  * Sequence 2: Interrupt dispatch using UART TX_EMPTY as trigger
  * 
  * The UART's TX FIFO is normally empty, so the TX_EMPTY interrupt
  * line is asserted by default. This is an "always-on" interrupt
  * source to test the complete PLIC claim/complete cycle without
  * needing UART interrupt support.
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>


//! CONSTANTS

//* PLIC register map
#define PLIC_BASE         DT_REG_ADDR(DT_NODELABEL(plic))
#define PLIC_IP0          (PLIC_BASE + 0x000u)
#define PLIC_IP1          (PLIC_BASE + 0x004u)
#define PLIC_LE0          (PLIC_BASE + 0x008u)
#define PLIC_PRIO_BASE    (PLIC_BASE + 0x010u)
#define PLIC_IE0_0        (PLIC_BASE + 0x200u)
#define PLIC_IE0_1        (PLIC_BASE + 0x204u)
#define PLIC_THRESHOLD0   (PLIC_BASE + 0x208u)
#define PLIC_CC0          (PLIC_BASE + 0x20Cu)

//* UART register map
#define UART_BASE         DT_REG_ADDR(DT_NODELABEL(uart0))
#define UART_INTR_STATE   (UART_BASE + 0x00U)
#define UART_INTR_ENABLE  (UART_BASE + 0x04U)
#define UART_STATUS       (UART_BASE + 0x10U)

// INTR_STATE/ENABLE bits
#define UART_INTR_TX_EMPTY_BIT BIT(2)


//! PLIC IRQ ENCODING MACROS

// Zephyr multi-level encoding: ((source + 1) << 8) | parent_irq
#define PLIC_IRQ_ENCODE(src) ((((src) + 1) << 8) | 11)

// PLIC source ID for UART TX_EMPTY (from core_v_mini_mcu.h)
#define UART_TX_EMPTY_SRC      3u

// UART ISR encoding (necesary to be encoded at compile time)
#define UART_TX_EMPTY_IRQ  PLIC_IRQ_ENCODE(UART_TX_EMPTY_SRC)


//! FUNCTIONS

//* ISR Handlers

static volatile bool     isr_fired;
static volatile uint32_t isr_source_id;
static volatile uint32_t isr_count;

// External IRQ handler from xheep_intc_plic.c
extern void xheep_plic_irq_handler(const void *arg);

// UART TX_EMPTY IRQ Handler
static void test_uart_tx_empty_isr(const void *arg) {
  /*
    This ISR will be registered for PLIC source 3 (UART TX_EMPTY).
    It disables the source in the UART so the level signal drops
    and the interrupt doesn't re-trigger endlessly.
  */

  ARG_UNUSED(arg);

  isr_fired = true;
  isr_source_id = UART_TX_EMPTY_SRC;
  isr_count++;

  // Disable UART interrupts to avoid re-triggering
  sys_write32(0u, UART_INTR_ENABLE);

  // Clear the interrupt state in the UART (W1C)
  sys_write32(UART_INTR_TX_EMPTY_BIT, UART_INTR_STATE);
}


//! TEST SEQUENCES

static int test_seq_1() {
  int pass = 0, fail = 0;

  printk("\n--- Register Access Test Sequence ---\n");

  // PLIC init already ran in PRE_KERNEL_1 — if we're here, it didn't crash
  printk("[PASS] PLIC driver initialized\n"); pass++;

  // Pending should be 0 (or reflect actual HW state)
  uint32_t ip0 = sys_read32(PLIC_IP0);
  uint32_t ip1 = sys_read32(PLIC_IP1);
  printk("[INFO] Pending: IP0=0x%08x IP1=0x%08x\n", ip0, ip1);

  // Enable regs should be 0 after init
  uint32_t ie0 = sys_read32(PLIC_IE0_0);
  uint32_t ie1 = sys_read32(PLIC_IE0_1);
  if (ie0 == 0 && ie1 == 0) {
    printk("[PASS] IE cleared: IE0=0x%08x IE1=0x%08x\n", ie0, ie1);
    pass++;
  } else {
    printk("[FAIL] IE not cleared: IE0=0x%08x IE1=0x%08x\n", ie0, ie1);
    fail++;
  }

  // Threshold should be 0
  uint32_t thr = sys_read32(PLIC_THRESHOLD0);
  if (thr == 0) {
    printk("[PASS] Threshold=0x%x\n", thr); pass++;
  } else {
    printk("[FAIL] Threshold=0x%x (expected 0)\n", thr); fail++;
  }

  //Claim should be 0 (nothing enabled/pending)
  uint32_t cc = sys_read32(PLIC_CC0);
  if (cc == 0) {
    printk("[PASS] Claim=0x%x (no pending)\n", cc); pass++;
  } else {
    printk("[FAIL] Claim=0x%x (expected 0)\n", cc); fail++;
    /* Complete whatever we claimed to avoid blocking */
    sys_write32(cc, PLIC_CC0);
  }

  // Priority write/readback
  sys_write32(5, PLIC_PRIO_BASE + 1 * 4);
  uint32_t p = sys_read32(PLIC_PRIO_BASE + 1 * 4);
  if (p == 5) {
    printk("[PASS] PRIO1 write=5 read=%u\n", p); pass++;
  } else {
    printk("[FAIL] PRIO1 write=5 read=%u\n", p); fail++;
  }
  sys_write32(0, PLIC_PRIO_BASE + 1 * 4);

  // Enable bit write/readback
  sys_write32(BIT(1), PLIC_IE0_0);
  uint32_t ie = sys_read32(PLIC_IE0_0);
  if (ie & BIT(1)) {
    printk("[PASS] IE0 bit 1 set/read OK\n"); pass++;
  } else {
    printk("[FAIL] IE0 bit 1 not set (read=0x%08x)\n", ie); fail++;
  }
  sys_write32(0, PLIC_IE0_0);

  // LE (level/edge) write/readback
  sys_write32(BIT(1), PLIC_LE0);
  uint32_t le = sys_read32(PLIC_LE0);
  if (le & BIT(1)) {
    printk("[PASS] LE0 bit 1 set/read OK\n");
    pass++;
  } else {
    printk("[FAIL] LE0 bit 1 not set (read=0x%08x)\n", le);
    fail++;
  }
  sys_write32(0, PLIC_LE0);

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_2() {
  int pass = 0, fail = 0;

  printk("\n--- Interrupt Dispatch Test Sequence ---\n");

  // Reset ISR handler tracking
  isr_fired = false;
  isr_source_id = 0;
  isr_count = 0;

  // Check if multi-level interrupt support is compiled in
  printk("Config check:");
  printk("\tMULTI_LEVEL_INTERRUPTS=%d 2ND_LEVEL=%d\n",
         IS_ENABLED(CONFIG_MULTI_LEVEL_INTERRUPTS), IS_ENABLED(CONFIG_2ND_LEVEL_INTERRUPTS));
  printk("\t2ND_LVL_ISR_TBL_OFFSET=%d 2ND_LVL_INTR_00_OFFSET=%d\n",
         CONFIG_2ND_LVL_ISR_TBL_OFFSET, CONFIG_2ND_LVL_INTR_00_OFFSET);

  // Check ISR Vector Table before any changes
  printk("\tISR Vector Table check:\n");

  printk("\t\t_sw_isr_table[11].isr = %p\n", _sw_isr_table[11].isr);
  printk("\t\txheep_plic_irq_handler = %p\n", xheep_plic_irq_handler);
  /* slot 35 = 32 + 3 = 2ND_LVL_ISR_TBL_OFFSET + source */
  printk("\t\t_sw_isr_table[35].isr = %p\n", _sw_isr_table[35].isr);
  printk("\t\ttest_uart_tx_empty_isr = %p\n", test_uart_tx_empty_isr);
  
  // FALLBACK: Hardcoding ISR Vector Table
  //? To be removed when 2nd-level IRQs are logged correctly
  ////_sw_isr_table[CONFIG_2ND_LVL_ISR_TBL_OFFSET + UART_TX_EMPTY_SRC].isr = test_uart_tx_empty_isr;
  ////_sw_isr_table[CONFIG_2ND_LVL_ISR_TBL_OFFSET + UART_TX_EMPTY_SRC].arg = NULL;

  // Check interrupt-related CSRs
  printk("\tInterrupt Related CSRs:\n");

  unsigned long mtvec_val;
  __asm__ volatile("csrr %0, mtvec" : "=r"(mtvec_val));
  printk("\t\tmtvec = 0x%08lx\n", mtvec_val);

  extern void _isr_wrapper(void);
  printk("\t\t\tInstruction subroutine address: _isr_wrapper = %p\n", _isr_wrapper);

  unsigned long mie_val;
  __asm__ volatile("csrr %0, mie" : "=r"(mie_val));
  printk("\t\tmie = 0x%08lx (MEIE bit11=%s)\n", mie_val,
         (mie_val & BIT(11)) ? "SET" : "CLEAR");

  unsigned long mstatus_val;
  __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus_val));
  printk("\t\tmstatus = 0x%08lx (MIE bit3=%s)\n", mstatus_val,
         (mstatus_val & BIT(3)) ? "SET" : "CLEAR");

  /*
    Step 1: Register our ISR for PLIC source 3 via Zephyr's IRQ API.

    The IRQ number for 2nd-level PLIC sources uses Zephyr's multi-level encoding.
    For PLIC source N on the first 2nd-level aggregator:
      encoded_irq = (N << 8) | 11
    where 11 = machine external interrupt (the 1st-level parent).
  */
  
  //? irq_connect_dynamic() is not allowed by Zephyr on this architecture
  //*//unsigned int encoded_irq = PLIC_IRQ_ENCODE(UART_TX_EMPTY_SRC);
  ////irq_connect_dynamic(encoded_irq, 1, test_uart_tx_empty_isr, NULL, 0);

  //// Check if slot 11 was corrupted by irq_connect_dynamic 
  ////printk("AFTER irq_connect_dynamic:\n");
  ////printk("\t_sw_isr_table[11].isr = %p (expect %p - xheep_plic_irq_handler)\n",
  ////       _sw_isr_table[11].isr, xheep_plic_irq_handler);
  ////printk("\t_sw_isr_table[15].isr = %p (expect %p - test_uart_tx_empty_isr)\n",
  ////       _sw_isr_table[15].isr, test_uart_tx_empty_isr);

  // FALLBACK: Registering ISRs at compile time
  printk("Registering ISR for PLIC source %u (encoded=0x%x)\n",
         UART_TX_EMPTY_SRC, UART_TX_EMPTY_IRQ);
  IRQ_CONNECT(UART_TX_EMPTY_IRQ, 1, test_uart_tx_empty_isr, NULL, 0);

  /*
    Step 2: Enable the interrupt through Zephyr's API.

    This calls our riscv_plic_irq_enable(3) + riscv_plic_set_priority(3, 1).
  */

  irq_enable(UART_TX_EMPTY_IRQ);
  printk("IRQ enabled via Zephyr API\n");

  /*
    Step 3: Check that UART's TX_EMPTY is already asserted.

    Read the UART INTR_STATE register — bit 2 should be set because the TX FIFO is empty right now.
  */

  uint32_t uart_state = sys_read32(UART_INTR_STATE);
  printk("UART INTR_STATE = 0x%08x (bit2=TX_EMPTY=%s)\n", uart_state,
         (uart_state & UART_INTR_TX_EMPTY_BIT) ? "asserted" : "NOT asserted");

  // Clear the interrupt state in the UART (W1C)
  sys_write32(0xFFFFFFFF, UART_INTR_STATE);

  /*
    Step 3: Enable the UART's TX_EMPTY interrupt output.

    This makes the UART actually drive its intr_tx_empty_o line high, which is connected to PLIC
    source 3.
  */
 
  sys_write32(UART_INTR_TX_EMPTY_BIT, UART_INTR_ENABLE);
  printk("UART INTR_ENABLE set for TX_EMPTY\n");

  /*
    Step 4: Check PLIC pending.
    
    Source 3 should now be pending.
  */

  uint32_t ip_before = sys_read32(PLIC_IP0);
  printk("PLIC IP0 = 0x%08x (bit3=%s)\n", ip_before,
         (ip_before & BIT(UART_TX_EMPTY_SRC)) ? "PENDING" : "not pending");

  /*
    Step 5: Wait briefly for the interrupt to fire.

    The TX_EMPTY line is level-triggered and already high, so the ISR should fire almost
    immediately.
  */

  ////k_busy_wait(1000);  /* 1 ms */

  // FALLBACK: Timer not yet available, active wait
  for (volatile int i = 0; i < 10000; i++) {
    if (isr_fired) {
      break;
    }
  }

  /*
    Step 6: Check results.
  */

  if (isr_fired) {
    printk("[PASS] ISR fired! source=%u count=%u\n",
           isr_source_id, isr_count);
    pass++;
  } else {
    printk("[FAIL] ISR did NOT fire\n");
    fail++;

    // Dump PLIC state
    printk("  DEBUG: IP0=0x%08x IE0=0x%08x THRESH=0x%x\n",
           sys_read32(PLIC_IP0),
           sys_read32(PLIC_IE0_0),
           sys_read32(PLIC_THRESHOLD0));
    printk("  DEBUG: PRIO3=0x%x CC0=0x%x\n",
           sys_read32(PLIC_PRIO_BASE + UART_TX_EMPTY_SRC * 4),
           sys_read32(PLIC_CC0));
  }

  // Disable the PLIC source
  irq_disable(UART_TX_EMPTY_IRQ);

  // Make sure the UART interrupt output is disabled
  sys_write32(0u, UART_INTR_ENABLE);

  // Verify claim returned to 0 (all completed)
  uint32_t cc = sys_read32(PLIC_CC0);
  if (cc == 0) {
    printk("[PASS] Claim=0 after completion\n"); pass++;
  } else {
    printk("[FAIL] Claim=%u still pending after ISR\n", cc); fail++;
    sys_write32(cc, PLIC_CC0);
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main() {

  printk("\n========================================\n");
  printk("\tX-HEEP PLIC Verification Test");
	printk("\t\tBoard: %s", CONFIG_BOARD);
  printk("\n========================================\n");

  int total_fail = 0;
  total_fail += test_seq_1();
  total_fail += test_seq_2();

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}