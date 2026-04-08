/*
  * X-HEEP rv_timer_ao Verification Test
  *
  * Sequence 1: Register state  — CTRL.ACTIVE_0=1 and CFG0.step=1 set by soc_early_init
  * Sequence 2: Counter running — mtime_lo increments between two reads
  * Sequence 3: k_busy_wait()  — 10 ms wait consumes ~1 000 000 mtime cycles (±20%)
  * Sequence 4: Kernel uptime  — k_uptime_get_32() advances after a timed wait
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>


//! CONSTANTS

//* rv_timer_ao register map (IP base via DT ctrl reg-name)
#define RV_TIMER_AO_BASE        DT_REG_ADDR_BY_NAME(DT_NODELABEL(mtimer), ctrl)
#define RV_TIMER_AO_CTRL        (RV_TIMER_AO_BASE + 0x000U)
#define RV_TIMER_AO_CFG0        (RV_TIMER_AO_BASE + 0x100U)
#define RV_TIMER_AO_MTIME_LO    (RV_TIMER_AO_BASE + 0x104U)
#define RV_TIMER_AO_MTIME_HI    (RV_TIMER_AO_BASE + 0x108U)
#define RV_TIMER_AO_INTR_ENABLE (RV_TIMER_AO_BASE + 0x114U)
#define RV_TIMER_AO_INTR_STATE  (RV_TIMER_AO_BASE + 0x118U)

// CTRL bits
#define CTRL_ACTIVE_0  BIT(0)

// CFG0 field positions
#define CFG0_PRESCALE_MASK  0xFFFU
#define CFG0_STEP_SHIFT     16U
#define CFG0_STEP_MASK      0xFFU

//* Timing parameters (100 MHz clock → 100 cycles/µs)
#define CLOCK_HZ        DT_PROP(DT_NODELABEL(mtimer), clock_frequency)
#define BUSY_WAIT_US    10000U    /* 10 ms → 1 000 000 cycles expected */
#define CYCLES_PER_US   (CLOCK_HZ / 1000000U)
#define CYCLES_EXPECTED (BUSY_WAIT_US * CYCLES_PER_US)
#define CYCLES_MIN      (CYCLES_EXPECTED * 8U / 10U)  /* -20% tolerance */
#define CYCLES_MAX      (CYCLES_EXPECTED * 12U / 10U) /* +20% tolerance */

#define UPTIME_WAIT_US  20000U   /* 20 ms → uptime should advance ≥ 18 ms */
#define UPTIME_WAIT_MS  (UPTIME_WAIT_US / 1000U)


//! TEST SEQUENCES

static int test_seq_1() {
  int pass = 0, fail = 0;

  printk("\n--- Register State Test Sequence ---\n");
  printk("[INFO] Verifying soc_early_init set CTRL and CFG0 correctly\n");

  // CTRL: ACTIVE_0 must be 1 (timer running)
  uint32_t ctrl = sys_read32(RV_TIMER_AO_CTRL);
  printk("\tCTRL = 0x%08x  (ACTIVE_0 = %u, expect 1)\n",
         ctrl, !!(ctrl & CTRL_ACTIVE_0));
  if (ctrl & CTRL_ACTIVE_0) {
    printk("[PASS] CTRL.ACTIVE_0 set\n"); pass++;
  } else {
    printk("[FAIL] CTRL.ACTIVE_0 not set — timer not running\n"); fail++;
  }

  // CFG0: prescale must be 0, step must be 1
  uint32_t cfg0 = sys_read32(RV_TIMER_AO_CFG0);
  uint32_t prescale = cfg0 & CFG0_PRESCALE_MASK;
  uint32_t step = (cfg0 >> CFG0_STEP_SHIFT) & CFG0_STEP_MASK;
  printk("\tCFG0 = 0x%08x  (prescale=%u expect 0, step=%u expect 1)\n",
         cfg0, prescale, step);
  if (step == 1U && prescale == 0U) {
    printk("[PASS] CFG0 prescale=0, step=1\n"); pass++;
  } else {
    printk("[FAIL] CFG0 unexpected (prescale=%u, step=%u)\n", prescale, step); fail++;
  }

  // INTR_ENABLE: must be 0 (machine timer interrupt managed by Zephyr driver, not this IP reg)
  uint32_t ie = sys_read32(RV_TIMER_AO_INTR_ENABLE);
  printk("\tINTR_ENABLE = 0x%08x  (expect 0x00000000)\n", ie);
  if (ie == 0U) {
    printk("[PASS] INTR_ENABLE=0\n"); pass++;
  } else {
    printk("[FAIL] INTR_ENABLE non-zero (0x%08x)\n", ie); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_2() {
  int pass = 0, fail = 0;

  printk("\n--- mtime Incrementing Test Sequence ---\n");

  uint32_t hi0 = sys_read32(RV_TIMER_AO_MTIME_HI);
  uint32_t lo0 = sys_read32(RV_TIMER_AO_MTIME_LO);

  /* Spin ~2000 iterations to give the counter time to advance */
  for (volatile int i = 0; i < 2000; i++) {}

  uint32_t lo1 = sys_read32(RV_TIMER_AO_MTIME_LO);
  uint32_t hi1 = sys_read32(RV_TIMER_AO_MTIME_HI);

  uint32_t delta = lo1 - lo0;  /* unsigned subtraction handles wraparound */

  printk("\tmtime before: hi=0x%08x lo=0x%08x\n", hi0, lo0);
  printk("\tmtime after:  hi=0x%08x lo=0x%08x\n", hi1, lo1);
  printk("\tdelta lo: %u cycles\n", delta);

  if (delta > 0U) {
    printk("[PASS] mtime is incrementing\n"); pass++;
  } else {
    printk("[FAIL] mtime did not advance — CTRL.ACTIVE_0 may not be set\n"); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_3() {
  int pass = 0, fail = 0;

  printk("\n--- k_busy_wait() Accuracy Test Sequence ---\n");
  printk("[INFO] Waiting %u µs, expecting ~%u mtime cycles (±20%%)\n",
         BUSY_WAIT_US, CYCLES_EXPECTED);

  uint32_t t0 = sys_read32(RV_TIMER_AO_MTIME_LO);
  k_busy_wait(BUSY_WAIT_US);
  uint32_t t1 = sys_read32(RV_TIMER_AO_MTIME_LO);

  uint32_t elapsed = t1 - t0;
  printk("\tElapsed: %u cycles  (min=%u, max=%u)\n",
         elapsed, CYCLES_MIN, CYCLES_MAX);

  if (elapsed >= CYCLES_MIN && elapsed <= CYCLES_MAX) {
    printk("[PASS] k_busy_wait() timing within ±20%%\n"); pass++;
  } else if (elapsed == 0U) {
    printk("[FAIL] No cycles elapsed — mtime not running\n"); fail++;
  } else {
    printk("[FAIL] Elapsed %u cycles out of tolerance\n", elapsed); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_4() {
  int pass = 0, fail = 0;

  printk("\n--- Kernel Uptime Test Sequence ---\n");
  printk("[INFO] Waiting %u ms, uptime delta should be >= %u ms\n",
         UPTIME_WAIT_MS, UPTIME_WAIT_MS - 2U);

  uint32_t u0 = k_uptime_get_32();
  k_busy_wait(UPTIME_WAIT_US);
  uint32_t u1 = k_uptime_get_32();

  uint32_t delta_ms = u1 - u0;
  printk("\tUptime before: %u ms\n", u0);
  printk("\tUptime after:  %u ms\n", u1);
  printk("\tDelta: %u ms\n", delta_ms);

  if (u0 == 0U && u1 == 0U) {
    /* Both zero means the kernel tick is not advancing at all */
    printk("[FAIL] Uptime stuck at 0 — machine timer interrupt not firing\n"); fail++;
  } else if (delta_ms >= UPTIME_WAIT_MS - 2U) {
    printk("[PASS] Uptime advanced correctly\n"); pass++;
  } else {
    printk("[FAIL] Uptime advanced only %u ms (expected ~%u ms)\n",
           delta_ms, UPTIME_WAIT_MS); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main() {
  printk("\n========================================\n");
  printk("\tX-HEEP rv_timer_ao Verification Test\n");
  printk("\t\tBoard: %s\n", CONFIG_BOARD);
  printk("\t\tClock: %u Hz", CLOCK_HZ);
  printk("\n========================================\n");

  int total_fail = 0;
  total_fail += test_seq_1();
  total_fail += test_seq_2();
  total_fail += test_seq_3();
  total_fail += test_seq_4();

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
