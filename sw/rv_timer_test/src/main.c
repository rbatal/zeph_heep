/*
 * X-HEEP rv_timer Verification Test
 *
 * Sequence 1: Register state    — CTRL.ACTIVE_0=1 and CFG0.step=1 set by driver init
 * Sequence 2: Counter running   — counter value increments between two reads
 * Sequence 3: Counter API       — counter_get_value() agrees with direct MMIO read
 * Sequence 4: Elapsed time      — k_busy_wait() duration matches counter delta (±20%)
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>


//! CONSTANTS

//* rv_timer register map (IP base from DT)
#define RV_TIMER_BASE        DT_REG_ADDR(DT_NODELABEL(rv_timer))
#define RV_TIMER_CTRL        (RV_TIMER_BASE + 0x000U)
#define RV_TIMER_CFG0        (RV_TIMER_BASE + 0x100U)
#define RV_TIMER_MTIME_LO    (RV_TIMER_BASE + 0x104U)
#define RV_TIMER_MTIME_HI    (RV_TIMER_BASE + 0x108U)
#define RV_TIMER_INTR_ENABLE (RV_TIMER_BASE + 0x114U)
#define RV_TIMER_INTR_STATE  (RV_TIMER_BASE + 0x118U)

// CTRL bits
#define CTRL_ACTIVE_0  BIT(0)

// CFG0 field positions
#define CFG0_PRESCALE_MASK  0xFFFU
#define CFG0_STEP_SHIFT     16U
#define CFG0_STEP_MASK      0xFFU

//* Timing parameters (100 MHz clock → 100 cycles/µs)
#define CLOCK_HZ        DT_PROP(DT_NODELABEL(rv_timer), clock_frequency)
#define BUSY_WAIT_US    10000U    /* 10 ms → 1 000 000 cycles expected */
#define CYCLES_PER_US   (CLOCK_HZ / 1000000U)
#define CYCLES_EXPECTED (BUSY_WAIT_US * CYCLES_PER_US)
#define CYCLES_MIN      (CYCLES_EXPECTED * 8U / 10U)  /* -20% tolerance */
#define CYCLES_MAX      (CYCLES_EXPECTED * 12U / 10U) /* +20% tolerance */


//! TEST SEQUENCES

static int test_seq_1() {
  int pass = 0, fail = 0;

  printk("\n--- Register State Test Sequence ---\n");
  printk("[INFO] Verifying driver init set CTRL and CFG0 correctly\n");

  // CTRL: ACTIVE_0 must be 1 (counter running)
  uint32_t ctrl = sys_read32(RV_TIMER_CTRL);
  printk("\tCTRL = 0x%08x  (ACTIVE_0 = %u, expect 1)\n",
         ctrl, !!(ctrl & CTRL_ACTIVE_0));
  if (ctrl & CTRL_ACTIVE_0) {
    printk("[PASS] CTRL.ACTIVE_0 set\n"); pass++;
  } else {
    printk("[FAIL] CTRL.ACTIVE_0 not set — counter not running\n"); fail++;
  }

  // CFG0: prescale must be 0, step must be 1
  uint32_t cfg0 = sys_read32(RV_TIMER_CFG0);
  uint32_t prescale = cfg0 & CFG0_PRESCALE_MASK;
  uint32_t step = (cfg0 >> CFG0_STEP_SHIFT) & CFG0_STEP_MASK;
  printk("\tCFG0 = 0x%08x  (prescale=%u expect 0, step=%u expect 1)\n",
         cfg0, prescale, step);
  if (step == 1U && prescale == 0U) {
    printk("[PASS] CFG0 prescale=0, step=1\n"); pass++;
  } else {
    printk("[FAIL] CFG0 unexpected (prescale=%u, step=%u)\n", prescale, step); fail++;
  }

  // INTR_ENABLE: must be 0 (alarm/IRQ not yet supported via fast-IRQ)
  uint32_t ie = sys_read32(RV_TIMER_INTR_ENABLE);
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

  printk("\n--- Counter Incrementing Test Sequence ---\n");

  uint32_t hi0 = sys_read32(RV_TIMER_MTIME_HI);
  uint32_t lo0 = sys_read32(RV_TIMER_MTIME_LO);

  /* Spin ~2000 iterations to give the counter time to advance */
  for (volatile int i = 0; i < 2000; i++) {}

  uint32_t lo1 = sys_read32(RV_TIMER_MTIME_LO);
  uint32_t hi1 = sys_read32(RV_TIMER_MTIME_HI);

  uint32_t delta = lo1 - lo0;  /* unsigned subtraction handles wraparound */

  printk("\tcounter before: hi=0x%08x lo=0x%08x\n", hi0, lo0);
  printk("\tcounter after:  hi=0x%08x lo=0x%08x\n", hi1, lo1);
  printk("\tdelta lo: %u cycles\n", delta);

  if (delta > 0U) {
    printk("[PASS] counter is incrementing\n"); pass++;
  } else {
    printk("[FAIL] counter did not advance — CTRL.ACTIVE_0 may not be set\n"); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_3(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Counter API vs MMIO Test Sequence ---\n");
  printk("[INFO] counter_get_value() should match direct MMIO read of MTIME_LO\n");

  uint32_t api_val = 0U;
  int ret = counter_get_value(dev, &api_val);
  uint32_t mmio_val = sys_read32(RV_TIMER_MTIME_LO);

  if (ret != 0) {
    printk("[FAIL] counter_get_value() returned error %d\n", ret); fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  /* Allow small drift between the two reads (counter is free-running) */
  uint32_t drift = (mmio_val >= api_val) ? (mmio_val - api_val)
                                         : (api_val - mmio_val);
  printk("\tAPI value:  %u\n", api_val);
  printk("\tMMIO value: %u\n", mmio_val);
  printk("\tDrift: %u cycles\n", drift);

  /* Drift should be negligible — two back-to-back reads with no work between them */
  if (drift < 1000U) {
    printk("[PASS] counter_get_value() agrees with MMIO read\n"); pass++;
  } else {
    printk("[FAIL] Drift %u cycles too large — API may not read MTIME_LO\n", drift); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_4() {
  int pass = 0, fail = 0;

  printk("\n--- Elapsed Time Test Sequence ---\n");
  printk("[INFO] Waiting %u µs, expecting ~%u cycles (±20%%)\n",
         BUSY_WAIT_US, CYCLES_EXPECTED);

  uint32_t t0 = sys_read32(RV_TIMER_MTIME_LO);
  k_busy_wait(BUSY_WAIT_US);
  uint32_t t1 = sys_read32(RV_TIMER_MTIME_LO);

  uint32_t elapsed = t1 - t0;
  printk("\tElapsed: %u cycles  (min=%u, max=%u)\n",
         elapsed, CYCLES_MIN, CYCLES_MAX);

  if (elapsed >= CYCLES_MIN && elapsed <= CYCLES_MAX) {
    printk("[PASS] k_busy_wait() timing within ±20%%\n"); pass++;
  } else if (elapsed == 0U) {
    printk("[FAIL] No cycles elapsed — counter not running\n"); fail++;
  } else {
    printk("[FAIL] Elapsed %u cycles out of tolerance\n", elapsed); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main() {
  printk("\n========================================\n");
  printk("\tX-HEEP rv_timer Verification Test\n");
  printk("\t\tBoard: %s\n", CONFIG_BOARD);
  printk("\t\tClock: %u Hz", CLOCK_HZ);
  printk("\n========================================\n");

  const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(rv_timer));
  if (!device_is_ready(dev)) {
    printk("[FAIL] rv_timer device not ready\n");
    return -1;
  }

  int total_fail = 0;
  total_fail += test_seq_1();
  total_fail += test_seq_2();
  total_fail += test_seq_3(dev);
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
