/*
 * X-HEEP rv_timer Verification Test
 *
 * Sequence 1: Register state    — CTRL.ACTIVE_0=1 and CFG0.step=1 set by driver init
 * Sequence 2: Counter running   — counter value increments between two reads
 * Sequence 3: Counter API       — counter_get_value() agrees with direct MMIO read
 * Sequence 4: Elapsed time      — k_busy_wait() duration matches counter delta (±20%)
 * Sequence 5: Alarm interrupt   — counter_set_channel_alarm() fires callback via FIC source 1
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
    printk("[PASS] CTRL.ACTIVE_0 set\n");
    pass++;
  }
  else {
    printk("[FAIL] CTRL.ACTIVE_0 not set — counter not running\n");
    fail++;
  }

  // CFG0: prescale must be 0, step must be 1
  uint32_t cfg0 = sys_read32(RV_TIMER_CFG0);
  uint32_t prescale = cfg0 & CFG0_PRESCALE_MASK;
  uint32_t step = (cfg0 >> CFG0_STEP_SHIFT) & CFG0_STEP_MASK;
  printk("\tCFG0 = 0x%08x  (prescale=%u expect 0, step=%u expect 1)\n",
         cfg0, prescale, step);
  if (step == 1U && prescale == 0U) {
    printk("[PASS] CFG0 prescale=0, step=1\n");
    pass++;
  }
  else {
    printk("[FAIL] CFG0 unexpected (prescale=%u, step=%u)\n", prescale, step);
    fail++;
  }

  // INTR_ENABLE: must be 0 at init (armed by set_alarm, cleared by ISR)
  uint32_t ie = sys_read32(RV_TIMER_INTR_ENABLE);
  printk("\tINTR_ENABLE = 0x%08x  (expect 0x00000000)\n", ie);
  if (ie == 0U) {
    printk("[PASS] INTR_ENABLE=0\n");
    pass++;
  }
  else {
    printk("[FAIL] INTR_ENABLE non-zero (0x%08x)\n", ie);
    fail++;
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
    printk("[PASS] counter is incrementing\n");
    pass++;
  }
  else {
    printk("[FAIL] counter did not advance — CTRL.ACTIVE_0 may not be set\n");
    fail++;
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
    printk("[FAIL] counter_get_value() returned error %d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  /* Allow small drift between the two reads (counter is free-running) */
  uint32_t drift = (mmio_val >= api_val) ? (mmio_val - api_val) : (api_val - mmio_val);
  printk("\tAPI value:  %u\n", api_val);
  printk("\tMMIO value: %u\n", mmio_val);
  printk("\tDrift: %u cycles\n", drift);

  /* Drift should be neglegible — two back-to-back reads with no work between them */
  if (drift < 1000U) {
    printk("[PASS] counter_get_value() agrees with MMIO read\n");
    pass++;
  }
  else {
    printk("[FAIL] Drift %u cycles too large — API may not read MTIME_LO\n", drift);
    fail++;
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
  printk("\tElapsed: %u cycles  (min=%u, max=%u)\n", elapsed, CYCLES_MIN, CYCLES_MAX);

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


static volatile int alarm_fired;
static volatile uint32_t alarm_cb_ticks;

static void alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data) {
  ARG_UNUSED(dev);
  ARG_UNUSED(chan_id);
  ARG_UNUSED(user_data);
  alarm_cb_ticks = ticks;
  alarm_fired = 1;
}


static int test_seq_5(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Alarm Interrupt Test Sequence ---\n");
  printk("[INFO] counter_set_channel_alarm() should fire via FIC source 1 (mcause 17)\n");

  alarm_fired = 0;
  alarm_cb_ticks = 0U;

  uint32_t now = 0U;
  counter_get_value(dev, &now);

  /* Schedule alarm 500 000 cycles (~5 ms) from now */
  uint32_t delay_ticks = 500000U;
  struct counter_alarm_cfg alarm_cfg = {
    .callback  = alarm_callback,
    .ticks     = delay_ticks,
    .user_data = NULL,
    .flags     = 0,  /* relative */
  };

  int ret = counter_set_channel_alarm(dev, 0, &alarm_cfg);
  printk("\tcounter_set_channel_alarm() ret=%d (expect 0)\n", ret);
  if (ret != 0) {
    printk("[FAIL] counter_set_channel_alarm() returned %d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }
  printk("[PASS] counter_set_channel_alarm() accepted\n");
  pass++;

  /* Verify INTR_ENABLE bit 0 was set by the driver */
  uint32_t ie = sys_read32(RV_TIMER_INTR_ENABLE);
  printk("\tINTR_ENABLE = 0x%08x  (expect 0x00000001)\n", ie);
  if (ie & BIT(0)) {
    printk("[PASS] INTR_ENABLE bit 0 set by driver\n");
    pass++;
  }
  else {
    printk("[FAIL] INTR_ENABLE bit 0 not set\n"); fail++;
  }

  /* Active-wait up to 50 ms for the alarm to fire */
  k_busy_wait(50000U);

  printk("\tAlarm fired: %s\n", alarm_fired ? "yes" : "no");
  if (alarm_fired) {
    printk("[PASS] Alarm callback invoked\n"); pass++;
  } else {
    printk("[FAIL] Alarm did not fire within 50 ms\n"); fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  /* Check timing: callback tick should be close to now + delay_ticks */
  uint32_t expected = now + delay_ticks;
  uint32_t drift = (alarm_cb_ticks >= expected) ? (alarm_cb_ticks - expected) : (expected - alarm_cb_ticks);
  printk("\tExpected tick: %u  Actual tick: %u  Drift: %u\n", expected, alarm_cb_ticks, drift);

  /* Allow ±10 000 cycles drift (~100 µs at 100 MHz) */
  if (drift <= 10000U) {
    printk("[PASS] Alarm fired within ±10 000 cycles of target\n");
    pass++;
  }
  else {
    printk("[FAIL] Alarm drift %u cycles exceeds tolerance\n", drift);
    fail++;
  }

  /* INTR_ENABLE should be cleared by the ISR after firing */
  ie = sys_read32(RV_TIMER_INTR_ENABLE);
  printk("\tINTR_ENABLE after fire = 0x%08x  (expect 0x00000000)\n", ie);
  if (ie == 0U) {
    printk("[PASS] INTR_ENABLE cleared by ISR\n");
    pass++;
  }
  else {
    printk("[FAIL] INTR_ENABLE not cleared after alarm fired\n");
    fail++;
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
  total_fail += test_seq_5(dev);

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
