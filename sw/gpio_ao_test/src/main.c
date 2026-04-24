/*
 * X-HEEP GPIO_AO Verification Test
 *
 * Employs the always-on GPIO controller (esl-epfl,x-heep-gpio-ao @ 0x20090000).
 * GPIO_AO has 8 pins (0-7); each pin is routed to a dedicated FIC source (6-13,
 * mcause 22-29) and supports edge and level interrupts.
 *
 * Sequence 1: Device ready + init register state
 * Sequence 2: Output write (pin 0 — set high, set low, verify via GPIO_OUT MMIO)
 * Sequence 3: Toggle (pin 1 — init low, toggle twice, verify GPIO_OUT MMIO)
 * Sequence 4: Port operations (pins 4-7 — masked set, set bits, clear bits, toggle)
 * Sequence 5: Input configure (pin 2 — verify GPIO_EN register)
 * Sequence 6: Open-drain configure (pin 3 — verify GPIO_MODE_0 field = OPEN_DRAIN1)
 * Sequence 7: Interrupt register verify (pin 0 — check GPIO + FIC enable regs)
 * Sequence 8: Interrupt dispatch (pin 0 — level-low via FIC, fires immediately in Verilator)
 *
 * Sequence 7/8 remarks:
 *  In Verilator simulation gpio_ao_in[0] is driven to 0 by default.
 *  Configuring LEVEL_LOW on pin 0 therefore fires immediately: the GPIO IP
 *  asserts INTRPT_STATUS[0], FIC source 6 fires hlic IRQ 22 (mcause 22),
 *  gpio_ao_xheep_isr reads INTRPT_STATUS, clears it, calls xheep_fic_irq_clear,
 *  then dispatches gpio_fire_callbacks.  The callback self-disables to prevent
 *  continuous re-triggering while gpio_ao_in[0] stays low.
 *
 * ? NOTE
 * GPIO_IN cannot be properly verified in Verilator simulation without an
 * external loopback since gpio_ao_in_i is driven by the testbench, not
 * automatically connected to gpio_ao_out_o.  Output state is therefore
 * verified via direct MMIO readback of the GPIO_OUT register.
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>


//! CONSTANTS

//* GPIO_AO and FIC base addresses from device tree
#define GPIO_AO_BASE  DT_REG_ADDR(DT_NODELABEL(gpio_ao))
#define FIC_BASE      DT_REG_ADDR(DT_NODELABEL(fic))

//* GPIO register offsets
#define GPIO_INFO_OFF       0x000U
#define GPIO_MODE_0_OFF     0x008U
#define GPIO_MODE_1_OFF     0x00CU
#define GPIO_EN_OFF         0x080U
#define GPIO_IN_OFF         0x100U
#define GPIO_OUT_OFF        0x180U
#define GPIO_RISE_EN_OFF    0x380U
#define GPIO_FALL_EN_OFF    0x400U
#define GPIO_LVL_HI_EN_OFF  0x480U
#define GPIO_LVL_LO_EN_OFF  0x500U

//* FIC register offsets
#define FIC_ENABLE_OFF      0x008U

//* IO mode values (2-bit per pin)
#define MODE_INPUT_ONLY     0U
#define MODE_OUTPUT_ACTIVE  1U
#define MODE_OPEN_DRAIN0    2U
#define MODE_OPEN_DRAIN1    3U

//* Helper: extract the 2-bit mode field for pin p from a mode register value
#define PIN_MODE(reg_val, pin_in_reg)  (((reg_val) >> ((pin_in_reg) * 2U)) & 0x3U)

//* Test pin assignments
#define PIN_OUT_WRITE   0U
#define PIN_OUT_TOGGLE  1U
#define PIN_INPUT       2U
#define PIN_OPEN_DRAIN  3U
#define PIN_PORT_BASE   4U
#define PIN_IRQ         0U

//* FIC source for PIN_IRQ (pin 0 - FIC source 6 = XHEEP_FIC_SRC_GPIO_AO0)
#define FIC_SRC_PIN_IRQ  6U


//! ISR

static volatile bool     gpio_ao_isr_fired;
static volatile uint32_t gpio_ao_isr_pins;
static struct gpio_callback gpio_ao_cb;

static void gpio_ao_dispatch_callback(
  const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  /*
    Called by gpio_fire_callbacks() from within gpio_ao_xheep_isr.
    Records the fired state and disables the level-low interrupt on pin 0 to
    prevent continuous re-triggering (gpio_ao_in[0] stays 0 in Verilator).
    Runs inside ISR context before mstatus.MIE is re-enabled.
  */
  gpio_ao_isr_fired = true;
  gpio_ao_isr_pins  = pins;
  gpio_pin_interrupt_configure(dev, PIN_IRQ, GPIO_INT_DISABLE);
}


//! TEST SEQUENCES

static int test_seq_1(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 1: Device Ready + Init Register State ---\n");

  // GPIO_AO driver initialized — if we're here it didn't crash
  printk("[PASS] gpio_ao device is ready\n");
  pass++;

  // GPIO_INFO: print version and pin count (read-only; content depends on RTL)
  uint32_t info = sys_read32(GPIO_AO_BASE + GPIO_INFO_OFF);
  printk("[INFO] GPIO_INFO = 0x%08x  (GPIO_CNT=%u, VERSION=%u)\n",
         info, info & 0x3FFU, (info >> 10) & 0x3FFU);

  // GPIO_MODE_0 must be 0 after init (all INPUT_ONLY)
  uint32_t mode0 = sys_read32(GPIO_AO_BASE + GPIO_MODE_0_OFF);
  if (mode0 == 0U) {
    printk("[PASS] GPIO_MODE_0 = 0x%08x (all INPUT_ONLY)\n", mode0);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_MODE_0 = 0x%08x (expected 0)\n", mode0);
    fail++;
  }

  // GPIO_EN must be 0 after init (all sampling disabled)
  uint32_t en = sys_read32(GPIO_AO_BASE + GPIO_EN_OFF);
  if (en == 0U) {
    printk("[PASS] GPIO_EN = 0x%08x (all disabled)\n", en);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_EN = 0x%08x (expected 0)\n", en);
    fail++;
  }

  // All interrupt enable registers must be 0 after init
  uint32_t rise = sys_read32(GPIO_AO_BASE + GPIO_RISE_EN_OFF);
  uint32_t fall = sys_read32(GPIO_AO_BASE + GPIO_FALL_EN_OFF);
  uint32_t lvhi = sys_read32(GPIO_AO_BASE + GPIO_LVL_HI_EN_OFF);
  uint32_t lvlo = sys_read32(GPIO_AO_BASE + GPIO_LVL_LO_EN_OFF);
  if (rise == 0U && fall == 0U && lvhi == 0U && lvlo == 0U) {
    printk("[PASS] All INTRPT_*_EN registers = 0\n");
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_*_EN not all zero: RISE=0x%08x FALL=0x%08x "
           "LVL_HI=0x%08x LVL_LO=0x%08x\n", rise, fall, lvhi, lvlo);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_2(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 2: Output Write (pin %u) ---\n", PIN_OUT_WRITE);

  // Configure pin as push-pull output, initially low
  int ret = gpio_pin_configure(dev, PIN_OUT_WRITE, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
  if (ret == 0) {
    printk("[PASS] gpio_pin_configure(OUTPUT | INIT_LOW) ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_configure returned %d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Verify GPIO_MODE_0 field for pin: should be OUTPUT_ACTIVE (1)
  uint32_t mode0 = sys_read32(GPIO_AO_BASE + GPIO_MODE_0_OFF);
  uint32_t pin_mode = PIN_MODE(mode0, PIN_OUT_WRITE);
  if (pin_mode == MODE_OUTPUT_ACTIVE) {
    printk("[PASS] MODE_0[pin %u] = %u (OUTPUT_ACTIVE)\n", PIN_OUT_WRITE, pin_mode);
    pass++;
  } else {
    printk("[FAIL] MODE_0[pin %u] = %u (expected %u OUTPUT_ACTIVE)\n",
           PIN_OUT_WRITE, pin_mode, MODE_OUTPUT_ACTIVE);
    fail++;
  }

  // Verify INIT_LOW: GPIO_OUT[pin] should be 0
  uint32_t out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_WRITE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 after INIT_LOW\n", PIN_OUT_WRITE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0 after INIT_LOW)\n",
           PIN_OUT_WRITE);
    fail++;
  }

  // Drive pin high, read back GPIO_OUT
  ret = gpio_pin_set_raw(dev, PIN_OUT_WRITE, 1);
  if (ret != 0) {
    printk("[FAIL] gpio_pin_set_raw(1) returned %d\n", ret);
    fail++;
  }
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if (out & BIT(PIN_OUT_WRITE)) {
    printk("[PASS] GPIO_OUT[pin %u] = 1 after set high\n", PIN_OUT_WRITE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 0 (expected 1 after set high)\n", PIN_OUT_WRITE);
    fail++;
  }

  // Drive pin low, read back GPIO_OUT
  ret = gpio_pin_set_raw(dev, PIN_OUT_WRITE, 0);
  if (ret != 0) {
    printk("[FAIL] gpio_pin_set_raw(0) returned %d\n", ret);
    fail++;
  }
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_WRITE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 after set low\n", PIN_OUT_WRITE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0 after set low)\n", PIN_OUT_WRITE);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_3(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 3: Toggle (pin %u) ---\n", PIN_OUT_TOGGLE);

  // Configure as output, init low
  int ret = gpio_pin_configure(dev, PIN_OUT_TOGGLE, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
  if (ret != 0) {
    printk("[FAIL] gpio_pin_configure returned %d\n", ret); fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  uint32_t out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_TOGGLE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 (init low)\n", PIN_OUT_TOGGLE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] != 0 before toggle\n", PIN_OUT_TOGGLE);
    fail++;
  }

  // First toggle: 0 → 1
  ret = gpio_pin_toggle(dev, PIN_OUT_TOGGLE);
  if (ret != 0) {
    printk("[FAIL] gpio_pin_toggle returned %d\n", ret); fail++;
  }
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if (out & BIT(PIN_OUT_TOGGLE)) {
    printk("[PASS] GPIO_OUT[pin %u] = 1 after first toggle\n", PIN_OUT_TOGGLE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 0 (expected 1 after toggle)\n", PIN_OUT_TOGGLE);
    fail++;
  }

  // Second toggle: 1 → 0
  ret = gpio_pin_toggle(dev, PIN_OUT_TOGGLE);
  if (ret != 0) {
    printk("[FAIL] gpio_pin_toggle returned %d\n", ret); fail++;
  }
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_TOGGLE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 after second toggle\n",  PIN_OUT_TOGGLE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0 after second toggle)\n", PIN_OUT_TOGGLE);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_4(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 4: Port Operations (pins %u-%u) ---\n", PIN_PORT_BASE, PIN_PORT_BASE + 3U);

  // Configure all four pins as output-low
  for (uint8_t p = PIN_PORT_BASE; p < PIN_PORT_BASE + 4U; p++) {
    gpio_pin_configure(dev, p, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
  }
  uint32_t nibble_mask = BIT(PIN_PORT_BASE + 3) | BIT(PIN_PORT_BASE + 2) |
                         BIT(PIN_PORT_BASE + 1) | BIT(PIN_PORT_BASE);

  // Verify all four start at 0
  uint32_t out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == 0U) {
    printk("[PASS] All nibble pins low after INIT_LOW\n");
    pass++;
  }
  else {
    printk("[FAIL] Nibble pins not all low: GPIO_OUT=0x%08x\n", out);
    fail++;
  }

  // port_set_bits_raw: set all four high
  gpio_port_set_bits_raw(dev, nibble_mask);
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == nibble_mask) {
    printk("[PASS] port_set_bits_raw: all nibble pins high\n");
    pass++;
  } else {
    printk("[FAIL] port_set_bits_raw: GPIO_OUT=0x%08x (expected mask 0x%08x)\n",
           out, nibble_mask);
    fail++;
  }

  // port_clear_bits_raw: clear all four
  gpio_port_clear_bits_raw(dev, nibble_mask);
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == 0U) {
    printk("[PASS] port_clear_bits_raw: all nibble pins low\n");
    pass++;
  }
  else {
    printk("[FAIL] port_clear_bits_raw: GPIO_OUT=0x%08x (expected nibble=0)\n", out);
    fail++;
  }

  // port_set_masked_raw: set only lower two pins of nibble
  gpio_port_pins_t lo_mask = BIT(PIN_PORT_BASE + 1) | BIT(PIN_PORT_BASE);
  gpio_port_set_masked_raw(dev, nibble_mask, lo_mask);  /* value = lo_mask */
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == lo_mask) {
    printk("[PASS] port_set_masked_raw: lower two set, upper two clear\n");
    pass++;
  }
  else {
    printk("[FAIL] port_set_masked_raw: GPIO_OUT=0x%08x (expected 0x%08x)\n",
           out, lo_mask);
    fail++;
  }

  // port_toggle_bits: toggle all four — lower two go low, upper two go high
  gpio_port_toggle_bits(dev, nibble_mask);
  uint32_t hi_mask = BIT(PIN_PORT_BASE + 3) | BIT(PIN_PORT_BASE + 2);
  out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == hi_mask) {
    printk("[PASS] port_toggle_bits: upper two set, lower two clear\n");
    pass++;
  }
  else {
    printk("[FAIL] port_toggle_bits: GPIO_OUT=0x%08x (expected 0x%08x)\n",
           out, hi_mask);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_5(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 5: Input Configure (pin %u) ---\n", PIN_INPUT);

  // Configure pin as input
  int ret = gpio_pin_configure(dev, PIN_INPUT, GPIO_INPUT);
  if (ret == 0) {
    printk("[PASS] gpio_pin_configure(INPUT) ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_configure(INPUT) returned %d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Verify GPIO_MODE_0 field = INPUT_ONLY (0)
  uint32_t mode0 = sys_read32(GPIO_AO_BASE + GPIO_MODE_0_OFF);
  uint32_t pin_mode = PIN_MODE(mode0, PIN_INPUT);
  if (pin_mode == MODE_INPUT_ONLY) {
    printk("[PASS] MODE_0[pin %u] = %u (INPUT_ONLY)\n", PIN_INPUT, pin_mode);
    pass++;
  }
  else {
    printk("[FAIL] MODE_0[pin %u] = %u (expected %u INPUT_ONLY)\n",
           PIN_INPUT, pin_mode, MODE_INPUT_ONLY);
    fail++;
  }

  // Verify GPIO_EN[pin] = 1 (input sampling enabled)
  uint32_t en = sys_read32(GPIO_AO_BASE + GPIO_EN_OFF);
  if (en & BIT(PIN_INPUT)) {
    printk("[PASS] GPIO_EN[pin %u] = 1 (sampling enabled)\n", PIN_INPUT);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_EN[pin %u] = 0 (expected 1, sampling disabled)\n",
           PIN_INPUT);
    fail++;
  }

  // Read the pin value (in simulation: likely 0 / undefined; just show it)
  int val = gpio_pin_get_raw(dev, PIN_INPUT);
  if (val >= 0) {
    printk("[PASS] gpio_pin_get_raw(pin %u) = %d\n", PIN_INPUT, val); 
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_get_raw(pin %u) returned error %d\n",
           PIN_INPUT, val);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_6(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 6: Open-Drain Configure (pin %u) ---\n", PIN_OPEN_DRAIN);

  /*
    Configure as output with open-drain (OPEN_DRAIN1 in PULP GPIO: 0 - Drive Low, 1 - High-Z).
    ? CAUTION
    Zephyr maps GPIO_OPEN_DRAIN to our MODE_OPEN_DRAIN1.
  */
  int ret = gpio_pin_configure(
    dev, PIN_OPEN_DRAIN, GPIO_OUTPUT | GPIO_OPEN_DRAIN | GPIO_OUTPUT_INIT_LOW);
  if (ret == 0) {
    printk("[PASS] gpio_pin_configure(OUTPUT | OPEN_DRAIN | INIT_LOW) ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_configure returned %d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Verify GPIO_MODE_0 field = OPEN_DRAIN1 (3)
  uint32_t mode0 = sys_read32(GPIO_AO_BASE + GPIO_MODE_0_OFF);
  uint32_t pin_mode = PIN_MODE(mode0, PIN_OPEN_DRAIN);
  if (pin_mode == MODE_OPEN_DRAIN1) {
    printk("[PASS] MODE_0[pin %u] = %u (OPEN_DRAIN1)\n", PIN_OPEN_DRAIN, pin_mode);
    pass++;
  }
  else {
    printk("[FAIL] MODE_0[pin %u] = %u (expected %u OPEN_DRAIN1)\n",
           PIN_OPEN_DRAIN, pin_mode, MODE_OPEN_DRAIN1);
    fail++;
  }

  // Verify output starts low (init_low set)
  uint32_t out = sys_read32(GPIO_AO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OPEN_DRAIN)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 (INIT_LOW)\n",
      PIN_OPEN_DRAIN); pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0)\n",PIN_OPEN_DRAIN);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_7(const struct device *dev) {
  /*
    Verify that pin_interrupt_configure correctly programs both the GPIO IP
    interrupt enable registers and the FIC FAST_INTR_ENABLE register.

    Two sub-steps:
      a) Rising edge: INTRPT_RISE_EN[0]=1, INTRPT_FALL_EN[0]=0, FIC_ENABLE[6]=1
      b) Disable: all INTRPT_*_EN[0]=0, FIC_ENABLE[6]=0
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 7: Interrupt Register Verify (pin %u, FIC src %u) ---\n",
         PIN_IRQ, FIC_SRC_PIN_IRQ);

  // Configure pin as input before arming interrupts
  gpio_pin_configure(dev, PIN_IRQ, GPIO_INPUT);

  int ret;
  uint32_t rise, fall, rise2, fall2, lvhi, lvlo, fic_en;

  // a) Rising edge
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ, GPIO_INT_EDGE_RISING);
  if (ret == 0) {
    printk("[PASS] configure(EDGE_RISING) ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] configure(EDGE_RISING) ret=%d\n", ret);
    fail++;
  }

  rise   = sys_read32(GPIO_AO_BASE + GPIO_RISE_EN_OFF);
  fall   = sys_read32(GPIO_AO_BASE + GPIO_FALL_EN_OFF);
  fic_en = sys_read32(FIC_BASE + FIC_ENABLE_OFF);

  if (rise & BIT(PIN_IRQ)) {
    printk("[PASS] INTRPT_RISE_EN[pin %u] = 1\n", PIN_IRQ);
    pass++;
  } else {
    printk("[FAIL] INTRPT_RISE_EN[pin %u] = 0 (expected 1)\n", PIN_IRQ);
    fail++;
  }
  if (!(fall & BIT(PIN_IRQ))) {
    printk("[PASS] INTRPT_FALL_EN[pin %u] = 0 (not set for rising-only)\n",
           PIN_IRQ);
    pass++;
  } else {
    printk("[FAIL] INTRPT_FALL_EN[pin %u] = 1 (expected 0)\n", PIN_IRQ);
    fail++;
  }
  if (fic_en & BIT(FIC_SRC_PIN_IRQ)) {
    printk("[PASS] FIC_ENABLE[src %u] = 1\n", FIC_SRC_PIN_IRQ);
    pass++;
  } else {
    printk("[FAIL] FIC_ENABLE[src %u] = 0 (expected 1, FIC=0x%08x)\n",
           FIC_SRC_PIN_IRQ, fic_en);
    fail++;
  }

  // b) Disable
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ, GPIO_INT_DISABLE);
  if (ret == 0) {
    printk("[PASS] configure(DISABLE) ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] configure(DISABLE) ret=%d\n", ret);
    fail++;
  }

  rise2  = sys_read32(GPIO_AO_BASE + GPIO_RISE_EN_OFF);
  fall2  = sys_read32(GPIO_AO_BASE + GPIO_FALL_EN_OFF);
  lvhi   = sys_read32(GPIO_AO_BASE + GPIO_LVL_HI_EN_OFF);
  lvlo   = sys_read32(GPIO_AO_BASE + GPIO_LVL_LO_EN_OFF);
  fic_en = sys_read32(FIC_BASE + FIC_ENABLE_OFF);

  if (!(rise2 & BIT(PIN_IRQ)) && !(fall2 & BIT(PIN_IRQ)) &&
      !(lvhi  & BIT(PIN_IRQ)) && !(lvlo  & BIT(PIN_IRQ))) {
    printk("[PASS] All INTRPT_*_EN[pin %u] cleared after DISABLE\n", PIN_IRQ);
    pass++;
  } else {
    printk("[FAIL] INTRPT_*_EN not all zero: RISE=0x%08x FALL=0x%08x "
           "LVL_HI=0x%08x LVL_LO=0x%08x\n", rise2, fall2, lvhi, lvlo);
    fail++;
  }
  if (!(fic_en & BIT(FIC_SRC_PIN_IRQ))) {
    printk("[PASS] FIC_ENABLE[src %u] cleared\n", FIC_SRC_PIN_IRQ);
    pass++;
  }
  else {
    printk("[FAIL] FIC_ENABLE[src %u] still 1 (expected 0)\n", FIC_SRC_PIN_IRQ);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_8(const struct device *dev) {
  /*
    Interrupt dispatch test via level-low on pin 0 (FIC source 6, mcause 22).
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 8: Interrupt Dispatch (pin %u, LEVEL_LOW via FIC) ---\n", PIN_IRQ);

  gpio_ao_isr_fired = false;
  gpio_ao_isr_pins  = 0U;

  // Configure pin as input (sampling required for interrupt detection)
  int ret = gpio_pin_configure(dev, PIN_IRQ, GPIO_INPUT);
  if (ret != 0) {
    printk("[FAIL] gpio_pin_configure(INPUT) ret=%d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Register the dispatch callback for pin 0
  gpio_init_callback(&gpio_ao_cb, gpio_ao_dispatch_callback, BIT(PIN_IRQ));
  ret = gpio_add_callback(dev, &gpio_ao_cb);
  if (ret == 0) {
    printk("[PASS] gpio_add_callback ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] gpio_add_callback ret=%d\n", ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Arm level-low — fires immediately since gpio_ao_in[0]=0 in Verilator
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ, GPIO_INT_LEVEL_LOW);
  if (ret == 0) {
    printk("[PASS] gpio_pin_interrupt_configure(LEVEL_LOW) ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_interrupt_configure(LEVEL_LOW) ret=%d\n", ret);
    fail++;
  }

  // Brief busy-wait for ISR to fire (~1 ms at 100 MHz)
  k_busy_wait(1000U);

  if (gpio_ao_isr_fired) {
    printk("[PASS] ISR fired: pins=0x%08x\n", gpio_ao_isr_pins);
    pass++;
  }
  else {
    printk("[FAIL] ISR did not fire within timeout\n");
    printk("\t(check: gpio_ao_in[0] must be 0 in testbench)\n");
    fail++;
  }

  if (gpio_ao_isr_pins & BIT(PIN_IRQ)) {
    printk("[PASS] Callback reported pin %u in mask\n", PIN_IRQ);
    pass++;
  }
  else {
    printk("[FAIL] Callback pin mask = 0x%08x (expected BIT(%u))\n",
           gpio_ao_isr_pins, PIN_IRQ);
    fail++;
  }

  // Clean up: remove callback
  gpio_remove_callback(dev, &gpio_ao_cb);

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main(void) {
  printk("\n========================================\n");
  printk("\tX-HEEP GPIO_AO Verification Test\n");
  printk("\t\tBoard: %s\n", CONFIG_BOARD);
  printk("========================================\n");

  const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gpio_ao));

  if (!device_is_ready(dev)) {
    printk("[FAIL] gpio_ao device not ready\n");
    return -1;
  }

  int total_fail = 0;
  total_fail += test_seq_1(dev);
  total_fail += test_seq_2(dev);
  total_fail += test_seq_3(dev);
  total_fail += test_seq_4(dev);
  total_fail += test_seq_5(dev);
  total_fail += test_seq_6(dev);
  total_fail += test_seq_7(dev);
  total_fail += test_seq_8(dev);

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
