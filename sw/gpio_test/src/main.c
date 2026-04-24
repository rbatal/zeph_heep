/*
 * X-HEEP GPIO Verification Test
 *
 * Employs the user-domain GPIO controller (esl-epfl,x-heep-gpio @ 0x30020000).
 * 24 pins; pins 8-31 are routed to PLIC sources 9-32 and support interrupts.
 *
 * Sequence 1: Device ready + init register state
 * Sequence 2: Output write (pin 0 — set high, set low, verify via GPIO_OUT MMIO)
 * Sequence 3: Toggle (pin 1 — init low, toggle twice, verify GPIO_OUT MMIO)
 * Sequence 4: Port operations (pins 4-7 — masked set, set bits, clear bits, toggle)
 * Sequence 5: Input configure (pin 2 — verify GPIO_EN register)
 * Sequence 6: Open-drain configure (pin 3 — verify GPIO_MODE_0 field = OPEN_DRAIN1)
 * Sequence 7: Interrupt register state (pins 9-12 — verify hardware enable regs + PLIC IE)
 * Sequence 8: Interrupt on non-IRQ pins 0-7 returns -ENOTSUP
 * Sequence 9: Interrupt dispatch via level-low callback (pin 8)
 *
 * Sequence 9 remarks:
 *  In Verilator simulation gpio_in_i is driven to 0 by the testbench,
 *  so pin 8 reads as low by default.
 *  Configuring LEVEL_LOW on pin 8 therefore fires immediate: the PLIC claims
 *  source 9, our PLIC ISR dispatches to gpio_xheep_isr, which reads
 *  INTRPT_STATUS and calls gpio_fire_callbacks. The callback self-disables
 *  to prevent continuous re-triggering.
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>


//! CONSTANTS

//* GPIO base address from device tree
#define GPIO_BASE   DT_REG_ADDR(DT_NODELABEL(gpio0))
#define PLIC_BASE   DT_REG_ADDR(DT_NODELABEL(plic))

//* Register offsets
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
#define GPIO_INTR_STAT_OFF  0x580U

//* PLIC interrupt-enable register
#define PLIC_IE0_0  (PLIC_BASE + 0x200U)

//* IO mode values (2-bit per pin)
#define MODE_INPUT_ONLY     0U
#define MODE_OUTPUT_ACTIVE  1U
#define MODE_OPEN_DRAIN0    2U
#define MODE_OPEN_DRAIN1    3U

//* Helper: extract the 2-bit mode field for pin p within its mode register
#define PIN_MODE(reg_val, pin_in_reg)  (((reg_val) >> ((pin_in_reg) * 2U)) & 0x3U)

//* Test pin assignments
#define PIN_OUT_WRITE    0U
#define PIN_OUT_TOGGLE   1U
#define PIN_INPUT        2U
#define PIN_OPEN_DRAIN   3U
#define PIN_PORT_BASE    4U
#define PIN_IRQ_DISPATCH 8U
#define PIN_IRQ_RISE     9U
#define PIN_IRQ_FALL    10U
#define PIN_IRQ_BOTH    11U
#define PIN_IRQ_LVLHI   12U


//! ISR

static volatile bool     gpio_isr_fired;
static volatile uint32_t gpio_isr_pins;
static struct gpio_callback gpio_test_cb;

static void gpio_dispatch_callback(
  const struct device *dev, struct gpio_callback *cb, uint32_t pins ){
  /*
    Called by gpio_fire_callbacks() from within gpio_xheep_isr.
    Sets the volatile flags, then disables the level-low interrupt for pin 8
    to prevent continuous re-triggering (pin stays low > hardware would keep
    asserting INTRPT_STATUS[8]).

    ? CAUTION
    This callback runs while the PLIC claim is still in flight
    (xheep_plic_irq_handler calls entry->isr BEFORE riscv_plic_irq_complete).
    Disabling the interrupt here ensures the PLIC source is no longer
    asserted at the moment the claim is completed.
  */
  ARG_UNUSED(cb);

  gpio_isr_fired = true;
  gpio_isr_pins  = pins;

  // Disable interrupt to stop level-triggered re-fire
  gpio_pin_interrupt_configure(dev, PIN_IRQ_DISPATCH, GPIO_INT_DISABLE);
}


//! TEST SEQUENCES

static int test_seq_1(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 1: Device Ready + Init Register State ---\n");

  printk("[PASS] gpio0 device is ready\n");
  pass++;

  uint32_t info = sys_read32(GPIO_BASE + GPIO_INFO_OFF);
  printk("[INFO] GPIO_INFO = 0x%08x  (GPIO_CNT=%u, VERSION=%u)\n",
    info, info & 0x3FFU, (info >> 10) & 0x3FFU);

  // GPIO_MODE_0 and GPIO_MODE_1 must be 0 after init (all INPUT_ONLY)
  uint32_t mode0 = sys_read32(GPIO_BASE + GPIO_MODE_0_OFF);
  uint32_t mode1 = sys_read32(GPIO_BASE + GPIO_MODE_1_OFF);
  if (mode0 == 0U && mode1 == 0U) {
    printk("[PASS] GPIO_MODE_0/1 = 0 (all INPUT_ONLY)\n");
    pass++;
  }
  else {
    printk("[FAIL] MODE_0=0x%08x MODE_1=0x%08x (expected 0)\n",
           mode0, mode1);
    fail++;
  }

  // GPIO_EN must be 0 (all sampling disabled)
  uint32_t en = sys_read32(GPIO_BASE + GPIO_EN_OFF);
  if (en == 0U) {
    printk("[PASS] GPIO_EN = 0x%08x (all disabled)\n", en);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_EN = 0x%08x (expected 0)\n", en);
    fail++;
  }

  // All interrupt enable registers must be 0
  uint32_t rise = sys_read32(GPIO_BASE + GPIO_RISE_EN_OFF);
  uint32_t fall = sys_read32(GPIO_BASE + GPIO_FALL_EN_OFF);
  uint32_t lvhi = sys_read32(GPIO_BASE + GPIO_LVL_HI_EN_OFF);
  uint32_t lvlo = sys_read32(GPIO_BASE + GPIO_LVL_LO_EN_OFF);
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

  // Verify MODE_0 field for pin = OUTPUT_ACTIVE (1)
  uint32_t mode0 = sys_read32(GPIO_BASE + GPIO_MODE_0_OFF);
  uint32_t pin_mode = PIN_MODE(mode0, PIN_OUT_WRITE);
  if (pin_mode == MODE_OUTPUT_ACTIVE) {
    printk("[PASS] MODE_0[pin %u] = %u (OUTPUT_ACTIVE)\n", PIN_OUT_WRITE, pin_mode);
    pass++;
  }
  else {
    printk("[FAIL] MODE_0[pin %u] = %u (expected %u)\n",
      PIN_OUT_WRITE, pin_mode, MODE_OUTPUT_ACTIVE);
    fail++;
  }

  // Verify INIT_LOW: GPIO_OUT[pin] = 0
  uint32_t out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_WRITE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 after INIT_LOW\n", PIN_OUT_WRITE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] != 0 after INIT_LOW\n", PIN_OUT_WRITE);
    fail++;
  }

  // Set high
  gpio_pin_set_raw(dev, PIN_OUT_WRITE, 1);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if (out & BIT(PIN_OUT_WRITE)) {
    printk("[PASS] GPIO_OUT[pin %u] = 1 after set high\n", PIN_OUT_WRITE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 0 (expected 1)\n", PIN_OUT_WRITE);
    fail++;
  }

  // Set low
  gpio_pin_set_raw(dev, PIN_OUT_WRITE, 0);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_WRITE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 after set low\n", PIN_OUT_WRITE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0)\n", PIN_OUT_WRITE);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_3(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 3: Toggle (pin %u) ---\n", PIN_OUT_TOGGLE);

  gpio_pin_configure(dev, PIN_OUT_TOGGLE, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);

  uint32_t out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_TOGGLE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 (init low)\n", PIN_OUT_TOGGLE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] != 0 before toggle\n", PIN_OUT_TOGGLE);
    fail++;
  }

  // Toggle 0 > 1
  gpio_pin_toggle(dev, PIN_OUT_TOGGLE);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if (out & BIT(PIN_OUT_TOGGLE)) {
    printk("[PASS] GPIO_OUT[pin %u] = 1 after first toggle\n", PIN_OUT_TOGGLE);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 0 (expected 1)\n", PIN_OUT_TOGGLE);
    fail++;
  }

  // Toggle 1 > 0
  gpio_pin_toggle(dev, PIN_OUT_TOGGLE);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OUT_TOGGLE)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 after second toggle\n", PIN_OUT_TOGGLE);
    pass++;
  } 
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0)\n", PIN_OUT_TOGGLE);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_4(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 4: Port Operations (pins %u-%u) ---\n", PIN_PORT_BASE, PIN_PORT_BASE + 3U);

  for (uint8_t p = PIN_PORT_BASE; p < PIN_PORT_BASE + 4U; p++) {
    gpio_pin_configure(dev, p, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
  }

  gpio_port_pins_t nibble_mask = BIT(PIN_PORT_BASE + 3) | BIT(PIN_PORT_BASE + 2) |
    BIT(PIN_PORT_BASE + 1) | BIT(PIN_PORT_BASE);

  uint32_t out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == 0U) {
    printk("[PASS] All nibble pins low after INIT_LOW\n");
    pass++;
  }
  else {
    printk("[FAIL] Nibble pins not all low: GPIO_OUT=0x%08x\n", out);
    fail++;
  }

  // Set all high
  gpio_port_set_bits_raw(dev, nibble_mask);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == nibble_mask) {
    printk("[PASS] port_set_bits_raw: all nibble pins high\n");
    pass++;
  }
  else {
    printk("[FAIL] port_set_bits_raw: GPIO_OUT=0x%08x (exp mask 0x%08x)\n",
      out, nibble_mask);
    fail++;
  }

  // Clear all
  gpio_port_clear_bits_raw(dev, nibble_mask);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == 0U) {
    printk("[PASS] port_clear_bits_raw: all nibble pins low\n");
    pass++;
  }
  else {
    printk("[FAIL] port_clear_bits_raw: GPIO_OUT=0x%08x (expected nibble=0)\n", out);
    fail++;
  }

  // Masked set: only lower two of nibble
  gpio_port_pins_t lo_mask = BIT(PIN_PORT_BASE + 1) | BIT(PIN_PORT_BASE);
  gpio_port_set_masked_raw(dev, nibble_mask, lo_mask);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == lo_mask) {
    printk("[PASS] port_set_masked_raw: lower two set, upper two clear\n");
    pass++;
  }
  else {
    printk("[FAIL] port_set_masked_raw: GPIO_OUT=0x%08x (exp 0x%08x)\n",
      out, lo_mask);
    fail++;
  }

  // Toggle all: lower two - low, upper two - high
  gpio_port_toggle_bits(dev, nibble_mask);
  gpio_port_pins_t hi_mask = BIT(PIN_PORT_BASE + 3) | BIT(PIN_PORT_BASE + 2);
  out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & nibble_mask) == hi_mask) {
    printk("[PASS] port_toggle_bits: upper two set, lower two clear\n");
    pass++;
  }
  else {
    printk("[FAIL] port_toggle_bits: GPIO_OUT=0x%08x (exp 0x%08x)\n",
      out, hi_mask);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_5(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 5: Input Configure (pin %u) ---\n", PIN_INPUT);

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

  // MODE_0[pin] = INPUT_ONLY (0)
  uint32_t mode0 = sys_read32(GPIO_BASE + GPIO_MODE_0_OFF);
  uint32_t pin_mode = PIN_MODE(mode0, PIN_INPUT);
  if (pin_mode == MODE_INPUT_ONLY) {
    printk("[PASS] MODE_0[pin %u] = %u (INPUT_ONLY)\n", PIN_INPUT, pin_mode);
    pass++;
  }
  else {
    printk("[FAIL] MODE_0[pin %u] = %u (expected 0 INPUT_ONLY)\n",
      PIN_INPUT, pin_mode);
    fail++;
  }

  // GPIO_EN[pin] = 1
  uint32_t en = sys_read32(GPIO_BASE + GPIO_EN_OFF);
  if (en & BIT(PIN_INPUT)) {
    printk("[PASS] GPIO_EN[pin %u] = 1 (sampling enabled)\n", PIN_INPUT);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_EN[pin %u] = 0 (expected 1)\n", PIN_INPUT);
    fail++;
  }

  int val = gpio_pin_get_raw(dev, PIN_INPUT);
  if (val >= 0) {
    printk("[PASS] gpio_pin_get_raw(pin %u) = %d\n", PIN_INPUT, val);
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_get_raw(pin %u) error %d\n", PIN_INPUT, val);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_6(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 6: Open-Drain Configure (pin %u) ---\n", PIN_OPEN_DRAIN);

  int ret = gpio_pin_configure(dev, PIN_OPEN_DRAIN,
    GPIO_OUTPUT | GPIO_OPEN_DRAIN | GPIO_OUTPUT_INIT_LOW);
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

  // MODE_0[pin] = OPEN_DRAIN1 (3)
  uint32_t mode0 = sys_read32(GPIO_BASE + GPIO_MODE_0_OFF);
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

  uint32_t out = sys_read32(GPIO_BASE + GPIO_OUT_OFF);
  if ((out & BIT(PIN_OPEN_DRAIN)) == 0U) {
    printk("[PASS] GPIO_OUT[pin %u] = 0 (INIT_LOW)\n", PIN_OPEN_DRAIN);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_OUT[pin %u] = 1 (expected 0)\n", PIN_OPEN_DRAIN);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_7(const struct device *dev) {
  /*
    Configure different interrupt types on pins 9-12 and verify the hardware
    enable registers AND the PLIC IE register reflect the expected state.

    PLIC source mapping: GPIO pin p-> PLIC source (p + 1)
      Pin  9 -> source 10 -> bit 10 of PLIC_IE0_0
      Pin 10 -> source 11 -> bit 11 of PLIC_IE0_0
      Pin 11 -> source 12 -> bit 12 of PLIC_IE0_0
      Pin 12 -> source 13 -> bit 13 of PLIC_IE0_0
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 7: Interrupt Register Verification (pins %u-%u) ---\n",
         PIN_IRQ_RISE, PIN_IRQ_LVLHI);

  // Configure all test pins as inputs first (interrupt detection requires sampling)
  gpio_pin_configure(dev, PIN_IRQ_RISE,  GPIO_INPUT);
  gpio_pin_configure(dev, PIN_IRQ_FALL,  GPIO_INPUT);
  gpio_pin_configure(dev, PIN_IRQ_BOTH,  GPIO_INPUT);
  gpio_pin_configure(dev, PIN_IRQ_LVLHI, GPIO_INPUT);

  int ret;

  // — Rising edge on pin 9 —
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ_RISE, GPIO_INT_EDGE_RISING);
  if (ret == 0) {
    printk("[PASS] configure(pin %u, EDGE_RISING) ret=0\n", PIN_IRQ_RISE);
    pass++;
  }
  else {
    printk("[FAIL] configure(pin %u, EDGE_RISING) ret=%d\n", PIN_IRQ_RISE, ret);
    fail++;
  }

  uint32_t rise_en = sys_read32(GPIO_BASE + GPIO_RISE_EN_OFF);
  uint32_t fall_en = sys_read32(GPIO_BASE + GPIO_FALL_EN_OFF);
  uint32_t plic_ie = sys_read32(PLIC_IE0_0);

  if (rise_en & BIT(PIN_IRQ_RISE)) {
    printk("[PASS] INTRPT_RISE_EN[pin %u] = 1\n", PIN_IRQ_RISE);
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_RISE_EN[pin %u] = 0 (expected 1)\n", PIN_IRQ_RISE);
    fail++;
  }

  if ((fall_en & BIT(PIN_IRQ_RISE)) == 0U) {
    printk("[PASS] INTRPT_FALL_EN[pin %u] = 0 (not set for rising-only)\n",
      PIN_IRQ_RISE);
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_FALL_EN[pin %u] = 1 (expected 0)\n", PIN_IRQ_RISE);
    fail++;
  }

  if (plic_ie & BIT(PIN_IRQ_RISE + 1U)) {
    printk("[PASS] PLIC_IE0_0[src %u] = 1 (PLIC source enabled)\n",
      PIN_IRQ_RISE + 1U);
    pass++;
  } else {
    printk("[FAIL] PLIC_IE0_0[src %u] = 0 (PLIC source not enabled)\n",
      PIN_IRQ_RISE + 1U);
    fail++;
  }

  // — Falling edge on pin 10 —
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ_FALL, GPIO_INT_EDGE_FALLING);
  if (ret == 0) {
    printk("[PASS] configure(pin %u, EDGE_FALLING) ret=0\n", PIN_IRQ_FALL);
    pass++;
  }
  else {
    printk("[FAIL] configure(pin %u, EDGE_FALLING) ret=%d\n",
      PIN_IRQ_FALL, ret);
    fail++;
  }

  fall_en = sys_read32(GPIO_BASE + GPIO_FALL_EN_OFF);
  if (fall_en & BIT(PIN_IRQ_FALL)) {
    printk("[PASS] INTRPT_FALL_EN[pin %u] = 1\n", PIN_IRQ_FALL);
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_FALL_EN[pin %u] = 0 (expected 1)\n", PIN_IRQ_FALL);
    fail++;
  }

  // — Both edges on pin 11 —
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ_BOTH, GPIO_INT_EDGE_BOTH);
  if (ret == 0) {
    printk("[PASS] configure(pin %u, EDGE_BOTH) ret=0\n", PIN_IRQ_BOTH);
    pass++;
  }
  else {
    printk("[FAIL] configure(pin %u, EDGE_BOTH) ret=%d\n",
      PIN_IRQ_BOTH, ret);
    fail++;
  }

  rise_en = sys_read32(GPIO_BASE + GPIO_RISE_EN_OFF);
  fall_en = sys_read32(GPIO_BASE + GPIO_FALL_EN_OFF);
  if ((rise_en & BIT(PIN_IRQ_BOTH)) && (fall_en & BIT(PIN_IRQ_BOTH))) {
    printk("[PASS] RISE_EN[pin %u] = 1 and FALL_EN[pin %u] = 1 (both edges)\n",
      PIN_IRQ_BOTH, PIN_IRQ_BOTH);
    pass++;
  }
  else {
    printk("[FAIL] RISE_EN[pin %u]=%u FALL_EN[pin %u]=%u (both expected 1)\n",
      PIN_IRQ_BOTH, !!(rise_en & BIT(PIN_IRQ_BOTH)),
      PIN_IRQ_BOTH, !!(fall_en & BIT(PIN_IRQ_BOTH)));
    fail++;
  }

  // — Level-high on pin 12 —
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ_LVLHI, GPIO_INT_LEVEL_HIGH);
  if (ret == 0) {
    printk("[PASS] configure(pin %u, LEVEL_HIGH) ret=0\n", PIN_IRQ_LVLHI);
    pass++;
  }
  else {
    printk("[FAIL] configure(pin %u, LEVEL_HIGH) ret=%d\n", PIN_IRQ_LVLHI, ret);
    fail++;
  }

  uint32_t lvhi_en = sys_read32(GPIO_BASE + GPIO_LVL_HI_EN_OFF);
  if (lvhi_en & BIT(PIN_IRQ_LVLHI)) {
    printk("[PASS] INTRPT_LVL_HI_EN[pin %u] = 1\n", PIN_IRQ_LVLHI);
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_LVL_HI_EN[pin %u] = 0 (expected 1)\n", PIN_IRQ_LVLHI);
    fail++;
  }

  // Disable all four and verify registers cleared
  gpio_pin_interrupt_configure(dev, PIN_IRQ_RISE,  GPIO_INT_DISABLE);
  gpio_pin_interrupt_configure(dev, PIN_IRQ_FALL,  GPIO_INT_DISABLE);
  gpio_pin_interrupt_configure(dev, PIN_IRQ_BOTH,  GPIO_INT_DISABLE);
  gpio_pin_interrupt_configure(dev, PIN_IRQ_LVLHI, GPIO_INT_DISABLE);

  rise_en = sys_read32(GPIO_BASE + GPIO_RISE_EN_OFF);
  fall_en = sys_read32(GPIO_BASE + GPIO_FALL_EN_OFF);
  lvhi_en = sys_read32(GPIO_BASE + GPIO_LVL_HI_EN_OFF);
  plic_ie  = sys_read32(PLIC_IE0_0);

  uint32_t test_mask = BIT(PIN_IRQ_RISE) | BIT(PIN_IRQ_FALL) |
                       BIT(PIN_IRQ_BOTH) | BIT(PIN_IRQ_LVLHI);
  uint32_t plic_mask = BIT(PIN_IRQ_RISE + 1U) | BIT(PIN_IRQ_FALL + 1U) |
                       BIT(PIN_IRQ_BOTH + 1U) | BIT(PIN_IRQ_LVLHI + 1U);

  if ((rise_en & test_mask) == 0U && (fall_en & test_mask) == 0U &&
      (lvhi_en & test_mask) == 0U) {
    printk("[PASS] All INTRPT_*_EN bits cleared after GPIO_INT_DISABLE\n");
    pass++;
  }
  else {
    printk("[FAIL] RISE=0x%08x FALL=0x%08x LVL_HI=0x%08x "
      "(expected test bits = 0)\n", rise_en, fall_en, lvhi_en);
    fail++;
  }

  if ((plic_ie & plic_mask) == 0U) {
    printk("[PASS] PLIC_IE0_0 sources cleared after GPIO_INT_DISABLE\n");
    pass++;
  }
  else {
    printk("[FAIL] PLIC_IE0_0=0x%08x (expected sources 10-13 = 0)\n", plic_ie);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_8(const struct device *dev) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 8: Interrupt on Non-IRQ Pins 0-7 Returns -ENOTSUP ---\n");

  int ret;

  // Pin 0 — rising edge
  ret = gpio_pin_interrupt_configure(dev, 0U, GPIO_INT_EDGE_RISING);
  if (ret == -ENOTSUP) {
    printk("[PASS] pin 0 EDGE_RISING -> -ENOTSUP\n");
    pass++;
  }
  else {
    printk("[FAIL] pin 0 EDGE_RISING _> %d (expected -ENOTSUP)\n", ret);
    fail++;
  }

  // Pin 7 — last non-IRQ pin, level-low
  ret = gpio_pin_interrupt_configure(dev, 7U, GPIO_INT_LEVEL_LOW);
  if (ret == -ENOTSUP) {
    printk("[PASS] pin 7 LEVEL_LOW -> -ENOTSUP\n");
    pass++;
  }
  else {
    printk("[FAIL] pin 7 LEVEL_LOW -> %d (expected -ENOTSUP)\n", ret);
    fail++;
  }

  // Disable on non-IRQ pin must be a safe no-op (returns 0)
  ret = gpio_pin_interrupt_configure(dev, 3U, GPIO_INT_DISABLE);
  if (ret == 0) {
    printk("[PASS] pin 3 GPIO_INT_DISABLE -> 0 (safe no-op)\n");
    pass++;
  }
  else {
    printk("[FAIL] pin 3 GPIO_INT_DISABLE -> %d (expected 0)\n", ret); 
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_9(const struct device *dev) {
  /*
    Interrupt dispatch via level-low on pin 8.

    GPIO IP (INTRPT_STATUS) > PLIC source 9 > CPU MEXT > xheep_plic_irq_handler
      > gpio_xheep_isr > gpio_fire_callbacks > gpio_dispatch_callback
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 9: Interrupt Dispatch — Level-Low on Pin %u ---\n", PIN_IRQ_DISPATCH);
  printk("[INFO] Relies on gpio_in_i[%u] = 0 (default in Verilator)\n", PIN_IRQ_DISPATCH);

  // Reset ISR tracking
  gpio_isr_fired = false;
  gpio_isr_pins  = 0U;

  // Configure pin 8 as input with sampling enabled
  int ret = gpio_pin_configure(dev, PIN_IRQ_DISPATCH, GPIO_INPUT);
  if (ret == 0) {
    printk("[PASS] gpio_pin_configure(pin %u, INPUT) ret=0\n",
      PIN_IRQ_DISPATCH);
    pass++;
  }
  else {
    printk("[FAIL] gpio_pin_configure(pin %u) returned %d\n",
      PIN_IRQ_DISPATCH, ret);
    fail++;
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Verify GPIO_EN[8] is set (required for interrupt detection)
  uint32_t en = sys_read32(GPIO_BASE + GPIO_EN_OFF);
  if (en & BIT(PIN_IRQ_DISPATCH)) {
    printk("[PASS] GPIO_EN[pin %u] = 1 (sampling enabled)\n", PIN_IRQ_DISPATCH);
    pass++;
  }
  else {
    printk("[FAIL] GPIO_EN[pin %u] = 0 (sampling not enabled)\n",
      PIN_IRQ_DISPATCH);
    fail++;
  }

  // Register the callback for pin 8
  gpio_init_callback(&gpio_test_cb, gpio_dispatch_callback, BIT(PIN_IRQ_DISPATCH));
  ret = gpio_add_callback(dev, &gpio_test_cb);
  if (ret == 0) {
    printk("[PASS] gpio_add_callback ret=0\n");
    pass++;
  }
  else {
    printk("[FAIL] gpio_add_callback returned %d\n", ret);
    fail++;
  }

  // Lock interrupts so ISR cannot fire between arming and register check
  unsigned int irq_key = irq_lock();

  // Arm the level-low interrupt — would fire immediately but IRQs are locked
  ret = gpio_pin_interrupt_configure(dev, PIN_IRQ_DISPATCH, GPIO_INT_LEVEL_LOW);
  if (ret == 0) {
    printk("[PASS] gpio_pin_interrupt_configure(LEVEL_LOW) ret=0\n");
    pass++;
  }
  else {
    irq_unlock(irq_key);
    printk("[FAIL] gpio_pin_interrupt_configure returned %d\n", ret);
    fail++;
    gpio_remove_callback(dev, &gpio_test_cb);
    printk("\nResult: %d passed, %d failed\n", pass, fail);
    return fail;
  }

  // Verify INTRPT_LVL_LO_EN[8] is set (ISR cannot fire yet)
  uint32_t lvlo = sys_read32(GPIO_BASE + GPIO_LVL_LO_EN_OFF);
  if (lvlo & BIT(PIN_IRQ_DISPATCH)) {
    printk("[PASS] INTRPT_LVL_LO_EN[pin %u] = 1\n", PIN_IRQ_DISPATCH);
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_LVL_LO_EN[pin %u] = 0 (interrupt not armed)\n",
      PIN_IRQ_DISPATCH);
    fail++;
  }

  // Verify PLIC source 9 is enabled (ISR cannot fire yet)
  uint32_t plic_ie = sys_read32(PLIC_IE0_0);
  if (plic_ie & BIT(PIN_IRQ_DISPATCH + 1U)) {
    printk("[PASS] PLIC_IE0_0[src %u] = 1\n", PIN_IRQ_DISPATCH + 1U);
    pass++;
  }
  else {
    printk("[FAIL] PLIC_IE0_0[src %u] = 0 (PLIC source not enabled)\n",
      PIN_IRQ_DISPATCH + 1U);
    fail++;
  }

  // Unlock — pending LEVEL_LOW interrupt fires now
  irq_unlock(irq_key);

  // Print current GPIO_IN value to help diagnose if interrupt does not fire
  uint32_t gpio_in_val = sys_read32(GPIO_BASE + GPIO_IN_OFF);
  printk("[INFO] GPIO_IN = 0x%08x  (pin %u = %u, need 0 for level-low)\n",
    gpio_in_val, PIN_IRQ_DISPATCH, !!(gpio_in_val & BIT(PIN_IRQ_DISPATCH)));

  // Wait up to ~10 000 iterations for the callback to fire
  for (volatile int i = 0; i < 10000; i++) {
    if (gpio_isr_fired) {
      break;
    }
  }

  if (gpio_isr_fired) {
    printk("[PASS] Interrupt callback fired! pins=0x%08x\n", gpio_isr_pins); pass++;

    if (gpio_isr_pins & BIT(PIN_IRQ_DISPATCH)) {
      printk("[PASS] Callback pins mask includes pin %u\n",
        PIN_IRQ_DISPATCH);
      pass++;
    }
    else {
      printk("[FAIL] Callback pins=0x%08x missing BIT(%u)\n",
        gpio_isr_pins, PIN_IRQ_DISPATCH);
      fail++;
    }
  }
  else {
    printk("[FAIL] Interrupt callback did NOT fire within timeout\n");
    fail++;
    printk("[INFO] Possible cause: gpio_in_i[%u] is HIGH in testbench "
      "(level-low requires low input)\n", PIN_IRQ_DISPATCH);
    printk("[INFO] GPIO_IN[%u] = %u at timeout\n",
      PIN_IRQ_DISPATCH, !!(sys_read32(GPIO_BASE + GPIO_IN_OFF) & BIT(PIN_IRQ_DISPATCH)));
    printk("[INFO] INTRPT_STATUS = 0x%08x\n", sys_read32(GPIO_BASE + GPIO_INTR_STAT_OFF));
  }

  // Verify interrupt was disabled by the callback (prevent re-fire)
  lvlo = sys_read32(GPIO_BASE + GPIO_LVL_LO_EN_OFF);
  if ((lvlo & BIT(PIN_IRQ_DISPATCH)) == 0U) {
    printk("[PASS] INTRPT_LVL_LO_EN[pin %u] = 0 (disabled by callback)\n",
      PIN_IRQ_DISPATCH);
    pass++;
  }
  else {
    printk("[FAIL] INTRPT_LVL_LO_EN[pin %u] = 1 (should be disabled)\n",
      PIN_IRQ_DISPATCH);
    fail++;
  }

  // Clean up
  gpio_remove_callback(dev, &gpio_test_cb);

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main(void)
{
  printk("\n========================================\n");
  printk("\tX-HEEP GPIO Verification Test\n");
  printk("\t\tBoard: %s\n", CONFIG_BOARD);
  printk("========================================\n");

  const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

  if (!device_is_ready(dev)) {
    printk("[FAIL] gpio0 device not ready\n");
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
  total_fail += test_seq_9(dev);

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
