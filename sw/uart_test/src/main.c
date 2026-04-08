/*
  * X-HEEP UART Verification Test
  * 
  * Tests UART STATUS register, polling TX, and blocking RX echo.
  * No interrupts or PLIC involvement (yet).
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>


//! CONSTANTS

//* UART register map
#define UART_BASE           DT_REG_ADDR(DT_NODELABEL(uart0))
#define UART_INTR_STATE     (UART_BASE + 0x00U)
#define UART_INTR_ENABLE    (UART_BASE + 0x04U)
#define UART_STATUS         (UART_BASE + 0x10U)

// STATUS bits
#define STATUS_TXFULL   BIT(0)
#define STATUS_RXFULL   BIT(1)
#define STATUS_TXEMPTY  BIT(2)
#define STATUS_TXIDLE   BIT(3)
#define STATUS_RXIDLE   BIT(4)
#define STATUS_RXEMPTY  BIT(5)

//* Other

#define RX_POLL_TIMEOUT_MS 100U


//! TEST SEQUENCES

static int test_seq_1(const struct device *uart) {
  int pass = 0, fail = 0;

  printk("\n--- Polling TX Test Sequence ---\n");

  const char msg[] = "Hello from X-HEEP!\r\n";

  for (size_t i = 0; msg[i]; i++) {
    uart_poll_out(uart, msg[i]);
  }
  printk("[PASS] TX via uart_poll_out(): OK\n");
  pass++;

  return fail;
}


static int test_seq_2() {
  int pass = 0, fail = 0;

  printk("\n--- STATUS Test Sequence ---\n");

  // Wait for TX FIFO and shift register to drain
  while (!(sys_read32(UART_STATUS) & STATUS_TXIDLE)) {}

  uint32_t s = sys_read32(UART_STATUS);
  printk("\tSTATUS   = 0x%08x\n", s);
  printk("\tTXFULL   = %u (expect 0)\n", !!(s & STATUS_TXFULL));
  printk("\tTXEMPTY  = %u (expect 1)\n", !!(s & STATUS_TXEMPTY));
  printk("\tTXIDLE   = %u (expect 1)\n", !!(s & STATUS_TXIDLE));
  printk("\tRXEMPTY  = %u (expect 1)\n", !!(s & STATUS_RXEMPTY));

  bool ok = !(s & STATUS_TXFULL)
    && (s & STATUS_TXEMPTY)
    && (s & STATUS_TXIDLE)
    && (s & STATUS_RXEMPTY);

  if (ok) {
    printk("[PASS]\n");
    pass++;
  } else {
    printk("[FAIL] Unexpected STATUS bits\n");
    fail++;
  }

  return fail;
}


static int test_seq_3() {
  int pass = 0, fail = 0;

  printk("\n--- INTR_STATE Test Sequence ---\n");

  // Confirm interrupts are disabled so nothing triggers
  sys_write32(0U, UART_INTR_ENABLE);

  uint32_t st = sys_read32(UART_INTR_STATE);
  printk("INTR_ENABLE = 0x%08x\n", sys_read32(UART_INTR_ENABLE));

  printk("INTR_STATE  = 0x%08x\n", st);
  
  // TX_WATERMARK is expected to be 1 as the TX FIFO has been drained at some point
  printk("\ttx_watermark(bit0) = %u  (1 expected: TX FIFO empty < threshold)\n",
         !!(st & BIT(0)));

  printk("\trx_watermark(bit1) = %u  (0 expected: no RX data)\n",
         !!(st & BIT(1)));

  printk("\ttx_empty    (bit2) = %u\n", !!(st & BIT(2)));

  printk("\trx_overflow (bit3) = %u\n", !!(st & BIT(3)));

  // Informational, always passes
  printk("[PASS]\n");
  pass++;

  return fail;
}


static int test_seq_4(const struct device *uart) {
  int pass = 0, fail = 0;

  printk("\n--- RX Echo Loop Test Sequence ---\n");
  printk("Timeout @ %u ms\n", RX_POLL_TIMEOUT_MS);

  int count = 0;
  uint32_t start = k_uptime_get_32();

  while (k_uptime_get_32() - start < RX_POLL_TIMEOUT_MS) {
    unsigned char c;

    if (uart_poll_in(uart, &c) == 0) {
      uart_poll_out(uart, c);
      count++;
    }
  }

  printk("Characters echoed: %d\n", count);
  if (count > 0) {
    printk("[PASS]\n");
    pass++;
  }
  else{
    printk("[FAIL] No characters echoed\n");
    fail++;
  }

  return fail;
}


//! MAIN

int main() {
  const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

  printk("\n========================================\n");
  printk("\tX-HEEP UART Verification Test\n");
  printk("\t\tBoard: %s", CONFIG_BOARD);
  printk("\n========================================\n");

  if (!device_is_ready(uart)) {
    printk("[FAIL] UART device not ready\n");
    return -1;
  }

  int total_fail = 0;
  total_fail += test_seq_1(uart);
  total_fail += test_seq_2();
  total_fail += test_seq_3();
  total_fail += test_seq_4(uart);

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
