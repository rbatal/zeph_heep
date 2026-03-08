/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * uart_plic_test — X-HEEP UART and PLIC functional test
 *
 * Build:
 *   west build -b x_heep sw/uart_plic_test \
 *       -- -DZEPHYR_EXTRA_MODULES=/path/to/z_heep_3
 *
 * What this tests
 * ---------------
 * TEST 1 — UART driver (polling TX)
 *   Verifies that the custom esl-epfl,x-heep-uart driver correctly
 *   initialises and can output characters through the Zephyr UART API.
 *   Expected output: "Hello from X-HEEP!" line visible in simulation console.
 *
 * TEST 2 — UART register sanity
 *   Reads the UART STATUS register directly and checks that the TX FIFO
 *   reports idle after the driver's init sequence (TXIDLE bit should be 1).
 *
 * TEST 3 — PLIC interrupt delivery (TX_WATERMARK)
 *   Directly stimulates a PLIC interrupt without involving the Zephyr
 *   interrupt-driven UART API:
 *     a) Connect a raw ISR to PLIC source XHEEP_UART_IRQ_TX_WATERMARK (1).
 *     b) Enable the UART TX_WATERMARK interrupt in the UART INTR_ENABLE
 *        register.  Because the TX FIFO is already empty (level = 0, which
 *        is below the reset watermark of 1), INTR_STATE.tx_watermark is
 *        already set, so the PLIC should deliver the interrupt immediately.
 *     c) Busy-wait up to PLIC_TEST_TIMEOUT_US for the ISR counter to
 *        increment.
 *     d) Disable the UART interrupt to prevent continuous re-firing.
 *   Pass criterion: ISR called at least once within the timeout window.
 *
 * TEST 4 — UART RX echo loop
 *   Polls for incoming characters for RX_LOOP_TIMEOUT_MS milliseconds,
 *   echoing each one back.  Also counts received characters.
 *   This test always "passes" (it just reports what it received).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/sys/sys_io.h>

/* -------------------------------------------------------------------------
 * X-HEEP UART register map (esl-epfl,x-heep-uart offsets)
 * These MUST match uart_x_heep.c exactly.
 * ---------------------------------------------------------------------- */
#define UART_BASE                 DT_REG_ADDR(DT_NODELABEL(uart0))

#define UART_INTR_STATE_ADDR      (UART_BASE + 0x00U)
#define UART_INTR_ENABLE_ADDR     (UART_BASE + 0x04U)
#define UART_CTRL_ADDR            (UART_BASE + 0x0CU)
#define UART_STATUS_ADDR          (UART_BASE + 0x10U)
#define UART_RDATA_ADDR           (UART_BASE + 0x14U)
#define UART_WDATA_ADDR           (UART_BASE + 0x18U)
#define UART_FIFO_CTRL_ADDR       (UART_BASE + 0x1CU)

/* INTR_STATE / INTR_ENABLE bits (W1C for INTR_STATE) */
#define UART_INTR_TX_WATERMARK    BIT(0)
#define UART_INTR_RX_WATERMARK    BIT(1)
#define UART_INTR_TX_EMPTY        BIT(2)
#define UART_INTR_RX_OVERFLOW     BIT(3)

/* STATUS register bits */
#define UART_STATUS_TXFULL        BIT(0)
#define UART_STATUS_RXFULL        BIT(1)
#define UART_STATUS_TXEMPTY       BIT(2)
#define UART_STATUS_TXIDLE        BIT(3)
#define UART_STATUS_RXIDLE        BIT(4)
#define UART_STATUS_RXEMPTY       BIT(5)

/* -------------------------------------------------------------------------
 * PLIC source numbers (from soc/x_heep/soc.h)
 * These are the hardware PLIC input IDs (1-indexed, 0 = no interrupt).
 * To get the Zephyr encoded multi-level IRQ: irq_to_level_2(source)
 * ---------------------------------------------------------------------- */
#define UART_PLIC_SRC_TX_WATERMARK   1U  /* TX FIFO below watermark      */
#define UART_PLIC_SRC_RX_WATERMARK   2U  /* RX FIFO above watermark      */
#define UART_PLIC_SRC_TX_EMPTY       3U  /* TX FIFO drained to empty     */

/* -------------------------------------------------------------------------
 * Test parameters
 * ---------------------------------------------------------------------- */
#define PLIC_TEST_TIMEOUT_US    100000U   /* 100 ms — plenty at 100 MHz   */
#define RX_LOOP_TIMEOUT_MS      5000U     /* 5 s RX echo window            */

/* -------------------------------------------------------------------------
 * Shared state between the test and its ISRs (volatile, single-hart)
 * ---------------------------------------------------------------------- */
static volatile int  plic_isr_count;
static volatile bool plic_isr_fired;

/* -------------------------------------------------------------------------
 * Helper: print a line prefix
 * ---------------------------------------------------------------------- */
static void test_header(int n, const char *name)
{
	printk("\n=== TEST %d: %s ===\n", n, name);
}

static void test_pass(void)  { printk("[PASS]\n"); }
static void test_fail(void)  { printk("[FAIL]\n"); }

/* =========================================================================
 * TEST 1 — UART driver polling TX
 * ====================================================================== */
static void test_uart_polling_tx(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

	test_header(1, "UART polling TX");

	if (!device_is_ready(uart)) {
		printk("  UART device not ready\n");
		test_fail();
		return;
	}

	const char msg[] = "Hello from X-HEEP!\r\n";

	for (size_t i = 0; msg[i]; i++) {
		uart_poll_out(uart, msg[i]);
	}

	printk("  TX string sent via uart_poll_out()\n");
	test_pass();
}

/* =========================================================================
 * TEST 2 — UART STATUS register sanity
 * ====================================================================== */
static void test_uart_status_reg(void)
{
	test_header(2, "UART STATUS register sanity");

	/* Wait for the TX FIFO and shift register to drain from previous printk
	 * output before sampling STATUS. At 256kbps each byte takes ~39 µs;
	 * a full 16-byte FIFO drains in under 1 ms. */
	while (!(sys_read32(UART_STATUS_ADDR) & UART_STATUS_TXIDLE)) {
		/* busy-wait */
	}

	uint32_t status = sys_read32(UART_STATUS_ADDR);

	printk("  STATUS = 0x%08x\n", status);
	printk("  TXFULL  = %u (expect 0)\n", !!(status & UART_STATUS_TXFULL));
	printk("  TXEMPTY = %u (expect 1)\n", !!(status & UART_STATUS_TXEMPTY));
	printk("  TXIDLE  = %u (expect 1)\n", !!(status & UART_STATUS_TXIDLE));
	printk("  RXEMPTY = %u (expect 1)\n", !!(status & UART_STATUS_RXEMPTY));

	/* TX should be idle and empty; RX should be empty */
	bool ok = !(status & UART_STATUS_TXFULL)
		  && (status & UART_STATUS_TXEMPTY)
		  && (status & UART_STATUS_TXIDLE)
		  && (status & UART_STATUS_RXEMPTY);

	if (ok) {
		test_pass();
	} else {
		printk("  Unexpected STATUS bits\n");
		test_fail();
	}
}

/* =========================================================================
 * TEST 3 — PLIC interrupt delivery
 *
 * Strategy: use the UART TX_WATERMARK interrupt (PLIC source 1).
 * After init the TX FIFO is empty (level 0 < watermark 1), so
 * INTR_STATE.tx_watermark is already asserted.  The moment we enable it in
 * INTR_ENABLE and the PLIC source is armed, the interrupt should fire.
 * ====================================================================== */

/*
 * ISR registered on PLIC source UART_PLIC_SRC_TX_WATERMARK.
 * Runs with interrupts disabled (standard ISR context in Zephyr).
 */
static void uart_tx_watermark_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/* Immediately disable UART interrupt to prevent continuous re-firing.
	 * (TX_WATERMARK is level-sensitive: stays asserted while FIFO < wmark) */
	sys_write32(0U, UART_INTR_ENABLE_ADDR);

	/* Clear the pending bit in UART INTR_STATE (W1C). */
	sys_write32(UART_INTR_TX_WATERMARK, UART_INTR_STATE_ADDR);

	plic_isr_count++;
	plic_isr_fired = true;
}

static void test_plic_interrupt(void)
{
	test_header(3, "PLIC interrupt delivery (UART TX_WATERMARK)");

	plic_isr_count = 0;
	plic_isr_fired = false;

	/*
	 * Step 1: make sure UART INTR_ENABLE is off (no stale enables).
	 */
	sys_write32(0U, UART_INTR_ENABLE_ADDR);

	/*
	 * Step 2: clear any pending UART interrupt state bits (W1C).
	 */
	sys_write32(0xFFFFFFFFU, UART_INTR_STATE_ADDR);

	/*
	 * Step 3: connect the ISR to the PLIC source.
	 *
	 * irq_to_level_2(S) produces the Zephyr multi-level encoded IRQ for
	 * PLIC source S.  Our driver dispatches isr_table[S] when CC0 returns S
	 * (hardware source ID), and irq_from_level_2(irq_to_level_2(S)) == S,
	 * so the table indices line up correctly.
	 *
	 * irq_connect_dynamic() is used instead of IRQ_CONNECT() so that the
	 * PLIC source number can be a runtime expression.
	 */
	uint32_t plic_irq = irq_to_level_2(UART_PLIC_SRC_TX_WATERMARK);

	irq_connect_dynamic(plic_irq, 1, uart_tx_watermark_isr, NULL, 0);

	printk("  Encoded PLIC IRQ = 0x%08x (L1=%u, source=%u)\n",
	       plic_irq,
	       irq_parent_level_2(plic_irq),
	       irq_from_level_2(plic_irq));

	/*
	 * Step 4: enable the PLIC source via Zephyr (sets IE bit + priority).
	 * riscv_plic_set_priority() is called inside z_riscv_irq_priority_set()
	 * during irq_connect_dynamic(), and irq_enable() delegates to
	 * riscv_plic_irq_enable() for L2 IRQs.
	 */
	irq_enable(plic_irq);

	/*
	 * Step 5: enable UART TX_WATERMARK in UART INTR_ENABLE.
	 * The TX FIFO is empty (level 0 < reset watermark of 1), so
	 * INTR_STATE.tx_watermark is already set — the interrupt should
	 * fire as soon as global interrupts are enabled with the PLIC
	 * source armed.
	 */
	sys_write32(UART_INTR_TX_WATERMARK, UART_INTR_ENABLE_ADDR);

	printk("  UART INTR_STATE before wait = 0x%08x\n",
	       sys_read32(UART_INTR_STATE_ADDR));

	/*
	 * Step 6: busy-wait for the ISR to fire.
	 * k_busy_wait() leaves interrupts enabled so the PLIC can deliver.
	 */
	uint32_t waited = 0U;
	const uint32_t step_us = 100U;

	while (!plic_isr_fired && waited < PLIC_TEST_TIMEOUT_US) {
		k_busy_wait(step_us);
		waited += step_us;
	}

	/* Step 7: ensure UART interrupt is disabled regardless of outcome. */
	sys_write32(0U, UART_INTR_ENABLE_ADDR);
	irq_disable(plic_irq);

	printk("  ISR fired: %s  (count=%d, waited=%u us)\n",
	       plic_isr_fired ? "YES" : "NO",
	       plic_isr_count, waited);

	if (plic_isr_fired) {
		test_pass();
	} else {
		printk("  ISR did not fire within %u us\n", PLIC_TEST_TIMEOUT_US);
		test_fail();
	}
}

/* =========================================================================
 * TEST 4 — UART RX echo loop
 * ====================================================================== */
static void test_uart_rx_echo(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

	test_header(4, "UART RX echo loop");
	printk("  Send characters within %u ms to test RX...\n",
	       RX_LOOP_TIMEOUT_MS);

	int rx_count = 0;
	uint32_t start = k_uptime_get_32();

	while (k_uptime_get_32() - start < RX_LOOP_TIMEOUT_MS) {
		unsigned char c;

		if (uart_poll_in(uart, &c) == 0) {
			uart_poll_out(uart, c);   /* echo */
			rx_count++;
		}
	}

	printk("  Characters received and echoed: %d\n", rx_count);
	test_pass();  /* RX test always passes — result is informational */
}

/* =========================================================================
 * Entry point
 * ====================================================================== */
int main(void)
{
	printk("\n");
	printk("==================================================\n");
	printk("  X-HEEP UART + PLIC functional test\n");
	printk("  Board: %s\n", CONFIG_BOARD);
	printk("  UART base: 0x%08x\n", (unsigned int)UART_BASE);
	printk("  PLIC base: 0x%08x\n",
	       (unsigned int)DT_REG_ADDR(DT_NODELABEL(plic)));
	printk("==================================================\n");

	test_uart_polling_tx();
	test_uart_status_reg();
	test_plic_interrupt();
	test_uart_rx_echo();

	printk("\n==================================================\n");
	printk("  All tests complete.\n");
	printk("==================================================\n");

	return 0;
}
