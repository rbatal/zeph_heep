/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP OpenTitan UART driver — based on Zephyr libraries uart_opentitan.c but
 * with corrected register offsets. X-HEEP's OpenTitan UART misses the ALERT_TEST register
 * (0x0C in the upstream IP), so CTRL and all subsequent registers are shifted 4 bytes
 * earlier relative to Zephyr's built-in driver.
 *
 * X-HEEP OpenTitan UART register map:
 *   0x00  INTR_STATE
 *   0x04  INTR_ENABLE
 *   0x08  INTR_TEST
 *   -- 0x0C  ALERT_TEST <-- Non-Existen register
 *   0x0C  CTRL          <-- In Zephyr libs [0x10 CTRL]
 *   0x10  STATUS
 *   0x14  RDATA
 *   0x18  WDATA
 *   0x1C  FIFO_CTRL
 *   0x20  FIFO_STATUS
 *   0x24  OVRD
 *   0x28  VAL
 *   0x2C  TIMEOUT_CTRL
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

/* Register offsets */
#define UART_INTR_STATE_REG_OFFSET   0x00
#define UART_INTR_ENABLE_REG_OFFSET  0x04
#define UART_CTRL_REG_OFFSET         0x0C
#define UART_STATUS_REG_OFFSET       0x10
#define UART_RDATA_REG_OFFSET        0x14
#define UART_WDATA_REG_OFFSET        0x18
#define UART_FIFO_CTRL_REG_OFFSET    0x1C
#define UART_OVRD_REG_OFFSET         0x24
#define UART_TIMEOUT_CTRL_REG_OFFSET 0x2C

/* Control register bits */
#define UART_CTRL_TX_BIT    BIT(0)
#define UART_CTRL_RX_BIT    BIT(1)
#define UART_CTRL_NCO_OFFSET 16

/* FIFO control register bits */
#define UART_FIFO_CTRL_RXRST_BIT BIT(0)
#define UART_FIFO_CTRL_TXRST_BIT BIT(1)

/* Status register bits. */
#define UART_STATUS_TXFULL_BIT  BIT(0)
#define UART_STATUS_RXEMPTY_BIT BIT(5)

#define DT_DRV_COMPAT esl_epfl_x_heep_uart

struct uart_x_heep_config {
	mem_addr_t base;
	uint32_t nco_reg;
};

static int uart_x_heep_init(const struct device *dev)
{
	const struct uart_x_heep_config *cfg = dev->config;

	// Reset settings
	sys_write32(0u, cfg->base + UART_CTRL_REG_OFFSET);

	// Clear FIFOs
	sys_write32(UART_FIFO_CTRL_RXRST_BIT | UART_FIFO_CTRL_TXRST_BIT,
		    cfg->base + UART_FIFO_CTRL_REG_OFFSET);

	// Clear other states
	sys_write32(0u, cfg->base + UART_OVRD_REG_OFFSET);
	sys_write32(0u, cfg->base + UART_TIMEOUT_CTRL_REG_OFFSET);

	// Disable interrupts [To be fixed when the PLIC is fully operative]
	sys_write32(0u, cfg->base + UART_INTR_ENABLE_REG_OFFSET);

	// Clear interrupts
	sys_write32(0xffffffffu, cfg->base + UART_INTR_STATE_REG_OFFSET);

	// Set baud and enable TX and RX
	sys_write32(UART_CTRL_TX_BIT | UART_CTRL_RX_BIT |
		    (cfg->nco_reg << UART_CTRL_NCO_OFFSET),
		    cfg->base + UART_CTRL_REG_OFFSET);
	return 0;
}

static int uart_x_heep_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_x_heep_config *cfg = dev->config;

	if (sys_read32(cfg->base + UART_STATUS_REG_OFFSET) & UART_STATUS_RXEMPTY_BIT) {
		/* Empty RX FIFO */
		return -1;
	}
	*c = sys_read32(cfg->base + UART_RDATA_REG_OFFSET);
	return 0;
}

static void uart_x_heep_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_x_heep_config *cfg = dev->config;

	// Wait for space in the TX FIFO
	while (sys_read32(cfg->base + UART_STATUS_REG_OFFSET) & UART_STATUS_TXFULL_BIT) {
		;
	}

	sys_write32(c, cfg->base + UART_WDATA_REG_OFFSET);
}

// API abstraction struct -> Links custom function to the generic driver handles
static DEVICE_API(uart, uart_x_heep_driver_api) = {
	.poll_in  = uart_x_heep_poll_in,
	.poll_out = uart_x_heep_poll_out,
};

/*
 * NCO value: baud ticks per system clock tick, scaled by 2^20
 * (same as the Zephyr libs OpenTitan driver):
 *    NCO = ((baud / 100) << 20) / (clk / 100) = BIT64(20) * baud / clk
 */
#define NCO_REG(baud, clk) (BIT64(20) * (baud) / (clk))


//* 
//* Instantiation macro
//* 
#define UART_X_HEEP_INIT(n)                                     \
	static struct uart_x_heep_config uart_x_heep_config_##n = {   \
		.base    = DT_INST_REG_ADDR(n),                             \
		.nco_reg = NCO_REG(DT_INST_PROP(n, current_speed),          \
				   DT_INST_PROP(n, clock_frequency)),                   \
	};                                                            \
	DEVICE_DT_INST_DEFINE(n, uart_x_heep_init, NULL, NULL,        \
			      &uart_x_heep_config_##n,                            \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,          \
			      &uart_x_heep_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_X_HEEP_INIT)
