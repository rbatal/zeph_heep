/*
 * author: rbatal (rodrigo.batal@gmail.com)
 * 
 * X-HEEP CV32E40P SoC initialization for Zephyr
 * 
 * Includes startup prints to check the progress
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
#include "soc.h"

//* Direct UART access for debugging
#define UART_BASE   0x30080000
#define UART_STATUS (*(volatile uint32_t *)(UART_BASE + 0x10))
#define UART_WDATA  (*(volatile uint32_t *)(UART_BASE + 0x18))

static void uart_putc(char c)
{
    while (UART_STATUS & 0x1) { }
    UART_WDATA = c;
    for (volatile int i = 0; i < 1000; i++) { }
}

static void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

__attribute__((used, noinline))
void debug_c_test(void)
{
    uart_puts("C!\n");
}

static int soc_early_init(void) {
    /*
      Activate rv_timer_ao before the machine timer driver reads mtime.
      Prior to this, the counter stays at 0 forever, (k_busy_wait() never
      ends up returning, as there is no increment).
    */
    sys_write32(XHEEP_RV_TIMER_AO_CFG0_STEP(1), XHEEP_RV_TIMER_AO_CFG0);
    sys_write32(XHEEP_RV_TIMER_AO_CTRL_ACTIVE_0, XHEEP_RV_TIMER_AO_CTRL);

    /*
      ? The lowRISC rv_timer IP gates its interrupt output via INTR_ENABLE
      ? This is a PRE-REQUISITE for any Zephyr-based project.
      Many functionalities depend on this timer interrupt being available at
      startup and Zephyr does not know how to access this register through its
      riscv_machine_timer.c library (ignores its existence). Also, it is safe
      to enable the interrupt as <mstatus.MIE>=0 at the moment, meaning that
      the interrupt is gated until machine-interrupts are enabled.
      This interrupt is also gated by <mtimecmp> (properly controlled by
      Zephyr), so there is no risk related to this enable.
    */
    sys_write32(BIT(0), XHEEP_RV_TIMER_AO_INTR_ENABLE);

    uart_puts("K1\n");
    return 0;
}
SYS_INIT(soc_early_init, PRE_KERNEL_1, 0);

static int soc_pre2_init(void)
{
    uart_puts("K2\n");
    return 0;
}
SYS_INIT(soc_pre2_init, PRE_KERNEL_2, 99);

static int soc_post_init(void)
{
    uart_puts("K3\n");
    return 0;
}
SYS_INIT(soc_post_init, POST_KERNEL, 99);

static int soc_app_init(void)
{
    uart_puts("K4\n");
    
    // Get the console UART device
    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    
    if (!device_is_ready(uart_dev)) {
        uart_puts("UART_NOT_RDY\n");
        return 0;
    }
    
    uart_puts("UART_RDY\n");
    
    // Test 1: Use Zephyr's UART driver poll_out directly
    uart_puts("T1:");
    uart_poll_out(uart_dev, 'D');
    uart_poll_out(uart_dev, 'R');
    uart_poll_out(uart_dev, 'V');
    uart_poll_out(uart_dev, '\r');
    uart_poll_out(uart_dev, '\n');
    
    // Small delay
    for (volatile int i = 0; i < 10000; i++) { }
    
    uart_puts("T2:");
    
    // Test 2: Try printk
    printk("PRN");
    
    uart_puts(":T3\n");
    
    return 0;
}
SYS_INIT(soc_app_init, APPLICATION, 0);