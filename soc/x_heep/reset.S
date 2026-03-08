/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP CV32E40P Reset Handler
 *
 * Includes a basic test to confirm C code runs before kernel setup
 */

#include <zephyr/arch/cpu.h>

//* X-HEEP memory map
#define SOC_CTRL_START_ADDRESS              0x20000000
#define SOC_CTRL_SYSTEM_FREQUENCY_HZ_OFFSET 0x1C

//* UART registers
#define UART_BASE       0x30080000
#define UART_CTRL       (UART_BASE + 0x0C)
#define UART_STATUS     (UART_BASE + 0x10)
#define UART_WDATA      (UART_BASE + 0x18)
#define UART_FIFO_CTRL  (UART_BASE + 0x1C)

//* System clock
#define REFERENCE_CLOCK_HZ  100000000

.section .text, "ax"
.align 2

.macro DEBUG_CHAR char
    li t0, UART_WDATA
    li t1, \char
    sw t1, 0(t0)
    li t2, 50000
1:  addi t2, t2, -1
    bnez t2, 1b
.endm

.global __reset
.type __reset, @function
__reset:
    // Initialize SOC_CTRL frequency
    li t0, SOC_CTRL_START_ADDRESS
    li t1, REFERENCE_CLOCK_HZ
    sw t1, SOC_CTRL_SYSTEM_FREQUENCY_HZ_OFFSET(t0)

    // Initialize UART
    li t0, UART_CTRL
    sw zero, 0(t0)
    
    li t0, UART_FIFO_CTRL
    li t1, 0x3
    sw t1, 0(t0)
    
    li t0, UART_CTRL
    li t1, 0x0A7C0003
    sw t1, 0(t0)

    DEBUG_CHAR '0'
    DEBUG_CHAR '\n'

    // Set up stack
    la sp, z_interrupt_stacks
    li t0, CONFIG_ISR_STACK_SIZE
    add sp, sp, t0

    DEBUG_CHAR '1'
    DEBUG_CHAR '\n'

    // Clear BSS
    la t0, __bss_start
    la t1, __bss_end
bss_loop:
    bge t0, t1, bss_done
    sw zero, 0(t0)
    addi t0, t0, 4
    j bss_loop
bss_done:

    DEBUG_CHAR '2'
    DEBUG_CHAR '\n'

    /* 
     * TEST: Call our debug C function directly
     * This tests if C code can execute at all.
     * Prints C!
     */
    call debug_c_test
    
    DEBUG_CHAR '3'
    DEBUG_CHAR '\n'

    // Now try z_cstart (normal Zephyr boot)
    DEBUG_CHAR '4'
    DEBUG_CHAR '\n'
    call z_cstart
    
    // Should never get here
    DEBUG_CHAR 'X'
    
spin:
    wfi
    j spin

.size __reset, . - __reset