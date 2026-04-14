/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP SoC Header
 *
 * Memory map and definitions derived from:
 * - /tools/x-heep/sw/device/lib/runtime/core_v_mini_mcu.h
 * - /tools/x-heep/config/z_heep.hjson
 * - /tools/x-heep/sw/device/target/sim/x-heep.h
 *
 * This header is core-agnostic (attempts-to not sure yet): it applies to all X-HEEP core variants
 * (cv32e20p, cv32e40p, cv32e40x, cv32e40px).
 */

#ifndef __RISCV_X_HEEP_SOC_H__
#define __RISCV_X_HEEP_SOC_H__

#include <zephyr/sys/util_macro.h>

//* ============================================================
//* Memory Map
//* ============================================================

/* RAM Banks: 4 × 32KB = 128KB total */
#define XHEEP_RAM_BASE              0x00000000
#define XHEEP_RAM_SIZE              0x00020000  /* 128KB */

#define XHEEP_RAM0_BASE             0x00000000
#define XHEEP_RAM0_SIZE             0x00008000  /* 32KB */
#define XHEEP_RAM1_BASE             0x00008000
#define XHEEP_RAM1_SIZE             0x00008000  /* 32KB */
#define XHEEP_RAM2_BASE             0x00010000
#define XHEEP_RAM2_SIZE             0x00008000  /* 32KB */
#define XHEEP_RAM3_BASE             0x00018000
#define XHEEP_RAM3_SIZE             0x00008000  /* 32KB */

/* Linker sections (from z_heep.hjson) */
#define XHEEP_CODE_BASE             0x00000000
#define XHEEP_CODE_SIZE             0x0000E800  /* 59KB */
#define XHEEP_DATA_BASE             0x0000E800
#define XHEEP_DATA_SIZE             0x00011800  /* 69KB */

/* Boot address - X-HEEP expects entry point at 0x180 */
#define XHEEP_BOOT_ADDRESS          0x00000180

/* Debug Module */
#define XHEEP_DEBUG_BASE            0x10000000
#define XHEEP_DEBUG_SIZE            0x00100000

//* ============================================================
//* Always-On (AO) Peripherals @ 0x20000000
//* ============================================================
#define XHEEP_AO_PERIPHERAL_BASE    0x20000000

#define XHEEP_SOC_CTRL_BASE         (XHEEP_AO_PERIPHERAL_BASE + 0x00000)
#define XHEEP_SOC_CTRL_SIZE         0x10000

#define XHEEP_BOOTROM_BASE          (XHEEP_AO_PERIPHERAL_BASE + 0x10000)
#define XHEEP_BOOTROM_SIZE          0x10000

#define XHEEP_SPI_FLASH_BASE        (XHEEP_AO_PERIPHERAL_BASE + 0x20000)
#define XHEEP_SPI_FLASH_SIZE        0x8000

#define XHEEP_SPI_MEMIO_BASE        (XHEEP_AO_PERIPHERAL_BASE + 0x28000)
#define XHEEP_SPI_MEMIO_SIZE        0x8000

#define XHEEP_DMA_BASE              (XHEEP_AO_PERIPHERAL_BASE + 0x30000)
#define XHEEP_DMA_SIZE              0x10000

#define XHEEP_POWER_MANAGER_BASE    (XHEEP_AO_PERIPHERAL_BASE + 0x40000)
#define XHEEP_POWER_MANAGER_SIZE    0x10000


//! RV_TIMER_AO
// Ref: x-heep/sw/device/lib/drivers/rv_timer/rv_timer_regs.h

#define XHEEP_RV_TIMER_AO_BASE      (XHEEP_AO_PERIPHERAL_BASE + 0x50000)
#define XHEEP_RV_TIMER_AO_SIZE      0x10000

#define XHEEP_RV_TIMER_AO_CTRL      (XHEEP_RV_TIMER_AO_BASE + 0x000)
#define XHEEP_RV_TIMER_AO_CFG0      (XHEEP_RV_TIMER_AO_BASE + 0x100)
#define XHEEP_RV_TIMER_AO_MTIME_LO  (XHEEP_RV_TIMER_AO_BASE + 0x104)
#define XHEEP_RV_TIMER_AO_MTIME_HI  (XHEEP_RV_TIMER_AO_BASE + 0x108)
#define XHEEP_RV_TIMER_AO_MTIMECMP_LO (XHEEP_RV_TIMER_AO_BASE + 0x10C)
#define XHEEP_RV_TIMER_AO_MTIMECMP_HI (XHEEP_RV_TIMER_AO_BASE + 0x110)
#define XHEEP_RV_TIMER_AO_INTR_ENABLE (XHEEP_RV_TIMER_AO_BASE + 0x114)
#define XHEEP_RV_TIMER_AO_INTR_STATE  (XHEEP_RV_TIMER_AO_BASE + 0x118)

/* CTRL register bits */
#define XHEEP_RV_TIMER_AO_CTRL_ACTIVE_0 BIT(0)

/* CFG0: prescale[11:0] | step[23:16]
 * For 100 MHz clock, prescale=0 (divide by 1), step=1 → mtime ticks at clock rate */
#define XHEEP_RV_TIMER_AO_CFG0_PRESCALE(p) ((p) & 0xFFFU)
#define XHEEP_RV_TIMER_AO_CFG0_STEP(s)     (((s) & 0xFFU) << 16)


//! FIC (Fast Interrupt Controller)

#define XHEEP_FAST_INTR_CTRL_BASE   (XHEEP_AO_PERIPHERAL_BASE + 0x60000)
#define XHEEP_FAST_INTR_CTRL_SIZE   0x10000

#define XHEEP_FIC_PENDING           (XHEEP_FAST_INTR_CTRL_BASE + 0x0)
#define XHEEP_FIC_CLEAR             (XHEEP_FAST_INTR_CTRL_BASE + 0x4)
#define XHEEP_FIC_ENABLE            (XHEEP_FAST_INTR_CTRL_BASE + 0x8)

#define XHEEP_EXT_PERIPHERAL_BASE   (XHEEP_AO_PERIPHERAL_BASE + 0x70000)
#define XHEEP_EXT_PERIPHERAL_SIZE   0x10000

#define XHEEP_PAD_CONTROL_BASE      (XHEEP_AO_PERIPHERAL_BASE + 0x80000)
#define XHEEP_PAD_CONTROL_SIZE      0x10000

#define XHEEP_GPIO_AO_BASE          (XHEEP_AO_PERIPHERAL_BASE + 0x90000)
#define XHEEP_GPIO_AO_SIZE          0x10000

//* ============================================================
//* User Peripherals @ 0x30000000
//* ============================================================
#define XHEEP_PERIPHERAL_BASE       0x30000000

#define XHEEP_PLIC_BASE             (XHEEP_PERIPHERAL_BASE + 0x00000)
#define XHEEP_PLIC_SIZE             0x10000

#define XHEEP_SPI_HOST_BASE         (XHEEP_PERIPHERAL_BASE + 0x10000)
#define XHEEP_SPI_HOST_SIZE         0x10000

#define XHEEP_GPIO_BASE             (XHEEP_PERIPHERAL_BASE + 0x20000)
#define XHEEP_GPIO_SIZE             0x10000

#define XHEEP_I2C_BASE              (XHEEP_PERIPHERAL_BASE + 0x30000)
#define XHEEP_I2C_SIZE              0x10000

#define XHEEP_RV_TIMER_BASE         (XHEEP_PERIPHERAL_BASE + 0x40000)
#define XHEEP_RV_TIMER_SIZE         0x10000

//! RV_TIMER
// Ref: x-heep/sw/device/lib/drivers/rv_timer/rv_timer_regs.h

#define XHEEP_RV_TIMER_CTRL      (XHEEP_RV_TIMER_BASE + 0x000)
#define XHEEP_RV_TIMER_CFG0      (XHEEP_RV_TIMER_BASE + 0x100)
#define XHEEP_RV_TIMER_MTIME_LO  (XHEEP_RV_TIMER_BASE + 0x104)
#define XHEEP_RV_TIMER_MTIME_HI  (XHEEP_RV_TIMER_BASE + 0x108)
#define XHEEP_RV_TIMER_MTIMECMP_LO (XHEEP_RV_TIMER_BASE + 0x10C)
#define XHEEP_RV_TIMER_MTIMECMP_HI (XHEEP_RV_TIMER_BASE + 0x110)
#define XHEEP_RV_TIMER_INTR_ENABLE (XHEEP_RV_TIMER_BASE + 0x114)
#define XHEEP_RV_TIMER_INTR_STATE  (XHEEP_RV_TIMER_BASE + 0x118)

/* CTRL register bits */
#define XHEEP_RV_TIMER_CTRL_ACTIVE_0 BIT(0)

/* CFG0: prescale[11:0] | step[23:16] */
#define XHEEP_RV_TIMER_CFG0_PRESCALE(p) ((p) & 0xFFFU)
#define XHEEP_RV_TIMER_CFG0_STEP(s)     (((s) & 0xFFU) << 16)

#define XHEEP_SPI2_BASE             (XHEEP_PERIPHERAL_BASE + 0x50000)
#define XHEEP_SPI2_SIZE             0x10000

#define XHEEP_I2S_BASE              (XHEEP_PERIPHERAL_BASE + 0x70000)
#define XHEEP_I2S_SIZE              0x10000

#define XHEEP_UART_BASE             (XHEEP_PERIPHERAL_BASE + 0x80000)
#define XHEEP_UART_SIZE             0x10000

//* ============================================================
//* External Memory
//* ============================================================
#define XHEEP_FLASH_MEM_BASE        0x40000000
#define XHEEP_FLASH_MEM_SIZE        0x01000000  /* 16MB */

#define XHEEP_EXT_SLAVE_BASE        0xF0000000
#define XHEEP_EXT_SLAVE_SIZE        0x01000000

//* ============================================================
//* Clock Configuration
//* ============================================================
#define XHEEP_REFERENCE_CLOCK_HZ    (100 * 1000 * 1000)  /* 100 MHz */
#define XHEEP_UART_BAUDRATE         256000

//* ============================================================
//* Interrupt Configuration
//* ============================================================
#define XHEEP_NUM_IRQS              64

/* UART Interrupts */
#define XHEEP_UART_IRQ_TX_WATERMARK     1
#define XHEEP_UART_IRQ_RX_WATERMARK     2
#define XHEEP_UART_IRQ_TX_EMPTY         3
#define XHEEP_UART_IRQ_RX_OVERFLOW      4
#define XHEEP_UART_IRQ_RX_FRAME_ERR     5
#define XHEEP_UART_IRQ_RX_BREAK_ERR     6
#define XHEEP_UART_IRQ_RX_TIMEOUT       7
#define XHEEP_UART_IRQ_RX_PARITY_ERR    8

/* GPIO Interrupts (GPIO 8-31 have interrupts) */
#define XHEEP_GPIO_IRQ_BASE             9   /* GPIO_INTR_8 */

/* I2C Interrupts */
#define XHEEP_I2C_IRQ_FMT_WATERMARK     33
#define XHEEP_I2C_IRQ_RX_WATERMARK      34

/* SPI2 Interrupt */
#define XHEEP_SPI2_IRQ_EVENT            49

/* I2S Interrupt */
#define XHEEP_I2S_IRQ_EVENT             50

/* External Interrupts */
#define XHEEP_EXT_IRQ_BASE              51

//* ============================================================
//* RISC-V CSR Addresses (for reference) DO NOT TOUCH - SENSIBLE
//* ============================================================
#define RISCV_CSR_MSTATUS       0x300
#define RISCV_CSR_MIE           0x304
#define RISCV_CSR_MTVEC         0x305
#define RISCV_CSR_MEPC          0x341
#define RISCV_CSR_MCAUSE        0x342
#define RISCV_CSR_MTVAL         0x343
#define RISCV_CSR_MIP           0x344

/* Machine interrupt enable bits */
#define RISCV_MIE_MSIE          BIT(3)   /* Machine Software Interrupt */
#define RISCV_MIE_MTIE          BIT(7)   /* Machine Timer Interrupt */
#define RISCV_MIE_MEIE          BIT(11)  /* Machine External Interrupt */

#endif /* __RISCV_X_HEEP_SOC_H__ */
