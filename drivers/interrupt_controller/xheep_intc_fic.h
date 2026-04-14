/*
  * X-HEEP Fast Interrupt Controller (FIC) — public driver API.
  * 
  * Other Zephyr drivers need to have access to call these function declarations
  * from the FIC. We need to declare the function prototypes in a header file,
  * so other driver sources (such as rv_timer) can use this functions in their
  * internal processes.
*/

#ifndef DRIVERS_INTERRUPT_CONTROLLER_XHEEP_INTC_FIC_H_
#define DRIVERS_INTERRUPT_CONTROLLER_XHEEP_INTC_FIC_H_

#include <stdint.h>


//! FIC SOURCE INDICES (from fast_intr_ctrl.sv)

#define XHEEP_FIC_SRC_TIMER1    0u   // rv_timer channel 1  - mcause 16
#define XHEEP_FIC_SRC_TIMER2    1u   // rv_timer channel 2  - mcause 17
#define XHEEP_FIC_SRC_TIMER3    2u   // rv_timer channel 3  - mcause 18
#define XHEEP_FIC_SRC_DMA_DONE  3u   // DMA done            - mcause 19
#define XHEEP_FIC_SRC_SPI       4u   // SPI host            - mcause 20
#define XHEEP_FIC_SRC_SPI_FLASH 5u   // SPI flash           - mcause 21
#define XHEEP_FIC_SRC_GPIO_AO0  6u   // AO GPIO 0           - mcause 22
#define XHEEP_FIC_SRC_GPIO_AO1  7u   // AO GPIO 1           - mcause 23
#define XHEEP_FIC_SRC_GPIO_AO2  8u   // AO GPIO 2           - mcause 24
#define XHEEP_FIC_SRC_GPIO_AO3  9u   // AO GPIO 3           - mcause 25
#define XHEEP_FIC_SRC_GPIO_AO4  10u  // AO GPIO 4           - mcause 26
#define XHEEP_FIC_SRC_GPIO_AO5  11u  // AO GPIO 5           - mcause 27
#define XHEEP_FIC_SRC_GPIO_AO6  12u  // AO GPIO 6           - mcause 28
#define XHEEP_FIC_SRC_GPIO_AO7  13u  // AO GPIO 7           - mcause 29
#define XHEEP_FIC_SRC_DMA_WIN   14u  // DMA window done     - mcause 30
#define XHEEP_FIC_SRC_EXT       15u  // External peripheral - mcause 31
#define XHEEP_FIC_NUM_SRC       16u


//! MCAUSE HELPER

// mcause for a given FIC source
#define XHEEP_FIC_MCAUSE(src)  (16u + (uint32_t)(src))


//! FUNCTION PROTOTYPES
// TODO: define with brief (or maybe do a doxygen...)

/*
  Enable a FIC source: sets FAST_INTR_ENABLE bit N and unmasks MIE[16+N]
  Call before arming any hardware interrupt path (e.g., IP INTR_ENABLE reg)
*/
void xheep_fic_irq_enable(uint8_t src);

/*
  Disable a FIC source: clears MIE[16+N] first, then FAST_INTR_ENABLE bit N
  Safe to call from within or outside an ISR
*/
void xheep_fic_irq_disable(uint8_t src);

/*
  Acknowledge a pending FIC interrupt (W1C on FAST_INTR_CLEAR bit N)
  Must be called from the ISR before re-enabling or returning
*/
void xheep_fic_irq_clear(uint8_t src);

/*
 Returns 1 if FIC source N is pending, 0 otherwise
*/
int xheep_fic_irq_is_pending(uint8_t src);


#endif
// End of DRIVERS_INTERRUPT_CONTROLLER_XHEEP_INTC_FIC_H_
