/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP OpenTitan PLIC interrupt controller driver.
 *
 * OpenTitan rv_plic register map (relative to PLIC base address):
 *   0x000  IP0    Interrupt Pending, sources  0-31 (read-only)
 *   0x004  IP1    Interrupt Pending, sources 32-63 (read-only)
 *   0x008  LE0    Level/Edge trigger config,  sources  0-31
 *   0x00C  LE1    Level/Edge trigger config,  sources 32-63
 *   0x010  PRIO0  Priority, source 0  (3-bit field)
 *   ...           (stride 4 per source)
 *   0x10C  PRIO63 Priority, source 63
 *   0x200  IE0_0  Interrupt Enable, target 0, sources  0-31
 *   0x204  IE0_1  Interrupt Enable, target 0, sources 32-63
 *   0x208  THRESHOLD0  Priority threshold, target 0 (3-bit field)
 *   0x20C  CC0    Claim/Complete, target 0
 *   0x210  MSIP0  Software interrupt pending, hart 0
 *
 * X-HEEP specifics: 64 IRQ sources, 1 target (hart 0), max-priority = 7.
 *
 * This driver exports the riscv_plic_* API so that Zephyr's RISC-V arch layer
 * (soc_common_irq.c) can delegate level-2 interrupt enable/disable operations
 * to it when CONFIG_RISCV_HAS_PLIC is set.
 */

#define DT_DRV_COMPAT esl_epfl_x_heep_plic

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/devicetree/interrupt_controller.h>

//* -------------------------------------------------------------------
//* Register offsets (all relative to the PLIC base address)
//* -------------------------------------------------------------------
#define PLIC_IP0_OFFSET        0x000U  /* Interrupt Pending, src  0-31 (RO) */
#define PLIC_IP1_OFFSET        0x004U  /* Interrupt Pending, src 32-63 (RO) */
#define PLIC_PRIO_BASE         0x010U  /* Priority[n] = base + 0x10 + n*4   */
#define PLIC_IE0_OFFSET        0x200U  /* IE target-0, src  0-31            */
#define PLIC_IE1_OFFSET        0x204U  /* IE target-0, src 32-63            */
#define PLIC_THRESHOLD_OFFSET  0x208U  /* Priority threshold, target 0       */
#define PLIC_CC_OFFSET         0x20CU  /* Claim/Complete, target 0           */

struct plic_config {
	mem_addr_t base;
	uint32_t max_prio;
	uint32_t nr_irqs;
	uint32_t irq;                             /* L1 parent IRQ = 11 */
	void (*irq_config_func)(void);
	const struct _isr_table_entry *isr_table; /* slice of _sw_isr_table */
};


// Per-claim state for riscv_plic_get_irq() / riscv_plic_get_dev().
// X-HEEP is single-hart so no per-CPU array is needed.
static uint32_t plic_saved_irq;
static const struct device *plic_saved_dev;

//*-------------------------------------------------------------------
//*Internal helpers
//*-------------------------------------------------------------------

// Return the IE register address that controls local_irq.
//   sources  0-31 → IE0_0 at base+0x200
//   sources 32-63 → IE0_1 at base+0x204
static inline mem_addr_t ie_addr(mem_addr_t base, uint32_t local_irq)
{
	return base + PLIC_IE0_OFFSET + (local_irq >> 5) * sizeof(uint32_t);
}

//* -------------------------------------------------------------------
//* ISR / init
//* -------------------------------------------------------------------

static void plic_irq_handler(const struct device *dev)
{
	const struct plic_config *config = dev->config;
	const mem_addr_t cc = config->base + PLIC_CC_OFFSET;

	// Claim: reading CC0 atomically dequeues the highest-priority pending IRQ
	const uint32_t local_irq = sys_read32(cc);

	if (local_irq == 0U) {
		// No pending interrupt (spurious)
		return;
	}

	// Save for riscv_plic_get_irq() / riscv_plic_get_dev()
	plic_saved_irq = local_irq;
	plic_saved_dev = dev;

	if (local_irq >= config->nr_irqs) {
		z_irq_spurious(NULL);
		// z_irq_spurious does not return
	}

	// Dispatch to the peripheral ISR registered via IRQ_CONNECT
	const struct _isr_table_entry *ite = &config->isr_table[local_irq];

	ite->isr(ite->arg);

	// Complete: write the IRQ id back to release the claim
	sys_write32(local_irq, cc);
}

static int plic_init(const struct device *dev)
{
	const struct plic_config *config = dev->config;

	// Disable all interrupt enables for target 0
	sys_write32(0U, config->base + PLIC_IE0_OFFSET);
	sys_write32(0U, config->base + PLIC_IE1_OFFSET);

	// Set threshold to 0 so all priority levels ≥ 1 are delivered
	sys_write32(0U, config->base + PLIC_THRESHOLD_OFFSET);

	// Set all source priorities to 0 (masked)
	for (uint32_t i = 0; i < config->nr_irqs; i++) {
		sys_write32(0U, config->base + PLIC_PRIO_BASE + i * sizeof(uint32_t));
	}

	// Connect plic_irq_handler to the CPU-level external interrupt and enable it
	config->irq_config_func();

	return 0;
}

//*
//* Public riscv_plic_* API
//* (required by arch/riscv soc_common_irq.c when CONFIG_RISCV_HAS_PLIC)
//*

void riscv_plic_irq_enable(uint32_t irq)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct plic_config *config = dev->config;
	const uint32_t local = irq_from_level_2(irq);
	const mem_addr_t addr = ie_addr(config->base, local);

	uint32_t val = sys_read32(addr);

	WRITE_BIT(val, local & 0x1fU, true);
	sys_write32(val, addr);
}

void riscv_plic_irq_disable(uint32_t irq)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct plic_config *config = dev->config;
	const uint32_t local = irq_from_level_2(irq);
	const mem_addr_t addr = ie_addr(config->base, local);

	uint32_t val = sys_read32(addr);

	WRITE_BIT(val, local & 0x1fU, false);
	sys_write32(val, addr);
}

int riscv_plic_irq_is_enabled(uint32_t irq)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct plic_config *config = dev->config;
	const uint32_t local = irq_from_level_2(irq);
	const mem_addr_t addr = ie_addr(config->base, local);

	return (int)((sys_read32(addr) >> (local & 0x1fU)) & 1U);
}

void riscv_plic_set_priority(uint32_t irq, uint32_t prio)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct plic_config *config = dev->config;
	const uint32_t local = irq_from_level_2(irq);

	if (prio > config->max_prio) {
		prio = config->max_prio;
	}

	sys_write32(prio, config->base + PLIC_PRIO_BASE + local * sizeof(uint32_t));
}

unsigned int riscv_plic_get_irq(void)
{
	return plic_saved_irq;
}

const struct device *riscv_plic_get_dev(void)
{
	return plic_saved_dev;
}

//* Stubs for optional API that X-HEEP does not support

int riscv_plic_irq_set_affinity(uint32_t irq, uint32_t cpumask)
{
	ARG_UNUSED(irq);
	ARG_UNUSED(cpumask);
	return -ENOTSUP;
}

void riscv_plic_irq_set_pending(uint32_t irq)
{
	/* X-HEEP OpenTitan PLIC: MSIP0 exists but software interrupts
	   are not used by any current driver.  No-op for now. */
	ARG_UNUSED(irq);
}


//* 
//* Instantiation macro
//* 
#define PLIC_X_HEEP_INIT(n)                                                            \
	static void plic_irq_config_func_##n(void)                                     \
	{                                                                              \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, plic_irq_handler,                     \
			    DEVICE_DT_INST_GET(n), 0);                                 \
		irq_enable(DT_INST_IRQN(n));                                           \
	}                                                                              \
                                                                                       \
	static const struct plic_config plic_config_##n = {                            \
		.base            = DT_INST_REG_ADDR(n),                                \
		.max_prio        = DT_INST_PROP(n, riscv_max_priority),                \
		.nr_irqs         = DT_INST_PROP(n, riscv_ndev),                        \
		.irq             = DT_INST_IRQN(n),                                    \
		.irq_config_func = plic_irq_config_func_##n,                           \
		.isr_table       = &_sw_isr_table[INTC_INST_ISR_TBL_OFFSET(n)],       \
	};                                                                             \
                                                                                       \
	IRQ_PARENT_ENTRY_DEFINE(plic##n, DEVICE_DT_INST_GET(n), DT_INST_IRQN(n),      \
				INTC_INST_ISR_TBL_OFFSET(n),                           \
				DT_INST_INTC_GET_AGGREGATOR_LEVEL(n));                 \
                                                                                       \
	DEVICE_DT_INST_DEFINE(n, plic_init, NULL, NULL, &plic_config_##n,             \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PLIC_X_HEEP_INIT)
