/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP Timer Driver
 *
 * Implements the Zephyr counter API for the lowRISC rv_timer IP in the
 * X-HEEP user peripheral domain (0x30040000).
 *
 * The rv_timer interrupt is routed through the Fast Interrupt Controller
 * (FIC), not the PLIC. set_alarm / cancel_alarm return -ENOTSUP
 * until fast-IRQ support is added.
 */


//! INCLUDES

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>


//! DRIVER COMPAT

#define DT_DRV_COMPAT esl_epfl_x_heep_rv_timer


//! REGISTER MAP

/* Offsets from IP base (confirmed against rv_timer_regs.h) */
#define RV_TIMER_CTRL        0x000U
#define RV_TIMER_CFG0        0x100U
#define RV_TIMER_MTIME_LO    0x104U
#define RV_TIMER_MTIME_HI    0x108U
#define RV_TIMER_MTIMECMP_LO 0x10CU
#define RV_TIMER_MTIMECMP_HI 0x110U
#define RV_TIMER_INTR_ENABLE 0x114U
#define RV_TIMER_INTR_STATE  0x118U

/* CTRL bits */
#define CTRL_ACTIVE_0  BIT(0)

/* CFG0 fields */
#define CFG0_STEP(s)  (((s) & 0xFFU) << 16)


//! DRIVER STRUCTURES

struct rv_timer_x_heep_config {
  /* counter_config_info MUST be first — counter API casts dev->config to it */
  struct counter_config_info info;
  mem_addr_t base;
};

/* No mutable driver state needed for polling-only mode */
struct rv_timer_x_heep_data {
};


//! REGISTER HELPERS

static inline uint32_t reg_read(const struct rv_timer_x_heep_config *cfg,
                                uint32_t offset)
{
  return sys_read32(cfg->base + offset);
}

static inline void reg_write(const struct rv_timer_x_heep_config *cfg,
                             uint32_t offset, uint32_t val)
{
  sys_write32(val, cfg->base + offset);
}


//! COUNTER API IMPLEMENTATION

static int rv_timer_start(const struct device *dev)
{
  const struct rv_timer_x_heep_config *cfg = dev->config;

  reg_write(cfg, RV_TIMER_CFG0, CFG0_STEP(1));
  reg_write(cfg, RV_TIMER_CTRL, CTRL_ACTIVE_0);
  return 0;
}

static int rv_timer_stop(const struct device *dev)
{
  const struct rv_timer_x_heep_config *cfg = dev->config;

  reg_write(cfg, RV_TIMER_CTRL, 0U);
  return 0;
}

static int rv_timer_get_value(const struct device *dev, uint32_t *ticks)
{
  const struct rv_timer_x_heep_config *cfg = dev->config;

  *ticks = reg_read(cfg, RV_TIMER_MTIME_LO);
  return 0;
}

static uint32_t rv_timer_get_top_value(const struct device *dev)
{
  /* 32-bit counter wraps at UINT32_MAX */
  return UINT32_MAX;
}

static uint32_t rv_timer_get_freq(const struct device *dev)
{
  const struct rv_timer_x_heep_config *cfg = dev->config;

  return cfg->info.freq;
}

static int rv_timer_set_alarm(const struct device *dev, uint8_t chan_id,
                              const struct counter_alarm_cfg *alarm_cfg)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(chan_id);
  ARG_UNUSED(alarm_cfg);
  /* Fast-IRQ not yet supported */
  return -ENOTSUP;
}

static int rv_timer_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(chan_id);
  return -ENOTSUP;
}

static int rv_timer_set_top_value(const struct device *dev,
                                  const struct counter_top_cfg *cfg)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(cfg);
  /* Hardware counter wraps at UINT32_MAX; top reconfiguration not supported */
  return -ENOTSUP;
}

static int rv_timer_get_pending_int(const struct device *dev)
{
  const struct rv_timer_x_heep_config *cfg = dev->config;

  return (reg_read(cfg, RV_TIMER_INTR_STATE) & BIT(0)) ? 1 : 0;
}


//! DRIVER API TABLE

static DEVICE_API(counter, rv_timer_x_heep_driver_api) = {
  .start            = rv_timer_start,
  .stop             = rv_timer_stop,
  .get_value        = rv_timer_get_value,
  .set_alarm        = rv_timer_set_alarm,
  .cancel_alarm     = rv_timer_cancel_alarm,
  .set_top_value    = rv_timer_set_top_value,
  .get_pending_int  = rv_timer_get_pending_int,
  .get_top_value    = rv_timer_get_top_value,
  .get_freq         = rv_timer_get_freq,
};


//! INIT AND INSTANTIATION

static int rv_timer_x_heep_init(const struct device *dev)
{
  /* Start counting immediately on init */
  return rv_timer_start(dev);
}

#define RV_TIMER_X_HEEP_INIT(n)                                              \
  static struct rv_timer_x_heep_data rv_timer_x_heep_data_##n;               \
                                                                              \
  static const struct rv_timer_x_heep_config rv_timer_x_heep_cfg_##n = {     \
    .info = {                                                                 \
      .max_top_value = UINT32_MAX,                                            \
      .freq          = DT_INST_PROP(n, clock_frequency),                      \
      .flags         = COUNTER_CONFIG_INFO_COUNT_UP,                          \
      .channels      = 0,                                                     \
    },                                                                        \
    .base = DT_INST_REG_ADDR(n),                                              \
  };                                                                          \
                                                                              \
  DEVICE_DT_INST_DEFINE(n,                                                   \
    rv_timer_x_heep_init,                                                     \
    NULL,                                                                     \
    &rv_timer_x_heep_data_##n,                                                \
    &rv_timer_x_heep_cfg_##n,                                                 \
    PRE_KERNEL_1,                                                             \
    CONFIG_COUNTER_INIT_PRIORITY,                                             \
    &rv_timer_x_heep_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RV_TIMER_X_HEEP_INIT)
