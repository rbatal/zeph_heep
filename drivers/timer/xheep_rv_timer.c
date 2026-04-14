/*
 * author: rbatal (rodrigo.batal@gmail.com)
 *
 * X-HEEP Timer driver for Zephyr.
 *
 * The rv_timer interrupt is routed through the Fast Interrupt Controller
 * (FIC) source 0 - mcause 16.
 */


//! INCLUDES

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include "../interrupt_controller/xheep_intc_fic.h"


//! DRIVER COMPAT

#define DT_DRV_COMPAT esl_epfl_x_heep_rv_timer


//! CONSTANTS

//* Register Offsets
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

struct rv_timer_x_heep_data {
  counter_alarm_callback_t alarm_cb;
  void                    *alarm_user_data;
};


//! REGISTER HELPERS

static inline uint32_t reg_read(const struct rv_timer_x_heep_config *cfg,
                                uint32_t offset) {
  return sys_read32(cfg->base + offset);
}

static inline void reg_write(const struct rv_timer_x_heep_config *cfg,
                             uint32_t offset, uint32_t val) {
  sys_write32(val, cfg->base + offset);
}


//! COUNTER API IMPLEMENTATION

static int rv_timer_start(const struct device *dev) {
  const struct rv_timer_x_heep_config *cfg = dev->config;

  reg_write(cfg, RV_TIMER_CFG0, CFG0_STEP(1));
  reg_write(cfg, RV_TIMER_CTRL, CTRL_ACTIVE_0);
  return 0;
}

static int rv_timer_stop(const struct device *dev) {
  const struct rv_timer_x_heep_config *cfg = dev->config;

  reg_write(cfg, RV_TIMER_CTRL, 0U);
  return 0;
}

static int rv_timer_get_value(const struct device *dev, uint32_t *ticks) {
  const struct rv_timer_x_heep_config *cfg = dev->config;

  *ticks = reg_read(cfg, RV_TIMER_MTIME_LO);
  return 0;
}

static uint32_t rv_timer_get_top_value(const struct device *dev) {
  /* 32-bit counter wraps at UINT32_MAX */
  return UINT32_MAX;
}

static uint32_t rv_timer_get_freq(const struct device *dev) {
  const struct rv_timer_x_heep_config *cfg = dev->config;

  return cfg->info.freq;
}

static int rv_timer_set_alarm(const struct device *dev, uint8_t chan_id,
                              const struct counter_alarm_cfg *alarm_cfg) {
  const struct rv_timer_x_heep_config *cfg = dev->config;
  struct rv_timer_x_heep_data *data = dev->data;
  uint32_t now, target;

  if (chan_id != 0) {
    return -EINVAL;
  }

  now = reg_read(cfg, RV_TIMER_MTIME_LO);

  if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
    target = alarm_cfg->ticks;
  }
  else {
    target = now + alarm_cfg->ticks;
  }

  data->alarm_cb        = alarm_cfg->callback;
  data->alarm_user_data = alarm_cfg->user_data;

  /*
    Write MTIMECMP: set HI to UINT32_MAX first so the compare never
                    triggers spuriously while LO is being written.
  */
  reg_write(cfg, RV_TIMER_MTIMECMP_HI, UINT32_MAX);
  reg_write(cfg, RV_TIMER_MTIMECMP_LO, target);
  reg_write(cfg, RV_TIMER_MTIMECMP_HI, 0U);

  /*
    Arm the IP interrupt output and the FIC source
  */
  reg_write(cfg, RV_TIMER_INTR_ENABLE, BIT(0));
  xheep_fic_irq_enable(XHEEP_FIC_SRC_TIMER2);

  return 0;
}

static int rv_timer_cancel_alarm(const struct device *dev, uint8_t chan_id) {
  const struct rv_timer_x_heep_config *cfg = dev->config;
  struct rv_timer_x_heep_data *data = dev->data;

  if (chan_id != 0) {
    return -EINVAL;
  }

  xheep_fic_irq_disable(XHEEP_FIC_SRC_TIMER2);
  reg_write(cfg, RV_TIMER_INTR_ENABLE, 0U);

  data->alarm_cb        = NULL;
  data->alarm_user_data = NULL;

  return 0;
}

static int rv_timer_set_top_value(const struct device *dev,
                                  const struct counter_top_cfg *cfg) {
  ARG_UNUSED(dev);
  ARG_UNUSED(cfg);
  /* Hardware counter wraps at UINT32_MAX; top reconfiguration not supported */
  return -ENOTSUP;
}

static uint32_t rv_timer_get_pending_int(const struct device *dev) {
  const struct rv_timer_x_heep_config *cfg = dev->config;

  return (reg_read(cfg, RV_TIMER_INTR_STATE) & BIT(0)) ? 1 : 0;
}


//! ISR

static void rv_timer_isr(const void *arg) {
  const struct device *dev = arg;
  const struct rv_timer_x_heep_config *cfg = dev->config;
  struct rv_timer_x_heep_data *data = dev->data;
  counter_alarm_callback_t cb = data->alarm_cb;
  void *user_data = data->alarm_user_data;
  uint32_t now;

  // Clear IP interrupt state before disabling FIC to avoid re-entry
  reg_write(cfg, RV_TIMER_INTR_STATE, BIT(0));
  xheep_fic_irq_clear(XHEEP_FIC_SRC_TIMER2);

  // Disarm: disable FIC source and IP interrupt output
  xheep_fic_irq_disable(XHEEP_FIC_SRC_TIMER2);
  reg_write(cfg, RV_TIMER_INTR_ENABLE, 0U);

  data->alarm_cb        = NULL;
  data->alarm_user_data = NULL;

  if (cb != NULL) {
    now = reg_read(cfg, RV_TIMER_MTIME_LO);
    cb(dev, 0, now, user_data);
  }
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


//! INITIALIZATION FUNCTION

static int rv_timer_x_heep_init(const struct device *dev) {
  const struct rv_timer_x_heep_config *cfg = dev->config;

  // Clear any stale interrupt state before arming IRQ line
  reg_write(cfg, RV_TIMER_INTR_ENABLE, 0U);
  reg_write(cfg, RV_TIMER_INTR_STATE, BIT(0));

  IRQ_CONNECT(DT_INST_IRQN(0), 0, rv_timer_isr, DEVICE_DT_INST_GET(0), 0);
  irq_enable(DT_INST_IRQN(0));

  return rv_timer_start(dev);
}


//! INSTANTIATION MACRO

#define RV_TIMER_X_HEEP_INIT(n)                                               \
  static struct rv_timer_x_heep_data rv_timer_x_heep_data_##n;                \
                                                                              \
  static const struct rv_timer_x_heep_config rv_timer_x_heep_cfg_##n = {      \
    .info = {                                                                 \
      .max_top_value = UINT32_MAX,                                            \
      .freq          = DT_INST_PROP(n, clock_frequency),                      \
      .flags         = COUNTER_CONFIG_INFO_COUNT_UP,                          \
      .channels      = 1,                                                     \
    },                                                                        \
    .base = DT_INST_REG_ADDR(n),                                              \
  };                                                                          \
                                                                              \
  DEVICE_DT_INST_DEFINE(n,                                                    \
    rv_timer_x_heep_init,                                                     \
    NULL,                                                                     \
    &rv_timer_x_heep_data_##n,                                                \
    &rv_timer_x_heep_cfg_##n,                                                 \
    PRE_KERNEL_1,                                                             \
    CONFIG_COUNTER_INIT_PRIORITY,                                             \
    &rv_timer_x_heep_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RV_TIMER_X_HEEP_INIT)
