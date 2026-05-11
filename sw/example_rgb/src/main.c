/*
 * X-HEEP RGB Blinky — Zephyr counterpart of example_freertos_blinky
 *
 * Two threads communicate through a message queue, mirroring the FreeRTOS
 * sender/receiver pattern:
 *
 *   Sender (TX): blocks for SEND_PERIOD_MS, then posts value 100 to the queue.
 *   Receiver (RX): blocks on the queue indefinitely; on receipt of 100 it
 *                  cycles the RGB LED through Red > Blue > Green > Off and
 *                  prints "BL". On any other value it prints "!Q".
 *
 * GPIO pin assignment (user-domain gpio0 @ 0x30020000, PYNQ-Z2 LED LD5):
 *   FPGA (PYNQ-Z2): Pin 11=Red, 12=Blue, 13=Green  (CONFIG_XHEEP_FPGA_PYNQZ2=y)
 *   Simulation:     Pin 29=Red, 30=Blue, 31=Green   (default)
 *
 * Priorities (Zephyr: lower number = higher priority):
 *   RX — K_PRIO_PREEMPT(1)  ← mirrors mainQUEUE_RECEIVE_TASK_PRIORITY (+2)
 *   TX — K_PRIO_PREEMPT(2)  ← mirrors mainQUEUE_SEND_TASK_PRIORITY    (+1)
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>


//! CONFIGURATION

//* LED toggle period (shorter for sim)
#if defined(CONFIG_XHEEP_FPGA_PYNQZ2)
  #define SEND_PERIOD_MS  200U // 200ms
#else
  #define SEND_PERIOD_MS  3U   // 3ms (~300k clks)
#endif

//* Trigger message
#define MSG_VALUE 100U

//* GPIO pin numbers
#if defined(CONFIG_XHEEP_FPGA_PYNQZ2)
  #define PIN_RED   11U
  #define PIN_BLUE  12U
  #define PIN_GREEN 13U
#else
  #define PIN_RED   29U
  #define PIN_BLUE  30U
  #define PIN_GREEN 31U
#endif


//! GPIO

static const struct device *gpio_dev;
static uint8_t led_state;

static void led_write(bool r, bool b, bool g) {
  gpio_pin_set_raw(gpio_dev, PIN_RED,   (int)r);
  gpio_pin_set_raw(gpio_dev, PIN_BLUE,  (int)b);
  gpio_pin_set_raw(gpio_dev, PIN_GREEN, (int)g);
}

static void toggle_led(void) {
  switch (led_state) {
    case 0:   // Red
      led_write(true,  false, false);
      break;
    case 1:   // Blue
      led_write(false, true,  false);
      break;
    case 2:   // Green
      led_write(false, false, true);
      break;
    default:  // Off
      led_write(false, false, false);
      break;
  }
  led_state = (led_state + 1) % 4;
}


//! THREADS

//* Priority presets
#define PRIO_RX K_PRIO_PREEMPT(1)
#define PRIO_TX K_PRIO_PREEMPT(2)

//* Stack definitions
#define STACK_SIZE  1024U
K_THREAD_STACK_DEFINE(tx_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(rx_stack, STACK_SIZE);

//* Thread structs
static struct k_thread tx_thd, rx_thd;

//* Data passing (message queue)
K_MSGQ_DEFINE(rgb_msgq, sizeof(uint32_t), 1, 4);


//* Thread functions

static void tx_thread(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

  const uint32_t msg = MSG_VALUE;

  while (1) {
    printk("T1\r\n");
    k_msleep(SEND_PERIOD_MS);
    k_msgq_put(&rgb_msgq, &msg, K_NO_WAIT);
  }
}

static void rx_thread(void *p1, void *p2, void *p3){
  ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

  uint32_t val;

  while (1) {
    printk("T2\r\n");
    k_msgq_get(&rgb_msgq, &val, K_FOREVER);

    if (val == MSG_VALUE) {
      printk("BL\r\n");
      toggle_led();
    }
    else {
      printk("!Q\r\n");
    }
  }
}


//! MAIN

int main(void) {
  printk("SS\r\n");

  gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  if (!device_is_ready(gpio_dev)) {
    printk("GPIO not ready\r\n");
    return -1;
  }

  gpio_pin_configure(gpio_dev, PIN_RED, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
  gpio_pin_configure(gpio_dev, PIN_BLUE, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
  gpio_pin_configure(gpio_dev, PIN_GREEN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);

  k_thread_create(&rx_thd, rx_stack, STACK_SIZE, rx_thread, NULL, NULL, NULL, PRIO_RX, 0, K_NO_WAIT);
  k_thread_name_set(&rx_thd, "RX");

  k_thread_create(&tx_thd, tx_stack, STACK_SIZE, tx_thread, NULL, NULL, NULL, PRIO_TX, 0, K_NO_WAIT);
  k_thread_name_set(&tx_thd, "TX");

  return 0;
}
