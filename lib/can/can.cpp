#include <Arduino.h>
#include <can.h>
#include <tuple>

namespace Can {
//
// struct node {
//  uint32_t can_id;
//  std::function<void(can2040_msg*)> callback;
//};
//
const int ID = 0, CALLBACK = 1;
static struct can2040 cbus;
static std::vector<std::tuple<uint32_t, std::function<void(can2040_msg *)>>>
    rx_nodes;

uint8_t add_receiver(uint32_t can_id,
                     std::function<void(can2040_msg *)> callback) {
  for (auto node : rx_nodes)
    if (can_id == std::get<ID>(node))
      return -1;

  rx_nodes.emplace_back(can_id, callback);
  return 0;
}

uint8_t add_listener(std::function<void(can2040_msg *)> callback) {
  // can IDs can either be 11 or 29-bit, 0xFFFFFFFF lies outside this space so
  // we can use it as a 'select all' marker
  rx_nodes.emplace_back(0xFFFFFFFF, callback);
  return 0;
}

void transmit(can2040_msg *message) { can2040_transmit(&cbus, message); }

static void can2040_cb(struct can2040 *cd, uint32_t notify,
                       struct can2040_msg *msg) {
  if (notify == CAN2040_NOTIFY_RX) {
    for (auto node : rx_nodes) {
      if (msg->id == std::get<ID>(node) || std::get<ID>(node) == 0xFFFFFFFF)
        std::get<CALLBACK>(node)(msg);
    }
  }
}

static void PIOx_IRQHandler(void) { can2040_pio_irq_handler(&cbus); }

void setup(uint32_t gpio_rx, uint32_t gpio_tx) {
  uint32_t pio_num = 0;
  uint32_t sys_clock = F_CPU, bitrate = 500000;

  // Setup canbus
  can2040_setup(&cbus, pio_num);
  can2040_callback_config(&cbus, can2040_cb);

  // Enable irqs
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

  // Start canbus
  can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void get_statistics(struct can2040_stats *stats) {
  can2040_get_statistics(&cbus, stats);
}
}; // namespace Can
