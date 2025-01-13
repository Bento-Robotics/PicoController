extern "C" {
#include <can2040.h>
}
#include <functional>

namespace Can {

/**
 * add callback for specified can ID
 * @ret -1 ⇒ ID already in use
 *
 * copy the CAN message and process it later, not inside the callback,
 * since callbacks run as interrupts, and the message pointer isn't valid
 * outside them.
 */
uint8_t add_receiver(uint32_t can_id,
                     std::function<void(can2040_msg *)> callback);

/**
 * add callback for all CAN traffic
 *
 * copy the CAN message and process it later, not inside the callback,
 * since callbacks run as interrupts, and the message pointer isn't valid
 * outside them.
 */
uint8_t add_listener(std::function<void(can2040_msg *)> callback);

// transmit message
void transmit(can2040_msg *message);

void setup(uint32_t gpio_rx = 4, uint32_t gpio_tx = 5);

void get_statistics(struct can2040_stats *stats);
}; // namespace Can
