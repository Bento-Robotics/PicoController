/**
 * CAN test code
 * @hardware: can transceiver correctly wired to pico + any other CAN device sending data
 *
**/

#include <Arduino.h>
#include <can.h>
#include <unity.h>

const int pin_can_rx = 18;
const int pin_can_tx = 19;
// const int can_id = 256;

void setup() {
  Serial.begin();
  // while (!Serial) ;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // set up CAN node/code
  Can::setup(pin_can_rx, pin_can_tx);

  // add listener - *not* a receiver - and print all traffic
  Can::add_listener([](can2040_msg *msg) -> void {
    Serial.printf("id: %i, ", msg->id);
    Serial.printf("dlc: %i, ", msg->dlc);
    Serial.printf(
        "data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
        msg->data[5], msg->data[6], msg->data[7]);
  });

  // transmit a message (if the other CAN node shows incoming traffic you should see this there)
  can2040_msg msg;
  msg.id = 42;
  msg.dlc = 1;
  msg.data[0] = 0xFF;
  Can::transmit(&msg);

  // give the transmission time to complete
  delay(10);

  // statistics, aka see how the bus is doing
  can2040_stats stats;
  Can::get_statistics(&stats);
  Serial.printf("tx_total: %i\nrx_total: %i\ntx_attempt: %i\nparse_error: %i\n",
                stats.tx_total, stats.tx_total, stats.tx_attempt,
                stats.parse_error);


  UNITY_BEGIN();
  // did the transmission work?
  TEST_ASSERT_EQUAL(0, stats.tx_attempt);
  // is data on the bus sensible?
  TEST_ASSERT_EQUAL(0, stats.parse_error);
  UNITY_END();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
