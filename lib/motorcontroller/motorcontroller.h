#include <Arduino.h>
#include <rclc_parameter/rclc_parameter.h>
#include <can2040.h>

// struct motorcontroller_config {
//
//   uint8_t max_pulse_width;
//   //uint16_t can_id;
//   uint16_t frequency_scale;
//   uint16_t timeout;
//
//   bool enable;
//   bool anti_windup;
//   bool invert_encoder;
//   //bool do_kinematics;
//   //CanResponse responseMode;
//
//   float input_weight;
//   float pid[3]; // {P,I,D}
//   float gear_ratio;
//   float encoder_ratio;
//   float rpm_max;
//
// };

enum motorcontroller_parameter {
  CAN_ID,
  STOP,
  ENABLE,
  FREQUENCY_SCALE,
  INPUT_WEIGHT,
  MAX_PULSE_WIDTH,
  TIMEOUT,
  PID_KP,
  PID_KI,
  PID_KD,
  PID_ANTI_WINDUP,
  INVERT_ENCODER,
  RESPONSE_MODE,
  GEAR_RATIO,
  ENCODER_RATIO,
  RPM,
};

class Motorcontroller {
private:
  int32_t can_input_address, can_output_address, can_broadcast_address;

public:
  Motorcontroller (uint32_t motorcontroller_id, rclc_parameter_server_t param_server);

  static const void calculate_can_ids (int32_t subsystem_id, int32_t node_id, int32_t* mc_input_address, int32_t* mc_output_address, int32_t* broadcast_address);

  template <typename T>
  bool set_value(motorcontroller_parameter what, T value);
};
