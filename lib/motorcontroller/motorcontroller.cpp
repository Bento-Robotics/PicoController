#include "micro_rosso.h"
#include "motorcontroller.h"
#include <can.h>
#include <cstdint>
#include "can2040.h"
#include "canprotocol.h"

#define RCCHECK(fn)                                         \
{                                                           \
  rcl_ret_t temp_rc = fn;                                   \
  if ((temp_rc != RCL_RET_OK))                              \
  {                                                         \
    D_print("motorcontroller.cpp failed on [line:code]: "); \
    D_print((int)__LINE__);                                 \
    D_print(":");                                           \
    D_print((int)temp_rc);                                  \
    D_println();                                            \
    rcutils_reset_error();                                  \
  }                                                         \
}

can2040_msg rec_data;
int32_t mtrctlr_can_id;

Motorcontroller::Motorcontroller (uint32_t motorcontroller_id, rclc_parameter_server_t param_server) {

  auto setup_parameter = [&](const char* name, rclc_parameter_type_t type, auto default_value) -> void {
    // add ID in front of parameter name to prevent name collisions between motorcontroller nodes
    // ` can_id + '/' + name`
    char* buf[sizeof(name) + 4];
    sprintf(*buf, "%i/%s", motorcontroller_id, name);
    RCCHECK(rclc_add_parameter(&param_server, *buf, type));

    // set default value (if motorcontroller_config is null, set sensible default)
    switch (type) {
      case RCLC_PARAMETER_BOOL:
        RCCHECK(rclc_parameter_set_bool(&param_server, name, default_value));
        break;
      case RCLC_PARAMETER_INT:
        RCCHECK(rclc_parameter_set_int(&param_server, name, default_value));
        break;
      case RCLC_PARAMETER_DOUBLE:
        RCCHECK(rclc_parameter_set_double(&param_server, name, default_value));
        break;
      case RCLC_PARAMETER_NOT_SET:
        break;
    }
  };

  setup_parameter("can_id",            RCLC_PARAMETER_INT,    0);
  setup_parameter("frequency_scale",   RCLC_PARAMETER_DOUBLE, 32); //TODO: constrain parameter (0..100)
  setup_parameter("input_weight",      RCLC_PARAMETER_DOUBLE, 0.2);
  setup_parameter("max_pulse_width",   RCLC_PARAMETER_INT,    100);
  setup_parameter("timeout",           RCLC_PARAMETER_INT,    300);
  setup_parameter("pid/kp",            RCLC_PARAMETER_DOUBLE, 0.5);
  setup_parameter("pid/ki",            RCLC_PARAMETER_DOUBLE, 40.0);
  setup_parameter("pid/kd",            RCLC_PARAMETER_DOUBLE, 0.0);
  setup_parameter("anti_windup",       RCLC_PARAMETER_BOOL,   true);
  setup_parameter("invert_encoder",    RCLC_PARAMETER_BOOL,   true);
  //setup_parameter("can_response_mode", RCLC_PARAMETER_INT,    , 0);
  setup_parameter("gear_ratio",        RCLC_PARAMETER_DOUBLE, 44.0);
  setup_parameter("encoder_ratio",     RCLC_PARAMETER_DOUBLE, 2048.0);
  setup_parameter("rpm_max",           RCLC_PARAMETER_DOUBLE, 204.0); //TODO: calculate from other parameters.
  //setup_parameter("do_kinematics",     RCLC_PARAMETER_BOOL,   mcfg->do_kinematics,   0);

  int32_t mtrctlr_can_receiving_id;
  Motorcontroller::calculate_can_ids(SYSID_MC2, motorcontroller_id, &mtrctlr_can_id, &mtrctlr_can_receiving_id, nullptr);

  Can::add_receiver(mtrctlr_can_receiving_id,
                    [&](can2040_msg* message) {if (message->dlc == 6) rec_data = *message;}
                    );
}



const void 
Motorcontroller::calculate_can_ids (int32_t subsystem_id, int32_t node_id, int32_t* mc_input_address,
                                    int32_t* mc_output_address, int32_t* broadcast_address) {
  int32_t sysID  = subsystem_id << 8;
  int32_t inBit  = INPUTBIT  << 7;
  int32_t outBit = OUTPUTBIT << 7;
  int32_t nodeID = node_id;

  *mc_input_address  = sysID | inBit  | nodeID;
  *mc_output_address = sysID | outBit | nodeID;
  *broadcast_address = sysID | inBit  | 0b0000000;
}


static const void send_float(int cmd, float f)
{
  can2040_msg can_msg;
  can_msg.id = mtrctlr_can_id;
  can_msg.dlc = 5;

  can_msg.data[0] = cmd;
  int* ival = (int*)&f;
  can_msg.data[1] = (*ival & 0xFF000000) >> 24;
  can_msg.data[2] = (*ival & 0x00FF0000) >> 16;
  can_msg.data[3] = (*ival & 0x0000FF00) >> 8;
  can_msg.data[4] = (*ival & 0x000000FF);

  Can::transmit(&can_msg);
}


// missing unsure if neccecary: broadcastExternalSync(), setPWM()
// TODO: check if static timeout_ms works
// TODO: is it ok to just keep overwriting the same message but send out pointers?
// TODO: make individual channels separately configurable (gear ratio, etc)
// TODO: check for successful transmission: no errors
template<typename T> bool
Motorcontroller::set_value(motorcontroller_parameter what, T value) {
  static can2040_msg can_msg;
  can_msg.id = mtrctlr_can_id;

  switch (what) {
    case CAN_ID:
      calculate_can_ids(SYSID_MC2, value, &can_input_address, &can_output_address, &can_broadcast_address);
      //TODO: update CAN listener ID
      can_msg.id = can_output_address;
      break;

    case STOP:
      (void) value;
      can_msg.dlc = 3;
      can_msg.data[0] = CMD_MOTOR_SETPWM;
      can_msg.data[1] = 0x00;
      can_msg.data[2] = 0x00;
      Can::transmit(&can_msg);
      break;

    case ENABLE:
      can_msg.dlc = 1;
      can_msg.data[0] = value ? CMD_MOTOR_ENABLE : CMD_MOTOR_DISABLE;
      Can::transmit(&can_msg);
      break;

    case FREQUENCY_SCALE:
      static uint16_t freq_scale = constrain(value, 0, 100);
      can_msg.dlc = 3;
      can_msg.data[0] = CMD_MOTOR_FREQ_SCALE; 
      can_msg.data[1] = (freq_scale >> 8) & 0xFF;
      can_msg.data[2] = freq_scale & 0xFF;
      Can::transmit(&can_msg);
      break;

    case INPUT_WEIGHT:
      send_float(CMD_MOTOR_CTL_INPUTFILTER, value);
      break;

    case MAX_PULSE_WIDTH: {
      uint8_t max_pulse_width = constrain(value, 0, 127);
      can_msg.dlc = 2;
      can_msg.data[0] = CMD_MOTOR_SETPWMMAX; 
      can_msg.data[1] = max_pulse_width;
      Can::transmit(&can_msg);
      break;
    }

    case TIMEOUT: {
      uint16_t timeout_ms = value;
      can_msg.dlc = 3;
      can_msg.data[0] = CMD_MOTOR_SETTIMEOUT;
      can_msg.data[1] = (timeout_ms >> 8) & 0xFF;
      can_msg.data[2] = timeout_ms & 0xFF;
      Can::transmit(&can_msg);
      break;
    }

    case PID_KP:
      send_float(CMD_MOTOR_CTL_KP, value);
      break;

    case PID_KI:
      send_float(CMD_MOTOR_CTL_KI, value);
      break;

    case PID_KD:
      send_float(CMD_MOTOR_CTL_KD, value);
      break;

    case PID_ANTI_WINDUP:
      can_msg.dlc = 2;
      can_msg.data[0] = CMD_MOTOR_CTL_ANTIWINDUP;
      can_msg.data[1] = (bool) value;
      Can::transmit(&can_msg);
      break;

    case INVERT_ENCODER:
      can_msg.dlc = 2;
      can_msg.data[0] = CMD_MOTOR_INVERTENC;
      can_msg.data[1] = value ? 1 : 0;
      Can::transmit(&can_msg);
      break;

    case RESPONSE_MODE:
      can_msg.dlc = 1;
      can_msg.data[0] = value ? CMD_MOTOR_SENDRPM : CMD_MOTOR_SENDPOS;
      Can::transmit(&can_msg);
      break;

    case GEAR_RATIO:
      send_float(CMD_MOTOR_GEARRATIO, value);
      send_float(CMD_MOTOR_GEARRATIO2, value);
      break;

    case ENCODER_RATIO:
      send_float(CMD_MOTOR_TICKSPERREV, value);
      send_float(CMD_MOTOR_TICKSPERREV2, value);
      break;

    case RPM: {
      can_msg.dlc = 3;
      int8_t vel1 = constrain(value[0], -100, 100);
      int8_t vel2 = constrain(value[1], -100, 100);
      //TODO: if (INVERT_ENCODER) vel = - vel;
      can_msg.data[0] = CMD_MOTOR_SETPWM;
      can_msg.data[1] = vel1;
      can_msg.data[2] = vel2;
      Can::transmit(&can_msg);
      break;
    }
  }
  return false;
}
