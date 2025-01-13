//#include "micro_rosso.h"
#include "eduart_drive.h"
#include "motorcontroller.h"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joy.h>

#include <std_srvs/srv/set_bool.h>

#include <rclc_parameter/rclc_parameter.h>

#define RCCHECK(fn)            \
{                              \
  rcl_ret_t temp_rc = fn;      \
  if ((temp_rc != RCL_RET_OK)) \
  {                            \
    return false;              \
  }                            \
}
#define RCNOCHECK(fn)     \
{                         \
  rcl_ret_t temp_rc = fn; \
  (void)temp_rc;          \
}



static std_msgs__msg__Bool msg_is_enabled;
static std_msgs__msg__Float32MultiArray msg_rpms;

static geometry_msgs__msg__Twist msg_max_velocities;
static std_msgs__msg__Float32MultiArray msg_max_rpms;

static geometry_msgs__msg__Twist msg_velocity;
static sensor_msgs__msg__Joy msg_joystick;

static std_msgs__msg__Float32MultiArray msg_rpm_override;
static std_srvs__srv__SetBool_Response srv_enable;

static rclc_parameter_server_t mc_param_server;

std::vector<Motorcontroller> motorcontrollers;


EduArt_Drive::EduArt_Drive() {
  msg_is_enabled.data = false;
}


bool
EduArt_Drive::setup(const char* topics_namespace, timer_descriptor &timer) {
  //micro_rosso::publishers.push_back();

  //TODO: add parameters to micro_rosso
  // initialize motorcontrollers
  for (int i = 0; i < motorcontrollers.size(); i++) {
    motorcontrollers[i] = Motorcontroller(i, micro_rosso::param_server);
  }
  return false;
}


static void
report_callback(int64_t last_call_time) {
  //TODO: mäth
}


//bool on_parameter_update(const Parameter * old_param, const Parameter * new_param, void * context) {
//  RCL_UNUSED(context);
//
//  if (old_param == NULL && new_param == NULL) {
//    printf("Callback error, both parameters are NULL\n");
//    return false;
//  }
//
//  if (old_param == NULL) {
//    printf("Creating new parameter %s\n", new_param->name.data);
//  } else if (new_param == NULL) {
//    printf("Deleting parameter %s\n", old_param->name.data);
//  } else {
//    printf("Parameter %s modified.", old_param->name.data);
//    switch (old_param->value.type) {
//      case RCLC_PARAMETER_BOOL:
//        printf(
//          " Old value: %d, New value: %d (bool)", old_param->value.bool_value,
//          new_param->value.bool_value);
//        break;
//      case RCLC_PARAMETER_INT:
//        printf(
//          " Old value: %ld, New value: %ld (int)", old_param->value.integer_value,
//          new_param->value.integer_value);
//        break;
//      case RCLC_PARAMETER_DOUBLE:
//        printf(
//          " Old value: %f, New value: %f (double)", old_param->value.double_value,
//          new_param->value.double_value);
//        break;
//      default:
//        break;
//    }
//    printf("\n");
//  }
//
//  return true;
//}
