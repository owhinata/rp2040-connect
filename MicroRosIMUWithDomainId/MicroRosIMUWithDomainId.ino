#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <builtin_interfaces/msg/time.h>
#include <rosidl_runtime_c/string_functions.h>

rcl_publisher_t imu_pub;
rcl_subscription_t bool_sub;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Bool bool_msg;

bool publish_enabled = false;

static void fill_time_now(builtin_interfaces__msg__Time *t) {
  int64_t ms = rmw_uros_epoch_millis();
  if (ms < 0) {
    t->sec = 0;
    t->nanosec = 0;
    return;
  }
  t->sec = ms / 1000;
  t->nanosec = (ms % 1000) * 1000000;
}

void bool_callback(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  publish_enabled = m->data;
  // LED制御
  digitalWrite(LED_BUILTIN, publish_enabled ? HIGH : LOW);
}

void on_timer(rcl_timer_t *, int64_t) {
  if (!publish_enabled)
    return;

  float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
  if (!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable()))
    return;
  if (!(IMU.readAcceleration(ax_g, ay_g, az_g) &&
        IMU.readGyroscope(gx_dps, gy_dps, gz_dps)))
    return;

  const float G = 9.80665f, DEG2RAD = 3.14159265358979323846f / 180.0f;

  imu_msg.linear_acceleration.x = ax_g * G;
  imu_msg.linear_acceleration.y = ay_g * G;
  imu_msg.linear_acceleration.z = az_g * G;

  imu_msg.angular_velocity.x = gx_dps * DEG2RAD;
  imu_msg.angular_velocity.y = gy_dps * DEG2RAD;
  imu_msg.angular_velocity.z = gz_dps * DEG2RAD;

  imu_msg.orientation.x = imu_msg.orientation.y = imu_msg.orientation.z = 0.0f;
  imu_msg.orientation.w = 1.0f;
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0] = -1.0;

  float sig_gyr = (0.02f * DEG2RAD), sig_acc = (0.05f * G);
  imu_msg.angular_velocity_covariance[0] =
      imu_msg.angular_velocity_covariance[4] =
          imu_msg.angular_velocity_covariance[8] = sig_gyr * sig_gyr;
  imu_msg.linear_acceleration_covariance[0] =
      imu_msg.linear_acceleration_covariance[4] =
          imu_msg.linear_acceleration_covariance[8] = sig_acc * sig_acc;

  fill_time_now(&imu_msg.header.stamp);
  rcl_publish(&imu_pub, &imu_msg, NULL);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  set_microros_transports();
  if (!IMU.begin()) {
    while (1) {
      delay(1000);
    }
  }

  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 43);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "nano_rp2040_imu_node", "", &support);

  // Publisher: /imu/data_raw
  rclc_publisher_init_default(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw");

  // Subscriber: /imu_enable (std_msgs/Bool)
  rclc_subscription_init_default(
      &bool_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/imu_enable");

  // frame_id
  rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // 100Hz Timer
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), on_timer);

  // Executor: 2 handles (1 subscriber, 1 timer)
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &bool_sub, &bool_msg,
                                 &bool_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)); }
