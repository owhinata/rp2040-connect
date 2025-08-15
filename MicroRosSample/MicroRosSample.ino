#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t pub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
std_msgs__msg__Int32 msg;

void on_timer(rcl_timer_t*, int64_t) {
  rcl_publish(&pub, &msg, NULL);
  msg.data++;
}

void setup() {
  // 1) transport: USB CDC（/dev/ttyACM*）
  set_microros_transports();

  // 2) micro-ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "nano_rp2040_node", "", &support);

  rclc_publisher_init_default(
    &pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "pico_counter"
  );

  // 3) 1秒周期タイマ + executor
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), on_timer);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  msg.data = 0;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(1);
}
