// TO BE DONE
#include <include_all.cpp>

void setup()
{
  //MOTORS
  pinMode(md_left_pwm, OUTPUT);
  pinMode(md_right_pwm, OUTPUT);

  pinMode(md_left_dir, OUTPUT);
  pinMode(md_right_dir, OUTPUT);

  // ENCODERS
  pinMode(enc_right_A, INPUT_PULLUP);
  pinMode(enc_right_B, INPUT_PULLUP);

  pinMode(enc_left_A, INPUT_PULLUP);
  pinMode(enc_left_B, INPUT_PULLUP);
  
  gpio_set_irq_enabled_with_callback(enc_right_A, GPIO_IRQ_EDGE_RISE, true, updateEncoder);
  gpio_set_irq_enabled_with_callback(enc_left_A, GPIO_IRQ_EDGE_RISE, true, updateEncoder);

  // IMU SETUP
  // Serial.begin(9600);
  bno.begin();

  set_microros_serial_transports(Serial);

  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "microros", "", &support));

  // executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  // cmd vel topic
  const char *pwm_topic_name = "/wheel_pwm";
  const rosidl_message_type_support_t *Twist_type = ROSIDL_GET_MSG_TYPE_SUPPORT(kpbot_interface, msg, WheelPWM);

  // sensor topic
  const char *sensor_topic_name = "/sensor";
  const rosidl_message_type_support_t *Sensor_msg_type =
          ROSIDL_GET_MSG_TYPE_SUPPORT(kpbot_interface, msg, Sensor);

  //publisher for odom
  RCCHECK(rclc_publisher_init_default(&sensor_publisher,&node,Sensor_msg_type, sensor_topic_name));

  //timer for odom publisher
  int timer_period = RCL_MS_TO_NS(100);

  RCCHECK(rclc_timer_init_default(&sensor_timer, &support, timer_period, &sensor_callback));
  rclc_executor_add_timer(&executor, &sensor_timer);

  // create cmd_vel subscriber
  RCCHECK(rclc_subscription_init_default(&pwm_subscriber, &node, Twist_type, pwm_topic_name));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &msg_pwm, &wheel_pwm_callback, ON_NEW_DATA));
}

void loop()
{ 
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // analogWrite(0, 1800);
  // digitalWrite(1, HIGH);

  // analogWrite(md_left_pwm, 60);
  // analogWrite(md_right_pwm, 50);

  // digitalWrite(md_left_dir, 1);
  // digitalWrite(md_right_dir, 0);

  // ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
}


