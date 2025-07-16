void sensor_callback(rcl_timer_t *timer, int64_t last_call_time){
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  yaw = (360 - orientationData.orientation.x) * (PI / 180);

  msg_sensor.enc_left_val = enc_val_left;
  msg_sensor.enc_right_val = enc_val_right;
  msg_sensor.imu_yaw = yaw;

  RCCHECK(rcl_publish(&sensor_publisher, &msg_sensor, NULL)); 

  // set encoder values back to zero 
  enc_val_left = 0;
  enc_val_right = 0;

}
