

// CMD VEL SUBSCRIBER CALLBACK
void wheel_pwm_callback(const void *msgin)
{
  float pwm_left = (float) msg_pwm.pwm_left;
  float pwm_right = (float) msg_pwm.pwm_right;
  int dir_left = (int) msg_pwm.dir_left;
  int dir_right = (int) msg_pwm.dir_right;

  analogWrite(md_left_pwm, pwm_left);
  analogWrite(md_right_pwm, pwm_right);

  digitalWrite(md_left_dir, dir_left);
  digitalWrite(md_right_dir, dir_right);

}