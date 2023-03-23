#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float wheel_dist, int pwm_bits):
  circumference_(PI * wheel_diameter),
  max_rpm_(motor_max_rpm),
  wheel_dist_(wheel_dist),
  pwm_res_(pow(2, pwm_bits) - 1)
{
}

Kinematics::output Kinematics::getRPM(float linear_x, float linear_y,float angular_z)
{
  //convert m/s to m/min
  linear_vel_x_mins_ = linear_x * 60;
  linear_vel_y_mins_ = linear_y * 60;
  
  //convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;

  //Vt = ω * radius
  tangential_vel_ = angular_vel_z_mins_ * (wheel_dist_ / 2);

  x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

  Kinematics::output rpm;

  //calculate for the target motor RPM and direction
  rpm.motor_left = x_rpm_ - y_rpm_ - tan_rpm_;
  rpm.motor_left = constrain(rpm.motor_left, -max_rpm_, max_rpm_);

  rpm.motor_right = x_rpm_ + y_rpm_ + tan_rpm_;
  rpm.motor_right = constrain(rpm.motor_right, -max_rpm_, max_rpm_);

  return rpm;

}

Kinematics::pwm Kinematics::getPWM(float linear_x, float linear_y, float angular_z)
{
  Kinematics::output rpm;
  Kinematics::pwm pwm;

  rpm = getRPM((float)linear_x, (float)linear_y, (float)angular_z);

  //conevert RPM to PWM
  pwm.motor_left = rpmToPWM(rpm.motor_left);
  pwm.motor_right = rpmToPWM(rpm.motor_right);

  return pwm;

}

Kinematics::velocities Kinematics::getVel(float motor_left, float motor_right)
{
  Kinematics::velocities vel;

  float avg_rpm_x;
  float avg_rps_x;
  float avg_rpm_a;
  float avg_rps_a;

  avg_rpm_x = (float)(motor_left + motor_right) / 2;    // RPM
  //convert rpm to revolutions per second
  avg_rps_x = avg_rpm_x / 60;                           // RPS
  vel.linear_x = (avg_rps_x * circumference_);          // m/s

  vel.linear_y = 0; //diff drive
  
  avg_rpm_a = (float)(motor_right - motor_left) / 2;    
  //convert rpm to revolutions per second
  avg_rps_a = avg_rpm_a / 60;
  vel.angular_z = (avg_rps_a * circumference_) / (wheel_dist_ / 2); // rad/s

  return vel;

}

int Kinematics::rpmToPWM(int rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
  return (((float) rpm / (float) max_rpm_) * pwm_res_);
}








