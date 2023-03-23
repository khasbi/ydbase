#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

class Kinematics
{
  public:
    struct output
    {
      int motor_left;
      int motor_right;
    };

    struct velocities
    {
      float linear_x;
      float linear_y;
      float angular_z;
    };

    struct pwm
    {
      int motor_left;
      int motor_right;
    };
    

    Kinematics(int motor_max_rpm, float wheel_diameter, float wheel_dist, int pwm_bits);
    velocities getVel(float motor_left, float motor_right);
    output getRPM(float linear_x, float linear_y, float angular_z);
    pwm getPWM(float linear_x, float linear_y, float angular_z);
    int rpmToPWM(int rpm);

  private:
    float linear_vel_x_mins_;
    float linear_vel_y_mins_;
    float angular_vel_z_mins_;
    float circumference_;
    float tangential_vel_;
    float x_rpm_;
    float y_rpm_;
    float tan_rpm_;
    int max_rpm_;
    float wheel_dist_;
    float pwm_res_;
};

#endif