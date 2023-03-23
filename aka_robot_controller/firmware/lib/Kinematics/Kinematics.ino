#include "Kinematics.h"
#include "base_config.h"

#define MOTOR_MAX_RPM 120
#define WHEEL_DIAMETER 0.06
#define WHEEL_DIST 0.2
#define PWM_BITS 8

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, WHEEL_DIST, PWM_BITS);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Kinematics::output rpm;

  //simulated vel
  float linear_vel_x = 0.5;
  float angular_vel_z = 0.5;

  //calculate rpm for each motor
  rpm = kinematics.getRPM(linear_vel_x, angular_vel_z);

  Serial.print("LEFT MOTOR: ");
  Serial.print(rpm.motor_left);
  Serial.print(" RIGHT MOTOR: ");
  Serial.println(rpm.motor_right);

  delay(5000);

  int leftMotor_feedback = rpm.motor_left;
  int rightMotor_feedback = rpm.motor_right;

  Kinematics:: velocities vel;

  vel = kinematics.getVel(leftMotor_feedback, rightMotor_feedback);
  Serial.print("VEL_X: ");
  Serial.print(vel.linear_x);
  Serial.print(" ANGULAR_Z: ");
  Serial.println(vel.angular_z);

}