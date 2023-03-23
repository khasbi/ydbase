//#define USE_USBCON
//#define _SAM3XA_

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "ArduinoHardware.h"

#include <Kinematics.h>
#include "base_config.h"
#include "Motor.h"
// #include "Encoder.h"
#include "Arduino.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, WHEELS_DISTANCE, PWM_BITS);

Controller left_control(MOTOR_LEFT_EN_F, MOTOR_LEFT_EN_R, MOTOR_LEFT_IN_F, MOTOR_LEFT_IN_R);
Controller right_control(MOTOR_RIGHT_EN_F, MOTOR_RIGHT_EN_R, MOTOR_RIGHT_IN_F, MOTOR_RIGHT_IN_R);

// Encoder encoder(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B, MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_A);


unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

volatile long encoder1pos = 0;
volatile long encoder2pos = 0;

float encoder1diff;
float encoder2diff;
float encoder1prev;
float encoder2prev;
float act_rpm_left;
float act_rpm_right;

float req_linear_vel_x;
float req_linear_vel_y = 0;
float req_angular_vel_z;
unsigned long prev_req_time = 0;

ros::NodeHandle nh;

void sub_cmd(const geometry_msgs::Twist& cmd_msg)
{
  //extract cmd_vel to linear and angular vel
  req_linear_vel_x = cmd_msg.linear.x;
  req_angular_vel_z = cmd_msg.angular.z;

  prev_req_time = millis();
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &sub_cmd);

geometry_msgs::Twist raw_vel_msg;
ros::Publisher raw_vel("raw_vel", &raw_vel_msg);

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600);
//  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel);
//  nh.advertise(raw_imu_pub);

  // while (!nh.connected())
  // {
  //   nh.spinOnce();
  // }
  // nh.loginfo("ROBOT CONNECTED");
  //delay(1);

  pinMode(MOTOR_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR_RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_A), enc_left_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_B), enc_left_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_A), enc_right_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_B), enc_right_B, CHANGE);
}

void loop()
{
  static unsigned long prev_control_time = 0;
  static unsigned long prev_debug_time = 0;
  currentMillis = millis();
  
  if(currentMillis - previousMillis >= 1000)
  {
      previousMillis = currentMillis;

      encoder1diff = encoder1pos - encoder1prev;
      encoder2diff = encoder2pos - encoder2prev;

      act_rpm_left = (encoder1diff / COUNTS_PER_REV) * 60;
      act_rpm_right = (encoder2diff / COUNTS_PER_REV) * 60;

      encoder1prev = encoder1pos;
      encoder2prev = encoder2pos;
  }

  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) 
  {
    moveBase();
    prev_control_time = millis();
  }

  if ((millis() - prev_control_time) >= 400) 
  {
    stopBase();
  }

  if(0)
  {
    if((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prev_debug_time = millis();
    }
  }

  nh.spinOnce();
}

void moveBase()
{
  //get required rpm for each motor based on required velocities
  Kinematics::output req_rpm = kinematics.getRPM(req_linear_vel_x, req_linear_vel_y, req_angular_vel_z);

  //get the current rpm of each motor
  int left_rpm = req_rpm.motor_left;
  int right_rpm = req_rpm.motor_right;

  char cek[50];

  sprintf(cek, "req left rpm = %d", left_rpm);
  nh.loginfo(cek);
  sprintf(cek, "req right rpm = %d", right_rpm);
  nh.loginfo(cek);

  // int left_pwm = map(((int)left_rpm), 0, MAX_RPM, 0, 255);
  left_control.spin(map(((int)left_rpm), 0, MAX_RPM, 0, 255));
  right_control.spin(map(((int)right_rpm), 0, MAX_RPM, 0, 255));

  int pwmLeft = map(((int)left_rpm), 0, MAX_RPM, 0, 255);
  int pwmRight = map(((int)right_rpm), 0, MAX_RPM, 0, 255);

  char pwm[50];

  sprintf(pwm, "left pwm = %d", pwmLeft);
  nh.loginfo(pwm);
  sprintf(pwm, "right pwm = %d", pwmRight);
  nh.loginfo(pwm);

  Kinematics::velocities current_vel;
  current_vel = kinematics.getVel(act_rpm_left, act_rpm_right);

  //pass velocities to publisher object
  raw_vel_msg.linear.x = current_vel.linear_x;
  raw_vel_msg.angular.z = current_vel.angular_z;
  
  char buffL[50];
  char msgL[50];
  char buffR[50];
  char msgR[50];

  dtostrf(act_rpm_left, 6, 2, buffL);
  sprintf(msgL, "left: %s", buffL);
  nh.loginfo(msgL);

  dtostrf(act_rpm_right, 6, 2, buffR);
  sprintf(msgR, "right: %s", buffR);
  nh.loginfo(msgR);

  //publih raw_vel_msg
  raw_vel.publish(&raw_vel_msg);  
}

void stopBase()
{
  req_linear_vel_x = 0;
  req_linear_vel_y = 0;
  req_angular_vel_z = 0;
}

// int mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
// {
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

void printDebug()
{
  char buffL[50];
  char msgL[50];
  char buffR[50];
  char msgR[50];

  dtostrf(act_rpm_left, 6, 2, buffL);
  sprintf(msgL, "left: %s", buffL);
  nh.loginfo(msgL);

  dtostrf(act_rpm_right, 6, 2, buffR);
  sprintf(msgR, "right: %s", buffR);
  nh.loginfo(msgR);
}

void enc_left_A()
{
  // look for a low-to-high on channel A
  if (digitalRead(MOTOR_LEFT_ENC_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(MOTOR_LEFT_ENC_B) == LOW) {encoder1pos = encoder1pos + 1;}      // CW
    else {encoder1pos = encoder1pos - 1;}         // CCW
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MOTOR_LEFT_ENC_B) == HIGH) {encoder1pos = encoder1pos + 1;}    // CW
    else {encoder1pos = encoder1pos - 1;}          // CCW
  }
}

void enc_left_B()
{  
  // look for a low-to-high on channel B
  if (digitalRead(MOTOR_LEFT_ENC_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(MOTOR_LEFT_ENC_A) == HIGH) {encoder1pos = encoder1pos + 1;}    // CW
    else {encoder1pos = encoder1pos - 1;}         // CCW
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MOTOR_LEFT_ENC_A) == LOW) {encoder1pos = encoder1pos + 1;}     // CW 
    else {encoder1pos = encoder1pos - 1;}          // CCW
  }
}

// ************** encoder 2 *********************

void enc_right_A()
{  
  // look for a low-to-high on channel A
  if (digitalRead(MOTOR_RIGHT_ENC_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(MOTOR_RIGHT_ENC_B) == LOW) {encoder2pos = encoder2pos - 1;}     // CW
    else {encoder2pos = encoder2pos + 1;}         // CCW
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MOTOR_RIGHT_ENC_B) == HIGH) {encoder2pos = encoder2pos - 1;}    // CW 
    else {encoder2pos = encoder2pos + 1;}          // CCW
  }
}

void enc_right_B()
{  
  // look for a low-to-high on channel B
  if (digitalRead(MOTOR_RIGHT_ENC_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(MOTOR_RIGHT_ENC_A) == HIGH) {encoder2pos = encoder2pos - 1;}    // CW
    else {encoder2pos = encoder2pos + 1;}         // CCW
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MOTOR_RIGHT_ENC_A) == LOW) {encoder2pos = encoder2pos - 1;}     // CW
    else {encoder2pos = encoder2pos + 1;}         // CCW
  }
}



