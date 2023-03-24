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
// #include "run_led.h"
// #include "Encoder.h"
#include "Arduino.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <FastLED.h>

#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

#define frontLed_pin 22
#define rearLed_pin 24
#define frontLED 15
#define rearLED 39

CRGB FLeds[frontLED];
CRGB RLeds[rearLED];

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
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel);

  FastLED.addLeds<WS2812,frontLed_pin, GRB>(FLeds, frontLED);
  FastLED.addLeds<WS2812,rearLed_pin, GRB>(RLeds, rearLED);
  // FastLED.setBrightness(200);

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

  if(1)
  {
    if((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      // printDebug();
      // runLed();
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

  // if (left_rpm > 0 && right_rpm > 0){runLed_forward();}
  // else if (left_rpm < 0 && right_rpm > 0){runLed_left();}
  // else if (left_rpm > 0 && right_rpm < 0){runLed_right();}

  // get actual rpm from encoder data
  Kinematics::velocities current_vel;
  current_vel = kinematics.getVel(act_rpm_left, act_rpm_right);

  char buffL[50];                                                           // print act rpm
  char msgL[50];
  char buffR[50];
  char msgR[50];

  dtostrf(act_rpm_left, 6, 2, buffL);
  sprintf(msgL, "before left: %s", buffL);
  nh.loginfo(msgL);

  dtostrf(act_rpm_right, 6, 2, buffR);
  sprintf(msgR, "before right: %s", buffR);
  nh.loginfo(msgR);

  
  char cek[50];                                                             // print req rpm

  sprintf(cek, "req left rpm = %d", left_rpm);
  nh.loginfo(cek);
  sprintf(cek, "req right rpm = %d", right_rpm);
  nh.loginfo(cek);

  // int left_pwm = map(((int)left_rpm), 0, MAX_RPM, 0, 255);
  left_control.spin(map(((int)left_rpm), 0, MAX_RPM, 0, 255));
  right_control.spin(map(((int)right_rpm), 0, MAX_RPM, 0, 255));

  int pwmLeft = map(((int)left_rpm), 0, MAX_RPM, 0, 255);
  int pwmRight = map(((int)right_rpm), 0, MAX_RPM, 0, 255);

  char pwm[50];                                                             // print req pwm

  sprintf(pwm, "left pwm = %d", pwmLeft);
  nh.loginfo(pwm);
  sprintf(pwm, "right pwm = %d", pwmRight);
  nh.loginfo(pwm);

  // get actual rpm from encoder data
  // Kinematics::velocities current_vel;
  // current_vel = kinematics.getVel(act_rpm_left, act_rpm_right);

  // pass velocities to publisher object
  // raw_vel_msg.linear.x = current_vel.linear_x;
  raw_vel_msg.linear.x = current_vel.linear_x;
  raw_vel_msg.angular.z = current_vel.angular_z;
  
  char buffLe[50];                                                           // print act rpm
  char msgLe[50];
  char buffRe[50];
  char msgRe[50];

  dtostrf(act_rpm_left, 6, 2, buffLe);
  sprintf(msgLe, "after left: %s", buffLe);
  nh.loginfo(msgLe);

  dtostrf(act_rpm_right, 6, 2, buffRe);
  sprintf(msgRe, "after right: %s", buffRe);
  nh.loginfo(msgRe);

  //publih raw_vel_msg
  raw_vel.publish(&raw_vel_msg);  
}

void stopBase()
{
  req_linear_vel_x = 0;
  req_linear_vel_y = 0;
  req_angular_vel_z = 0;
  // runLed_steady();  

  // raw_vel_msg.linear.x = req_linear_vel_x;
  // raw_vel_msg.angular.z = req_angular_vel_z;

  // //publish raw_vel_msg
  // raw_vel.publish(&raw_vel_msg);  
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

void runLed_forward()
{
  for (int i = 0; i <= (rearLED - 1) / 2; i++)
  {
    for (int j = 0; j <= 6; j++)
    {
      RLeds[i + j] = CRGB (0, 0, 0);
      RLeds[((rearLED - 1) - i) - j] = CRGB (0, 0, 0);
    }
    if (i > 6)
    {
      RLeds[i - 7] = CRGB (0, 0, 0); 
      RLeds[((rearLED - 1) - i) + 7] = CRGB (0, 0, 0);
    }
    RLeds[i] = CRGB (0, 200, 0);
    RLeds[(rearLED - 1) - i] = CRGB (0, 200, 0);

    for (int k = 0; k <= frontLED - 1; k++)
    {
      FLeds[k] = CRGB (0, 50, 0);
    }
    FastLED.show();
    FastLED.setBrightness(200);
    delay(30);
  }
}

void runLed_right()
{
  for (int i = 0; i < frontLED; i++)   // front
  {
    if (i < 3)
    {
      FLeds[i] = CRGB (0, 50, 0);
    }
    else if (i > frontLED - 4)
    {
      FLeds[i] = CRGB (0, 50, 0);
    }
    else
    {
      FLeds[i] = CRGB (0, 0, 0);
    }
  }
  for (int i = 0; i < rearLED - 1; i++)    // rear
  {
    RLeds[i] = CRGB (0, 100, 0);
  }
  FastLED.show();
  FastLED.setBrightness(200);
  delay(200);

  for (int i = 0; i < frontLED; i++)   // front
  {
    if (i < 3)
    {
      FLeds[i] = CRGB (0, 50, 0);
    }
    else if (i > frontLED - 4)
    {
      FLeds[i] = CRGB (0, 0, 0);
    }
    else
    {
      FLeds[i] = CRGB (0, 0, 0);
    }
  }
  for (int i = 0; i < rearLED ; i++)    // rear
  {
    if (i > 26)
    {
      RLeds[i] = CRGB (0, 0, 0);
    }
    else
    {
      RLeds[i] = CRGB (0, 100, 0);
    }   
  }  
  FastLED.show();
  delay(200);
}

void runLed_left()
{
  for (int i = 0; i < frontLED; i++)   // front
  {
    if (i < 3)
    {
      FLeds[i] = CRGB (0, 50, 0);
    }
    else if (i > frontLED - 4)
    {
      FLeds[i] = CRGB (0, 50, 0);
    }
    else
    {
      FLeds[i] = CRGB (0, 0, 0);
    }
  }
  for (int i = 0; i < rearLED - 1; i++)    // rear
  {
    RLeds[i] = CRGB (0, 100, 0);
  }
  FastLED.show();
  FastLED.setBrightness(200);
  delay(200);

  for (int i = 0; i < frontLED; i++)   // front
  {
    if (i < 3)
    {
      FLeds[i] = CRGB (0, 0, 0);
    }
    else if (i > frontLED - 4)
    {
      FLeds[i] = CRGB (0, 50, 0);
    }
    else
    {
      FLeds[i] = CRGB (0, 0, 0);
    }
  }
  for (int i = 0; i < rearLED ; i++)    // rear
  {
    if (i < 12)
    {
      RLeds[i] = CRGB (0, 0, 0);
    }
    else
    {
      RLeds[i] = CRGB (0, 100, 0);
    }   
  }  
  FastLED.show();
  delay(200);
}

void runLed_steady()
{
  for (int i = 10; i <= 200; i = i + 10)
  {
    for (int j = 0; j <= rearLED - 1; j ++)
    {
      RLeds[j] = CRGB (0, 100, 0);
    }
    for (int k = 0; k <= frontLED - 1; k++)
    {
      FLeds[k] = CRGB (0, 50, 0);
    }
    FastLED.setBrightness(i);
    FastLED.show();
    delay(60);    
  }
  for (int i = 200; i >= 10; i = i - 10)
  {
    for (int j = 0; j <= rearLED - 1; j ++)
    {
      RLeds[j] = CRGB (0, 100, 0);
    }
    for (int k = 0; k <= frontLED - 1; k++)
    {
      FLeds[k] = CRGB (0, 50, 0);
    }
    FastLED.setBrightness(i);
    FastLED.show();
    delay(60);    
  }
}

// void runLed()
// {
//   //get required rpm for each motor based on required velocities
//   Kinematics::output readReq = kinematics.getRPM(req_linear_vel_x, req_linear_vel_y, req_angular_vel_z);

//   //get the current rpm of each motor
//   int leftReq = readReq.motor_left;
//   int rightReq = readReq.motor_right;

//   char cek[50];                                                             // print req rpm

//   sprintf(cek, "req left led = %d", leftReq);
//   nh.loginfo(cek);
//   sprintf(cek, "req right led = %d", rightReq);
//   nh.loginfo(cek);


//   if (leftReq > 0 && rightReq > 0)             // front and rear leds in forward movement
//   {
//     for (int i = 0; i <= (rearLED - 1) / 2; i++)
//     {
//       for (int j = 0; j <= 6; j++)
//       {
//         RLeds[i + j] = CRGB (0, 0, 0);
//         RLeds[((rearLED - 1) - i) - j] = CRGB (0, 0, 0);
//       }
//       if (i > 6)
//       {
//         RLeds[i - 7] = CRGB (0, 0, 0); 
//         RLeds[((rearLED - 1) - i) + 7] = CRGB (0, 0, 0);
//       }
//       RLeds[i] = CRGB (0, 200, 0);
//       RLeds[(rearLED - 1) - i] = CRGB (0, 200, 0);

//       for (int k = 0; k <= frontLED - 1; k++)
//       {
//         FLeds[k] = CRGB (0, 50, 0);
//       }
//       FastLED.show();
//       FastLED.setBrightness(200);
//       delay(30);
//     }
//   }

//   if (leftReq < 0 && rightReq > 0)            // front and rear leds when turn left
//   {
//     for (int i = 0; i < frontLED; i++)   // front
//     {
//       if (i < 3)
//       {
//         FLeds[i] = CRGB (0, 50, 0);
//       }
//       else if (i > frontLED - 4)
//       {
//         FLeds[i] = CRGB (0, 50, 0);
//       }
//       else
//       {
//         FLeds[i] = CRGB (0, 0, 0);
//       }
//     }
//     for (int i = 0; i < rearLED - 1; i++)    // rear
//     {
//       RLeds[i] = CRGB (0, 100, 0);
//     }
//     FastLED.show();
//     FastLED.setBrightness(200);
//     delay(200);

//     for (int i = 0; i < frontLED; i++)   // front
//     {
//       if (i < 3)
//       {
//         FLeds[i] = CRGB (0, 0, 0);
//       }
//       else if (i > frontLED - 4)
//       {
//         FLeds[i] = CRGB (0, 50, 0);
//       }
//       else
//       {
//         FLeds[i] = CRGB (0, 0, 0);
//       }
//     }
//     for (int i = 0; i < rearLED ; i++)    // rear
//     {
//       if (i < 12)
//       {
//         RLeds[i] = CRGB (0, 0, 0);
//       }
//       else
//       {
//         RLeds[i] = CRGB (0, 100, 0);
//       }   
//     }  
//     FastLED.show();
//     delay(200);
//   }
  
//   if (leftReq > 0 && rightReq < 0)             // front and rear leds when turn right
//   {
//     for (int i = 0; i < frontLED; i++)   // front
//     {
//       if (i < 3)
//       {
//         FLeds[i] = CRGB (0, 50, 0);
//       }
//       else if (i > frontLED - 4)
//       {
//         FLeds[i] = CRGB (0, 50, 0);
//       }
//       else
//       {
//         FLeds[i] = CRGB (0, 0, 0);
//       }
//     }
//     for (int i = 0; i < rearLED - 1; i++)    // rear
//     {
//       RLeds[i] = CRGB (0, 100, 0);
//     }
//     FastLED.show();
//     FastLED.setBrightness(200);
//     delay(200);

//     for (int i = 0; i < frontLED; i++)   // front
//     {
//       if (i < 3)
//       {
//         FLeds[i] = CRGB (0, 50, 0);
//       }
//       else if (i > frontLED - 4)
//       {
//         FLeds[i] = CRGB (0, 0, 0);
//       }
//       else
//       {
//         FLeds[i] = CRGB (0, 0, 0);
//       }
//     }
//     for (int i = 0; i < rearLED ; i++)    // rear
//     {
//       if (i > 26)
//       {
//         RLeds[i] = CRGB (0, 0, 0);
//       }
//       else
//       {
//         RLeds[i] = CRGB (0, 100, 0);
//       }   
//     }  
//     FastLED.show();
//     delay(200);
//   }

//   if (leftReq == 0 && rightReq == 0)                       // front and rear leds in steady
//   {
//     for (int i = 10; i <= 200; i = i + 10)
//     {
//       for (int j = 0; j <= rearLED - 1; j ++)
//       {
//         RLeds[j] = CRGB (0, 100, 0);
//       }
//       for (int k = 0; k <= frontLED - 1; k++)
//       {
//         FLeds[k] = CRGB (0, 50, 0);
//       }
//       FastLED.setBrightness(i);
//       FastLED.show();
//       delay(60);    
//     }
//     for (int i = 200; i >= 10; i = i - 10)
//     {
//       for (int j = 0; j <= rearLED - 1; j ++)
//       {
//         RLeds[j] = CRGB (0, 100, 0);
//       }
//       for (int k = 0; k <= frontLED - 1; k++)
//       {
//         FLeds[k] = CRGB (0, 50, 0);
//       }
//       FastLED.setBrightness(i);
//       FastLED.show();
//       delay(60);    
//     }
//   }
// }

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



