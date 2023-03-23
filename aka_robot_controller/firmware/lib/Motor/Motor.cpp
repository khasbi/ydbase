#include "Motor.h"
#include "base_config.h"
// #include "Arduino.h"

Controller::Controller(int en_pinA, int en_pinB, int motor_pinA, int motor_pinB):
  en_pinA_(en_pinA),
  en_pinB_(en_pinB),
  motor_pinA_(motor_pinA),
  motor_pinB_(motor_pinB)
{
  pinMode(en_pinA_, OUTPUT);
  pinMode(en_pinB_, OUTPUT);
  pinMode(motor_pinA_, OUTPUT);
  pinMode(motor_pinB_, OUTPUT);
}

void Controller::spin(int pwm)
{
  if (pwm > 0)
  {
    digitalWrite(en_pinA_, HIGH);
    digitalWrite(en_pinB_, HIGH);
    analogWrite(motor_pinA_, abs(pwm));
    analogWrite(motor_pinB_, 0);
  }
  else if (pwm < 0)
  {
    digitalWrite(en_pinA_, HIGH);
    digitalWrite(en_pinB_, HIGH);
    analogWrite(motor_pinA_, 0);
    analogWrite(motor_pinB_, abs(pwm));
  }
  else
  {
    digitalWrite(en_pinA_, HIGH);
    digitalWrite(en_pinB_, HIGH);
    analogWrite(motor_pinA_, 0);
    analogWrite(motor_pinB_, 0);
  }
}