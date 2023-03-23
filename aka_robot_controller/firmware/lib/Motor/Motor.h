#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Controller
{
  public:
    Controller(int en_pinA, int en_pinB, int motor_pinA, int motor_pinB);
    void spin(int pwm);

  private:
    int motor_pinA_;
    int motor_pinB_;
    int en_pinA_;
    int en_pinB_;
};

#endif