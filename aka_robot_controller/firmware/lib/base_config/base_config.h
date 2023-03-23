#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

#define MAX_RPM 25               // motor's maximum RPM
#define COUNTS_PER_REV 2040       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.075       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define WHEELS_DISTANCE 0.255       // distance between left and right wheels

#define MOTOR_LEFT_ENC_A 2        // Encoder Pins
#define MOTOR_LEFT_ENC_B 4
#define MOTOR_RIGHT_ENC_A 3
#define MOTOR_RIGHT_ENC_B 5

#define MOTOR_LEFT_EN_F 12         // Motor Pins
#define MOTOR_LEFT_EN_R 13
#define MOTOR_LEFT_IN_F 10
#define MOTOR_LEFT_IN_R 11
#define MOTOR_RIGHT_EN_F 8         // Motor Pins
#define MOTOR_RIGHT_EN_R 9
#define MOTOR_RIGHT_IN_F 6
#define MOTOR_RIGHT_IN_R 7

#endif