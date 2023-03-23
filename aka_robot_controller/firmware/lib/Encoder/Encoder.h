#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

class Encoder
{
    public:
        struct ticks
        {
            float left_enc;
            float right_enc;
        };
        
        Encoder(volatile long left_enc_pos, volatile long right_enc_pos);
        ticks getTicks(unsigned long currentMillis);

    private:
        volatile long left_enc_pos_;
        volatile long right_enc_pos_;
        float encoder1diff;
        float encoder2diff;
        float encoder1prev;
        float encoder2prev;
        volatile long encoder1pos;
        volatile long encoder2pos;
        unsigned long currentMillis;
        unsigned long previousMillis;
};

#endif