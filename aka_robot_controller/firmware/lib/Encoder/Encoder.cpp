#include "Encoder.h"
#include "base_config.h"
#include "Arduino.h"

Encoder::Encoder(volatile long left_enc_pos, volatile long right_enc_pos):
    left_enc_pos_(left_enc_pos),
    right_enc_pos_(right_enc_pos_)
{    
}

Encoder::ticks Encoder::getTicks(unsigned long currentMillis)
{
    Encoder::ticks enc;
    enc = getTicks((unsigned long) currentMillis);
    currentMillis = millis();
    if(currentMillis - previousMillis >= 1000)
    {
        previousMillis = currentMillis;

        enc.left_enc = encoder1pos - encoder1prev;
        enc.right_enc = encoder2pos - encoder2prev;

        encoder1prev = encoder1pos;
        encoder2prev = encoder2pos;
    }

    return enc;
}

