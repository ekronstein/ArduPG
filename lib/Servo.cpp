#pragma once

#include "Servo.h"


void Servo::init(uint8_t inp, uint8_t out, uint8_t reverse, uint8_t encA, uint8_t encB) 
{
    encA_ = encA;
    encB_ = encB;
    inp_ = inp;
    out_ = out;
    rev_ = reverse;
    count_ = INIT_COUNT;
    tgt_ = INIT_TGT;
    p_ = DEFAULT_P;
    i_ = DEFAULT_I;
    d_ = DEFAULT_D;               
    pinMode(inp_, INPUT_PULLUP);
    pinMode(encA_, INPUT_PULLUP);
    pinMode(encB_, INPUT_PULLUP);
    pinMode(out_, OUTPUT);
    pinMode(rev_, OUTPUT);
    analogWrite(out_, dutyCycle(INIT_TGT));
    Serial.write("a");
}

    



