#pragma once

#include "Servo.h"
#include <analogWrite.h>
#include <SimplyAtomic.h>

#define DEFAULT_KP 1
#define DEFAULT_KI 0.1
#define DEFAULT_KD 0.5

#define INIT_COUNT 0
#define MAX_COUNT 1000 // TODO measure actual values
#define MIN_COUNT -1000

#define INIT_TGT 0

#define FWD HIGH
#define BCK LOW

#define MAX_DUTY_CYCLE 255U

#define PULSEIN_TIMEOUT 1e5  // us

// PWM time constants:
#define PWM_INP_MIN 900  // us
#define PWM_INP_MAX 2000  // us
#define PWM_OUT_FREQ 1000  // Hz
//#define todo conversion from pulse width to counts - TGT_INP_CONV
#define TGT_INP_CONV 0 // todo



uint8_t Servo::getEncBPin() {
    return encB_;
}

void Servo::addToCount(int toAdd) {
    count_ += toAdd;
}

void Servo::init(uint8_t inp, uint8_t out, uint8_t dir, uint8_t encA, uint8_t encB, float outfreq) 
{
    encA_ = encA;
    encB_ = encB;
    inp_ = inp;
    out_ = out;
    dir_ = dir;
    count_ = INIT_COUNT;
    tgt_ = INIT_TGT;
    Kp_ = DEFAULT_KP;
    Ki_ = DEFAULT_KI;
    Kd_ = DEFAULT_KD;
    eint_ = 0;               
    pinMode(inp_, INPUT);  // todo INPUT_PULLUP/DOWN?
    pinMode(encA_, INPUT);
    pinMode(encB_, INPUT);
    pinMode(out_, OUTPUT);
    pinMode(dir_, OUTPUT);
    // attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), readEncoder, RISING);
    analogWriteFrequency(out_, outfreq);
    updateCMD(INIT_TGT);
    // analogWrite(out_, calcDutyCycle(INIT_TGT), MAX_DUTY_CYCLE);
}

void Servo::updateTgt() {
    int inp;
    ATOMIC() {
        inp = pulseIn(inp_, HIGH, PULSEIN_TIMEOUT);
    }
    tgt_ = TGT_INP_CONV * inp;
}

void Servo::updateCMD(int tgt) 
{
    if (tgt > MAX_COUNT)
        tgt = MAX_COUNT;
    if (tgt < MIN_COUNT)
        tgt = MIN_COUNT;
    long t = micros();
    float dt = (float)(tprev_ - t);
    float e = (float)(tgt - count_);
    eint_ += e * dt;
    float d = ((float)(e - eprev_)) / dt;
    eprev_ = e;
    tprev_ = t;
    float u = Kp_*e + Ki_*eint_ + Kd_*d;
    int dir = u > 0 ? FWD : BCK;
    u = fabs(u);
    if (u > MAX_DUTY_CYCLE)
        u = MAX_DUTY_CYCLE;
    digitalWrite(dir_, dir);
    analogWrite(out_, u, MAX_DUTY_CYCLE);
}


    



