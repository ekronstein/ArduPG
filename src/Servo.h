// #ifndef __SERVO_H__
// #define __SERVO_H__
#pragma once

#include "common_types.h"
#include <Arduino.h>
#define DEFAULT_KP 1 
#define DEFAULT_KI 0.1
#define DEFAULT_KD 0.5

#define ROT_MAX 3  //TODO max rotations
#define COUNTS_PER_ROT 10000 // TODO

#define INIT_COUNT 0
#define INIT_TGT 0

#define FWD HIGH
#define BCK LOW

#define MAX_DUTY_CYCLE 255U


// PWM time constants:
#define PWM_INP_MIN 1000  // us
#define PWM_INP_MAX 2000  // us  (pulse length roughly)
#define PWM_OUT_FREQ 1000  // Hz todo ??

/*
 *  PID controller for a DC motor with a quadrature encoder.
 *  Does the control calculations and heldels inputs and outputs.
 *  The motor parameters are set to a specific model but can be 
 *  adjusted according to needs.
 *  // todo! missing attaching to interrupt from encoder
 *     from within the class.
 */
class Servo {
    public:
        //Servo();
        void init(uint8_t inp,
                  uint8_t fwd,
                  uint8_t back,
                  uint8_t encA, 
                  uint8_t encB);
        
        void checkUpdateTgt();  //todo use pulseIn()
        void updateCMD();
        uint8_t getEncBPin();
        void addToCount(int toAdd);

    private:
        void calcDutyCycle(int tgt);
        void setDirection(int dir);
        void readEncoder();
        void setMotor(uint8_t dutyCycle, int direction);
        int count_;
        int tgt_;
        float Kp_;
        float Ki_;
        float Kd_;
        float eint_;  // accumulated err integral
        float eprev_;  // stores the last err for err deriv
        long tprev_;  // previous PID err time in us

        long tprevtgt_;  // previous time tgt was updated

        bool currEncA_;
        bool currEncB_;
        
        bool prevEncA_;
        bool PrevEncB_;

        // pins:
        uint8_t inp_;
        uint8_t fwd_;
        uint8_t back_;
        uint8_t encA_;
        uint8_t encB_;

};


// #endif