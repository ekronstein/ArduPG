// #ifndef __SERVO_H__
// #define __SERVO_H__
#pragma once

#include "common_types.h"
#include <Arduino.h>

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
                  uint8_t out,
                  uint8_t reverse, 
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
        uint8_t out_;
        uint8_t dir_;
        uint8_t encA_;
        uint8_t encB_;

};


// #endif