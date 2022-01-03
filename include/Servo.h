// #ifndef __SERVO_H__
// #define __SERVO_H__
#pragma once

#include "common_types.h"
#include <Arduino.h>

/*
 *  Does the servo calculations. Position and target are 
 *  between 0 and 1.  Speed is in 1/s. i.e. speed of 1 
 *  corresponds to a full travel (min to max)
 */
class Servo {
    public:
        //Servo();
        void init(uint8_t inp,uint8_t out,uint8_t reverse, 
              uint8_t encA, uint8_t encB, float outfreq);
        
        void updateCount();
        void updateTgt();  //todo use pulseIn()
        void updateCMD(int tgt);

    private:
        void calcDutyCycle(int tgt);
        void setDirection(int dir);
        int count_;
        int tgt_;
        float Kp_;
        float Ki_;
        float Kd_;
        float eint_; // accumulated err integral
        float eprev_; // stores the last err for err deriv
        long tprev_; // previous time in us

        bool currEncA_;
        bool currEncB_;
        
        bool prevEncA_;
        bool PrevEncB_;

        // pins:
        unsigned inp_;
        unsigned out_;
        unsigned rev_;
        unsigned encA_;
        unsigned encB_;

};


// #endif