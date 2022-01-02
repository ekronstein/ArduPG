// #ifndef __SERVO_H__
// #define __SERVO_H__
#pragma once

#include "common_types.h"
#include <Arduino.h>
#define DEFAULT_P 1
#define DEFAULT_I 0.1
#define DEFAULT_D 0.5

#define INIT_COUNT 0
#define INIT_TGT 0

/*
 *  Does the servo calculations. Position and target are 
 *  between 0 and 1.  Speed is in 1/s. i.e. speed of 1 
 *  corresponds to a full travel (min to max)
 */
class Servo {
    public:
        //Servo();
        void init(uint8_t inp,uint8_t out,uint8_t reverse, 
              uint8_t encA, uint8_t encB);
        
        void updateCount();
        void updateTgt();
        void updateCMD();

    private:
        uint8_t dutyCycle(int tgt);
        int count_;
        int tgt_;
        double p_;
        double i_;
        double d_;

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