#include <Arduino.h>
#include <analogWrite.h>
#include <SimplyAtomic.h>
#include "common_types.h"
#include "Servo.h"



// Motor pins:
#define MOTOR_FWD_RIGHT 0
#define MOTOR_FWD_LEFT 0

#define MOTOR_BACK_RIGHT 0
#define MOTOR_BACK_LEFT 0

#define MOTOR_INP_RIGHT 25 
#define MOTOR_INP_LEFT 26

// Encoder pins:
#define ENCA_RIGHT 17 
#define ENCB_RIGHT 5  

#define ENCA_LEFT 18  
#define ENCB_LEFT 19

static Servo right;
static Servo left;

int readEncoder(uint8_t pinB) {
  int b = digitalRead(pinB);
  if (b > 0)
    return 1;
  return -1;
}

void readRightEncoder() {
  ATOMIC() {
    right.addToCount(readEncoder(right.getEncBPin()));
    right.updateCMD();
  }
}

void readLeftEncoder() {
  ATOMIC() {
    left.addToCount(readEncoder(left.getEncBPin()));
    left.updateCMD();
  }
}

void setup() {
  // put your setup code here, to run once:
  right.init(MOTOR_INP_RIGHT,MOTOR_FWD_RIGHT, MOTOR_BACK_RIGHT, ENCA_RIGHT, ENCB_RIGHT);
  left.init(MOTOR_INP_LEFT,MOTOR_FWD_LEFT, MOTOR_BACK_LEFT, ENCA_LEFT, ENCB_LEFT);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), readRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), readLeftEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  right.checkUpdateTgt();
  left.checkUpdateTgt();
}