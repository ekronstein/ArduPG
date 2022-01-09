#include <Arduino.h>
#include <analogWrite.h>
#include <SimplyAtomic.h>
#include "common_types.h"
#include "Servo.h"



// Motor pins:
#define MOTOR_OUT_RIGHT 0
#define MOTOR_OUT_LEFT 0

#define MOTOR_REVERSE_RIGHT 0
#define MOTOR_REVERSE_LEFT 0

#define MOTOR_INP_RIGHT 0
#define MOTOR_INP_LEFT 0

// Encoder pins:
#define ENCA_RIGHT 0
#define ENCB_RIGHT 0

#define ENCA_LEFT 0
#define ENCB_LEFT 0

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
  right.init(MOTOR_INP_RIGHT,MOTOR_OUT_RIGHT, MOTOR_REVERSE_RIGHT, ENCA_RIGHT, ENCB_RIGHT);
  left.init(MOTOR_INP_LEFT,MOTOR_OUT_LEFT, MOTOR_REVERSE_LEFT, ENCA_LEFT, ENCB_LEFT);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), readRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), readLeftEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  right.checkUpdateTgt();
  left.checkUpdateTgt();
}