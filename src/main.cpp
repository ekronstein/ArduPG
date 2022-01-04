#include <Arduino.h>
#include <unordered_map>
#include <analogWrite.h>
#include <SimplyAtomic.h>
#include "common_types.h"
#include "Servo.h"

// PWM time constants:
#define PWM_INP_PERIOD 20000 // us  // TODO
#define PWM_INP_MIN 900  // us
#define PWM_INP_MAX 1500  // us
#define PWM_OUT_FREQ 1000  // Hz

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



// static const int in_pin_asg[] = {15, 16};  // input pin assignment
// static const int out_pin_asg[] = {17, 18};  // output pin assignment


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
  }
}

void readLeftEncoder() {
  ATOMIC() {
    left.addToCount(readEncoder(left.getEncBPin()));
  }
}

void setup() {
  // put your setup code here, to run once:
  // todo:
  // setup pid's, motor speeds params etc for the Servos
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), readRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), readLeftEncoder, RISING);
  right.init(MOTOR_INP_RIGHT,MOTOR_OUT_RIGHT, MOTOR_REVERSE_RIGHT, ENCA_RIGHT, ENCB_RIGHT, PWM_OUT_FREQ);
  left.init(MOTOR_INP_LEFT,MOTOR_OUT_LEFT, MOTOR_REVERSE_LEFT, ENCA_LEFT, ENCB_LEFT, PWM_OUT_FREQ);

}

void loop() {
  // put your main code here, to run repeatedly:
  time_t curr_millis = millis();
 
  right.updateCount();
  right.updateTgt();
  // right.updateCMD();

  left.updateCount();
  left.updateTgt();
  // left.updateCMD();
  
}