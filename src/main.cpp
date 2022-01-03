#include <Arduino.h>
#include <unordered_map>
#include <analogWrite.h>
#include "common_types.h"
#include "Servo.h"

#define MAX_CHANNELS 16
#define NUM_CHANNELS 2  // each channel is a pwm input. 0, 1 are saved for right and left brakes

// Input PWM time constants:
#define PWM_INP_PERIOD 20000 // us  // TODO
#define PWM_INP_MIN 900  // us
#define PWM_INP_MAX 1500  // us
#define PWM_INP_INTERREAD 10 * PWM_INP_PERIOD // us


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

// static Servo right;
// static Servo left;
enum Channel {RightCH=0, LeftCH};
static Channel all_channels[] = {RightCH, LeftCH};
static pwm_t pwm[MAX_CHANNELS] = {};
static long last_pwm;

static Servo right;
static Servo left;


void setup() {
  // put your setup code here, to run once:
  // todo:
  // setup pid's, motor speeds params etc for the Servos
  right.init(MOTOR_INP_RIGHT,MOTOR_OUT_RIGHT, MOTOR_REVERSE_RIGHT, ENCA_RIGHT, ENCB_RIGHT);
  left.init(MOTOR_INP_LEFT,MOTOR_OUT_LEFT, MOTOR_REVERSE_LEFT, ENCA_LEFT, ENCB_LEFT);

}

void loop() {
  // put your main code here, to run repeatedly:
  time_t curr_millis = millis();
  if (last_pwm > curr_millis + PWM_INP_INTERREAD) {
    last_pwm =curr_millis;
    
      // validate_pwms(pwm, ch)
  }

  right.updateCount();
  right.updateTgt();
  right.updateCMD();

  left.updateCount();
  left.updateTgt();
  left.updateCMD();
  
}