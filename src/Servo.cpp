

#include "Servo.h"
#include <analogWrite.h>
#include <SimplyAtomic.h>

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
#define PWM_INP_MIN 900  // us
#define PWM_INP_MAX 2000  // us  (pulse length roughly)
#define PWM_OUT_FREQ 1000  // Hz
static const long PULSEIN_TIMEOUT = 5 *  PWM_INP_MAX; // us
static const long TGT_UPDATE_PERIOD = 20 * PWM_INP_MAX; // us


// Encoder counts constants:
static const int COUNT_MAX = ROT_MAX * COUNTS_PER_ROT;
static const int COUNT_MIN = - COUNT_MAX;

// Pulse conversion constants:
static const float c = (1.0 / (PWM_INP_MAX - PWM_INP_MIN));
static const float PULSE_CONVERT = c * (COUNT_MAX - COUNT_MIN);
static const float PULSE_BIAS = c * (PWM_INP_MAX * COUNT_MIN - 
                                    PWM_INP_MIN * COUNT_MAX);


uint8_t Servo::getEncBPin() {
    return encB_;
}

void Servo::addToCount(int toAdd) {
    count_ += toAdd;
}

void Servo::init(uint8_t inp, 
                 uint8_t out, 
                 uint8_t dir, 
                 uint8_t encA, 
                 uint8_t encB) {
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
    eprev_ = 0;
    long t = micros();
    tprev_ = t;
    tprevtgt_ = t;
    tgt_ = INIT_TGT;
    pinMode(inp_, INPUT);  // todo INPUT_PULLUP/DOWN?
    pinMode(encA_, INPUT);
    pinMode(encB_, INPUT);
    pinMode(out_, OUTPUT);
    pinMode(dir_, OUTPUT);
    // attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), readEncoder, RISING);  // todo check why doesn't compile
    analogWriteFrequency(out_, PWM_OUT_FREQ);
    
}

// Updates cmd as well
void Servo::checkUpdateTgt() {
    long t = micros();
    if (t < tprevtgt_ + TGT_UPDATE_PERIOD)
        return;  // not yet time to update again
    tprevtgt_ = t;
    int inp;
    ATOMIC() {  // check if really necessary - interrupts disturb? 
        inp = pulseIn(inp_, HIGH, PULSEIN_TIMEOUT);
    }
    if (inp == 0)
        inp = (PWM_INP_MAX + PWM_INP_MIN) / 2;
    tgt_ = PULSE_CONVERT * inp + PULSE_BIAS;
    if (tgt_ > COUNT_MAX)
        tgt_ = COUNT_MAX;
    if (tgt_ < COUNT_MIN)
        tgt_ = COUNT_MIN;
    ATOMIC() {
        updateCMD();
    }   
}

void Servo::setMotor(uint8_t dutyCycle, int direction) {
    digitalWrite(dir_, direction);  // todo, perhaps more than one pin?
    analogWrite(out_, dutyCycle, MAX_DUTY_CYCLE);
}

void Servo::updateCMD() {
    long t = micros();
    float dt = (float)(tprev_ - t);
    float e = (float)(tgt_ - count_);
    eint_ += e * dt;
    float d = ((float)(e - eprev_)) / dt;
    eprev_ = e;
    tprev_ = t;
    float u = Kp_*e + Ki_*eint_ + Kd_*d;
    int dir = u > 0 ? FWD : BCK;
    u = fabs(u);
    if (u > MAX_DUTY_CYCLE)
        u = MAX_DUTY_CYCLE;
    setMotor(u, dir);
}


    



