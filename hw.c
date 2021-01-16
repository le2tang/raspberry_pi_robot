#include "hw.h"

#include <math.h>
#include <wiringPi.h>

#include "config.h"

void init_hw() {
  wiringPiSetup();

  pinMode(LMTR_FWD, OUTPUT);
  pinMode(LMTR_BCK, OUTPUT);
  pinMode(RMTR_FWD, OUTPUT);
  pinMode(RMTR_BCK, OUTPUT);

  pinMode(LMTR_PWM, PWM_OUTPUT);
  pinMode(RMTR_PWM, PWM_OUTPUT);
}

void motors_set(float left_pwr, float right_pwr) {
  unsigned int left_pwm = 1023 * abs(left_pwr);
  unsigned int right_pwm = 1023 * abs(right_pwr);
  
  if (left_pwr > pwr_tol) {
    digitalWrite(LMTR_FWD, HIGH);
    digitalWrite(LMTR_BCK, LOW);
  }
  else if (left_pwr < -pwrt_tol) {
    digitalWrite(LMTR_FWD, LOW);
    digitalWrite(LMTR_BCK, HIGH);
  }
  else {
    digitalWrite(LMTR_FWD, LOW);
    digitalWrite(LMTR_BCK, LOW);
  }

  if (right_pwr > pwr_tol) {
    digitalWrite(RMTR_FWD, HIGH);
    digitalWrite(RMTR_BCK, LOW);
  }
  else if (right_pwr < -pwrt_tol) {
    digitalWrite(RMTR_FWD, LOW);
    digitalWrite(RMTR_BCK, HIGH);
  }
  else {
    digitalWrite(RMTR_FWD, LOW);
    digitalWrite(RMTR_BCK, LOW);
  }

  pwmWrite(LMTR_PWM, left_pwm);
  pwmWrite(RMTR_PWM, right_pwm);
}