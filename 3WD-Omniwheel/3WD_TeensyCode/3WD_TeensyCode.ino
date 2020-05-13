/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - April 27, 2020
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MAX_PWM_FREQ 1600
#define FULL_ON_VAL 4096

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test");

  pwm1.begin();
  pwm1.setPWMFreq(MAX_PWM_FREQ);
  pwm1.setPWM(2, FULL_ON_VAL, 0);
  pwm1.setPWM(3, 0, FULL_ON_VAL);
  delay(1000);
}

void loop() {
  pwm1.setPWM(2, 0, FULL_ON_VAL);
  pwm1.setPWM(3, 0, FULL_ON_VAL);
}
