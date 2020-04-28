/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - April 27, 2020
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Connected"); //put this in a loop to confirm connection

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(1000);  // Set frequency to 1kHz

  delay(10);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  pwm.setPWM(15,0,2000);
  pwm.setPWM(16,0,4096);
  
}
