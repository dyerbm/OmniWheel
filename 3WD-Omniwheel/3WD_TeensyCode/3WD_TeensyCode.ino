/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - April 27, 2020
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MAX_PWM_FREQ 1600
#define FULL_ON_VAL 4096

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

const int interruptPin_a_MSB = 2;
const int interruptPin_a_LSB = 3;

const int motor_a_p = 9;
const int motor_a_n = 8;
const int motor_b_p = 3;
const int motor_b_n = 2;
const int motor_c_p = 13;
const int motor_c_n = 14;

String echoString;

volatile int tics[4] = {0, 0, 0, 0};
volatile double velocity[4] = {0, 0, 0, 0};
char velocity_output[17];

int encoder_state[4];
int encoder_state_prev[4];

int incomingByte;

void setup() {
  Serial.begin(9600);

  pwm1.begin();
  pwm1.setPWMFreq(MAX_PWM_FREQ);

  pinMode(interruptPin_a_MSB, INPUT);
  pinMode(interruptPin_a_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_a_MSB), ENCODER_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_a_LSB), ENCODER_A, CHANGE);
  
  pwm1.setPWM(motor_a_p, 0, FULL_ON_VAL);
  pwm1.setPWM(motor_a_n, 0, FULL_ON_VAL);
  pwm1.setPWM(motor_b_p, 0, FULL_ON_VAL);
  pwm1.setPWM(motor_b_n, 0, FULL_ON_VAL);
  pwm1.setPWM(motor_c_p, 0, FULL_ON_VAL);
  pwm1.setPWM(motor_c_n, 0, FULL_ON_VAL);
  delay(1000);
  
}

void loop() {

  while (Serial.available()) {
        delay(10); 
      if (Serial.available() >0) {
        char c = Serial.read();
        echoString += c;}
        }

  if (echoString.length() >0) {
    if (echoString=="encoder"){
      Serial.println(tics[0]);
      tics[0]=0;
    }
    else{
      int motor = echoString.toInt();
      Serial.println(echoString);
      pwm1.setPWM(motor_a_p, motor, 0);
      pwm1.setPWM(motor_b_p, motor, 0);
      pwm1.setPWM(motor_c_p, motor, 0);
    }
    echoString="";
  }
}

// Encoder A
void ENCODER_A() {
  noInterrupts();
  UPDATE_STATES_A(); //updates encoder values
  if (encoder_state[0] != encoder_state_prev[0]) { //update tics if change detected
    if ( encoder_state[0]==3&&encoder_state_prev[0]==1 || encoder_state[0]==2&&encoder_state_prev[0]==3 || encoder_state[0]==0&&encoder_state_prev[0]==2 || encoder_state[0]==1&&encoder_state_prev[0]==0 ) {
      tics[0] = tics[0] + 1;
    } else {
      tics[0] = tics[0] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_A() {
  encoder_state_prev[0] = encoder_state[0]; // Update previous int
  encoder_state[0] = digitalRead(interruptPin_a_MSB) * 2 + digitalRead(interruptPin_a_LSB);
}
