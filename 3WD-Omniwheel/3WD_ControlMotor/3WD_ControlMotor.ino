/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - May 25, 2020
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MAX_PWM_FREQ 1600
#define FULL_ON_VAL 4096

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

const int interruptPin_a_MSB = 2;
const int interruptPin_a_LSB = 3;
const int interruptPin_b_MSB = 4;
const int interruptPin_b_LSB = 5;
const int interruptPin_c_MSB = 6;
const int interruptPin_c_LSB = 7;

const int motor_a_p = 9;
const int motor_a_n = 8;
const int motor_b_p = 3;
const int motor_b_n = 2;
const int motor_c_p = 13;
const int motor_c_n = 14;

String echoString;

volatile int tics[3] = {0, 0, 0};
volatile double velocity[3] = {0, 0, 0};
char velocity_output[17];

int encoder_state[3];
int encoder_state_prev[3];

int incomingByte;

void setup() {
  Serial.begin(9600);

  pwm1.begin();
  pwm1.setPWMFreq(MAX_PWM_FREQ);

  //Initialize interupt pins for encoders
  pinMode(interruptPin_a_MSB, INPUT);
  pinMode(interruptPin_a_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_a_MSB), ENCODER_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_a_LSB), ENCODER_A, CHANGE);
  pinMode(interruptPin_b_MSB, INPUT);
  pinMode(interruptPin_b_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_b_MSB), ENCODER_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_b_LSB), ENCODER_B, CHANGE);
  pinMode(interruptPin_c_MSB, INPUT);
  pinMode(interruptPin_c_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_c_MSB), ENCODER_C, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_c_LSB), ENCODER_C, CHANGE);

  //set all motors to not move
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

// Encoder B
void ENCODER_B() {
  noInterrupts();
  UPDATE_STATES_B(); //updates encoder values
  if (encoder_state[1] != encoder_state_prev[1]) { //update tics if change detected
    if ( encoder_state[1]==3&&encoder_state_prev[1]==1 || encoder_state[1]==2&&encoder_state_prev[1]==3 || encoder_state[1]==0&&encoder_state_prev[1]==2 || encoder_state[1]==1&&encoder_state_prev[1]==0 ) {
      tics[1] = tics[1] + 1;
    } else {
      tics[1] = tics[1] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_B() {
  encoder_state_prev[1] = encoder_state[1]; // Update previous int
  encoder_state[1] = digitalRead(interruptPin_b_MSB) * 2 + digitalRead(interruptPin_b_LSB);
}

// Encoder C
void ENCODER_C() {
  noInterrupts();
  UPDATE_STATES_C(); //updates encoder values
  if (encoder_state[2] != encoder_state_prev[2]) { //update tics if change detected
    if ( encoder_state[2]==3&&encoder_state_prev[2]==1 || encoder_state[2]==2&&encoder_state_prev[2]==3 || encoder_state[2]==0&&encoder_state_prev[2]==2 || encoder_state[2]==1&&encoder_state_prev[2]==0 ) {
      tics[2] = tics[2] + 1;
    } else {
      tics[2] = tics[2] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_C() {
  encoder_state_prev[2] = encoder_state[2]; // Update previous int
  encoder_state[2] = digitalRead(interruptPin_c_MSB) * 2 + digitalRead(interruptPin_c_LSB);
}
