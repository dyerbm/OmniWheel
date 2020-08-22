/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - May 25, 2020
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MAX_PWM_FREQ 1600
#define FULL_ON_VAL 4096

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

const int interruptPin_1_MSB = 8; //Encoder 1
const int interruptPin_1_LSB = 9;
const int interruptPin_2_MSB = 20; //Encoder 2
const int interruptPin_2_LSB = 21;
const int interruptPin_3_MSB = 23; //Encoder 3
const int interruptPin_3_LSB = 22;
const int interruptPin_4_MSB = 10; //Encoder 4
const int interruptPin_4_LSB = 11;

const int motor_1_p = 0; //Motor 1
const int motor_1_n = 1;
const int motor_2_p = 2; //Motor 2
const int motor_2_n = 3;
const int motor_3_p = 4; //Motor 3
const int motor_3_n = 5;
const int motor_4_p = 6; //Motor 4
const int motor_4_n = 7;

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

  //Initialize interupt pins for encoders
  pinMode(interruptPin_1_MSB, INPUT);
  pinMode(interruptPin_1_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_1_MSB), ENCODER_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_1_LSB), ENCODER_1, CHANGE);
  pinMode(interruptPin_2_MSB, INPUT);
  pinMode(interruptPin_2_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_2_MSB), ENCODER_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_2_LSB), ENCODER_2, CHANGE);
  pinMode(interruptPin_3_MSB, INPUT);
  pinMode(interruptPin_3_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_3_MSB), ENCODER_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_3_LSB), ENCODER_3, CHANGE);
  pinMode(interruptPin_4_MSB, INPUT);
  pinMode(interruptPin_4_LSB, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin_4_MSB), ENCODER_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_4_LSB), ENCODER_4, CHANGE);

  //set all motors to not move
  pwm1.setPWM(0, 0, FULL_ON_VAL);
  pwm1.setPWM(1, 0, FULL_ON_VAL);
  pwm1.setPWM(2, 0, FULL_ON_VAL);
  pwm1.setPWM(3, 0, FULL_ON_VAL);
  pwm1.setPWM(4, 0, FULL_ON_VAL);
  pwm1.setPWM(5, 0, FULL_ON_VAL);
  pwm1.setPWM(6, 0, FULL_ON_VAL);
  pwm1.setPWM(7, 0, FULL_ON_VAL);
  pwm1.setPWM(8, 0, FULL_ON_VAL);
  pwm1.setPWM(9, 0, FULL_ON_VAL);
  pwm1.setPWM(10, 0, FULL_ON_VAL);
  pwm1.setPWM(11, 0, FULL_ON_VAL);
  pwm1.setPWM(12, 0, FULL_ON_VAL);
  pwm1.setPWM(13, 0, FULL_ON_VAL);
  pwm1.setPWM(14, 0, FULL_ON_VAL);
  //pwm1.setPWM(6, FULL_ON_VAL, 0);
  delay(1000);
  
}

void loop() {

  /*while (Serial.available()) {
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
  }*/
  if (millis()%1000==0){
    Serial.println(tics[3]);
    tics[3]=0;
    delay(1);
  }
}

// Encoder 1
void ENCODER_1() {
  noInterrupts();
  UPDATE_STATES_1(); //updates encoder values
  if (encoder_state[0] != encoder_state_prev[0]) { //update tics if change detected
    if ( encoder_state[0]==3&&encoder_state_prev[0]==1 || encoder_state[0]==2&&encoder_state_prev[0]==3 || encoder_state[0]==0&&encoder_state_prev[0]==2 || encoder_state[0]==1&&encoder_state_prev[0]==0 ) {
      tics[0] = tics[0] + 1;
    } else {
      tics[0] = tics[0] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_1() {
  encoder_state_prev[0] = encoder_state[0]; // Update previous int
  encoder_state[0] = digitalRead(interruptPin_1_MSB) * 2 + digitalRead(interruptPin_1_LSB);
}

// Encoder 2
void ENCODER_2() {
  noInterrupts();
  UPDATE_STATES_2(); //updates encoder values
  if (encoder_state[1] != encoder_state_prev[1]) { //update tics if change detected
    if ( encoder_state[1]==3&&encoder_state_prev[1]==1 || encoder_state[1]==2&&encoder_state_prev[1]==3 || encoder_state[1]==0&&encoder_state_prev[1]==2 || encoder_state[1]==1&&encoder_state_prev[1]==0 ) {
      tics[1] = tics[1] + 1;
    } else {
      tics[1] = tics[1] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_2() {
  encoder_state_prev[1] = encoder_state[1]; // Update previous int
  encoder_state[1] = digitalRead(interruptPin_2_MSB) * 2 + digitalRead(interruptPin_2_LSB);
}

// Encoder 3
void ENCODER_3() {
  noInterrupts();
  UPDATE_STATES_3(); //updates encoder values
  if (encoder_state[2] != encoder_state_prev[2]) { //update tics if change detected
    if ( encoder_state[2]==3&&encoder_state_prev[2]==1 || encoder_state[2]==2&&encoder_state_prev[2]==3 || encoder_state[2]==0&&encoder_state_prev[2]==2 || encoder_state[2]==1&&encoder_state_prev[2]==0 ) {
      tics[2] = tics[2] + 1;
    } else {
      tics[2] = tics[2] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_3() {
  encoder_state_prev[2] = encoder_state[2]; // Update previous int
  encoder_state[2] = digitalRead(interruptPin_3_MSB) * 2 + digitalRead(interruptPin_3_LSB);
}

// Encoder 4
void ENCODER_4() {
  noInterrupts();
  UPDATE_STATES_4(); //updates encoder values
  if (encoder_state[3] != encoder_state_prev[3]) { //update tics if change detected
    if ( encoder_state[3]==3&&encoder_state_prev[3]==1 || encoder_state[3]==2&&encoder_state_prev[3]==3 || encoder_state[3]==0&&encoder_state_prev[3]==2 || encoder_state[3]==1&&encoder_state_prev[3]==0 ) {
      tics[3] = tics[3] + 1;
    } else {
      tics[3] = tics[3] - 1;
    }
  }
  interrupts();
}

void UPDATE_STATES_4() {
  encoder_state_prev[3] = encoder_state[3]; // Update previous int
  encoder_state[3] = digitalRead(interruptPin_4_MSB) * 2 + digitalRead(interruptPin_4_LSB);
}
