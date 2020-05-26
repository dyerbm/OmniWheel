/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - May 25, 2020
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CircularBuffer.h>

#define MAX_PWM_FREQ 1600
#define FULL_ON_VAL 4096

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

char* output_string;

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

//Variables for the controller
const float T=20; //Desired time step in milliseconds
double omega_desired[3] = {0,0,0};
double u_m[3] = {0,0,0};

const double K_p = -10;
const double K_d = -0;
const double K_i = -500;

int time_m=0; //Current time (for the motors)
int time_previous_m=0; //Last time step for the motors

const int eint_m_length=100; //set bufer size based on desired integral loop
CircularBuffer<double,eint_m_length> e_m_a; //Initialize buffer
double eint_m_a=0;
double ed_m_a=0;


CircularBuffer<double,100> e_m_b; 
CircularBuffer<double,100> e_m_c; 


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

  for (int i=0;i<=eint_m_length;i++){
    e_m_a.unshift(0);
  }
}

void loop() {
  time_m=millis();

  if (time_m-time_previous_m>=T) {
    CALC_VELOCITY(time_m-time_previous_m); //calculate each wheel velocity
    time_previous_m=time_m;
    //MOTOR_CONTROLLER(velocity[0], omega_desired[0]); //Call Motor Controller function, send desired state, real state, motor pins
    //Serial.println(velocity[0]);
    
    e_m_a.unshift(velocity[0]-omega_desired[0]);
    eint_m_a = eint_m_a+(e_m_a[0]-e_m_a[eint_m_length-1])*(T/1000)/eint_m_length;
    ed_m_a = (e_m_a[0]-e_m_a[1])/(T/1000);

    u_m[0] = K_p*e_m_a[0]+K_d*ed_m_a+K_i*eint_m_a; //calculate the controller in PWM
    if (abs(u_m[0])>12 || abs(u_m[1])>12 || abs(u_m[2])>12) {
      float quickcounter = u_m[0];
      for (int i=1; i<3;i++){
        if (abs(quickcounter)<abs(u_m[i])) (quickcounter=u_m[i]);
      }
      u_m[0]=u_m[0]/abs(quickcounter)*12;
    }

    if (u_m[0]>0) {
      pwm1.setPWM(motor_a_n, 0, (int) ((u_m[0])/12*4000));
      pwm1.setPWM(motor_a_p, 0, 0);
    }
    else {
      pwm1.setPWM(motor_a_n, 0, 0);
      pwm1.setPWM(motor_a_p, 0, (int) abs(u_m[0])/12*4000);
    }

    //sprintf(output_string, "Error: %f\tController: %d",e_m_a[0],u_m[0]);
    //Serial.println(output_string);
    Serial.print("Error: ");
    Serial.print(e_m_a[0]);
    Serial.print("\t Controller: ");
    Serial.print(u_m[0]);
    Serial.print("\n");
  }

  while (Serial.available()) {
        delay(10); 
      if (Serial.available() >0) {
        char c = Serial.read();
        echoString += c;}
        }

  if (echoString.length() >0) {
    /*if (echoString=="encoder"){
      Serial.println(tics[0]);
      tics[0]=0;
    }
    else{*/
      int motor = echoString.toFloat();
      Serial.println(echoString);
      omega_desired[0]=motor; //Currently casting float to double array, should maybe fix this
    //}
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

void CALC_VELOCITY(float timestep) {
  noInterrupts();
  velocity[0] = tics[0] / (timestep); //give time in tics per time persiod
  /*Serial.println("--------------");
  Serial.println(tics[0]);
  Serial.println(timestep);
  Serial.println(tics[0]/timestep);
  Serial.println(tics[0] / (timestep)/256*6);*/
  velocity[1] = tics[1] / (timestep);
  velocity[2] = tics[2] / (timestep);
  tics[0] = 0;
  tics[1] = 0;
  tics[2] = 0;
  interrupts();
}

/*void MOTOR_CONTROLLER(double velocity, double desired_velocity, double k_p, double k_d, double k_i) {
  double error = 0;
}*/
