/**************************************************************

  Code ot run 3WD - Omniwheel Robot using custom Motor Drivers

  Benjamin Dyer - May 25, 2020

  Trevor Smith ^^ -  hehe ;)

  This man, sneaking in here, he wrote the encoder bit, mostly
 
 *****************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CircularBuffer.h>

#define MAX_PWM_FREQ 1600
#define FULL_ON_VAL 4096

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

char* output_string;

const int interruptPin_a_MSB = 3;
const int interruptPin_a_LSB = 2;
const int interruptPin_b_MSB = 7;
const int interruptPin_b_LSB = 6;
const int interruptPin_c_MSB = 5;
const int interruptPin_c_LSB = 4;

const int motor_a_p = 9;
const int motor_a_n = 8;
const int motor_b_p = 13;
const int motor_b_n = 14;
const int motor_c_p = 3;
const int motor_c_n = 2;

String echoString;

volatile double tics[3] = {0, 0, 0};
volatile double velocity[3] = {0, 0, 0};
char velocity_output[17];

int encoder_state[3];
int encoder_state_prev[3];

int incomingByte;

//Variables for the feedback lienarization controller
double v_r[3]={0, 0, 0}; //Linearized controller
double u_r[3]={0, 0, 0}; //Non-linear controller
double x_r[3]={0,0,0}; //Position of the robot
double x_r_desired[3]={0,0,0}; //desired position of robot
double xd_r_desired[3]={0,0,0}; //desired velocity of robot
const double wr=0.0508;//define wheel radius
const double rr=0.2632456;//define robot radius

const double lambda[3]={7,7,6}; //SMC lambda
const double K_r[3]={0.01,0.01,0.01}; //SMC K
double s[3] = {0,0,0}; //define sliding surface
double vshat[3] = {0,0,0}; //define measured slip

const int eint_r_length=100;
CircularBuffer<double,eint_r_length> e_r_x; //defines robot x error
double eint_r_x=0; //define integral error
double ed_r_x; //define derivative error
CircularBuffer<double,eint_r_length> e_r_y; //defines robot x error
double eint_r_y=0; //define integral error
double ed_r_y; //define derivative error
CircularBuffer<double,eint_r_length> e_r_t; //defines robot x error
double eint_r_t=0; //define integral error
double ed_r_t; //define derivative error

//Variables for the motor controller
const float T=20.0; //Desired time step in milliseconds
double omega_desired[3] = {0,0,0};
double u_m[3] = {0,0,0};

const double K_p = -10;
const double K_d = -0;
const double K_i = -30;

float time_m=0; //Current time (for the motors)
float time_previous_m=0; //Last time step for the motors

const int eint_m_length=100; //set bufer size based on desired integral loop
CircularBuffer<double,eint_m_length> e_m_a; //Initialize buffer
double eint_m_a=0;
double ed_m_a=0;
CircularBuffer<double,100> e_m_b;
double eint_m_b=0;
double ed_m_b=0;
CircularBuffer<double,100> e_m_c;
double eint_m_c=0;
double ed_m_c=0; 


void setup() {
  Serial1.begin(9600);
  Serial.begin(115200); //Begin serial for XBee module
  delay(3000);

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
    e_m_b.unshift(0);
    e_m_c.unshift(0);
    e_r_x.unshift(0);
    e_r_y.unshift(0);
    e_r_t.unshift(0);
  }

  delay(1000); //make sure motors stop turning so the robot starts at origin
}

int start_time = millis();
String cString; //TODO:Move this
int ind1,ind2,ind3,ind4,ind5,ind6; //TODO: Move this

void loop() {
  time_m=millis();

  if (Serial1.available()>0) {
//    digitalWrite(ledPin, HIGH); //indicate serial is being read
    char c = Serial1.read();
    
    if (c=='*'){
      Serial.println(cString);
      ind1 = cString.indexOf(',');  //finds location of first,
      x_r_desired[0] = cString.substring(0, ind1).toFloat();   //captures first data String
      ind2 = cString.indexOf(',', ind1+1 ); //finds location of second ,
      x_r_desired[1] = cString.substring(ind1 + 1,ind2).toFloat(); //captures second data String
      ind3 = cString.indexOf(',', ind2 +1  ); //finds location of second ,
      x_r_desired[2] = cString.substring(ind2+1,ind3).toFloat(); //finds location of second
      /*ind4 = cString.indexOf(',');  //finds location of first,
      xd_r_desired[0] = cString.substring(ind3+1, ind4).toFloat();   //captures first data String
      ind5 = cString.indexOf(',', ind1+1 ); //finds location of second ,
      xd_r_desired[1] = cString.substring(ind4 + 1,ind5).toFloat(); //captures second data String
      ind6 = cString.indexOf(',', ind2 +1  ); //finds location of second ,
      xd_r_desired[2] = cString.substring(ind5+1,ind6).toFloat(); //finds location of second*/
      
      
      cString="";
    }

    else {
      cString += c; //makes the string controlString
    }
  }
//  digitalWrite(ledPin, LOW);
  
  // Main control loop, should be changed to run on a watchdog interupt
  if (time_m-time_previous_m>=T) {

    /*if (millis()-start_time<20000){ #hard program path into robot
      x_r_desired[0] = -0.3*sin((millis()-start_time)/1000.*2.*3.14159/10);
      x_r_desired[1] = 0.3*cos((millis()-start_time)/1000.*2.*3.14159/10);
    }*/  
    
    //Serial.println(velocity[0]);
    //Serial.println(velocity[1]);
    //Serial.println(velocity[2]);
    CALC_VELOCITY(time_m-time_previous_m); //calculate each wheel velocity
    //MOTOR_CONTROLLER(velocity[0], omega_desired[0]); //Call Motor Controller function, send desired state, real state, motor pins
    //Serial.println(velocity[0]);

    //-----------Calculate Sliding Mode Control--------------//
    //Calculate current position


    /*Serial.print(x_r[1]);
    Serial.print("\n");
    Serial.print((-2.0/3.0*cos(x_r[2]))*velocity[0]);
    Serial.print("\n");
    Serial.print((sin(x_r[2])/sqrt(3)+cos(x_r[2])/3.0)*velocity[1]);
    Serial.print("\n");
    Serial.print((-sin(x_r[2])/sqrt(3)+cos(x_r[2])/3.0)*velocity[2]);
    Serial.print("\n");
    Serial.print((time_m-time_previous_m)/1000.0*wr);
    Serial.print("\n");*/

    
    x_r[0] = x_r[0] + ((2.0/3.0*sin(x_r[2]))*velocity[0]+(cos(x_r[2])/sqrt(3.0)-sin(x_r[2])/3.0)*velocity[1]+(-cos(x_r[2])/sqrt(3.0)-sin(x_r[2])/3.0)*velocity[2])*(time_m-time_previous_m)/1000.0*wr;
    x_r[1] = x_r[1] + ((-2.0/3.0*cos(x_r[2]))*velocity[0]+(sin(x_r[2])/sqrt(3.0)+cos(x_r[2])/3.0)*velocity[1]+(-sin(x_r[2])/sqrt(3.0)+cos(x_r[2])/3.0)*velocity[2])*(time_m-time_previous_m)/1000.0*wr;
    x_r[2] = x_r[2] + (-1./(3.*rr)*(velocity[0]+velocity[1]+velocity[2]))*(time_m-time_previous_m)/1000.*wr;
    //x_r[1] = x_r[1] + ((-2/3*cos(x_r[2]))*velocity[0]+(sin(x_r[2])/sqrt(3)+cos(x_r[2])/3)*velocity[1]+(cos(x_r[2])/3)*velocity[2])*(time_m-time_previous_m)/1000.0*wr;

    e_r_x.unshift(x_r[0]-x_r_desired[0]);//calculate new error
    e_r_y.unshift(x_r[1]-x_r_desired[1]);
    e_r_t.unshift(x_r[2]-x_r_desired[2]);
    eint_r_x = eint_r_x+(e_r_x[0]-e_r_x[eint_r_length-1])*(T/1000.0)/(double)eint_r_length;//recalculate integral error
    eint_r_y = eint_r_y+(e_r_y[0]-e_r_y[eint_r_length-1])*(T/1000.0)/(double)eint_r_length;
    eint_r_t = eint_r_t+(e_r_t[0]-e_r_t[eint_r_length-1])*(T/1000.0)/(double)eint_r_length;
    ed_r_x = (e_r_x[0]-e_r_x[1])/(T/1000.0); //recalculate derivative error
    ed_r_y = (e_r_y[0]-e_r_y[1])/(T/1000.0);
    ed_r_t = (e_r_t[0]-e_r_t[1])/(T/1000.0);

    //PUT UR CONTROLLER HERE!

    s[0]=e_r_x[0]+lambda[0]*eint_r_x; //calculate sliding surface
    s[1]=e_r_y[0]+lambda[1]*eint_r_y;
    s[2]=e_r_t[0]+lambda[2]*eint_r_t;

    omega_desired[0]=1/wr*(sin(x_r[2])*(-lambda[0]*e_r_x[0]+xd_r_desired[0]-K_r[0]*sgn(s[0]))-cos(x_r[2])*(-lambda[1]*e_r_y[0]+xd_r_desired[1]-K_r[1]*sgn(s[1]))-rr*(-lambda[2]*e_r_t[0]+xd_r_desired[2]-K_r[2]*sgn(s[2])))-vshat[0];
    omega_desired[1]=1/wr*((sqrt(3.)/2.*cos(x_r[2])-sin(x_r[2])/2.)*(-lambda[0]*e_r_x[0]+xd_r_desired[0]-K_r[0]*sgn(s[0]))+(sqrt(3.)/2.*sin(x_r[2])+cos(x_r[2])/2.)*(-lambda[1]*e_r_y[0]+xd_r_desired[1]-K_r[1]*sgn(s[1]))-rr*(-lambda[2]*e_r_t[0]+xd_r_desired[2]-K_r[2]*sgn(s[2])))-vshat[1];
    omega_desired[2]=1/wr*((-sqrt(3.)/2.*cos(x_r[2])-sin(x_r[2])/2.)*(-lambda[0]*e_r_x[0]+xd_r_desired[0]-K_r[0]*sgn(s[0]))+(-sqrt(3.)/2.*sin(x_r[2])+cos(x_r[2])/2.)*(-lambda[1]*e_r_y[0]+xd_r_desired[1]-K_r[1]*sgn(s[1]))-rr*(-lambda[2]*e_r_t[0]+xd_r_desired[2]-K_r[2]*sgn(s[2])))-vshat[2];

    
    if (abs(omega_desired[0])>4 || abs(omega_desired[1])>4 || abs(omega_desired[2])>4) {
      float quickcounter = omega_desired[0];
      for (int i=1; i<3;i++){
        if (abs(quickcounter)<abs(omega_desired[i])) (quickcounter=omega_desired[i]);
      }
      omega_desired[0]=omega_desired[0]/abs(quickcounter)*4;
      omega_desired[1]=omega_desired[1]/abs(quickcounter)*4;
      omega_desired[2]=omega_desired[2]/abs(quickcounter)*4;
    }
    
    //Calculate Motor Controllers
    e_m_a.unshift(velocity[0]-omega_desired[0]);
    eint_m_a = eint_m_a+(e_m_a[0]-e_m_a[eint_m_length-1])*(T/1000)/eint_m_length;
    ed_m_a = (e_m_a[0]-e_m_a[1])/(T/1000);
    u_m[0] = K_p*e_m_a[0]+K_d*ed_m_a+K_i*eint_m_a; //calculate the controller in PWM

    e_m_b.unshift(velocity[1]-omega_desired[1]);
    eint_m_b = eint_m_b+(e_m_b[0]-e_m_b[eint_m_length-1])*(T/1000)/eint_m_length;
    ed_m_b = (e_m_b[0]-e_m_b[1])/(T/1000);
    u_m[1] = K_p*e_m_b[0]+K_d*ed_m_b+K_i*eint_m_b; //calculate the controller in PWM

    e_m_c.unshift(velocity[2]-omega_desired[2]);
    eint_m_c = eint_m_c+(e_m_c[0]-e_m_c[eint_m_length-1])*(T/1000)/eint_m_length;
    ed_m_c = (e_m_c[0]-e_m_c[1])/(T/1000);
    u_m[2] = K_p*e_m_c[0]+K_d*ed_m_a+K_i*eint_m_a; //calculate the controller in PWM
    
    if (abs(u_m[0])>12 || abs(u_m[1])>12 || abs(u_m[2])>12) {
      float quickcounter = u_m[0];
      for (int i=1; i<3;i++){
        if (abs(quickcounter)<abs(u_m[i])) (quickcounter=u_m[i]);
      }
      u_m[0]=u_m[0]/abs(quickcounter)*12;
      u_m[1]=u_m[1]/abs(quickcounter)*12;
      u_m[2]=u_m[2]/abs(quickcounter)*12;
      //Serial.println(millis());
    }

    /*if (abs(u_m[0])<3.5) u_m[0]=0; //Cut off voltage, save battery, stop ringing
    if (abs(u_m[1])<3.5) u_m[1]=0;
    if (abs(u_m[2])<3.5) u_m[2]=0;*/

    //u_m[0]=8;u_m[1]=-4;u_m[2]=-4; //This forces a spinning controller

    //-----Set each motor accordingly-----//
    if (u_m[0]>0) {
      pwm1.setPWM(motor_a_n, 0, (int) (abs(u_m[0])/12*4096));
      pwm1.setPWM(motor_a_p, 0, 0);
    }
    else {
      pwm1.setPWM(motor_a_n, 0, 0);
      pwm1.setPWM(motor_a_p, 0, (int) (abs(u_m[0])/12*4096));
    }
    if (u_m[1]>0) {
      pwm1.setPWM(motor_b_n, 0, (int) (abs(u_m[1])/12*4096));
      pwm1.setPWM(motor_b_p, 0, 0);
    }
    else {
      pwm1.setPWM(motor_b_n, 0, 0);
      pwm1.setPWM(motor_b_p, 0, (int) (abs(u_m[1])/12*4096));
    }
    if (u_m[2]>0) {
      pwm1.setPWM(motor_c_n, 0, (int) (abs(u_m[2])/12*4096));
      pwm1.setPWM(motor_c_p, 0, 0);
    }
    else {
      pwm1.setPWM(motor_c_n, 0, 0);
      pwm1.setPWM(motor_c_p, 0, (int) (abs(u_m[2])/12*4096));
    }


    time_previous_m=time_m; //set previous time to current time

    
    /*//sprintf(output_string, "Error: %f\tController: %d",e_m_a[0],u_m[0]);
    //Serial.println(output_string);
    Serial.print("Error: ");
    //Serial.print(e_m_a[0]);
    Serial.print(e_r_y[0]);
    Serial.print("\t Controller: ");
    Serial.print(u_m[0]);
    Serial.print("\t Velocity: ");
    Serial.print(velocity[0]);
    Serial.print("\n");*/

    Serial.print("x Position: ");
    Serial.print(x_r[0]);
    Serial.print("\t y Position: ");
    Serial.print(x_r[1]);
    Serial.print("\t theta Position: ");
    Serial.print(x_r[2]);
    Serial.print("\n");

    /*Serial.print("xd Position: ");
    Serial.print(x_r_desired[0]);
    Serial.print("\t yd Position: ");
    Serial.print(x_r_desired[1]);
    Serial.print("\t thetad Position: ");
    Serial.print(x_r_desired[2]);
    Serial.print("\n");*/

    /*Serial.print(x_r[0],5);
    Serial.print(",");
    Serial.print(x_r[1],5);
    Serial.print(",");
    Serial.print(x_r[2],5);
    Serial.print(",");
    Serial.println(sqrt(pow(x_r[0],2)+pow(x_r[1],2))-0.3,5);*/
  }

  /*while (Serial.available()) {
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
    else{
      double motor = echoString.toFloat();
      Serial.println(echoString);
      //omega_desired[0]=motor; //Currently casting float to double array, should maybe fix this
      //omega_desired[1]=motor;
      //omega_desired[2]=motor;
      x_r_desired[2]=motor;
    //}
    echoString="";
  }*/
}

static inline double sgn(double val) {
 if (val < 0) return -1.;
 if (val==0) return 0.;
 return 1.;
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
  velocity[0] = tics[0] / ((timestep)/1000) / 10000*2*3.14156; //give time in tics per time period
  velocity[1] = tics[1] / ((timestep)/1000) / 10000*2*3.14156;
  velocity[2] = tics[2] / ((timestep)/1000) / 10000*2*3.14156;
  tics[0] = 0;
  tics[1] = 0;
  tics[2] = 0;
  interrupts();
}
