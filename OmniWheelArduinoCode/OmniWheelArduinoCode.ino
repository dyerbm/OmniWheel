#include <math.h>

int M2pwm = 5;
int M2dir = 4;
int M1pwm = 6;
int M1dir = 7;
int M3pwm = 9;
int M3dir = 8;
int M4pwm = 10;
int M4dir = 11;

String controlString,v1, v2, v3, v4;

int ind1,ind2,ind3,ind4;

int v1int = 0;
int v2int = 0;
int v3int = 0;
int v4int = 0;

int endflag = 0;



void setup()
{
    pinMode(M1dir, OUTPUT);
    pinMode(M1pwm, OUTPUT);
    pinMode(M2dir, OUTPUT);
    pinMode(M2pwm, OUTPUT);
    pinMode(M3dir, OUTPUT);
    pinMode(M3pwm, OUTPUT);
    pinMode(M4dir, OUTPUT);
    pinMode(M4pwm, OUTPUT);

    analogWrite(M1pwm, 0);   //PWM Speed Control
    analogWrite(M2pwm, 0);   //PWM Speed Control
    analogWrite(M3pwm, 0);   //PWM Speed Control
    analogWrite(M4pwm, 0);   //PWM Speed Control


    Serial.begin(9600);
}

void loop()
{ //string should be in form 255,255,255,255*
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='*'){
      ind1 = controlString.indexOf(',');  //finds location of first ,                                                      //saperating the data in formof motor on off signal and the degree input
      v1 = controlString.substring(0, ind1);   //captures first data String
      ind2 = controlString.indexOf(',', ind1+1 ); //finds location of second ,
      v2 = controlString.substring(ind1 + 1,ind2); //captures second data String
      ind3 = controlString.indexOf(',', ind2 +1  ); //finds location of second ,
      v3 = controlString.substring(ind2+1,ind3); //finds location of second ,
      ind4 = controlString.indexOf(',', ind3 + 1 ); //finds location of second ,
      v4 = controlString.substring(ind3 + 1 ); //finds location of second ,

      v1int = v1.toInt();
      v2int = v2.toInt();
      v3int = v3.toInt();
      v4int = v4.toInt();

      v1 = "";
      v2 = "";
      v3 = "";
      v4 = "";
      endflag = 0;
      
      //Serial.println(String(v1int)+ "," + String(v2int)+ "," +String(v3int)+ "," +String(v4int));
      
      controlString="";
    }

    else {
      controlString += c; //makes the string controlString
    }
    //Serial.println("i" + controlString);
  }


  //This is going to be for directly controlling the bot
  /*int circleTime=8;
  int speedd=250;
  int v1int=(sin(2*M_PI/circleTime*millis()/1000)*speedd);
  int v3int=(-sin(2*M_PI/circleTime*millis()/1000)*speedd);
  int v2int=(cos(2*M_PI/circleTime*millis()/1000)*speedd);
  //int v4int=(-cos(2*M_PI/circleTime*millis()/1000)*speedd);
  int v4int=(-cos(2*M_PI/circleTime*millis()/1000)*speedd);;

  endflag=0;*/


  //if ((abs(v1int) > 0 || abs(v2int) > 0 || abs(v3int) > 0 || abs(v4int) > 0) && endflag==0)
  if (endflag==0)
  {

    //Set direction pins
    if (v1int>=0) digitalWrite(M1dir, HIGH);
    if (v1int<0) digitalWrite(M1dir, LOW);
    if (v2int>=0) digitalWrite(M2dir, HIGH);
    if (v2int<0) digitalWrite(M2dir, LOW);
    if (v3int>=0) digitalWrite(M3dir, HIGH);
    if (v3int<0) digitalWrite(M3dir, LOW);
    if (v4int>=0) digitalWrite(M4dir, HIGH);
    if (v4int<0) digitalWrite(M4dir, LOW);

    analogWrite(M1pwm,abs(v1int));
    analogWrite(M2pwm,abs(v2int));
    analogWrite(M3pwm,abs(v3int));
    analogWrite(M4pwm,abs(v4int));

    endflag=1;

    Serial.println(v4int);

  }

/*  if ((v1int == 0 && v2int == 0 && v3int == 0 && v4int == 0) && endflag==0)
  {
    stopbot();
  }*/
  
}

void stopbot(){
  analogWrite(M1pwm, 0);
  analogWrite(M2pwm, 0);
  analogWrite(M3pwm, 0);
  analogWrite(M4pwm, 0);
  endflag=1;
}
