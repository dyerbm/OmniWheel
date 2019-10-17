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
      
      Serial.println(String(v1int)+ "," + String(v2int)+ "," +String(v3int)+ "," +String(v4int));

      controlString="";
    }

    else {
      controlString += c; //makes the string controlString
    }
    //Serial.println(controlString);
  }


  if ((abs(v1int) > 0 || abs(v2int) > 0 || abs(v3int) > 0 || abs(v4int) > 0) && endflag==0)
  {

    //Set direction pins
    if (v1int>0) digitalWrite(M1dir, HIGH);
    if (v1int<0) digitalWrite(M1dir, LOW);
    if (v2int>0) digitalWrite(M2dir, HIGH);
    if (v2int<0) digitalWrite(M2dir, LOW);
    if (v3int>0) digitalWrite(M3dir, HIGH);
    if (v3int<0) digitalWrite(M3dir, LOW);
    if (v4int>0) digitalWrite(M4dir, HIGH);
    if (v4int<0) digitalWrite(M4dir, LOW);

    analogWrite(M1pwm,abs(v1int));
    analogWrite(M2pwm,abs(v2int));
    analogWrite(M3pwm,abs(v3int));
    analogWrite(M4pwm,abs(v4int));

  }

  if ((v1int == 0 && v2int == 0 && v3int == 0 && v4int == 0) && endflag==0)
  {
    stopbot();
  }
  
}

void stopbot(){
  analogWrite(M1pwm, 0);
  analogWrite(M2pwm, 0);
  analogWrite(M3pwm, 0);
  analogWrite(M4pwm, 0);
  endflag=1;
}



/*int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;
int E3 = 9;
int M3 = 8;
int E4 = 10;
int M4 = 11;

void setup()
{
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
    pinMode(E3, OUTPUT);
    pinMode(E4, OUTPUT);

    analogWrite(E1, 0);   //PWM Speed Control
    analogWrite(E2, 0);   //PWM Speed Control
    analogWrite(E3, 0);   //PWM Speed Control
    analogWrite(E4, 0);   //PWM Speed Control
}

void loop()
{
  int value;
  for(value = 0 ; value <= 255; value+=1)
  {
    digitalWrite(M1,HIGH);
    digitalWrite(M2, HIGH);
    digitalWrite(M3,HIGH);
    digitalWrite(M4, HIGH);
    analogWrite(E1, 0);   //PWM Speed Control
    analogWrite(E2, 0);   //PWM Speed Control
    analogWrite(E3, 0);   //PWM Speed Control
    analogWrite(E4, 100);   //PWM Speed Control
    delay(30);
  }
}*/
