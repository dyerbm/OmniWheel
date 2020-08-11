const int ledPin =  LED_BUILTIN;// the number of the LED pin

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
}

String cString;
float v1,v2,v3;
int ind1,ind2,ind3;
int timer = micros();

void loop() {
  // put your main code here, to run repeatedly:
  timer=micros();
  while (Serial1.available()>0) {
    digitalWrite(ledPin, HIGH);
    char c = Serial1.read();
    /*cString = cString + (char)c;
    if (c !='\n') {
      cString += (char)c;
    }

    else if (c == '\n') {
      Serial.print(cString);
      cString = "";
    }*/

    if (c=='*'){
      ind1 = cString.indexOf(',');  //finds location of first ,                                                      //saperating the data in formof motor on off signal and the degree input
      v1 = cString.substring(0, ind1).toFloat();   //captures first data String
      ind2 = cString.indexOf(',', ind1+1 ); //finds location of second ,
      v2 = cString.substring(ind1 + 1,ind2).toFloat(); //captures second data String
      ind3 = cString.indexOf(',', ind2 +1  ); //finds location of second ,
      v3 = cString.substring(ind2+1,ind3).toFloat(); //finds location of second ,
      
      //Serial.println(String(v1int)+ "," + String(v2int)+ "," +String(v3int)+ "," +String(v4int));
      Serial.print(v1);
      Serial.print(",\t");
      Serial.print(v2);
      Serial.print(",\t");
      Serial.print(v3);
      Serial.print("\n");
      cString="";
      v1 = 0;
      v2 = 0;
      v3 = 0;
      Serial.println(micros()-timer);
    }

    else {
      cString += c; //makes the string controlString
    }
    
    //Serial.println(sizeof(cString));
  }
  digitalWrite(ledPin, LOW);
  
  if (Serial.available()) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
    char d = Serial.read();
    Serial.print((char)d);
  }
}
