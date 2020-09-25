/*
  For second teensy to take data from SD card and feed it to the control teensy
  Also recieves data from control teensy and saves it to the SD card

  Ben Dyer - 2020-09-25
 */

#include <DS3231.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

const int chipSelect = 10; //CS pin on Teensy4.0

DS3231 Clock;

byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second;


void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, 
    byte& Hour, byte& Minute, byte& Second) {
  // Call this if you notice something coming in on 
  // the serial port. The stuff coming in should be in 
  // the order YYMMDDwHHMMSS, with an 'x' at the end.
  boolean GotString = false;
  char InChar;
  byte Temp1, Temp2;
  char InString[20];

  byte j=0;
  while (!GotString) {
    if (Serial.available()) {
      InChar = Serial.read();
      InString[j] = InChar;
      j += 1;
      if (InChar == 'x') {
        GotString = true;
      }
    }
  }
  Serial.println(InString);
  // Read Year first
  Temp1 = (byte)InString[0] -48;
  Temp2 = (byte)InString[1] -48;
  Year = Temp1*10 + Temp2;
  // now month
  Temp1 = (byte)InString[2] -48;
  Temp2 = (byte)InString[3] -48;
  Month = Temp1*10 + Temp2;
  // now date
  Temp1 = (byte)InString[4] -48;
  Temp2 = (byte)InString[5] -48;
  Day = Temp1*10 + Temp2;
  // now Day of Week
  DoW = (byte)InString[6] - 48;   
  // now Hour
  Temp1 = (byte)InString[7] -48;
  Temp2 = (byte)InString[8] -48;
  Hour = Temp1*10 + Temp2;
  // now Minute
  Temp1 = (byte)InString[9] -48;
  Temp2 = (byte)InString[10] -48;
  Minute = Temp1*10 + Temp2;
  // now Second
  Temp1 = (byte)InString[11] -48;
  Temp2 = (byte)InString[12] -48;
  Second = Temp1*10 + Temp2;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);

  SPI.setMOSI(12);
  SPI.setMISO(11);
  SPI.setSCK(13);

  // Start the I2C interface
  Wire.begin();

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized."); 
}

String positionString = "";
File dataFile = SD.open("datalog.txt", O_CREAT | O_WRITE);
File pathFile = SD.open("Younng10Hz10.dat", O_READ);
char line[50];
int n;

int time_previous_r=millis();
int time_previous_w=millis();
int T=20;
int Twrite=1000;

void loop() {
  /*// put your main code here, to run repeatedly:
  
  if (Serial.available()) {
    GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);

    Clock.setClockMode(false);  // set to 24h
    //setClockMode(true); // set to 12h

    Clock.setYear(Year);
    Clock.setMonth(Month);
    Clock.setDate(Date);
    Clock.setDoW(DoW);
    Clock.setHour(Hour);
    Clock.setMinute(Minute);
    Clock.setSecond(Second);

  }

  if (Serial1.available()) {
     if (dataFile) {
      dataFile.print(Serial1.read());
      dataFile.flush();
     }

   }

  if (millis()-time_previous_r>=T) { //run every 20 milliseconds
    time_previous_r=millis();

    while (line[n-1]!='\n') {
      line[n]=pathFile.read();
      n++;
    }
    Serial.println(line);
    
    
  }

  if (millis()-time_previous_w>=Twrite) { //reset the files every minute
    time_previous_w=millis();
  }*/
}
