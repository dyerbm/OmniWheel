const int ledPin =  LED_BUILTIN;// the number of the LED pin

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600);
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available()) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
    char c = Serial2.read();
    Serial.print((char)c);
  }
  if (Serial.available()) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
    char d = Serial.read();
    Serial.print((char)d);
  }
}
