#include <AltSoftSerial.h>

AltSoftSerial altSerial;

void setup() {
  Serial.begin(9600);
  Serial.println("AltSoftSeria Test Begin");
  while(Serial.available()){Serial.read();};
  altSerial.begin(9600);
  altSerial.println("Hello World");
  while(altSerial.available()){altSerial.read();};
}

void loop() {
  char c;
  //while(Serial.available()){
  //Serial.print((char)Serial.read());
  //Serial.println("");
  //}
//Serial.println(Serial.available());

  if (Serial.available()) {
    c = Serial.read();
    Serial.print(c);
  }
  
  if (altSerial.available()) {
    c = altSerial.read();
    Serial.print(c);
  }
}
