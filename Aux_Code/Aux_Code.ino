int auxA = 22;

String stringFromConsole;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(auxA, OUTPUT);
}

void loop() {
  if(Serial.available()){
    stringFromConsole = Serial.readStringUntil('\r');
    if(stringFromConsole[0] == 'A'){
      if(stringFromConsole[1] == 'A'){
        digitalWrite(auxA, HIGH);
      }
      else{
        digitalWrite(auxA, LOW);
      }
    }
    Serial.println(stringFromConsole);
    Serial2.println(stringFromConsole);
  }

}
