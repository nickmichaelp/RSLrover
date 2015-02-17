int auxA = 22;

String stringFromConsole;

const long interval = 500;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(auxA, OUTPUT);
}

unsigned long previous = 0;
void loop() {
  if(Serial1.available()){
    unsigned long current = millis();
    stringFromConsole = Serial.readStringUntil('\r');
    if(stringFromConsole[0] == 'A'){
      if(stringFromConsole[1] == 'A'){
        digitalWrite(auxA, HIGH);
      }
      else{
        digitalWrite(auxA, LOW);
      }
    }
    if(current - previous < interval) {
       Serial2.println(stringFromConsole);
       previous = current;
    }
    else{
       digitalWrite(auxA, LOW);
    }
  }
}
