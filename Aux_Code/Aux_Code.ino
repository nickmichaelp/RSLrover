#include <QueueList.h>
QueueList<String> inputBuffer;
int auxA = 22;

String stringFromConsole;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(auxA, OUTPUT);
  pinMode(19, INPUT);
  attachInterrupt(4,add2Queue,LOW);
  pinMode(13,OUTPUT);
  pinMode(11 ,OUTPUT);
}

void add2Queue(){
  if(Serial1.available()){
    digitalWrite(11,HIGH);
    //inputBuffer.push(Serial1.readStringUntil('\r'));
    inputBuffer.push("C,G,P");
    digitalWrite(13,HIGH);
  }
}

void loop() {
  //digitalWrite(13,HIGH);
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  if(!inputBuffer.isEmpty()){
    //digitalWrite(13,HIGH);
    //stringFromConsole = Serial1.readStringUntil('\r');
    stringFromConsole = inputBuffer.pop();
    if(stringFromConsole[1] == 'A'){
      if(stringFromConsole[2] == 'A'){
        digitalWrite(auxA, HIGH);
      }
      else{
        digitalWrite(auxA, LOW);
      }
    }
    stringFromConsole.trim();
    Serial2.println(stringFromConsole);
    //Serial.println(stringFromConsole);
  }
  if(Serial2.available()){
    //digitalWrite(13,HIGH);
    stringFromConsole = Serial2.readStringUntil('\r');
    stringFromConsole.trim();
    Serial1.println(stringFromConsole);
   
  }
//  if(Serial3.available()){
//    stringFromConsole = Serial3.readStringUntil('\r');
//    Serial.println(stringFromConsole);
//  }
  digitalWrite(13,LOW);
}
