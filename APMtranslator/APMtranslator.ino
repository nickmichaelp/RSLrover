int Serial_timeout = 25; //Serial Timout

/*serial strings*/
String commandType = "C";
String commandMode;
String throttleORsteer;
String toSend;

//input Pin initializations
int steering_pot_pin = 0;
int speed_pot_pin = 1;

//flags
bool speedFlag = true;


void setup(){
  Serial.begin(9600);
  Serial.setTimeout(Serial_timeout);
}

int mapAPM(int analog){
  /*
  *this converts the 0-5V output of the APM to eqivalent serial string commands, [-1000,1000]
  */
  return map(analog, 0, 5, -1000, 1000);
}

int rc_apm_function(){
  /*
  *This function turns the APM output into the serial sgrings the Rover understands
  */
  int analogSpeed, analogSteering;
  int desiredSpeed, desiredSteering;
  if(true)//this will be act_gear == desiredGear
  {
    commandMode = "A"; //We think we need it in acatuator but are not sure
    if(speedFlag)
    {
      analogSpeed = analogRead(speed_pot_pin);
      desiredSpeed = mapAPM(analogSpeed);
      throttleORsteer = "V";
      toSend = commandType + "," + commandMode + "," + throttleORsteer + "," + desiredSpeed;
      Serial.println(toSend);
    }
    else{
      analogSteering = analogRead(steering_pot_pin);
      desiredSteering = mapAPM(analogSteering);
      throttleORsteer = "W";
      toSend = commandType + "," + commandMode + "," + throttleORsteer + "," + desiredSpeed;
      Serial.println(toSend);
    }
    speedFlag = !speedFlag;
  }
  else{}//if the desired gear does not match actual
  
}
  
void loop(){
  rc_apm_function();
}
