int Serial_timeout = 25; //Serial Timout

//serial Strings
String commandType = "C";
String commandMode;
String throttleORsteer;

//input Pin initializations
int steering_pot_pin = 0;
int speed_pot_pin = 1;


void setup(){
  Serial.begin(9600);
  Serial.setTimeout(Serial_timeout);
}

int mapAPM(analogSpeed){
  /*
  *this converts the 0-5V output of the APM to eqivalent joystick-esqu commands
  */
}

int rc_apm_function(){
  /*
  *This function turns the APM output into the serial sgrings the Rover understands
  */
  int speedFlag = 1;
  int analogSpeed;
  int desiredSpeed;
  if(true)//this will be act_gear == desiredGear
  {
    if(speedFlag)
    {
      command_mode = "A"; //We think we need it in acatuator but are not sure
      analogSpeed = analogRead(speed_pot_pin);
    }
  }
  
}
  
void loop(){
  
}
