int Serial_timeout = 25; //Serial Timout

//serial Strings
String commandType = "C";
String commandMode;

//input Pin initializations
int steering_pot_pin = 0;
int speed_pot_pin = 1;


void setup(){
  Serial.begin(9600);
  Serial.setTimeout(Serial_timeout);
}

int rc_apm_function(){
  int speedFlag = 1;
  if(true)//this will be act_gear == desiredGear
  {
    if(speedFlag)
    {
      
    }
  }
  
}
  
void loop(){
  
}
