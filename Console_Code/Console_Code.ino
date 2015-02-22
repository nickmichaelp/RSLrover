//RSL Rover 2014
//Console Code

// Serial 1: To Vehicle (drive by wire) 
// Serial 2: To Vehicle (XBee)

#include <LiquidCrystal.h>                               //Library for LCD screen
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);                   //LCD Screen initialization

//Strings (for more info see string initializations in setup)
String command_type;   
String command_mode;
String throttle_or_steer;
String steering_string_to_send;
String speed_string_to_send;
String gear_string_to_send;
String desired_gear;
String previous_desired_gear;
String comma;
String aux_string;
String aux_string_previous;
String A_status;
String B_status;
String C_status;
String D_status;
String E_status;
String horn_status;
String F_status;
String return_string;
String string_from_usb;
String string_from_usb_steering;
String string_from_usb_speed;
String string_from_usb_gear;
String act_speed;
String act_steer;
String act_gear;
String armed_status; 
String temp; 
String voltage; 

int counter = 0;                                          //used to control the intermittent sending of data
int Serial_timout = 25;                                   //Set the serial timeout for hardware serial ports
int comma_index_1 = 0;                                    //Index of first comma in a string (for parsing)
int comma_index_2 = 0;                                    //Index of second comma in a string (for parsing)
int comma_index_3 = 0;                                    //Index of third comma in a string (for parsing)
int comma_index_4 = 0;                                    //Index of fourth comma in a string (for parsing)

int desired_steering = 0;                                 //Desired steering position (int from -1000(left) to 1000(right) with center at 0)
int steer_pos = 0;                                        //Used to turn string act_steer into an int to display actual steering position in real time 
int desired_speed;                                        //The variable desired_speed holds the input for the speed potentiometer on the joystick, then is mapped from -1000 to 1000 and sent in a string to the Vehicle Mega
int governor;                                             //Reading from governor potentiometer used to saturate throttle commands
int max_speed = 40;                                       //Maximum speed of vehicle (gives the governor something to map to in speed mode)

int steering_pot_pin = 0;                                 //Analog input pin associated with the steering potentiometer on the joystick
int speed_pot_pin = 1;                                    //Analog input pin associated with the steering potentiometer on the joystick
int gov_pin = 2;                                          //Analog input pin associated with the governor potentiometer

int high_gear = 26;                                       //Digital input pin number that reads position of high gear pushbutton
int low_gear = 25;                                        //Digital input pin number that reads position of high gear pushbutton
int neutral_gear = 23;                                    //Digital input pin number that reads position of high gear pushbutton
int reverse_gear = 22;                                    //Digital input pin number that reads position of high gear pushbutton
int park_brake = 24;                                      //Digital input pin number that reads position of high gear pushbutton

int speed_vs_actuator=27;                                 //Digital input pin number that reads position of speed versus actuator mode rocker switch
int serial_dbw_rc=29;                                     //Digital input pin number that reads position of serial versus drive by wire or remote control mode rocker switch

int aux_A=30;                                             //Digital input pin number for auxilliary rocker switch A 
int aux_B=31;                                             //Digital input pin number for auxilliary rocker switch B
int aux_C=32;                                             //Digital input pin number for auxilliary rocker switch C
int aux_D=33;                                             //Digital input pin number for auxilliary rocker switch D 
int aux_E=34;                                             //Digital input pin number for auxilliary rocker switch E

int horn=35;                                              //Digital input pin number for auxilliary pushbutton H which is currently the horn 
int aux_F=36;                                             //Digital input pin number for auxilliary pushbutton F

int flash_rate = 300;                                     //Rate in milliseconds for LED flashing to indicate that the vehicle is in the process of shifting into a perspective gear
int LED_delay=175;                                        //Time in milliseconds for initial LED flash to indicate that the vehicle is about to begin the process of shifting gears
int start_time=0;                                         //Used to store starting time to meter the flash rate
int stop_time=0;                                          //Used to store ending time to meter the flash rate
int elapsed_time=0;                                       //Used to calculate elapsed time to meter the flash rate
int LED_state=LOW;                                        //Used to set the state of an LED (HIGH for on, LOW for off)
  
int ind_H=37;                                             //LED pin associated with High Gear
int ind_L=38;                                             //LED pin associated with Low Gear
int ind_N=39;                                             //LED pin associated with Neutral Gear
int ind_R=40;                                             //LED pin associated with Reverse Gear
int ind_P=41;                                             //LED pin associated with Park Gear
int ind_1=42;                                             //LED pin to indicate that the console is on
int ind_2=43;                                             //LED pin to indicate that the vehicle is armed and ready to accept commands
int ind_3=44;                                             //LED pin to indicate low battery on vehicle (This LED will illuminate if the 24 volt system falls below 21 volts or if the 12 volt system falls below 10 volts
int ind_4=45;                                             //LED pin to indicate that the vehicle's temperature light is on (if this light stays on for more than 20 seconds, the emergency stop will be activated)
int ind_5=46;                                             //Auxiliary LED pin number
int ind_6=47;                                             //Auxiliary LED pin number

int voltage_input = 0;                                    //Used to store the initial analog input reading for console battery voltage
float Batt_Voltage = 0;                                   //Used to store the value of the calculated battery voltage from the voltage_input
int Batt_Voltage_pin = 3;                                 //Digital pin number to read battery voltage from 
  
  
void setup()                                              //Runs before the main loop to initialize everything
{
  Serial.begin(9600);                                     //Serial to/from USB or serial monitor (sets baud rate and opens serial port)
  Serial.setTimeout(Serial_timout);                       //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller receives a long garbage string, it will not waste time reading it
  Serial1.begin(9600);                                    //Serial to/from the Vehicle via drive by wire (sets baud rate and opens serial port)
  Serial1.setTimeout(Serial_timout);                      //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller recieves a long garbage string, it will not waste time reading it
  Serial2.begin(9600);                                    //Serial to/from the Vehicle via x-bee radio (sets baud rate and opens serial port)
  Serial2.setTimeout(Serial_timout);                      //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller recieves a long garbage string, it will not waste time reading it
  
  lcd.begin(20, 4);                                       //Sets up and opens port to LCD screen
  
  pinMode(ind_H, OUTPUT);                                 //Sets up digital pin ind_H as a digital output
  pinMode(ind_L, OUTPUT);                                 //Sets up digital pin ind_L as a digital output
  pinMode(ind_N, OUTPUT);                                 //Sets up digital pin ind_N as a digital output
  pinMode(ind_R, OUTPUT);                                 //Sets up digital pin ind_R as a digital output
  pinMode(ind_P, OUTPUT);                                 //Sets up digital pin ind_P as a digital output
  pinMode(ind_1, OUTPUT);                                 //Sets up digital pin ind_1 as a digital output
  pinMode(ind_2, OUTPUT);                                 //Sets up digital pin ind_2 as a digital output
  pinMode(ind_3, OUTPUT);                                 //Sets up digital pin ind_3 as a digital output
  pinMode(ind_4, OUTPUT);                                 //Sets up digital pin ind_4 as a digital output
  pinMode(ind_5, OUTPUT);                                 //Sets up digital pin ind_5 as a digital output
  pinMode(ind_6, OUTPUT);                                 //Sets up digital pin ind_6 as a digital output

  
  lcd.setCursor(0, 0); lcd.print("READY"); delay(80);     //Startup procedure for LCD screen on console 
  lcd.setCursor(0, 1); lcd.print("FIRE"); delay(80);
  lcd.setCursor(0, 2); lcd.print("AIM"); delay(150);
  lcd.setCursor(0, 0); lcd.print("     ");
  lcd.setCursor(0, 1); lcd.print("     ");
  lcd.setCursor(0, 2); lcd.print("     ");
  
  
  for (int flash=37; flash<=47; flash++)                  //Startup procedure for LED flash cycle on console
    {
    digitalWrite(flash, HIGH);
    delay(LED_delay);
    digitalWrite(flash, LOW);
    }
  digitalWrite(ind_1, HIGH);
  
   command_type = String("C");                            //Sent at the beginning of a string sent to the vehicle: C for command, ? for queries (not yet involved), etc
   command_mode = String("");                             //Sent in string to vehicle to indicate mode: A for actuator, S for speed control modes
   throttle_or_steer = String("");                        //Sent in command string to vehicle to indicate whether the command is a steering or speed related command:  V for speed related commands, W for steering related commands
   steering_string_to_send= String("C,A,W,0");            //Steering command string sent to vehicle (initialized to center command)
   speed_string_to_send = String("C,A,V,0");              //Speed related command sent ot vehicle (initialized to zero meaning no brake and no throttle)
   gear_string_to_send = String("C,G,P");                 //Gear change command sent to vehicle (initialized to park)
   desired_gear = String("P");                            //Sent in the gear_string_to_send to indicate the desired gear (initialized ot park)
   previous_desired_gear = String("P");                   //Stores the previously desired gear to insure that the gear change string only gets sent if a different desired gear is input
   comma = String(",");                                   //Used to separate different parts of a command string sent to vehicle so the string can be parsed
   aux_string = String("XXXXXXX");                        //String to send to vehicle to indicate the position of the auxiliary pushbuttons or rockers
   aux_string_previous = String("XXXXXXX");               //String to store aux_string to determine if the status of any buttons has changed
   A_status = String("");                                 //String sent in aux_string to indicate that auxiliary button A is in the on position ("A" if read HIGH, "X" if read LOW)
   B_status = String("");                                 //String sent in aux_string to indicate that auxiliary button B is in the on position ("B" if read HIGH, "X" if read LOW)
   C_status = String("");                                 //String sent in aux_string to indicate that auxiliary button C is in the on position ("C" if read HIGH, "X" if read LOW)
   D_status = String("");                                 //String sent in aux_string to indicate that auxiliary button D is in the on position ("D" if read HIGH, "X" if read LOW)
   E_status = String("");                                 //String sent in aux_string to indicate that auxiliary button E is in the on position ("E" if read HIGH, "X" if read LOW)
   F_status = String("");                                 //String sent in aux_string to indicate that auxiliary button F is in the on position ("F" if read HIGH, "X" if read LOW)
   horn_status = String("");                              //String sent in aux_string to indicate that auxiliary button H (Horn) is in the on position ("H" if read HIGH, "X" if read LOW)
   return_string = String("");                            //Stores string sent back to console from vehicle either as feedback or a fault code
   string_from_usb = String("");                          //Stores string sent to console from the USB input 
   act_speed = String("x");                               //Stores the wheel speed value from the vehicle feedback to display on LCD screen
   act_steer = String("x");                               //Stores the steering position value from the vehicle feedback to display on LCD screen
   act_gear = String("x");                                //Stores the current gear that the vehicle is in from the vehicle feedback to display on the console LEDs
   armed_status = String("X");                            //Sent from vehicle to console to alert console that the vehicle is armed and ready to take commands ("A" for armed, "X" for not armed)
   temp = String("X");                                    //Sent from vehicle to console to alert console that the vehicle's temperature warning light is on ("T" for temperature light on, "X" for temperature light off)
   voltage = String("X");                                 //Sent from vehicle to console to alert console that one of the vehicle's systems is at a low voltage ("V" for under voltage, "X" for healthy voltage levels)
 
 
   pinMode(high_gear, INPUT);                             //Sets up digital pin high_gear as a digital input 
   pinMode(low_gear, INPUT);                              //Sets up digital pin low_gear as a digital input 
   pinMode(neutral_gear, INPUT);                          //Sets up digital pin neutral_gear as a digital input 
   pinMode(reverse_gear, INPUT);                          //Sets up digital pin reverse_gear as a digital input 
   pinMode(park_brake, INPUT);                            //Sets up digital pin park_brake as a digital input 
   pinMode(speed_vs_actuator, INPUT);                     //Sets up digital pin speed_vs_actuator as a digital input 
   pinMode(serial_dbw_rc, INPUT);                         //Sets up digital pin serial_dbw_rc as a digital input 
   pinMode(aux_A, INPUT);                                 //Sets up digital pin aux_A as a digital input 
   pinMode(aux_B, INPUT);                                 //Sets up digital pin aux_B as a digital input 
   pinMode(aux_C, INPUT);                                 //Sets up digital pin aux_C as a digital input 
   pinMode(aux_D, INPUT);                                 //Sets up digital pin aux_D as a digital input 
   pinMode(aux_E, INPUT);                                 //Sets up digital pin aux_E as a digital input 
   pinMode(horn, INPUT);                                  //Sets up digital pin horn as a digital input 
   pinMode(aux_F, INPUT);                                 //Sets up digital pin aux_F as a digital input 
   
    
}


int map_joystick(int minimum, int min_dead, int max_dead, int maximum, int pos)      //Maps joystick input from minimum to maximum value (-1000 to 1000) for speed and steering inputs while taking into account deadband 
{
  if(pos>max_dead)                                                                   
  {pos=map(pos, max_dead, maximum, 0, 1000);}
  else if(pos<min_dead)
  {pos=map(pos, min_dead, minimum, 0, -1000);}
  else 
  {pos=0;}
  return -1*pos;
  
}


void aux_switch_read()                                                              //Function to read the auxilliary switch digital inputs and send a string to Vehicle Mega indicating their status
{
  if(digitalRead(aux_A)==HIGH)                                                      //Checks to see if aux_A pin is HIGH
  {A_status= String("A");}                                                          //If the button is on, assign the "A" character to its place in aux_string
  else{A_status= String("X");}                                                      //If the button is off, assign the "X" character to its place in aux_string
  if(digitalRead(aux_B)==HIGH)                                                      //Checks to see if aux_B pin is HIGH                                                 
  {B_status= String("B");}                                                          //If the button is on, assign the "B" character to its place in aux_string
  else{B_status= String("X");}                                                      //If the button is off, assign the "X" character to its place in aux_string
  if(digitalRead(aux_C)==HIGH)                                                      //Checks to see if aux_C pin is HIGH                                                 
  {C_status= String("C");}                                                          //If the button is on, assign the "C" character to its place in aux_string
  else{C_status= String("X");}                                                      //If the button is off, assign the "X" character to its place in aux_string
  if(digitalRead(aux_D)==HIGH)                                                      //Checks to see if aux_D pin is HIGH                                                 
  {D_status= String("D");}                                                          //If the button is on, assign the "D" character to its place in aux_string
  else{D_status= String("X");}                                                      //If the button is off, assign the "X" character to its place in aux_string
  if(digitalRead(aux_E)==HIGH)                                                      //Checks to see if aux_E pin is HIGH                                                 
  {E_status= String("E");}                                                          //If the button is on, assign the "E" character to its place in aux_string
  else{E_status= String("X");}                                                      //If the button is off, assign the "X" character to its place in aux_string
  if(digitalRead(aux_F)==HIGH)                                                      //Checks to see if aux_F pin is HIGH                                                 
  {F_status= String("F");}                                                          //If the button is on, assign the "F" character to its place in aux_string
  else{F_status= String("X");}                                                      //If the button is off, assign the "X" character to its place in aux_string
  if(digitalRead(horn)==HIGH)                                                       //Checks to see if horn pin is HIGH                                                 
  {horn_status= String("H");}                                                       //If the button is on, assign the "H" character to its place in aux_string
  else{horn_status= String("X");}                                                   //If the button is off, assign the "X" character to its place in aux_string
  
  aux_string = String("A") + A_status + B_status + C_status + D_status + E_status + F_status + horn_status;  //Formulate auxilliary switch string
  
  if(aux_string != aux_string_previous)                                             //So the string is only sent when the status of a button or rocker changes
  { 
     Serial1.println(aux_string);                                                   //Send auxilliary switch string over drive by wire
     Serial2.println(aux_string);                                                   //Send auxilliary switch string over x-bees
  }
  
  aux_string_previous = aux_string;
}


void return_command(String return_string)                                            //Function to parse string coming from vehicle and assign variables based on string
{
  //Serial.println('.' + return_string + '.');
  if(return_string.startsWith("F"))                                                //If the string starts with "F" it is a feedback string not an error string
  {
    //Serial.println("I get the F command");
  comma_index_1 = return_string.indexOf(',');                                        //Records index value of first comma in string for parsing
  comma_index_2 = return_string.indexOf(',', comma_index_1 + 1);                     //Records index value of second comma in string for parsing
  comma_index_3 = return_string.indexOf(',', comma_index_2 + 1);                     //Records index value of third comma in string for parsing
  
  act_steer = return_string.substring(comma_index_1 + 1, comma_index_2);             //Parses and records value of the current steering position
  act_speed = return_string.substring(comma_index_2 + 1, comma_index_3);             //Parses and records value of the current wheel speed
  act_gear = return_string.substring(comma_index_3 + 1);                             //Parses and records value of the current gear
  //Serial.println(act_gear);
  }
  
  else if(return_string.startsWith("E"))                                             //If the vehicle is sending the console an error code
  {
  armed_status = return_string.substring(1, 2);                                      //Parse armed_status out of error code
  voltage = return_string.substring(2, 3);                                           //Parse voltage out of error code
  temp = return_string.substring(3, 4);                                              //Parse temp out of error code
  }
  
  else if(return_string.charAt(1)=='E')                                              //Same as previous else if statement.  This should not be necessary but every once in a while the previous statement lets an error code go and this catches it and keeps it from going undetected
  {
  armed_status = return_string.substring(2, 3);
  voltage = return_string.substring(3, 4);
  temp = return_string.substring(4, 5);
  }
  
  if(armed_status == "A"){digitalWrite(ind_2, HIGH);}                                //Check if the vehicle is armed.  Illuminate ARMED LED if it is
  else {digitalWrite(ind_2, LOW);}
  
  if(temp == "T"){digitalWrite(ind_4, HIGH);}                                        //Check if the vehicle's temperature warning light is on.  Illuminate ERROR LED if it is
  else {digitalWrite(ind_4, LOW);}
  
  if(voltage == "V"){digitalWrite(ind_3, HIGH);}                                     //Check if the vehicle is under voltage.  Illuminate LOW BATT LED if it is
  else {digitalWrite(ind_3, LOW);}
}


void serial_interface_function()                                                     //If the serial versus remote control or drive by wire rocker is in the serial position, this function is called to take read the input string from the USB port and deal with it
{ 
  //Serial.println(string_from_usb);
  if(Serial.available())
  {
    string_from_usb = Serial.readStringUntil('\r');                                  //Read string from USB port until return character
  }
  
  if (string_from_usb.substring(2,3) == "G")                                         //Checks to see if it is a gear change command
  {
    gear_string_to_send = string_from_usb;                                           //If the input string is a gear change string, set it equal to gear_string_to_send
    if(gear_string_to_send.substring(4,5) == "P")
    {
      desired_gear = String("P");
    }
    else if(gear_string_to_send.substring(4,5) == "N")
    {
      desired_gear = String("N");
    }
    else if(gear_string_to_send.substring(4,5) == "R")
    {
      desired_gear = String("R");
    }
    else if(gear_string_to_send.substring(4,5) == "L")
    {
      desired_gear = String("L");
    }
    else if(gear_string_to_send.substring(4,5) == "H")
    {
      desired_gear = String("H");
    }
    if(gear_string_to_send != "Clear")                                               //Checks to make sure that the gear change string only gets sent once (sets equal to "Clear" later in the code after it has been sent)
    {
      //Serial.println(gear_string_to_send);
      Serial1.println(gear_string_to_send);                                          //Send gear_string_to_send to vehicle via drive by wire
      Serial2.println(gear_string_to_send);                                          //Send gear_string_to_send to vehicle via x-bees
       //Serial.println(gear_string_to_send);
      digitalWrite(ind_H, LOW);                                                      //turns off all gear indicator LEDs that may have been on
      digitalWrite(ind_L, LOW);
      digitalWrite(ind_N, LOW);
      digitalWrite(ind_R, LOW);
      digitalWrite(ind_P, LOW);
    }
  }
  
  else if (string_from_usb.substring(4,5) == "W")                                    //If the USB input string is not a gear change command this checks to see if it is a steering command 
  {
    steering_string_to_send = string_from_usb;                                       //If the input string is a steering string, set it equal to steering_string_to_send
  }
  
  else if (string_from_usb.substring(4,5) == "V")                                    //If the USB input string is not a gear change or steering command this checks to see if it is a speed related command 
  {
    speed_string_to_send = string_from_usb;                                          //If the input string is a speed related command, set it equal to speed_string_to_send
  }
  
  if(act_gear == desired_gear)                                                       //Once the vehicle has shifted gears, stop sending gear change string and get back to sending steering and speed commands
  {
    string_from_usb = String("C");                                                   //Clears gear_string_to_send so that the previous part of the code knows the vehicle is done changing gears and stops sending the gear change command
    
    if (counter == 1)                                                                //Used to alternate between sending steering and speed commands (sends the steering command on the first iteration and speed related command on the second)
    {
      //Serial.println(steering_string_to_send);
      Serial1.println(steering_string_to_send);                                      //Send steering command over drive by wire
      Serial2.println(steering_string_to_send); 
      //Serial.println(steering_string_to_send);      //Send steering command over x-bees
    }
    
    else if (counter >= 2)                                                           //Used to alternate between sending steering and speed commands (sends the steering command on the first iteration and speed related command on the second)
    {
      //Serial.println(speed_string_to_send);
      Serial1.println(speed_string_to_send);                                         //Send speed related command over drive by wire
      Serial2.println(speed_string_to_send);                                         //Send speed related command over x-bees
      //Serial.println(speed_string_to_send);
      counter = 0;                                                                   //Reset counter
    }
      
  counter = counter + 1;                                                             //Count iterations
  }

 
  if(act_gear == "H") {digitalWrite(ind_H, HIGH);digitalWrite(ind_L, LOW);digitalWrite(ind_N, LOW);digitalWrite(ind_R, LOW);digitalWrite(ind_P, LOW);}        //Illuminates LED corresponding to the current gear of the vehicle and writes all other LEDs LOW
  else if (act_gear == "L") {digitalWrite(ind_L, HIGH);digitalWrite(ind_H, LOW);digitalWrite(ind_N, LOW);digitalWrite(ind_R, LOW);digitalWrite(ind_P, LOW);}
  else if (act_gear == "N") {digitalWrite(ind_N, HIGH);digitalWrite(ind_L, LOW);digitalWrite(ind_H, LOW);digitalWrite(ind_R, LOW);digitalWrite(ind_P, LOW);}
  else if (act_gear == "R") {digitalWrite(ind_R, HIGH);digitalWrite(ind_N, LOW);digitalWrite(ind_L, LOW);digitalWrite(ind_H, LOW);digitalWrite(ind_P, LOW);}
  else {digitalWrite(ind_P, HIGH);digitalWrite(ind_R, LOW);digitalWrite(ind_N, LOW);digitalWrite(ind_L, LOW);digitalWrite(ind_H, LOW);} 

}

void rc_dbw_function()                                                               //Separate function for when the vehicle is operating in drive by wire or remote control mode as opposed to serial mode
{
  string_from_usb = String("C");                                                     //Clears string_from_usb 

  if(digitalRead(high_gear)==HIGH)                                                   //Reads digital inputs from gear pushbuttons and assigns the desired_gear to a string indicating the gear if one of the pushbuttons is pressed
  {desired_gear= String("H");}
  else if(digitalRead(low_gear)==HIGH)
  {desired_gear= String("L");}
  else if(digitalRead(neutral_gear)==HIGH)
  {desired_gear= String("N");}
  else if(digitalRead(reverse_gear)==HIGH)
  {desired_gear= String("R");}
  else if(digitalRead(park_brake)==HIGH)
  {desired_gear= String("P");}
  else
  {desired_gear=previous_desired_gear;}                                            //If no gear pushbuttons are pressed, this sets the desired_gear=previous_desired_gear                                              
  
  //Serial.println(desired_gear);
  
  if(desired_gear != previous_desired_gear)                                         //Checks to make sure the new desired gear is not equal to the previously desired gear
  {
    for (int flash=ind_H; flash<=ind_P; flash++)                                    //Flash sequence during gear change
    {digitalWrite(flash, LOW);}
    //Serial.println("d != p");
  }
  previous_desired_gear=desired_gear;                                               //Sets previous_desired_gear=desired_gear
  
  if(act_gear != desired_gear)                                                      //Checks to make sure the desired gear is different from the actual gear and does not do anythign further if the vehicle is currently in the gear desired
  {
    command_mode = "G";                                                               //Sets the command mode to "G" for gear (This will tell the Vehicle Mega that the command is a gear change)
    gear_string_to_send = command_type + comma + command_mode + comma + desired_gear; //Formulates the gear change command to send to the vehicle
    Serial1.println(gear_string_to_send);                                             //Sends the gear change command to the vehicle via drive by wire
    Serial2.println(gear_string_to_send);                                             //Sends the gear change command to the vehicle via x-bee
    //Serial.println(gear_string_to_send);
    
    stop_time=millis();                                                               //Assigns the stop time used to calculate the elapsed time which enforces the flash rate
    elapsed_time=stop_time-start_time;                                                //Calcualtes the elapsed time used to enforces the flash rate
    if (elapsed_time > flash_rate)                                                    //Checks to see if the elapsed time is greater than the flash rate
    {
      start_time=millis();                                                            //Assigns the start time used to calculate the elapsed time which enforces the flash rate
      if(LED_state==HIGH)                                                             //If the elapsed time is greater than the flash rate, the LED switches states
      { LED_state=LOW;}
      else
      { LED_state=HIGH;}
      
      if(desired_gear == "H") {digitalWrite(ind_H, LED_state);}                       //Writes LED_state (HIGH or LOW) to digital pin corresponding to the gear that is changing
      else if (desired_gear == "L") {digitalWrite(ind_L, LED_state);}
      else if (desired_gear == "N") {digitalWrite(ind_N, LED_state);}
      else if (desired_gear == "R") {digitalWrite(ind_R, LED_state);}
      else {digitalWrite(ind_P, LED_state);}
    }
  }
  else if(act_gear == "H") {digitalWrite(ind_H, HIGH);digitalWrite(ind_L, LOW);digitalWrite(ind_N, LOW);digitalWrite(ind_R, LOW);digitalWrite(ind_P, LOW);}  //If the vehicle is in the gear that is desired, illuminate the corresponding LED
  else if (act_gear == "L") {digitalWrite(ind_L, HIGH);digitalWrite(ind_H, LOW);digitalWrite(ind_N, LOW);digitalWrite(ind_R, LOW);digitalWrite(ind_P, LOW);}
  else if (act_gear == "N") {digitalWrite(ind_N, HIGH);digitalWrite(ind_L, LOW);digitalWrite(ind_H, LOW);digitalWrite(ind_R, LOW);digitalWrite(ind_P, LOW);}
  else if (act_gear == "R") {digitalWrite(ind_R, HIGH);digitalWrite(ind_N, LOW);digitalWrite(ind_L, LOW);digitalWrite(ind_H, LOW);digitalWrite(ind_P, LOW);}
  else {digitalWrite(ind_P, HIGH);digitalWrite(ind_R, LOW);digitalWrite(ind_N, LOW);digitalWrite(ind_L, LOW);digitalWrite(ind_H, LOW);} 
  
  governor = analogRead(gov_pin);                                                      //Read governor analog input
  governor = map(analogRead(gov_pin), 1023, 0, 0, 1000);                               //Map or scale the governor's input from 0 to 1000
  
  if(act_gear == desired_gear)                                                         //Checks to ensure that the vehicle is in the gear that is desired
  {
     if(counter==1)                                                                    //If on the first iteration, formulate and send speed related commands (counter used to alternate between speed related commands and steering related commands)
     {
       if(digitalRead(speed_vs_actuator)==HIGH)                                         //Checks whether the console indicates speed or actuator mode and assigns the command mode to the desired mode
         {command_mode = String("S");}
       else
         {command_mode = String("A");}
     
       desired_speed = analogRead(speed_pot_pin);                                        //Reads speed analog input from joystick
       desired_speed= map_joystick(3, 311, 379, 813, desired_speed);                     //Maps speed analog input from joystick with speed potentiometer deadband
     
       if(act_gear == "R")                                                               //If the vehicle is currently in reverse, the desired speed commands should be reversed so it intuitively makes sense to drive (in reverse if you push the joystick back, you will recieve throttle input and if you push it forward the brakes will be activated)
       {
         desired_speed = -1*desired_speed;
       }
       //GOVERNOR:
       if(governor<desired_speed)                                                        //If the desired speed is greater than the governor input, govern the speed or throttle input from the joystick 
       {
         desired_speed = governor;
       }
    
       throttle_or_steer = String("V");                                                  //Set throttle_or_steer equal to "V" to indicate that this is a speed related command
       speed_string_to_send= command_type + comma + command_mode + comma + throttle_or_steer + comma + desired_speed; //Formulate speed related command
       Serial1.println(speed_string_to_send);                                            //Send speed related command to vehicle via drive by wire
       Serial2.println(speed_string_to_send);                                            //Send speed related command to vehicle via x-bee
     }
     
     if(counter>=2)                                                                      //If on the second iteration, formulate and send steering commands (counter used to alternate between speed related commands and steering related commands)
     {
       desired_steering = analogRead(steering_pot_pin);                                  //Reads analog input from steering potentiometer on joystick
       desired_steering= map_joystick(1, 316, 395, 939, desired_steering);               //Maps or scales the analog steering input from joystick
       throttle_or_steer = String("W");                                                  //Set throttle_or_steer equal to "W" to indicate that this is a steering command
     
       steering_string_to_send= command_type + comma + command_mode + comma + throttle_or_steer + comma + desired_steering; //Formulate steering command
       Serial1.println(steering_string_to_send);                                         //Send steering command to vehicle via drive by wire
       Serial2.println(steering_string_to_send);                                         //Send steering command to vehicle via x-bee
       counter = 0;                                                                      //Reset counter
     }
      
     counter = counter + 1;                                                            //Keep count of iteration
  }
  
}

void batt_check()                                                                      //Function to check battery voltage in console
{
  voltage_input = analogRead(Batt_Voltage_pin);                                        //Reads analog input of console battery voltage if in remote control mode or 12 volt voltage from vehicle if in drive by wire mode
  Batt_Voltage = voltage_input * .01468;                                               //Scales battery voltage input to display numerical value in volts
  lcd.setCursor(0, 3); lcd.print(Batt_Voltage);                                        //Prints battery voltage or 12 volt input to LCD screen
}
    
    
void loop()                                                                            //Main loop (iterates over and over)
{
  if(digitalRead(serial_dbw_rc)==HIGH)                                                 //If the vehicle is in serial mode, call the serial_interface_function
  {
   serial_interface_function();
   //Serial.println("Serial");
  }
  
  else                                                                                 //If the vehicle is in drive by wire or remote control mode, call the rc_dbw_function
  {
     //Serial.println("dbw");
    rc_dbw_function();
    
  }
  
  if(Serial1.available())                                                              //If the Vehicle Mega is sending serial data back to the console via wired serial link, read it and store it as return_string
  {return_string = Serial1.readStringUntil('\r');
  return_command(return_string);}                                                      //Call function return_command and pass the string return_string to it
  
  if(Serial2.available())                                                              //If the Vehicle Mega is sending serial data back to the consol via x-bee, read it and store it as return_string
  {return_string = Serial2.readStringUntil('\r');
  return_command(return_string);}                                                      //Call function return_command and pass the string return_string to it
 
  lcd.setCursor(0, 0); lcd.print("Speed =");                                                                      //write "Speed =" to LCD screen
  lcd.setCursor(8, 0); lcd.print("      ");                                                                       //write space to LCD screen
  lcd.setCursor(8, 0); lcd.print(act_speed);                                                                      //write current wheel speed to LCD screen

  steer_pos=act_steer.toInt();                                                                                    //Transform steering feedback from a string to an integer
  steer_pos=steer_pos / 10;                                                                                       //Transofrm steering feedback from -1000 to 1000 value from -100 to 100 so it can be displayed as a percentage
  lcd.setCursor(0, 1); lcd.print("Steering =");                                                                   //write "Steering =" to LCD screen
  lcd.setCursor(11, 1); lcd.print("         ");                                                                   //write space to LCD screen
    if (steer_pos<0) {lcd.setCursor(11, 1); lcd.print("L"); steer_pos=-steer_pos;                                 //If steering is negative, make positive percentage and put a L before it to indicate left
    lcd.setCursor(12,1); lcd.print(steer_pos); lcd.print("%");}                                                   //write "%" to LCD screen
    else if (steer_pos>0) {lcd.setCursor(11, 1); lcd.print("R");                                                  //If steering is negative, put a R before it to indicate right
    lcd.setCursor(12,1); lcd.print(steer_pos); lcd.print("%");}                                                   //write "%" to LCD screen
    else {lcd.setCursor(11, 1); lcd.print("Center");}                                                             //If steering feedback is zero, write center
  lcd.setCursor(0,2); lcd.print("Governor =");                                                                    //write "Governor =" to LCD screen
  lcd.setCursor(11, 2); lcd.print("         ");                                                                   //write space to LCD screen
  lcd.setCursor(11, 2);
  if(digitalRead(speed_vs_actuator)==HIGH)                                                                        //If in speed mode, map governor from 0 to maximum vehicle speed and display this in miles per hour
  {
    governor = map(governor, 0, 1000, 0, max_speed);
    lcd.print(governor); lcd.print(" MPH");
  }
  else                                                                                                            //If in actuator mode, map governor from 0 to 100 indicating a percentage of throttle input
  {
    governor = map(governor, 0, 1000, 0, 100);
    lcd.print(governor); lcd.print("%");
  }
    
  lcd.setCursor(17, 3); lcd.print("RSL");                                                                         //Write RSL to the LCD screen to indicate ownership
 
  aux_switch_read();                                                                                              //Reads status of auxiliary rocker switches and pushbuttons and sends string to vehicle 
}
