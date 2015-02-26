//RSL Rover 2014
//Vehicle Mega code

//Serial1: From Consol
//Serial2: To Steering and Transmission Motor Controller
//Serial3: To Speed Controller
//mySerial: AutoPilot

#include <AltSoftSerial.h>
AltSoftSerial mySerial; //RX:48, TX:46

//Strings (for more info see string initializations in setup)
String consol_input_string;
String selfDrive_string;
String command_type;   
String command_mode;
String throttle_or_steer;
String steering_command;
String speed_string_to_send;
String steering_string_to_send;
String gear_string_to_send;
String desired_gear;
String current_gear;
String string_from_motor_controller;
String mc_state;
String steering_query;
String feedback_to_consol;
String feedback_to_consol_prefix;
String position_prefix;
String suffix;
String space;
String comma;

String A_status;
String B_status;
String C_status;
String D_status;
String E_status;
String horn_status;
String F_status;

//Auxilery Flags
bool selfDrive;

String error_string;
String error_string_previous;
String armed_status;
String temp;
String voltage;

int voltage_input = 0;                                            //Integer placeholder for analogRead of voltage divider circuits
float Twenty_Four_V_Voltage = 0;                                  //24 Volt system voltage
int Twenty_Four_V_Voltage_pin = 8;                                //24 Volt system voltage input pin
float Twelve_V_Voltage = 0;                                       //12 Volt system voltage
int Twelve_V_Voltage_pin = 9;                                     //12 Volt system voltage input pin

int temp_count = 0;                                               //Counter used to ensure temp_start_time begins timing when the temperature light first comes on
unsigned long temp_start_time = 0;                                //Absolute time recorded when the temperature light first turns on
unsigned long temp_end_time = 0;                                  //Absolute time recorded every time an iteration occurs with the temperature light on
unsigned long temp_time = 0;                                      //Time that the temperature light has been on (temp_end_time - temp_start_time)
int temp_time_limit = 20000;                                      //Time in milliseconds to allow the temperature light to stay on before activating emergency stop (important safeguard to prevent serious engine damage)

int threshold_warning_12_v = 11;                                  //12 Volt threshold to give low battery warning on console
int threshold_warning_24_v = 21;                                  //24 Volt threshold to give low battery warning on console
int threshold_e_stop_12_v = 9;                                    //12 volt threshold to activate emergency stop
int threshold_e_stop_24_v = 19;                                   //24 volt threshold to activate emergency stop
int e_stop_state = LOW;                                           //if high, e-stop will be activated (acts as a toggle and if statements can be added anywhere in the code to toggle emergency stop mode on)

int Serial_timout = 100;                                          //Set the serial timeout for hardware serial ports
int temp_warning = 10;                                            //Digital input pin for vehicle's temp warning light  
int reverse = 9;                                                  //Digital input pin for vehicle's reverse gear light  
int neutral = 8;                                                  //Digital input pin for vehicle's neutral gear light                                          
int low = 7;                                                      //Digital input pin for vehicle's low gear light
int high = 6;                                                     //Digital input pin for vehicle's high gear light

int ebrake_relay_pin = 2;                                         //Emergency brake relay pin
int horn_relay_pin = 3;                                           //Horn relay pin
int e_stop_relay_pin = 4;                                         //Emergency stop relay pin
int e_brake_state = HIGH;                                         //Emergency brake state:  HIGH is on, LOW is off
int contact_with_consol = LOW;                                    //Once the Vehicle Mega makes initial contact with the console, this state turns HIGH 
int beacon_relay_pin = 5;                                         //Relay pin for beacon on rollcage

int counter = 0;                                                  //used to control the intermittent sending of data

int desired_speed = 0;                                            //Used to send a desired speed to the speed controller when changing gears                          
int wheel_speed = 0;                                              //Current wheel speed
int wheel_speed_pin = 11;                                         //PWM wheel speed pin from the Axle Tachometer Interpreter

int gear_position = 0;                                            //Desired position to send to gear actuator
int channel=1;                                                    //Channel used to formulate strings to send to motor controllers (either 1 for steering or 2 for transmission command)
int steering_position=0;                                          //Desired steering position parsed from console
int act_steering_position = 0;                                    //Current steering position as querried from the steering motor controller (to be sent as feedback to the console)                                      
int comma_index_1;                                                //Index of first comma in a string (for parsing)
int comma_index_2;                                                //Index of second comma in a string (for parsing)
int comma_index_3;                                                //Index of third comma in a string (for parsing)
int dead_man_timout = 750;                                        //If the Vehicle Mega loses contact with the console for more than the dead_man_timout (in milliseconds), the emergency stop will be hit 
unsigned long e_stop_time_1 = 0;                                  //Records the start time when the last console contact occurred
unsigned long e_stop_time_2 = 0;                                  //Records the end time when the next console contact occurred
unsigned long e_stop_time = 0;                                    //Difference between e_stop_time_2 and e_stop_time_1 (compared to dead_man_timout)


void setup()                                                      //Runs before the main loop to initialize everything
{
  
  pinMode(ebrake_relay_pin, OUTPUT);                              //Sets the emergency brake relay pin to output (same as parking brake)
  digitalWrite(ebrake_relay_pin, e_brake_state);                  //Writes the startup emergency brake state to emergency brake pin
  
  pinMode(e_stop_relay_pin, OUTPUT);                              //Sets the emergency stop relay pin to output
  digitalWrite(e_stop_relay_pin, e_stop_state);                   //Writes the startup emergency stop state to emergency stop pin
  
  pinMode(horn_relay_pin, OUTPUT);                                //Sets the horn relay pin to output
  digitalWrite(horn_relay_pin, LOW);                              //Writes the startup horn state to horn pin (LOW is off)
  
  pinMode(beacon_relay_pin, OUTPUT);                              //Sets the beacon relay pin to output 
  digitalWrite(beacon_relay_pin, LOW);                            //Writes the startup emergency stop state to emergency stop pin
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);                                             //Serial to/from USB or serial monitor (sets baud rate and opens serial port)
  Serial.setTimeout(Serial_timout);                               //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller recieves a long garbage string, it will not waste time reading it
  Serial1.begin(9600);                                            //Serial to/from the Console (sets baud rate and opens serial port)
  Serial1.setTimeout(Serial_timout);                              //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller recieves a long garbage string, it will not waste time reading it
  Serial2.begin(115200);                                          //Serial to/from steering and transmission motor controller (sets baud rate and opens serial port)
  Serial2.setTimeout(Serial_timout);                              //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller recieves a long garbage string, it will not waste time reading it
  Serial3.begin(9600);                                            //Serial to/from Speed Controller (sets baud rate and opens serial port)
  Serial3.setTimeout(Serial_timout);                              //If the serial buffer misses the '\r' character, it will read a really long string.  Setting the timeout ensures that if the controller recieves a long garbage string, it will not waste time reading it
  mySerial.begin(2400);
  mySerial.setTimeout(Serial_timout);
  //pinMode(48, OUTPUT);
  
  pinMode(temp_warning, INPUT);                                   //Sets the temp_warning pin as an input (HIGH or LOW)
  pinMode(reverse, INPUT);                                        //Sets the reverse gear pin as an input (HIGH or LOW)
  pinMode(neutral, INPUT);                                        //Sets the neutral gear pin as an input (HIGH or LOW)
  pinMode(low, INPUT);                                            //Sets the low gear pin as an input (HIGH or LOW)
  pinMode(high, INPUT);                                           //Sets the high gear pin as an input (HIGH or LOW)
  pinMode(wheel_speed_pin, INPUT);                                //Sets the wheel speed pin as an input (PWM)
 
  consol_input_string = String("");                               //String from console
  selfDrive_string = "";
  command_type = String("");                                      //Parsed from consol_input_string: C for command, ? for querries (not yet involved), etc
  command_mode = String("");                                      //Parsed from consol_input_string: A for actuator, S for speed control modes
  throttle_or_steer = String("");                                 //Parsed from consol_input_string: W for steering command, V for speed related commands
  steering_command = String("");                                  //Parsed from consol_input_string: Value from -1000 to 1000 
  steering_string_to_send = String("");                           //Formulated string to send as a steering motor command to steering and transmission motor controller
  speed_string_to_send = String("");                              //Formulated string to send as a command to speed controller
  gear_string_to_send = String("");                               //Formulated string to send as a transmission motor command to steering and transmission motor controller
  desired_gear = String("");                                      //Parsed from consol_input_string: H for high, L for low, N for neutral, R for reverse, P for park
  current_gear = String("N");                                     //Current gear that the vehicle is in: H for high, L for low, N for neutral, R for reverse, P for park
  
  steering_query = String("?TR 1");                               //Query to be sent to steering and transmission motor controller (Asks motor controller what the current steering position is as a value from -1000 to 1000)
  string_from_motor_controller = String("");                      //String sent from steering and transmission motor controller
  mc_state = String("Ready");                                     //State of the steering and transmission motor controller: "Ready" when ready to take commands "Starting" when performing startup procedure
  feedback_to_consol = String("");                                //Feedback string to console includes wheel speed, current gear, and current steering position
  feedback_to_consol_prefix = String("F");                        //Prefix for feedback_to_consol so console recognizes this as feedback and not an error string

  suffix = String ("\r");                                         //Return character to send at the end of command or query to motor controller (denotes the end of a string of data)
  space= String(" ");                                             //Space needed in motor controller commands
  comma = String(",");                                            //Comma used mainly to separate variables for data logging
  position_prefix = String("!g");                                 //"!g" is how absolute position commands to motor controllers begin
 
  A_status = String("");                                 //String parsed from aux_string to indicate that auxiliary button A is in the on position ("A" if read HIGH, "X" if read LOW)
  B_status = String("");                                 //String parsed from aux_string to indicate that auxiliary button B is in the on position ("B" if read HIGH, "X" if read LOW)
  C_status = String("");                                 //String parsed from aux_string to indicate that auxiliary button C is in the on position ("C" if read HIGH, "X" if read LOW)   
  D_status = String("");                                 //String parsed from aux_string to indicate that auxiliary button D is in the on position ("D" if read HIGH, "X" if read LOW)
  E_status = String("");                                 //String parsed from aux_string to indicate that auxiliary button E is in the on position ("E" if read HIGH, "X" if read LOW)
  F_status = String("");                                 //String parsed from aux_string to indicate that auxiliary button F is in the on position ("F" if read HIGH, "X" if read LOW)
  horn_status = String("");                              //String parsed from aux_string to indicate that auxiliary button H (Horn) is in the on position ("H" if read HIGH, "X" if read LOW)
  
  error_string = String("XXXX");
  error_string_previous = String("XXXX");
  armed_status = String("X");
  temp = String("X");
  voltage = String("X");
   
 //Casues Vehicle Mega to wait for contact from the consol to enter the main loop so that the emergency stop is not hit if the console is powered up after the vehicle
 
  top:
  if(Serial1.available() == 0)  
  {
    delay(50);
    goto top;
  }
  else
  {delay(4500);}

}

void error_check()                                        //Checks the vehicle's systmes for errors (currently the only errors being checked for are temperature light, under voltage, and letting the console know that the vehicle is armed)
{
  
 if(digitalRead(temp_warning) == HIGH)                    //Checks the digital pin that is tapped into the temperature light on the dash (if the temperature light is on, the pin will read HIGH)                       
 { 
   temp = String("T");                                    //Puts a "T" in the designated temperature warning place in error_string (indicating that the temperature light is on)
   temp_count = temp_count + 1;                           //Counts iterations that the temperature light has been on
   
   if(temp_count == 1)                                    //Ensures that the timer will start for the temperature light when the light first comes on
   {
    temp_start_time = millis();                           //Assigns the start time on the first iteration that the temperature light has been on
   }
   
   temp_end_time = millis();                              //Assigns the end time each iteration that the temperature light is on for
   temp_time = temp_end_time - temp_start_time;           //Calculates the total time that the temperature light has been on for
   
   if(temp_time >= temp_time_limit)                       //Checks to see if the temperature light has been on for longer than the designated temperature time limit
   {
    e_stop_state = HIGH;                                  //If the temperature light has been on for longer than the temperature time limit, the emergency stop is activated
   }
 }
 
 else                                                     //If the temperature light is off
 {
   temp = String("X");                                    //Puts a "X" in the designated temperature warning place in error_string (indicating that the temperature light is off)
   temp_count = 0;                                        //Resets the counter
 }

 error_string = "E" + armed_status + voltage + temp;      //Formulates error string
 
 if(error_string != error_string_previous)                //Only send error string to the console if the status of errors has changed
 {
 Serial1.println(error_string);                           //Send the error string to the console
 }
 
 error_string_previous = error_string;                    //Set error_string_previous = error_string

}

void check_voltage()                                              //Function to check the on-vehicle voltage levels
{
  voltage_input = analogRead(Twelve_V_Voltage_pin);               //Reads analog value from Twelve_V_Voltage_pin
  Twelve_V_Voltage = voltage_input * .01468;                      //Scales voltage_input 
  voltage_input = analogRead(Twenty_Four_V_Voltage_pin);          //Reads analog value from Twelve_V_Voltage_pin
  Twenty_Four_V_Voltage = voltage_input * .03205;                 //Scales voltage_input 

  if((Twelve_V_Voltage <= threshold_e_stop_12_v) || (Twenty_Four_V_Voltage <= threshold_e_stop_24_v)) //Checks to make sure voltage levels are above emergency stop threshold levels and activates emergency stop if they are not
  {
    e_stop_state = HIGH;
  }
  
  else if((Twelve_V_Voltage <= threshold_warning_12_v) || (Twenty_Four_V_Voltage <= threshold_warning_24_v)) //Checks to make sure voltage levels are above temperature light warning threshold levels and activates temperature light if they are not
  {
    voltage = String("V");                                        //Puts a "V" in the designated temperature warning place in error_string (indicating that one of the vehicle's systems is under voltage)
  }
  
  else
  {
    voltage = String("X");                                        //Puts a "X" in the designated temperature warning place in error_string (indicating that the vehicle's voltage levels are not too low)
  }

}

void aux_switch_parse()                                           //Function to parse the auxiliary switch string from console
{

  A_status = consol_input_string.substring(2, 3);                 //Status of rocker switch A on console("A" for on, "X" for off)
  B_status = consol_input_string.substring(3, 4);                 //Status of rocker switch B on console("B" for on, "X" for off)
  C_status = consol_input_string.substring(4, 5);                 //Status of rocker switch C on console("C" for on, "X" for off)
  D_status = consol_input_string.substring(5, 6);                 //Status of rocker switch D on console("D" for on, "X" for off)
  E_status = consol_input_string.substring(6, 7);                 //Status of rocker switch E on console("E" for on, "X" for off)
  F_status = consol_input_string.substring(7, 8);                 //Status of pushbutton F on console("F" for on, "X" for off)
  horn_status = consol_input_string.substring(8, 9);              //Status of pushbutton H on console("H" for on, "X" for off)
  
  if(horn_status == "H")                                          //If the horn button on the console has been pressed, activate the horn relay
  {
   digitalWrite(horn_relay_pin, HIGH);                           
  }
  else                                                            //If the horn button on the console has not been pressed, make sure the horn is off
  {
   digitalWrite(horn_relay_pin, LOW);
  }
  if(A_status == "A"){                                              //sets the autopiolot flag
    selfDrive = true;
  }
  else{
     selfDrive = false; 
  }
  
}

void gear_change(String consol_input_string)                      //Function called when a gear change is desired
{

  desired_gear = consol_input_string.substring(comma_index_2+1);  //Parses the desired gear from consol_input_string
  
  if (desired_gear != current_gear)                               //Only changes if the desired gear and current gear are differnet
  {
   desired_speed = -500;                                          //Sets up a string to sent to the Speed Controller to apply the brake while the gear is changed so that throttle will be zero and the brake will be applied for you if you are on a hill changing gears
   command_type = String("C");
   command_mode = String("A");
   throttle_or_steer = String("V");
   speed_string_to_send= space + command_type + comma + command_mode + comma + throttle_or_steer + comma + desired_speed;
  
   Serial3.println(speed_string_to_send);                         //Sends string to Speed Controller to apply brakes


    if (desired_gear == "H")                                      //If desired gear is High
    {
      gear_position = 1000;                                       //Transmission actuator position associated with High
      e_brake_state = LOW;                                        //Ensures emergency brake is off
      digitalWrite(ebrake_relay_pin, e_brake_state);              
    }
    
    else if(desired_gear == "L")                                  //If desired gear is Low
    {
      gear_position = 103;                                        //Transmission actuator position associated with Low
      e_brake_state = LOW;                                        //Ensures emergency brake is off
      digitalWrite(ebrake_relay_pin, e_brake_state);
    }
    
    else if(desired_gear == "N")                                  //If desired gear is Neutral
    {
      gear_position = -449;                                       //Transmission actuator position associated with Neutral
      e_brake_state = LOW;                                        //Ensures emergency brake is off
      digitalWrite(ebrake_relay_pin, e_brake_state);
    }
    
    else if(desired_gear == "R")                                  //If desired gear is Reverse
    {
      gear_position = -1000;                                      //Transmission actuator position associated with Reverse
      e_brake_state = LOW;                                        //Ensures emergency brake is off
      digitalWrite(ebrake_relay_pin, e_brake_state);
    }
    
    else if(desired_gear == "P")                                  //If desired gear is Park
    {
      gear_position = -449;                                       //Transmission actuator position associated with Neutral
      e_brake_state = HIGH;                                       //Applies parking brake
      digitalWrite(ebrake_relay_pin, e_brake_state);
    }
    
    else
    {
     desired_gear = current_gear;
    }
    
    channel = 2; 
    gear_string_to_send = position_prefix + space + channel + space + gear_position;
    Serial2.println(gear_string_to_send);
    current_gear_function();
  }
    
    
    else                                                          //When the gear is done changing, the brake is released
    {
    desired_speed = 0;                                            //Brake off and throttle at zero
    speed_string_to_send= space + space + command_type + comma + command_mode + comma + throttle_or_steer + comma + desired_speed;
    if(Serial1.available()){Serial1.readStringUntil('\r');}       //clear serial buffer
    Serial3.println(speed_string_to_send);                        //Send the speed command on the the Speed Controller
    }
}

  
void current_gear_function()                                      //Updates the current gear
{

 if(digitalRead(high) == HIGH)                                    //If the high gear indicator light on the vehicle is on
 {
   current_gear = "H";                                            
 }
 
  else if(digitalRead(low) == HIGH)                               //If the low gear indicator light on the vehicle is on
 {
   current_gear = "L";
 }
 
  else if(digitalRead(neutral) == HIGH)                           //If the neutral gear indicator light on the vehicle is on
 {
     if (e_brake_state == LOW)                                    //If parking brake is off, and vehicle in neutral, the vehicle is simply in neutral
     {
     current_gear = "N";
     }
     else if(e_brake_state == HIGH)                               //If parking brake is on, and vehicle in neutral, the vehicle is in park
     {
       current_gear = "P";
     }
 }
 
  else if(digitalRead(reverse) == HIGH)                           //If the reverse gear indicator light on the vehicle is on
 {
   current_gear = "R";
 }
 
}


void control_command(String consol_input_string)                                               //Called when the string from Vehicle Mega is a control command
{
  comma_index_1 = consol_input_string.indexOf(',');                                            //Index for parsing 
  comma_index_2 = consol_input_string.indexOf(',', comma_index_1 + 1);                         //Index for parsing 
  
  command_mode = consol_input_string.substring(comma_index_1+1, comma_index_2);                //Parses the second character to determine the command mode 

  if(command_mode == "A");                                                                     //If in actuator mode
  {
  comma_index_3 = consol_input_string.indexOf(',', comma_index_2 + 1);                         //Index for parsing 
  throttle_or_steer = consol_input_string.substring(comma_index_2+1, comma_index_3);           //Parses the third character to determine whether the command is a speed or steering command (V or W)
  
    if(throttle_or_steer == "W")                                                               //If a steering command
    {
      channel = 1;                                                                             //Channel 1 for steering commands to steering and transmission motor controller
      steering_command = consol_input_string.substring(comma_index_3+1);                       //Parses steering command value from -1000 to 1000
      steering_position = steering_command.toInt();                                            //Transforms this value from string to intiger
       steering_string_to_send = position_prefix + space + channel + space + steering_position;//Formulation of string
       Serial2.println(steering_string_to_send);                                               //Sending steering command to steering and transmission motor controller 
    }
  
    else if(throttle_or_steer == "V")                                                           //If a speed related command
    {
      if(current_gear == desired_gear)                                                          //If the vehicle is in the correct gear, send on speed commands (helps ensure that vehicle doesnt give throttle input during gear change)
      {
         Serial3.println(consol_input_string);                                                  //String to send to speed controller
      }
    }
  }
  
  if(command_mode == "G" && abs(wheel_speed)<=1)                                                //If the console input string is a gear change string and the absolute value of wheel speed is below 1 mph (Trying not to grind gears!)
  { 
    gear_change(consol_input_string);                                                           //Call gear change function
  }
}
  
    
void loop()                                                                                     //Main loop (iterates over and over)
{

  
  if(Serial1.available())                                                                       //If there is contact with console
  {
  e_stop_time_1 = millis();                                                                     //Record emergency stop start time (if contact with console is lost, the start time will stop updating itself)
  consol_input_string = Serial1.readStringUntil('\r');                                          //Read consol_input_string
  }
 //Serial.println(mySerial.available());
 if(mySerial.available())
  {
     selfDrive_string = mySerial.readStringUntil('\r');
     //Serial.println(selfDrive_string);
     //Serial.println("New LINE");
  }
  
  e_stop_time_2 = millis();                                                                     //Record emergency stop end time                   
  e_stop_time = e_stop_time_2 - e_stop_time_1;                                                  //Time between start and stop for dead man switch 
  
  if(e_stop_time >= dead_man_timout)                                                            //If difference in time is greater than the dead-man timeout, toggle on the emergency stop system
  {
  e_stop_state = HIGH;                                                                          //Indicates the emergency stop system is engaged
  }
  
  if(e_stop_state == HIGH)                                                                      //If emergency stop state is HIGH, write low to the emergency stop relay to engage the emergency stop system
  {
    digitalWrite(e_stop_relay_pin, LOW);
  }
  
  else                                                                                          //Otherwise write the emergency stop relay pin HIGH to keep the emergency stop system off
  {
     digitalWrite(e_stop_relay_pin, HIGH);
  }
  
  if(mc_state == "Ready")                                                                        //Ensuring that the motor controller is not executing its startup procedure
  {
    
    if(consol_input_string.charAt(1)=='C' || consol_input_string.startsWith("C"))                                                      //If the consol_input_string is a command, call control_command function
    {
      if(selfDrive){
        //digitalWrite(48,HIGH);
        //Serial.println("2");
        control_command(selfDrive_string);
      }
      else{
        control_command(consol_input_string);
      }
    }
    
   /* else if(consol_input_string.startsWith("C"))                                                //These should both be identical if statements but this one is needed when the serial interface is used (Serial interface will not get through without this)
    {
      control_command(consol_input_string);
    }*/
    
    else if(consol_input_string.startsWith("A") || consol_input_string.charAt(1)=='A')                                                //If the consol_input_string is a auxiliary switch string, call aux_switch_parse function 
    {
      aux_switch_parse();
    }
    /*
    else if(consol_input_string.charAt(1)=='A')                                                 //These should both be identical if statements but this one is needed when the serial interface is used (Serial interface will not get through without this)                                                  
    {
      aux_switch_parse();
    }
    */
  }
  
  
  if (Serial2.available())                                                                      //Read String from transmission and steering motor controller
  {
  string_from_motor_controller = Serial2.readStringUntil('\r');
  }
  
  if(string_from_motor_controller.startsWith("TR"))                                             //If the string is a response to the steering position query
  {
  string_from_motor_controller = string_from_motor_controller.substring(3);                     //Parse the numerical value from the query response
  act_steering_position = string_from_motor_controller.toInt();                                 //Convert this numerical value from a string to intiger
  }
  
  else if(string_from_motor_controller.startsWith("Starting"))                                  //If the motor controller is executing its startup procedure
  {
    mc_state = string_from_motor_controller;                                                    //Set mc_state to "Starting"
    armed_status = String("X");
  }
  
  else if(string_from_motor_controller.startsWith("Ready"))                                     //When the motor controller is finished executing the startup procedure, the mc_state changes to "Ready"
  {
    mc_state = string_from_motor_controller;
    digitalWrite(beacon_relay_pin, HIGH);                                                       //Lights up beacon to demonstrate vehicle is armed and ready
    armed_status = String("A");
  }
  
  if(counter>10)                                                                                //Only sends the steering query and console feedback every 10 iterations
  {
    counter = 0;
    Serial2.println(steering_query);                                                            //Prints steering_query to transmission and steering motor controller
    feedback_to_consol = feedback_to_consol_prefix + comma + act_steering_position + comma + wheel_speed + comma + current_gear;  
    Serial1.println(feedback_to_consol);                                                        //Prints console feedback 
  }
  
  counter = counter +1;                                                                         //Iteration counter 
  
  wheel_speed = pulseIn(wheel_speed_pin, HIGH, 5000);                                           //Read PWM (in nano-seconds) from Axle Tachometer Interpreter
  wheel_speed = wheel_speed*(.0398)-40;                                                         //Turn PWM into a speed from -40 to +40 mph
  current_gear_function();                                                                      //Function call to determine the current gear
  
  check_voltage();                                                                              //Function call to check voltages
  error_check();                                                                                //Function call to check the vehicle for errors and either alert the console or activate the emergency stop
}
  
  
 
