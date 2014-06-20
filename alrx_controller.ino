#include <SoftwareSerial.h>
#include <Stepper.h>
#include <String.h>
#include <SD.h>
#include <Sensor.h>
#include <Heater.h>

boolean run = false;
boolean stringComplete = false;
int index = 0;
char message [] = "";
SoftwareSerial serial = SoftwareSerial(0,1); //put in correct RX and TX pins
Sd2Card card;
const int chipSelect = 10;
Sensor h2sensor = Sensor (pin, "hydrogen"); //put in correct pin # for hydrogen sensor
Sensor o2sensor = Sensor (pin, "oxygen"); //put in correct pin # for oxygen sensor
Stepper motor (steps, in1Pin, in2Pin, in3Pin, in4Pin); //put in correct # of steps and pin #'s for stepper motor
Heater heater = Heater (pin); //put in correct pin # for (oxygen system) heater

void setup() {
  serial.begin (9600);
  initializeSDCard ();
  setupStepper ();
}

void loop() {
  
  if (stringComplete){
    parseMessage();
    stringComplete = false;
    message = "";
  }
  
  if (run == true)
  {
    serial.println ("System started");
    reactionChamber ();
    hydrogenSystem ();
    oxygenSystem ();
    sendStatus (); //send status report every 1 second
  } 
  else if (run == false)
    serial.println ("System stopped");
 
 
}
void serialEvent ()
{
  while (serial.available()&& stringComplete == false) {
    char inChar = serial.read();
    message [index] = inChar;
    index++;
    if (inChar == -1){
      index = 0;
      stringComplete = true;
    }
  }
}

void initializeSDCard ()
//initializes SD card
{
  serial.print ("Initializing SD card...");
  pinMode (10, OUTPUT);
  if (!card.init(SPI_HALF_SPEED, chipSelect))
    {
      serial.println ("Initialization failed.");
    }  
  else
    {
      serial.println ("Initialization complete.");
    }
}

void setupStepper ()
{
  pinMode (in1Pin, OUTPUT);
  pinMode (in2Pin, OUTPUT);
  pinMode (in3Pin, OUTPUT);
  pinMode (in4Pin, OUTPUT);
  motor.setSpeed (speed); //put in correct initial speed for motor
}

void parseMessage ()
//parses message from serial input
{
  char *p = message; //the message to be parsed
  char *str;
  int count = 0;
  char inParse []= "";
  
  while ((str=strok_r(p, "_", &p)) != NULL)
  {
    inParse [count] = str;
    count++;
  }
  if (inParse [0] != "ALRX") 
  break;
  if (inParse [1]== "START")
    run == true;
  else if (inParse [1]=="STOP")
    run == false;
//add full list of commands 
}

void sendStatus ()
//sends status report to Serial output (ALRX_STATUS 1111 for overall status OK, reaction is ON, H2 side is OK, O2 side is OK)
{
}

void reactionChamber ()
{
   //pressure sensor 
   //thermocouple 
}
void hydrogenSystem ()
{
  h2sensor.log();
  h2sensor.print();
  if (h2sensor.hitMax(maxValue)) //put in correct max value for hydrogen sensor
  {
    //slow pump
  }
  
  //pressure switch
  //pressure sensor

void oxygenSystem ()
//heater-- should be turned on when oxygen level is lower than certain value
  //and turned off when level is above certain value
  //stepper motor-- two speeds-- switch speeds when oxygen level drops below certain value
{
  o2sensor.log();
  o2sensor.print();
  
  if (o2sensor.read()< heaterMin) //put in correct heaterMin when value is known
    heater.turnOn();
  else if (o2sensor.read()> heaterMax) //put in correct heaterMax when value is known
    heater.turnOff(); 
  
  if (o2sensor.read()< value) //put in correct value for variable 'value'
    motor.setSpeed (speed1);
  else if (o2sensor.read()>value) //put in correct value for variable 'value'
    motor.setSpeed (speed2);
  
    
}
