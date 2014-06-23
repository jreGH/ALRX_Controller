#include <Time.h>
#include <SoftwareSerial.h>
#include <Stepper.h>
#include <String.h>
#include <SD.h>
#include <Sensor.h>
#include <Heater.h>
#include <ctype.h>
#include <MuxShield.h>


boolean run = false;
boolean log = true;
boolean print = false;
boolean stringComplete = false;
int index = 0;
char message [] = "";
SoftwareSerial serial = SoftwareSerial(0,1); //put in correct RX and TX pins
MuxShield muxShield;
Sd2Card card;
const int chipSelect = 10;
Sensor h2sensor = Sensor (pin, "hydrogen"); //put in correct pin # for hydrogen sensor
Sensor o2sensor = Sensor (pin, "oxygen"); //put in correct pin # for oxygen sensor
Stepper motor (steps, in1Pin, in2Pin, in3Pin, in4Pin); //put in correct # of steps and pin #'s for stepper motor
Heater heater = Heater (pin); //put in correct pin # for (oxygen system) heater
time_t t = now();
int timeSeconds = second(t);

void setup() {
  serial.begin (9600); //printing to device
  Serial.begin (9600); //printing to serial monitor
  setupMuxShield();
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
    Serial.println ("System started");
    reactionChamber ();
    hydrogenSystem ();
    oxygenSystem ();
    if (second (t) == (timeSeconds +1))
     {   
      timeSeconds = second (t);
      sendStatus (); //send status report every 1 second
     }
  } 
  else if (run == false)
    Serial.println ("System stopped");
 
}
void serialEvent ()
{
  while (serial.available()&& stringComplete == false) {
    char inChar = serial.read();
    message [index] = toupper(inChar);
    index++;
    if (inChar == -1){
      index = 0;
      stringComplete = true;
    }
  }
}

void setupMuxShield ()
//sets up mux shield board
{
  muxShield.setMode (1,ANALOG_IN); //analog input
  muxShield.setMode (2,ANALOG_IN); //analog input
  muxShield.setMode (3,DIGITAL_OUT); //digital output
}

void initializeSDCard ()
//initializes SD card
{
  Serial.print ("Initializing SD card...");
  pinMode (10, OUTPUT);
  if (!card.init(SPI_HALF_SPEED, chipSelect))
    {
      Serial.println ("Initialization failed.");
    }  
  else
    {
      Serial.println ("Initialization complete.");
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
  {
    Serial.print ("Received unknown command: ");
      for (int i=0;i<inParse.length();i++)
      {
        Serial.print(inParse[i]);
      }
      Serial.println();
  break;
  }
switch (inParse[1])
{
case "START" :
run ==true;
break;
case "STOP" :
run==false;
break;
case "STATUS":
 
}

//add full list of commands 
}

void sendStatus ()
//sends status report to serial output (ALRX_STATUS 1111 for overall status OK, reaction is ON, H2 side is OK, O2 side is OK)
{
  String statusRep= "ALRX_STATUS";
  int overallStat = 0;
  int reactionStat = 0;
  int H2Stat = 0;
  int O2Stat = 0;

if (checkOverallStat)
overallStat=1;

if (checkReactionStat)
reactionStat=1;

if (checkH2Stat)
H2Stat=1;

if (checkO2Stat)
O2Stat=1;

 statusRep = statusRep.concat(overallStat); 
 statusRep = statusRep.concat(reactionStat);
 statusRep = statusRep.concat(H2Stat);
 statusRep = statusRep.concat(O2Stat);
 
 serial.println (statusRep);
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
//heater-- should be turned on/off determined by temperature 
  //stepper motor-- two speeds-- switch speeds when oxygen level drops below certain value
{
  o2sensor.log();
  o2sensor.print();
  
  if (temperature < heaterMin) //put in correct heaterMin when value is known
    heater.turnOn();
  else if (temperature > heaterMax) //put in correct heaterMax when value is known
    heater.turnOff(); 
  
  if (o2sensor.read()< value) //put in correct value for variable 'value'
    motor.setSpeed (speed1);
  else if (o2sensor.read()>value) //put in correct value for variable 'value'
    motor.setSpeed (speed2);
  
    
}
