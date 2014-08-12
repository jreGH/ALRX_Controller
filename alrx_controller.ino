#include <Time.h>
#include <SoftwareSerial.h>
#include <String.h>
#include <SD.h>
#include <ctype.h>
#include <MuxShield.h>
#include <OneWire.h> //for thermocouple
#include <DallasTemperature.h> //for thermocouple
#include <PID_v1.h> //for heater control

#define mdir 4 // motor direction
#define msp 5 // motor speed 62.5
#define RelayPin 6 //connected to MOSFET G

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9

#define impulse 6 //impulse generator 6 pulse/mech.rotation (for pump)
#define H2PressurePin A5 //(for AST pressure sensor)
#define P_HI 36 //H2 max pressure (psi)
#define P_LO 8 //H2 min pressure (psi)
double H2Pressure; //AST pressure reading

boolean run = false;
boolean log = true;
boolean print = false;
boolean stringComplete = false;
int index = 0;
char message [] = "";
SoftwareSerial serial = SoftwareSerial(0,1); //put in correct RX and TX pins
MuxShield muxShield;
Sd2Card card;
const int chipSelect = 10; //CS pin (SD Breakout Board)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature thermocouples(&oneWire);
DeviceAddress thermo1, thermo2, thermo3, thermo4;
double tempC; //thermocouple reading

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,10,10,1, DIRECT);
Setpoint = 370.0; //PID heater Setpoint-- temp Celsius
 
time_t t = now();
int timeSeconds = second(t);

void setup() {
  serial.begin(9600); //printing to device
  Serial.begin(9600); //printing to serial monitor
  setupMuxShield();
  initializeSDCard();
  setupThermocouples();
  setupStepper(); //turns stepper on
  setupPID();
  pinMode (vctrl, INPUT);
 	pinMode (impulse, OUTPUT);

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
    t=now();
    if (second (t) >= (timeSeconds +1))
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
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  else
    {
      Serial.println ("Initialization complete.");
    }
}
void setupThermocouples()
{
 thermocouples.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if (!sensors.getAddress(thermo1, 1)) Serial.println("Unable to find address for Device 1");
  if (!sensors.getAddress(thermo2, 2)) Serial.println("Unable to find address for Device 2");
  if (!sensors.getAddress(thermo3, 3)) Serial.println("Unable to find address for Device 3");
  if (!sensors.getAddress(thermo4, 4)) Serial.println("Unable to find address for Device 4");
  
  // set the resolution to 9 bit
  sensors.setResolution(thermo1, TEMPERATURE_PRECISION);
  sensors.setResolution(thermo2, TEMPERATURE_PRECISION);
  sensors.setResolution(thermo3, TEMPERATURE_PRECISION);
  sensors.setResolution(thermo4, TEMPERATURE_PRECISION);
  
  //call thermocouples.requestTemperatures() to issue a global temperature 
  thermocouples.requestTemperatures();

}
void setupStepper ()
{
  pinMode(mdir,OUTPUT);
  digitalWrite(mdir,LOW); // correct direction is LOW
  analogWrite(msp,75); //turn stepper motor on
}

void setupPID()
{
  tempC = thermocouples.getTempC (thermo1);
  Input = tempC;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void parseMessage ()
//parses message from serial input
{
  char *p = message; //the message to be parsed
  char *str;
  int count = 0;
  char inParse []= "";
  
  while ((str=strtok_r(p, "_", &p)) != NULL)
  {
    inParse [count] = *str;
    count++;
  }
 while ((str=strtok_r(p, " ", &p)) != NULL)
  {
    inParse [count] = *str;
    count++;
  }
  if (inParse [0] != 'ALRX') 
  {
    Serial.print ("Received unknown command: ");
      for (int i=0;i<sizeof(inParse);i++)
      {
        Serial.print(inParse[i]);
      }
      Serial.println();
  }
switch (inParse[1])
{
case 'START' :
       run ==true;
       break;
case 'STOP' :
       run==false;
       break;
case 'STATUS':
       switch (inParse[count])
       {
        }
 
}

//add full list of commands 
}

void sendStatus ()
//sends status report to serial output (ALRX_STATUS 1111 for overall status OK, reaction is ON, H2 side is OK, O2 side is OK)
{
  String statusRep= "ALRX_STATUS ";
  int overallStat = 0;
  int reactionStat = 0;
  int H2Stat = 0;
  int O2Stat = 0;
  

if (checkOverallStat())
overallStat=1;

if (checkReactionStat())
reactionStat=1;

if (checkH2Stat())
H2Stat=1;

if (checkO2Stat())
O2Stat=1;

 statusRep = statusRep.concat(overallStat); 
 statusRep = statusRep.concat(reactionStat);
 statusRep = statusRep.concat(H2Stat);
 statusRep = statusRep.concat(O2Stat);
 
 serial.println (statusRep);
}

boolean checkOverallStat ()
//checks overall status of system
{
}

boolean checkReactionStat ()
//checks status of reaction chamber
{
}

boolean checkH2Stat ()
//checks status of hydrogen system
{
}

boolean checkO2Stat ()
//checks status of oxygen system
{
}

void reactionChamber ()
{
   //pressure sensor 
   //thermocouple 
}
void hydrogenSystem ()
//AST pressure sensor, Sensirion mass flow meter, KNF liquid pump
{
double RPM = (digitalRead (impulse)) / 6;
	Serial.print ("Water Pump RPM: ");
	Serial.println (RPM);

	double v = (analogRead (H2PressurePin) / 1023.0) * 5;
	if (v == 1)
		H2Pressure = 0;
	else
		H2Pressure = (v * 74.402) - 74.2;

	Serial.print ("H2 Pressure: ");
	Serial.println (H2Pressure);
	delay (1000);

	if (H2Pressure >= P_HI)
	{
//turn off pump
		digitalWrite (vctrl, LOW);
	}
	else if (H2Pressure <= P_LO)
	{
//turn on pump
		digitalWrite (vctrl, HIGH);
	}

}

void oxygenSystem ()
  //rope heater, LuminOx oxygen sensor, stepper motor/driver
{
  tempC = thermocouples.getTempC (thermo1);
  Input = tempC;
  Serial.print ("Heater temp (C): ");
  Serial.println (Input);
  myPID.Compute();
  Serial.print ("PID Output: ");
  Serial.println (Output);
  analogWrite(6,Output);
  
}
