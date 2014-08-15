
#include <Time.h>
#include <SoftwareSerial.h>
#include <String.h>
#include <SD.h>
#include <SPI.h>
#include <ctype.h>
#include <MuxShield.h>
#include <OneWire.h> //for thermocouple (needed for DallasTemperature)
#include <DallasTemperature.h> //for thermocouple
#include <PID_v1.h> //for heater control
#include <Wire.h>
//NOTE:  Need to edit this library due to conflict with Time.h
//The function dayOfWeek() needs to be renamed (I used dayOfWeek_rtc())
#include <RTClib.h> 

#define I2C_SDAPIN 4
#define I2C_SCLPIN 5
const double H2CAL = 1.0; 
double pressure = 1.0; //put in correct value here
double H2_sccm;
char msg[120];

#define _SFM4100_I2C_ADDR_ 0x01 //I2C address!
#define _SFM4100_DATAREQ_ 0xF1
#define CRC_POLYNOMIAL 0x131

#define mdir 4 //NEED TO SET PIN! On MUX// motor direction
#define msp 5 // motor speed 62.5
#define RelayPin 10 //connected to MOSFET G

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 9

#define _H2O_PUMP_VCTRL_ 9 
#define _H2O_PUMP_IMPULSE_ 6 //NEED TO SET PIN! On MUX //impulse generator 6 pulse/mech.rotation (for pump)

#define MAX_STRLEN 130


#define H2PressurePin A0 //(for AST pressure sensor)
#define P_HI 36 //H2 max pressure (psi)
#define P_LO 8 //H2 min pressure (psi)
double H2Pressure; //AST pressure reading

boolean run = false;
boolean logData = true;
boolean print = false;
boolean stringComplete = false;
int index = 0;
char message[MAX_STRLEN]; //ZB:  try to reuse this buffer for all string variables

SoftwareSerial serial = SoftwareSerial(0,1); //put in correct RX and TX pins
MuxShield muxShield;
Sd2Card card;
const int chipSelect = 8; //CS pin (SD Breakout Board)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature thermocouples(&oneWire);
DeviceAddress thermo1, thermo2, thermo3, thermo4;
double tempC; //thermocouple reading

double Input, Output;

double Setpoint = 370.0; //PID heater Setpoint-- temp Celsius

PID myPID(&Input, &Output, &Setpoint,10,10,1, DIRECT);

RTC_DS1307 rtc;
 
time_t t = now();
int timeSeconds = second(t);

void setup() {
  serial.begin(9600); //printing to device
  Serial.begin(9600); //printing to serial monitor
  setupMuxShield();
  initializeSDCard();
  setupClock();
  setupThermocouples();
  setupStepper(); //turns stepper on
  setupPID();
  pinMode (_H2O_PUMP_VCTRL_, INPUT);
  pinMode (_H2O_PUMP_IMPULSE_, OUTPUT);

}

void loop() {
  if (stringComplete){
    parseMessage();
    stringComplete = false;
    sprintf(message, "");
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
void setupClock()
{
Wire.begin();
if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

}

void logTime()
{
 DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

void setupThermocouples()
{
 thermocouples.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(thermocouples.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if (!thermocouples.getAddress(thermo1, 1)) Serial.println("Unable to find address for Device 1");
  if (!thermocouples.getAddress(thermo2, 2)) Serial.println("Unable to find address for Device 2");
  if (!thermocouples.getAddress(thermo3, 3)) Serial.println("Unable to find address for Device 3");
  if (!thermocouples.getAddress(thermo4, 4)) Serial.println("Unable to find address for Device 4");
  
  // set the resolution to 9 bit
  thermocouples.setResolution(thermo1, TEMPERATURE_PRECISION);
  thermocouples.setResolution(thermo2, TEMPERATURE_PRECISION);
  thermocouples.setResolution(thermo3, TEMPERATURE_PRECISION);
  thermocouples.setResolution(thermo4, TEMPERATURE_PRECISION);
  
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
  //Use the message buffer
  sprintf(message, "ALRX_STATUS ");

  int overallStat = 0;
  int reactionStat = 0;
  int H2Stat = 0;
  int O2Stat = 0;
  
  if (checkOverallStat()) {
    sprintf(message, "%s 1", message); 
  }
  else {
    sprintf(message, "%s 0", message);
  }

  if (checkReactionStat()) {
    sprintf(message, "%s1", message); 
  }
  else {
    sprintf(message, "%s0", message);
  }
  
  if (checkH2Stat()) {
    sprintf(message, "%s1", message); 
  }
  else {
    sprintf(message, "%s0", message);
  }
  
  if (checkO2Stat()) {
    sprintf(message, "%s1", message); 
  }
  else {
    sprintf(message, "%s0", message);
  }
  
 
 Serial.println (message);
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
 if (!readSFM4100()) {
    Serial.println("Failed to read SFM4100 data.");
  }
  else {
    sprintf (msg, "H2 flow = %d.%.2d", (int)H2_sccm, (int)((H2_sccm-(int)(H2_sccm))*100));
    Serial.println(msg);
    
  }

double RPM = (digitalRead (_H2O_PUMP_IMPULSE_)) / 6;
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
		digitalWrite (_H2O_PUMP_VCTRL_, LOW);
	}
	else if (H2Pressure <= P_LO)
	{
//turn on pump
		digitalWrite (_H2O_PUMP_VCTRL_, HIGH);
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
boolean readSFM4100() {
  uint8_t flowData[2];
  uint8_t checkSum;
  
  //Request a measurement
  Wire.beginTransmission(_SFM4100_I2C_ADDR_);
  
  //Request data
  Wire.write(_SFM4100_DATAREQ_);
  
  //Close the request
  Wire.endTransmission();
  
  //Now read a response
  Wire.requestFrom(_SFM4100_I2C_ADDR_, 3);
  flowData[0] = Wire.read();
  flowData[1] = Wire.read();
  checkSum = Wire.read();
    
  //Do the checksum
  if (checkCRC(flowData, 0x02, checkSum)) {
    int H2int = flowData[1] | (flowData[0] << 8);
    sensirionCal ((double)H2int, pressure);
  }
  else {
    Serial.println("Invalid data received from the SFM4100!");
    return false;
  }
  
  return true;
}

void sensirionCal(double measurement, double pressure)
{
//pressure is the measured gauge pressure (psi)
//Linear fit ([slope intercept]) to the data
double p[2] = {1.1523,-283.7969};
double p7[2] = {1.4488,-213.9459};
 H2_sccm= (measurement - p[1] - (p7[1]-p[1])/7 * pressure) / (p[0] + (p7[0] - p[0])/7 * pressure);
}

boolean checkCRC(uint8_t data[], uint8_t nBytes, uint8_t checkSum) {
  uint8_t crc = 0;
  uint8_t byteCount;
  
  for (byteCount=0;byteCount < nBytes;byteCount++) {
    crc ^= (data[byteCount]);
    
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      }
      else {
        crc = (crc << 1);
      }
    }
    
  }
  
  if (crc != checkSum) {
    return false;
  }
  
  return true;
}
