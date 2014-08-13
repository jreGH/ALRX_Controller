/*
  Sensor.cpp - Library for using sensors.
*/

#include "Arduino.h"
#include "Sensor.h"

Sensor::Sensor (int pin, String type)
{
  pinMode (pin, OUTPUT);
  _pin = pin;
  _type = type;
}

int Sensor::read()
//code to read sensor
{
  reading = analogRead (_pin);
  return reading;
}

void Sensor::log()
//code to log sensor reading to SD card
{  
  dataFile = SD.open ("datalog.txt", FILE_WRITE);
 if (dataFile){
   dataFile.println (_type +" reading: " + reading);
   dataFile.close();
  } 
 else
   Serial.println ("error opening datalog.txt");
}

void Sensor::print()
//prints reading to Serial monitor
{
  Serial.println (_type + " reading: " + reading);
}

boolean Sensor::hitMax (int maxValue)
//code to check if sensor reading has reached its max value
//returns true if sensor reading is greater than max value
{
  if (reading>maxValue)
    return true;
  else 
    return false;
}
