/*
  Heater.cpp - Library for using heaters.
*/

#include "Arduino.h"
#include "Heater.h"

Heater::Heater (int pin)
{
  pinMode (pin, OUTPUT);
  _pin = pin;
}

void Heater::turnOn()
//code to turn on heater
{ 
}

void Heater::turnOff()
//code to turn off heater
{
}
