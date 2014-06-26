/*
Heater.h - Library for using heaters.
*/

#ifndef Heater_h
#define Heater_h

#include "Arduino.h"

class Heater
{
  public:
    Heater (int pin)
    void turnOn();
    void turnOff();
    
   private:
     int _pin;
};

#endif
