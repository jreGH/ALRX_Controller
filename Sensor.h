/*
Sensor.h - Library for using sensors.
*/

#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include <SD.h>

class Sensor
{
  public:
    Sensor (int pin, String type);
    int read();
    void log();
    void print();
    boolean hitMax(int maxValue);
    //void stopFlow();
    //void turnValve();
    int reading;
    File dataFile;
    
   private:
     int _pin;
     String _type;
};

#endif
