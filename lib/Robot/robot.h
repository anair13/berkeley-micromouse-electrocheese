/*
* Robot.h - Library for representing a robot
*/
#ifndef Robot_h
#define Robot_h

#include "Arduino.h"

class Robot
{
  public:
    Robot(int ix, int iy, int it);
    int x;
    int y;
    int t; // theta
};

#endif
