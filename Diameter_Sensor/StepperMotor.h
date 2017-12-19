/*
  StepperMotor.h - Library for controlling the outfeed Stepper
  . The motor is driven using a fast PWM signal
  (Timer1). PWM is used because angular velocity
  is more important than angular position the PWM signal works well
  and frees up the processor for other jobs.

  Created by Matthew P. Rogge, Jan 10, 2017.
  Released into the public domain.
*/

#ifndef StepperMotor_h
#define StepperMotor_h

#include "Arduino.h"
#include "FastPWM.h"


class StepperMotor
{
  public:

    StepperMotor();//constructor
    void setRPM(double rpm);
    //float getRPM();
    void disable();
    void enable();
    void setDirection();
    float getFrequency();
    double _rpm;


  
  private:
    FastPWM _timer;
    float _stepsPerRev = 200 *16;
    boolean enabled;
};

#endif
