/*
  StepperMotor.cpp - Library for controlling the outfeed Stepper
  . The motor is driven using a fast PWM signal
  (Timer1). PWM is used because angular velocity
  is more important than angular position the PWM signal works well
  and frees up the processor for other jobs.

  Created by Matthew P. Rogge, Jan 10, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "StepperMotor.h"


StepperMotor::StepperMotor() : _timer()
{

  DDRB |= B00001110; //set the Step, direction and enable pins to output
  disable(); // Make sure that the steppers begin disabled
}

float StepperMotor::getFrequency() {

  //return _timer.getFrequency();
  return _timer._frequency;
}

void StepperMotor::setRPM(double RPM) {
  _rpm = RPM;
  float freq = RPM / 60.0 * _stepsPerRev;
  _timer.setFrequency(freq);
  if (!enabled && _rpm > 0.0){
    enable();
  }else if (_rpm == 0.0){
    disable();
  }
}

//float StepperMotor::getRPM() {
  //return _rpm;
//}


//The following should be done before Setting the Spool RPM

//Make sure the spoolRPM is updated
//   calcSpoolRPM();

void StepperMotor::disable() {
  PORTB |= B00001000;
  enabled = false;
}


void StepperMotor::enable() {
  PORTB &= B11110111;
  enabled = true;
}


void StepperMotor::setDirection() {

}

