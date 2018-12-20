/*
  Timer.cpp - Library for setting up timers 1,
  which are used for generating the step signal for the
  stepper.

  Created by Matthew P. Rogge, Jan 10, 2016.
  Released into the public domain.
*/

#include "Arduino.h"
#include "FastPWM.h"


//setFrequency(float frequency) sets the PWM to the designated frequency
//and activates it.
void FastPWM::setFrequency(float frequency)
{
  //Store frequency in private variable
  _frequency = frequency;

  //Define Modes
  byte outModeA = 1; //Fast PWM toggles pin OC1A
  byte outModeB = 0 ;//Normal pin operation on OC1B
  byte captureMode = 0 ;//I think that this is used for interrupts


  int prescaler;

  //Timer 1

  byte mode = 15 ;//Fast PWM

  int prescalers[] = {1, 8, 64, 256, 1024};

  //Select Prescaler assumes using 16bit fast PWM
  for (int i = 0; i < 5; i++) {
    if (16000000 / (2 * frequency * *(prescalers + i)) < 65535) {
      prescaler = *(prescalers + i);
      break;
    }
  }

  byte clockMode = 0 ; // 0 means no clocking - the counter is frozen.
  switch (prescaler)
  {
    case 1: clockMode = 1 ; break ;
    case 8: clockMode = 2 ; break ;
    case 64: clockMode = 3 ; break ;
    case 256: clockMode = 4 ; break ;
    case 1024: clockMode = 5 ; break ;
    default:
      if (prescaler < 0)
        clockMode = 7 ; // external clock
  }


  TCCR1A = (outModeA << 6) | (outModeB << 4) | (mode & 3) ;
  TCCR1B = (captureMode << 6) | ((mode & 0xC) << 1) | clockMode ;
  OCR1A = (short)(16000000.0 / (2 * frequency * prescaler)); //OCR1A because signal will be on pin OC1A (pin 11)


}

//float FastPWM::getFrequency()
//{
  //return _frequency;
//}

void FastPWM::off()
{
  // for all timers, bits 6 and 7 designate the Compare Output Mode
  // 00 designates normal operation with OCnA disconnected
  TCCR1A = TCCR1A & B00111111;
}
