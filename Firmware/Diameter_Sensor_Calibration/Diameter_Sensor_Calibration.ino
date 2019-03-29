//Diameter sensor UI by Matt Rogge August 25, 2016
//Based on example from http://playground.arduino.cc/Main/TSL1402R

//I've added three stars *** to many of the origional comments in to differentiate them from my own like this
//*** Comment from the origional tutorial code.

#include <Wire.h>
#define oldPins //define oldPins to use the old pin definitions


// Define various ADC prescaler:
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

#ifdef oldPins
//Old PIN DEFS
int CLKpin = 3;    // <-- Arduino pin delivering the clock pulses to pin 3 (CLK) of the TSL1402R
int SIpin = 2;     // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 & 10 of the TSL1402R
int AOpin1 = 1;    // <-- Arduino pin connected to pin 4 (analog output 1)of the TSL1402R
int AOpin2 = 2;    // <-- Arduino pin connected to pin 8 (analog output 2)of the TSL1402R
int stepPin = 9;    //pin 9 is OC1A and PB1
int dirPin = 10;    //pin D10 is OC1b and PB2 on the same timer as pin 9 but not used for PWM
int enablePin = 11; //pin D11 is PB3
int laserPin = 12; // <-- the pin that the laser is connected to.
#endif

#ifndef oldPins
//PIN DEFS
int CLKpin = 2;    // <-- Arduino pin delivering the clock pulses to pin 3 (CLK) of the TSL1402R NOTE: clockPuls() uses direct port manipulation.
int SIpin = 3;     // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 & 10 of the TSL1402R
int AOpin1 = 6;    // <-- Arduino pin connected to pin 4 (analog output 1)of the TSL1402R
int AOpin2 = 7;    // <-- Arduino pin connected to pin 8 (analog output 2)of the TSL1402R
int stepPin = 9;    //pin 9 is OC1A and PB1. direct port manipulatioin is used
int dirPin = 8;    //pin D10 is OC1b and PB2 on the same timer as pin 9 but not used for PWM
int enablePin = 10; //pin D11 is PB3
#endif

 int integrationTime = 30;//should be const after calibration
const int ADCDelay = 0;

//The following are values used for determining the diameter based on output from the sensor
int adcVals[256];// The array where the readout of the photodiodes is stored, as integers

//Jump detection
// The adc values seem to jump up and down by a somewhat fixed amount.
// They all jump together.
// Not sure why. Laser intensity? Something in the electronics?
// Maybe we need to put a capacitor some where..
// The software solution has been to discard the higher values and keep the low ones
// jumpDetected is true when an high value has been detected.
// pixel 0 is used to test if a jump has occurred
// see function detectJump()
boolean jumpDetected = false;

//timing and control
unsigned long now; //The time right now.

//sensorReadingTimeMicros is used to pause the loop in order to make sure each loop is as similar in length as possible.
//The pause is done just before integration starts.
//The time in microseconds is determined by the time needed to execute the loop in diameterSensor.ino
const int sensorReadingTimeMicros = 13000;

//If something was done other than just collect data, the loop will be different and the data is no good
//sendData
boolean sendData = false;
boolean skip = false;

void setup()
{

  // Initialize two Arduino pins as digital output:
  pinMode(CLKpin, OUTPUT);
  pinMode(SIpin, OUTPUT);
#ifdef oldPins
  pinMode(laserPin, OUTPUT);
#endif
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);



  // To set up the ADC, first remove bits set by Arduino library, then choose
  // a prescaler: PS_16, PS_32, PS_64 or PS_128:
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_32; // <-- Using PS_32 makes a single ADC conversion take ~30 us

  //Should the following be changed to set all unused pins to have internal pullup resters enabled?
  // Set all IO pins low:
  for ( int i = 0; i < 14; i++ )
  {
    digitalWrite(i, LOW);
  }

  //*** Next, assert default setting:
  analogReference(DEFAULT);

#ifdef oldPins
  //turn on the laser
  digitalWrite(laserPin, HIGH);
#endif

  //Begin serial communication
  Serial.begin(250000);
  Serial.println("STANDBY");


  // The following clears out the CCD I think.

  //*** Clock out any existing SI pulse through the ccd register:
  for (int i = 0; i < 260; i++)
  {
    ClockPulse();
  }

  //*** Create a new SI pulse and clock out that same SI pulse through the sensor register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);
  for (int i = 0; i < 260; i++)
  {
    ClockPulse();
  }
}

void loop() {

  //*** Stop the ongoing integration of light quanta from each photodiode by clocking in a SI pulse
  //*** into the sensors register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);

  //*** Next, read all 256 pixels in parallel. Store the result in the array. Each clock pulse
  //*** causes a new pixel to expose its value on the two outputs:
  for (int i = 0; i < 128; i++) {
    delayMicroseconds(ADCDelay);//*** <-- We add a delay to stabilize the AO output from the sensor
    //The ADC delay didn't seem to have much effect on stability that I could tell

    adcVals[i] = analogRead(AOpin1);
    adcVals[i + 128] = analogRead(AOpin2);
    ClockPulse();
  }

  //*** Next, stop the ongoing integration of light quanta from each photodiode by clocking in a
  //*** SI pulse:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);


  //*** Next, send the measurement stored in the array to host computer using serial (rs-232).
  //*** communication. This takes ~80 ms during which time no clock pulses reaches the sensor.
  //*** No integration is taking place during this time from the photodiodes as the integration
  //*** begins first after the 18th clock pulse after a SI pulse is inserted:
// The timing is not necessarily the same.
  //*** Put Control Here
  
  if (!skip) {
    detectJump();//always check if there is a jump to keep the average up to date. And to be ready.
  }

  long stps; //The number of steps to send to the stepper motor.

  //Check to see if a command has been recieved from Processing
  //If on has, act on it.
  if (Serial.available() > 0) {
    char key = (char)Serial.read();

    switch (key) {
      // 'h' is the command to move 1 microstep.
      // Only done during homing to try and get as clost to the starting point as possible.
      case 'h':
        PORTB = PORTB | B00000010;
        delayMicroseconds(5);
        PORTB = PORTB & B11111101;
        delayMicroseconds(5);
        skip = true;
        sendData = true;
        break;

      // 'm' is the small movement command.
      // It moves the stepper by 4 steps.
      // In the Processing program, this is the variable "delta".
      case 'm':
        for (int i = 0; i < 4; i++) {
          PORTB = PORTB | B00000010;
          delayMicroseconds(1000);
          PORTB = PORTB & B11111101;
          delayMicroseconds(1000);
        }
        skip = true;
        sendData = true;
        break;

      // 'M' is the big move command
      // It is immediately followed by the number of steps to take.
      case 'M':
        stps = Serial.parseInt();
        stepMotor(stps);
        Serial.println("M");
        skip = true;
        break;

      //unused cases that were used to adjust the integration time.
      //Could be used during calibration.

      case 'I':
        integrationTime += 10;
        Serial.println("I");
        break;

      case 'L':
        if (integrationTime >= 10) {
          integrationTime -= 10;
        }
        Serial.println("L");
        break;

      case 'F':
        digitalWrite(dirPin, LOW);//set the direction for collecting data
        Serial.println('F');
        skip = true;
        break;

      case 'R':
        digitalWrite(dirPin, HIGH);//set the direction for collecting data
        Serial.println("R");
        skip = true;
        break;


      case 'D':
        sendData = true;
        break;

      default:
        Serial.println(key);
        break;
    }

    //Clear out the serial buffer
    //
    while (Serial.available()) {
      Serial.read();
    }
  }

  // If this this cycle was skipped, unset the skip flag
  // If it wasn't skiped, there may be data to send. If so, send it.
  if (skip) {
    skip = false;
  } else if (sendData) {
    sendCCDData();
  }

  while (micros() - now < sensorReadingTimeMicros) {}

  now = micros();

  // Next, a new measuring cycle is starting once 18 clock pulses have passed. At
  // that time, the photodiodes are once again active. We clock out the SI pulse through
  // the 256 bit register in order to be ready to halt the ongoing measurement at our will
  // (by clocking in a new SI pulse):

  for (int i = 0; i < 260; i++)
  {
    if (i == 18)
    {
      // Now the photodiodes goes active..
      // An external trigger can be placed here
    }

    ClockPulse();
  }

  // The integration time of the current program / measurement cycle is ~3ms. If a larger time
  // of integration is wanted, uncomment the next line:
  delayMicroseconds(integrationTime);// <-- Add integration time in microseconds
}

// This function generates an outgoing clock pulse from the Arduino digital pin 'CLKpin'. This clock
// pulse is fed into pin 3 of the linear sensor:
void ClockPulse()
{
  //Using direct port manipulation to get clock as fast as possible.
#ifndef oldPins
  PORTD = PORTD | B00000100;
  PORTD = PORTD & B11111011;
#endif

#ifdef oldPins
    //Old pin Def
    PORTD = PORTD | B00001000;
    PORTD = PORTD & B11110111;
#endif
}



void stepMotor(long steps) {
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    if (steps > 1) {
      delayMicroseconds(300);
    }
  }
}

//sendCCDData() sends the data to Processing via serial.
// The 'D' indicates that
void sendCCDData() {
  skip = true;

  if (jumpDetected) {
    //    Serial.print("getting more data: ");
    //    Serial.println(average);
    return; // don't bother prepping the data, just get another set of data
  }

  sendData = false;//reset flag
  Serial.print('D');
  Serial.print(' ');

  //Shift data on certain pixels here.
  //adcVals[129] = adcVals[129] * 1.76600441501104-53.6203090507726;


  //Send all sensor Data
  for (int i = 0; i < 256; i++) {//256; i++) {
    Serial.print(adcVals[i]);
    Serial.print(" ");
  }
  Serial.println();// <-- Send a linebreak to indicate the measurement is transmitted.
}

//detectJump() detects the mysterious jump in intensity that is seen.
//it keeps an array of the 10 most recent measurements for pixel 1.
//A rolling average is made from this array.
//If the current measurement is higher than the average, then a jump is detected.
void detectJump() {
   //return; //Uncomment this line to skip jump detection
  static int index = 0;
  static float boxCar[10];
  static int sum;
  static float average;
  int val = adcVals[1];
  sum += val;
  sum -= boxCar[index];

  average = sum / 10.0;
  boxCar[index] = val;
  index++;
  if (index == 10) {
    index = 0;
  }
  if (val > average) {
    jumpDetected = true; 
  } else {
    jumpDetected = false;
  }
}




