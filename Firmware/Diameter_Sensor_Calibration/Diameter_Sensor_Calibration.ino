//Diameter sensor UI by Matt Rogge August 25, 2016
//Based on example from http://playground.arduino.cc/Main/TSL1402R

/*
      Thoughts
      When the master Arduino requests the diameter, if a diameter
      is currently being measured, it will have to be neglected.
      There should be a flag set during reading so if an interrupt
      is received during this time, the measurement is ignored

      This could be set up with the proper preprocessor directives
      to make it configurable to work with an I2C LCD that would just
      display the current measurement.
*/


#include <Wire.h>



#define REG_MAP_SIZE 47
#define MAX_ENTRY_SIZE 8 //The largest size in bytes of any registerMap entry
#define MAX_SENT_BYTES 5
#define MIN_WRITE_INDEX 4 //The highest index at which writing can be initiated in the register map
#define MAX_WRITE_INDEX 42 //The highest index at which writing can be initiated in the register map
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define CALIBRATE


// Define various ADC prescaler:
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//PIN DEFS
int CLKpin = 2;    // <-- Arduino pin delivering the clock pulses to pin 3 (CLK) of the TSL1402R NOTE: clockPuls() uses direct port manipulation.
int SIpin = 3;     // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 & 10 of the TSL1402R
int AOpin1 = 6;    // <-- Arduino pin connected to pin 4 (analog output 1)of the TSL1402R
int AOpin2 = 7;    // <-- Arduino pin connected to pin 8 (analog output 2)of the TSL1402R
int stepPin = 9;    //pin 9 is OC1A and PB1. direct port manipulatioin is used
int dirPin = 8;    //pin D10 is OC1b and PB2 on the same timer as pin 9 but not used for PWM
int enablePin = 10; //pin D11 is PB3
//int laserPin = 20; // <-- the pin that the laser is connected to.


const int integrationTime = 30;//should be const after calibration
const int ADCDelay = 0;
const int leftMargin = 10;//The number of pixels to omit from the left side of the sensor. Sometimes the pixels at the edge of the sensor don't read correctly.
const int rightMargin = 10;//The number of pixels to omit from the right side of the sensor. Sometimes the pixels at the edge of the sensor don't read correctly.

//The following are values used for determining the diameter based on output from the sensor
int adcVals[256];// The array where the readout of the photodiodes is stored, as integers


//Globals used for calculating/reporting filament diameter
const float pixelPitch = 0.0635; //the pitch of the CCD array
const int leftGap = 5;//The number of pixels to omit from the sensor. Some times the pixels at the edge of the sensor don't read correctly.
const int rightGap = 235;//The number of pixels to omit from the right side of the sensor.
const int minBackgroundValue = 350;
const int jumpIndex = 0;

float dia; //shouldn't b needed
float actDia; //should be return value
float lPos; //should be local
float rPos;//should be local
float leftX; //The x position of the left side of the shadow as calculated using the line and the y offset.
float rightX;//The x position of the right side of the shadow as calculated using the line and the y offset.
int lIndex;//should be local
int rIndex;//should be local
float lValue;//should be local
float rValue;//should be local

//Jump detection
boolean jumpDetected = false;

//timing and control
unsigned long now;
unsigned long previous;
const int sensorReadingTimeMicros = 13000; //This and the following should just be combined.
boolean sendData = false;
boolean skip = false;


void setup()
{

  // Initialize two Arduino pins as digital output:
  pinMode(CLKpin, OUTPUT);
  pinMode(SIpin, OUTPUT);
//  pinMode(laserPin, OUTPUT);

  //Pins for coordinating with 3D printer during calibration
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);



  // To set up the ADC, first remove bits set by Arduino library, then choose
  // a prescaler: PS_16, PS_32, PS_64 or PS_128:
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_32; // <-- Using PS_32 makes a single ADC conversion take ~30 us
  //// set prescaler to 16
  //sbi(ADCSRA,ADPS2) ;
  //cbi(ADCSRA,ADPS1) ;
  //cbi(ADCSRA,ADPS0) ;



  // Set all IO pins low:
  for ( int i = 0; i < 14; i++ )
  {
    digitalWrite(i, LOW);
  }

  // Next, assert default setting:
  analogReference(DEFAULT);
//  digitalWrite(laserPin, HIGH);//turn on the laser
  Serial.begin(250000);
  Serial.println("STANDBY");

  // Clock out any existing SI pulse through the ccd register:
  for (int i = 0; i < 260; i++)
  {
    ClockPulse();
  }

  // Create a new SI pulse and clock out that same SI pulse through the sensor register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);
  for (int i = 0; i < 260; i++)
  {
    ClockPulse();
  }
}

void loop() {

  // Stop the ongoing integration of light quanta from each photodiode by clocking in a SI pulse
  // into the sensors register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);
  //  digitalWrite(laserPin, LOW);
  // Next, read all 256 pixels in parallel. Store the result in the array. Each clock pulse
  // causes a new pixel to expose its value on the two outputs:
  for (int i = 0; i < 128; i++) {
    delayMicroseconds(ADCDelay);// <-- We add a delay to stabilize the AO output from the sensor //Didn't seem to have much effect on stability
    adcVals[i] = analogRead(AOpin1);
    adcVals[i + 128] = analogRead(AOpin2);
    ClockPulse();
  }

  // Next, stop the ongoing integration of light quanta from each photodiode by clocking in a
  // SI pulse:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);
  // Next, send the measurement stored in the array to host computer using serial (rs-232).
  // communication. This takes ~80 ms during which time no clock pulses reaches the sensor.
  // No integration is taking place during this time from the photodiodes as the integration
  // begins first after the 18th clock pulse after a SI pulse is inserted:

  //Put Control Here
  if (!skip) {
    detectJump();//always check if there is a jump to keep the average up to date. And to be ready.
  }
  long stps;
  if (Serial.available() > 0) {
    char key = (char)Serial.read();

    switch (key) {
      case 'h':
        PORTB = PORTB | B00000010;
        delayMicroseconds(5);
        PORTB = PORTB & B11111101;
        delayMicroseconds(5);
        skip = true;
        sendData = true;
        break;

      case 'm':
        for (int i = 0; i < 4; i++) {
          PORTB = PORTB | B00000010;
          delayMicroseconds(1000);
          PORTB = PORTB & B11111101;
          delayMicroseconds(1000);
        }
        //        Serial.println("M");
        skip = true;
        sendData = true;
        break;

      case 'M':
        stps = Serial.parseInt();
        stepMotor(stps);
        Serial.println("M");
        skip = true;
        break;

      //      case 'I':
      //        integrationTime += 10;
      //        Serial.println("I");
      //        break;
      //
      //      case 'L':
      //        if (integrationTime >= 10) {
      //          integrationTime -= 10;
      //        }
      //        Serial.println("L");
      //        break;

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
    while (Serial.available()) {
      Serial.read();
    }
  }


  //  if ((millis() - previous) > 50) {
  //    sendData = true;
  //    previous = millis();
  ////    Serial.println("Trying to send data");
  //  }
  if (skip) {
    //    Serial.print(micros() - now);
    //    Serial.println();

    skip = false;
  } else if (sendData) {
    prepData();
  }

  while (micros() - now < sensorReadingTimeMicros) {}
  //    Serial.print(micros() - now);
  //    Serial.print("\t");
  //    Serial.println(adcVals[1]);
  //  Serial.println(micros() - now);

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
      //      digitalWrite(laserPin, HIGH);
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
  PORTD = PORTD | B00000100;
  PORTD = PORTD & B11111011;
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


void prepData() {
  skip = true;

  if (jumpDetected) {
    //    Serial.print("getting more data: ");
    //    Serial.println(average);
    return; //get another set of data
  }

  sendData = false;//reset flag
  Serial.print('D');
  Serial.print(' ');

  //Shift data on certain pixels here.
adcVals[129] = adcVals[129] * 1.76600441501104-53.6203090507726;
adcVals[130] = adcVals[130] * 1.64609053497942-45.2263374485597;
adcVals[131] = adcVals[131] * 1.51228733459357-35.8601134215501;
adcVals[132] = adcVals[132] * 1.78173719376392-54.7216035634744;
adcVals[133] = adcVals[133] * 1.67014613778706-46.9102296450939;
adcVals[134] = adcVals[134] * 1.78970917225951-55.2796420581655;
adcVals[138] = adcVals[138] * 1.66320166320166-46.4241164241164;


  //Send all sensor Data
  for (int i = 0; i < 256; i++) {//256; i++) {
    Serial.print(adcVals[i]);
    Serial.print(" ");
  }
  Serial.println();// <-- Send a linebreak to indicate the measurement is transmitted.
}


void detectJump() {
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




