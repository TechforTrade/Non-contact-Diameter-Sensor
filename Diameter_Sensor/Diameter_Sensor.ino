//Diameter sensor UI by Matt Rogge August 25, 2016
//Based on example from http://playground.arduino.cc/Main/TSL1402R

/*


*/


#include <Wire.h>
#include <avr/pgmspace.h>
#include "LeftMap.h"
#include "RightMap.h"
#include "DiameterCalcs.h"
#include "PID_v1.h"
#include "FastPWM.h"
#include "StepperMotor.h"

#define DIAMETER 0
#define RPM 4
#define DIA_SETPOINT 8
#define MODE 12
#define SAMPLE_TIME 13
#define KP 15
#define KI 19
#define KD 23
#define OUTPUT_MIN 27
#define OUTPUT_MAX 31
#define LASER_ON 35
#define VERSION 36

#define SLAVE_ADDRESS 4
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

// Define various ADC prescaler:
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//PIN DEFS
const int CLKpin = 3;    // <-- Arduino pin delivering the clock pulses to pin 3 (CLK) of the TSL1402R
const int SIpin = 2;     // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 & 10 of the TSL1402R
const int AOpin1 = 1;    // <-- Arduino pin connected to pin 4 (analog output 1)of the TSL1402R
const int AOpin2 = 2;    // <-- Arduino pin connected to pin 8 (analog output 2)of the TSL1402R
const int laserPin = 12; // <-- the pin that the laser is connected to.
const int stepPin = 9;    //pin 9 is OC1A and PB1
const int dirPin = 10;    //pin D10 is OC1b and PB2 on the same timer as pin 9 but not used for PWM
const int enablePin = 11; //pin D11 is PB3
const int waterSensorPin = A6; //The pin used to detect water in the casing.

// Water sensor constants
const int waterPresentThreshold = 800; //The ADC value below which water is assumed to be in the case.
boolean waterPresent = false;

//Constants used for diameter sensor
const int integrationTime = 30;//should be determined during calibration set as low as possible
const int ADCDelay = 20;

//Stepper Related Constants
const int stepsPerRev = 200 * 16;

//Jump detection
boolean jumpDetected = false;

//timing and control
unsigned long now;
unsigned long nowMillis;
unsigned long previous;
unsigned long computeTime;
const int sensorReadingTimeMicros = 13000; //This and the following should just be combined.
const int sensorReadingTimeMillis = 13;

boolean sendData = false;
boolean skip = false;

//Diameter averaging
float diaSum = 0.0;
int numDiameters = 0;

typedef struct RegisterMap_S {
  double diameter; //byte 0
  double rpm;//byte 4 //disable stepper if RPM = 0
  double diaSetpoint; //byte 8
  byte mode; //byte 12
  int sampleTime; //byte 13
  double kp;//byte 15
  double ki; //byte 19
  double kd; //byte 23
  double outputMin; //byte 27
  double outputMax; //byte 31
  boolean laserOn;//byte 35
  char version[8]; //byte 36
};

typedef union RegisterMap_u {
  RegisterMap_S s;
  byte b[sizeof(RegisterMap_S)];
};

RegisterMap_u registerMap;//The register map which is a union of a byte array and a struct

byte receivedCommands[5];//enough space to hold data to write to one register

volatile boolean changeMade = true;//A write instruction has been received. Check flags should be called.
volatile boolean updateRPM = true;
volatile boolean updateMode = true;
volatile boolean updateSampleTime = true;
volatile boolean updateTunings = true;
volatile boolean updateLimits = true;
volatile boolean updateLaser = true;
volatile boolean updateDiaSetpoint = true;

//testing
volatile boolean requestMade = false;
volatile byte requestAddress;

//Stepper and PID setup
StepperMotor outfeedMotor;
PID pid(&registerMap.s.diameter, &registerMap.s.rpm, &registerMap.s.diaSetpoint, registerMap.s.kp,  registerMap.s.ki,  registerMap.s.kd, REVERSE);

void setup()
{
  //Load register Map with default values
  registerMap.s.laserOn = true;
  registerMap.s.rpm = 0.0;
  registerMap.s.mode = MANUAL;
  registerMap.s.diaSetpoint = 1.61;
  registerMap.s.sampleTime = 100;
  registerMap.s.kp = 10.0;
  registerMap.s.ki = 0.001;
  registerMap.s.kd = 0.0;
  registerMap.s.outputMin = 0.0; //byte 38
  registerMap.s.outputMax = 100.0; //byte 42
  registerMap.b[VERSION] = 'V';
  registerMap.b[VERSION + 1] = '1';
  registerMap.b[VERSION + 2] = '.';
  registerMap.b[VERSION + 3] = '0';
  registerMap.b[VERSION + 4] = ' ';
  registerMap.b[VERSION + 5] = ' ';
  registerMap.b[VERSION + 6] = ' ';
  registerMap.b[VERSION + 7] = ' ';



  // Initialize two Arduino pins as digital output:
  pinMode(CLKpin, OUTPUT);
  pinMode(SIpin, OUTPUT);
  pinMode(laserPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);


  // To set up the ADC, first remove bits set by Arduino library, then choose
  // a prescaler: PS_16, PS_32, PS_64 or PS_128:
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_32; // <-- Using PS_32 makes a single ADC conversion take ~30 us
  //// set prescaler to 16
  //sbi(ADCSRA,ADPS2) ;
  //cbi(ADCSRA,ADPS1) ;
  //cbi(ADCSRA,ADPS0) ;

  // Next, assert default setting:
  analogReference(DEFAULT);

  // Set all IO pins low:
  for ( int i = 0; i < 14; i++ )
  {
    digitalWrite(i, LOW);
  }


  digitalWrite(laserPin, HIGH);//turn on the laser
 //Serial.begin(250000);
 //Serial.println("STANDBY");
  //  currentState = STANDBY;

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

  checkFlags();

}

void loop() {

  // Stop the ongoing integration of light quanta from each photodiode by clocking in a SI pulse
  // into the sensors register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);

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
  detectJump();
  if (skip) {
    skip = false;
  } else {
    if (jumpDetected) {
      skip = false;
    } else {
      diaSum += getDia();
      numDiameters++;
    }
    skip = true;
  }

  //The following is commented out for calibration
  //  //Have the PID compute. It returns true if it actually computes.
  //  //If true, set the RPM with the new rpm.




  if (requestMade) {
    requestMade = false;
    reportRequest();
  }

  //  while (micros() - now < 11000L) {}
  while (micros() - now < sensorReadingTimeMicros) {}//wait so that each sensor reading cycle takes the same amount of time. value should be just longer than longest time to read sensor
  //  Serial.println(micros()-now);
  now = micros();
  nowMillis = millis();

  //Check to see if it is with in one sensor reading (durration) of time to run PID calculate.
  //If so, just wait until it is time to calculate.
  if (nowMillis + sensorReadingTimeMillis > computeTime) { //Possibly put a while here to pause until just the correct time.
    while (nowMillis < computeTime) nowMillis = millis();
    computeTime = nowMillis + registerMap.s.sampleTime;
    if (!waterPresent) {
      if (numDiameters > 0) {
        registerMap.s.diameter = diaSum / numDiameters;
        if (pid.Compute()) {
          outfeedMotor.setRPM(registerMap.s.rpm);
        }
      }
      //    Serial.println(registerMap.s.diameter,4);
      //    Serial.println(registerMap.s.rpm,4);
      //    Serial.println(numDiameters);
      //    Serial.println();

      diaSum = 0.0;
      numDiameters = 0;
      //check flags and take necessary actions;
      if (changeMade)checkFlags();
    } else {
      //water has been detected so just keep sending the water detected value
      registerMap.s.diameter = 6666;
    }

    //Check for water infiltration
    if (analogRead(A6) < 800 && !waterPresent) {
      //Water has infiltrated.
      waterShutdown();
    }
    skip = true;
  }
  //      Serial.println(micros() - now);
  //      Serial.println();
  //    Serial.print("\t");
  //    Serial.println(adcVals[1]);
  //  Serial.println(micros() - now);


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
  PORTD = PORTD | B00001000;
  PORTD = PORTD & B11110111;
}



void  linReg(float* slope, float* intercept, int Vals[][2]) {
  //Variables
  int n;
  float sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0, sumYY = 0.0;
  float sXY, sXX;

  //Determine n
  for (int i = 0; i < 8; i++) {
    n = i;
    if (Vals[i][0] == 0) {
      break;
    }
  }

  //Find Sums
  for (int i = 0; i < n; i++) {
    sumX = sumX + float(Vals[i][0]);
    sumY = sumY + float(Vals[i][1]);
    sumXY = sumXY + float(Vals[i][0]) * float(Vals[i][1]);
    sumXX = sumXX + float(Vals[i][0]) * float(Vals[i][0]);
    sumYY = sumYY + float(Vals[i][1]) * float(Vals[i][1]);
  }

  //Compute slope and intercept
  sXY = sumXY - sumX * sumY / float(n);
  sXX = sumXX - sumX * sumX / float(n);
  *slope = sXY / sXX;
  *intercept = sumY / float(n) - *slope * (sumX / float(n));
}


void reportRequest() {
 //Serial.print("Requested Address: ");
 //Serial.println(requestAddress);
 //Serial.print("Dia: ");
 //Serial.println(registerMap.s.diameter, 4);
}

void requestEvent() {
  Wire.write(registerMap.b + receivedCommands[0], MAX_ENTRY_SIZE);
  requestMade = true;
  requestAddress = receivedCommands[0];
}

void receiveEvent(int bytesReceived) {
  int bytesKept = 0;

  //Load all possible bytes into receivedCommands and throw away the rest
  for (int i = 0; i < bytesReceived; i++) {
    if (i < MAX_SENT_BYTES) {
      receivedCommands[i] = Wire.read();
      bytesKept++;
    } else {
      Wire.read();
    }
  }

  //If only one byte was received and it is smaller than  REG_MAP_SIZE,
  // the master is probably going to read data, in which case just return.
  if (bytesReceived == 1 && receivedCommands[0] < REG_MAP_SIZE) {
    return;
  }

  //If the one byte is larger than the REG_MAP_SIZE set the pointer to 0
  //and return
  if (bytesReceived == 1 && receivedCommands[0] >= REG_MAP_SIZE) {
    receivedCommands[0] = 0;
    return;
  }
  //if the data falls within the writable range, write it to the register.
  if (receivedCommands[0] >= MIN_WRITE_INDEX && receivedCommands[0] <= MAX_WRITE_INDEX) {
    for (int i = 0; i < bytesKept - 1; i++) {
      registerMap.b[receivedCommands[0] + i] = receivedCommands[i + 1];
    }
  }
  //set flags for changes that require action
  switch (receivedCommands[0]) {
    //double diameter; //byte 0
    //double rpm;//byte 4 //disable stepper if RPM = 0
    //double diaSetpoint; //byte 8
    //byte mode; //byte 12
    //int sampleTime; //byte 13
    //double kp;//byte 15
    //double ki; //byte 19
    //double kd; //byte 23
    //double outputMin; //byte 27
    //double outputMax; //byte 31
    //boolean laserOn;//byte 35
    //char version[8]; //byte 36

    case 4:
      updateRPM = true;
      break;

    case 8:
      updateDiaSetpoint = true;
      break;

    case 12:
      updateMode = true;
      break;

    case 13:
      updateSampleTime = true;
      break;

    case 15:
    case 19:
    case 23:
      updateTunings = true;
      break;

    case 27:
    case 31:
      updateLimits = true;
      break;
    case 35:
      updateLaser = true;
      break;
  }
  changeMade = true;
}

void checkFlags() {
  changeMade = false;

  if (updateLaser) {
    if (registerMap.s.laserOn) {
      digitalWrite(laserPin, HIGH);
     //Serial.println("Laser ON");
    } else {
      digitalWrite(laserPin, LOW);
     //Serial.println("Laser OFF");
    }
    updateLaser = false;
  }

  if (updateRPM) {
    outfeedMotor.setRPM(registerMap.s.rpm);
   //Serial.print("RPM Set to ");
   //Serial.println(registerMap.s.rpm);
    updateRPM = false;
  }

  if (updateDiaSetpoint) {
   //Serial.print("diaSetpoint Set to ");
   //Serial.println(registerMap.s.diaSetpoint);
    updateDiaSetpoint = false;
  }

  if (updateMode) {
    pid.SetMode(registerMap.s.mode);
   //Serial.print("Mode Set to ");
   //Serial.println(registerMap.s.mode);
    updateMode = false;
  }

  if (updateSampleTime) {
    pid.SetSampleTime(registerMap.s.sampleTime);
   //Serial.print("Sample time set to ");
   //Serial.println(registerMap.s.sampleTime);
    updateSampleTime = false;
  }

  if (updateTunings) {
    pid.SetTunings(registerMap.s.kp, registerMap.s.ki, registerMap.s.kd);
   //Serial.print("PID tunings set to: Kp = ");
   //Serial.print(registerMap.s.kp);
   //Serial.print("   Ki = ");
   //Serial.print(registerMap.s.ki);
   //Serial.print("   Kd = ");
   //Serial.println(registerMap.s.kd);
    updateTunings = false;
  }

  if (updateLimits) {
    pid.SetOutputLimits(registerMap.s.outputMin, registerMap.s.outputMax);
   //Serial.print("Output limits set to: min = ");
   //Serial.print(registerMap.s.outputMin);
   //Serial.print("   max = ");
   //Serial.println(registerMap.s.outputMax);
    updateLimits = false;
  }
}

void stepMotor(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
  }
}



boolean detectJump() {
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

void waterShutdown() {
  outfeedMotor.setRPM(0);//Shut down motor
  digitalWrite(laserPin, LOW);//turn off laser
  waterPresent = true;
}

