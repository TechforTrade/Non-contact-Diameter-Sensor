
//Diameter sensor UI by Matt Rogge August 25, 2016

//Diameter_Sensor_UI is used to visualize the data from the diameter sensor.
//It is also used to calibrate the diameter sensor.
//
//How it works:
//  Draw()
//  The draw function continuousily draws a graph (XYChart lineChart) of the dataY array
//  If it can find a shadow, it graphs the width of the shadow using the XYChart diaChart.
//  So draw is just constantly showing the most recent info it has for all of the variables.
//
//  serialEvent(Serial p)
//    serialEvent is called any time Processing receives serial info from the arduino
//    The arduino sends  two types of information to Processing:
//        characters that signify a type of event
//        Integers that are the ADC values recorded from the CCD array
//    serialEvent(Serial p) is works like the loop that calls the state functions.
//   
//  State Functions:
//    There are many different states for the finite state machine here.
//    Each state takes the serial data that Processing just recieved and decides what to do.
//    The current state will continue executing each time it is called by serialEvent until the state is changed.
//    Some state functions set a new state at some point in ther execution. 
//        E.G. beginHome() sets the state to HOME once the initial homing tasks have been done.
//    Some state functions do not specify a change to another state.
//    The only way that the program will enter another state is with user input.
//        E.g. standby() just updates the dataY array.
//        To switch to a different state, the user must press a key which triggers the keyPressed() function.
//        The keyPressed() function in turn may set a new state.
//    Many state functions send serial info to the arduino.
//        E.G.standby() sends a 'D' to the arduino telling it to send data.
//  Serial data character sent from the arduino (confirmation that something happened. the first character of every serial event):
//    M - A big move by the stepper has completed.
//    D - data sent
//    s - does nothing that I can tell
//    I - response from arduino that integration time was increased by 10us.
//    L - response from arduino that integration time was decreased by 10us
//    F - Stepper direction pin was set to low
//    R - Stepper direction pin was set to high
//
//  Serial data characters sent to the arduio. Typically a command to do something in the form of serial.write("D"):
//    D - send data
//    s - does nothing
//    I - increase integration time by 10us
//    L - decrease integration time by 10us
//    M - big stepper move. Must be followed by an integer number of steps e.g. M350
//    F - Set stepper direction pin to low
//    R - Set stepper direction pin to high
//    E - does nothing... used when background should be taken and filament is not in front of sensor.
//    h - Move one microstep
//    m - move 4 microsteps i.e. delta. Used during a scan.
//    W - does nothing
//
//  User input - Pressing the following keys triggers the keyPressed function:
//    h - home the calibration tool. The filament shadow must be detected befor starting the homing process.
//    s - switch to the STANDBY state
//    d - print raw data coming from the arduino to the console. 
//        It is also saved to rawData.txt, but you must press 'f' to save it.
//    b - collect background data. It is written to a file called background.txt
//    m - do a multi scan. I.e. scan the filament accross the sensor x times also collect background data.
//        data is saved in scan0.txt, scan1.txt etc.
//    1 - do one scan and also collect background.
//    i - increase integration time by 10 us.
//    l - decrease integration time bo 10 us.
//    f - flush the rawData writer. IE Save it
//    p - work up data. This creates the leftMap.txt and rightMap.txt that are ready to copy and paste into leftMap.pde and rightMap.pde
//    c - Works up the data but formats it for arduino in c. Uses leftMap.txt and rightMap.txt again.
//    t - tests the calibration by scanning the filament accross the sensor and measurs the diameter using the left and right map
//        makes a file called "Calibration Test.txt" that has screw position and diameter data.
//
//    


import java.util.*;
import processing.serial.*; 
import org.gicentre.utils.stat.*;    // For chart classes.

import java.util.ArrayList;
import static java.util.Arrays.asList;
import java.util.List;



LinkedList<Float> diaList = new LinkedList<Float>();

Serial serial;    // The serial port
boolean serialInited;
int now;
int lastSerial;

PFont myFont;     // The display font
String inString;  // Input string from serial port
int linefeed = 10;      // ASCII linefeed 
float[] vals = new float[256];

//Chart Variables
XYChart lineChart;
float[] dataX = new float[256];//The data for the x axis
float[] dataY = new float[256];



;//The data for the y axis
XYChart diaChart;
float[] diaDataX = new float[2];
float []diaDataY = new float[2];
XYChart calibrationChartLeft;
float[] calibrationChartLeftX = new float[300];
float []calibrationChartLeftY = new float[300];
XYChart calibrationChartRight;
float []calibrationChartRightY = new float[300];


//Globals used for calculating/reporting filament diameter
float pixelPitch = 0.05707; //the pitch of the CCD array. This is appearent NOT FROM DATASHEET
int leftGap = 1;//The number of pixels to omit from the sensor. Some times the pixels at the edge of the sensor don't read correctly.
int rightGap = 254;//The number of pixels to omit from the right side of the sensor.
int minBackgroundValue = 400;

float dia;
float actDia;
float lPos;
float rPos;
float leftX; //The x position of the left side of the shadow as calculated using the line and the y offset.
float rightX;//The x position of the right side of the shadow as calculated using the line and the y offset.
int lIndex;
int rIndex;
float lValue;
float rValue;
float averageJumpValue;
int[] setToAverage = new int[8];
int averageIndex = 0;


float averageDia;
float[] dias = new float[10];



//Calibration variables
boolean homing = false;
int calNumSamples = 1;
int calSampleNum = 0;// a reading of the output from all of the sensor pixels
float startPos = 12.9;
float endPos = .1;
int startLIndex = 225;

float dist = startPos-endPos;
int steps = 4;
float position;
float delta = steps*(0.5/(200*16)); //The distance to move for each measurement. 0.5 mm pitch, 200 steps per rev, 16 microsteps per step.
int numMeasurements = int((dist/delta)); //the numberof measurements to make
float[] rightEdges = new float[calNumSamples];
float[] leftEdges = new float[calNumSamples];
float[] diameters = new float[calNumSamples];
//float[][] rightEdgeMap = new float[numMeasurements][2];
//float[][] leftEdgeMap = new float[numMeasurements][2];
float filamentDia = 1.62;//Measured diameter of the filament being used for calibration.
int measurementNum = numMeasurements-1; //The current Measurement (a group of samples)
boolean stopRecording = false;
boolean newData = false;
long integral;
long nextSampleTime;
long startTime;

//background variables
int bgNUM = 0;
int bgSampleNum = 0;//current background sample
int bgNumSamples = 20;//number of bg samples to collect per test.
int[] bgReading = new int[256]; // location to store bg scans. 
char backgroundDir;
int backgroundStepsTaken;
float backgroundPosition = -2.64;
//float backgroundPosition = -.22;
int skipData = 15;
int skipNum = 0;

//multiscan Variables
int scan = 0;
int scans = 10;
PrintWriter scanWriter;
boolean multiscan;

PrintWriter rawDataWriter;

PrintWriter backgroundWriter;
PrintWriter testWriter;


//state Machine
enum states {
  STANDBY, 
    BEGIN_HOME, 
    HOME, 
    SCAN, 
    BEGIN_SCAN, 
    MULTISCAN, 
    GET_RAW_DATA, 
    PROCESS_DATA, 
    COLLECT_BACKGROUND, 
    BEGIN_COLLECT_BACKGROUND, 
    EXIT_COLLECT_BACKGROUND, 
    TEST_CALIBRATION
}

states currentState = states.STANDBY;
states previousState = states.STANDBY;

void setup() {

  initializeLeftMap();
  initializeRightMap();

  frameRate(60);
  size(1000, 700);
  //Serial port setup
  // List all the available serial ports: 
  printArray(Serial.list());




  //Chart Setup

  textFont(createFont("Arial", 10), 10);

  // Both x and y data set here.  
  lineChart = new XYChart(this);
  dataX = new float[256];
  dataY = new float[256];
  for (int i=0; i<dataX.length; i++) {
    dataX[i] = i*pixelPitch;
  }
  lineChart.setData(dataX, dataY);

  // Axis formatting and labels.
  lineChart.showXAxis(true); 
  lineChart.showYAxis(true); 
  lineChart.setMinY(0);
  lineChart.setMaxY(1024);
  lineChart.setMinX(0);
  lineChart.setMaxX(256*pixelPitch);
  // Symbol colours
  lineChart.setPointColour(color(50, 180, 50, 100));
  lineChart.setPointSize(5);
  lineChart.setLineWidth(2);
  lineChart.setYAxisLabel("Intensity");

  //diameter line
  diaChart = new XYChart(this);
  diaDataX = new float[2];
  diaDataY = new float[2];
  for (int i=0; i<dataX.length; i++) {
    dataX[i] = i*pixelPitch;
  }
  diaChart.setData(diaDataX, diaDataY);

  // Axis formatting and labels.
  diaChart.showXAxis(true); 
  diaChart.showYAxis(true); 
  diaChart.setMinY(0);
  diaChart.setMaxY(1024);
  diaChart.setMinX(0);
  diaChart.setMaxX(256*pixelPitch);


  // Symbol colours
  diaChart.setPointColour(color(255, 0, 0, 100));
  diaChart.setPointSize(8);
  diaChart.setLineWidth(4);
  diaChart.setYAxisLabel("Intensity");

  // Open the port that the arduino uses.
  initSerial();
  //workupData();
  //testAlg();
  serial.write('D');
} 

void draw() { 

  background(255);
  textSize(9);
  //text("received: " + inString, 10,50);

  lineChart.setData(dataX, dataY);
  diaChart.setData(diaDataX, diaDataY);

  lineChart.draw(15, 15, (width-30), (height)/2-40);
  diaChart.draw(15, 15, (width-30), (height)/2-40);


  // Draw a title over the top of the chart.
  fill(120);
  textSize(20);
  text("Diameter Sensor", 70, 30);
  textSize(11);
  text(dia, 70, 45);
  text("Act Dia", 70, 500);
  text(actDia+" "+ averageDia, 70, 515);
  text("Measurement Num:", 15, 380);
  text(measurementNum, 115, 380);
  text("Position:", 70, 395);
  text(position, 115, 395);
  text("Sample Num:", 45, 410);
  text(calSampleNum, 115, 410);  
  text("Left X:", 70, 425);
  text(String.format("%.4f", leftX), 115, 425); 
  text("lIndex:", 180, 425);
  text(lIndex, 220, 425); 
  float sum = 0;
  for (int i = 0; i<diaList.size(); i++) {
    sum += diaList.get(i);
  }
  text(sum/diaList.size(), 70, 75);
} 


//This function is called when there is a serial event.
//the serial data is read into the variable inString
//Each time serial data is recieved, the current state function is called
//The current state function handles the serial data as is suitable for the state.
void serialEvent(Serial p) { 
  try {
    inString = p.readString();
    serial.clear();
  
    if (inString == null) {
      return;
    }
    
    //run the current state function now that data has been received.
    switch (currentState) {

    case STANDBY:
      standby();
      return;
    case BEGIN_HOME:
      beginHome();
      return;
    case HOME:
      home();
      return;

    case SCAN:
      scan();
      return;

    case BEGIN_SCAN:
      beginScan();
      return;

    case MULTISCAN:
      multiscan();
      return;

    case GET_RAW_DATA:
      getRawData();
      return;

    case PROCESS_DATA:
      workupData();
      return;

    case BEGIN_COLLECT_BACKGROUND:
      beginCollectBackground();
      return;

    case EXIT_COLLECT_BACKGROUND:
      exitCollectBackground();
      return;

    case COLLECT_BACKGROUND:
      collectBackground();
      return;

    case TEST_CALIBRATION:
      testCalibration();
      return;
    }
  }
  catch (Exception e) {
    println(e);
    println("error");
  }
} 


void keyPressed() {
  switch (key) {
  case 'h':
    previousState = states.STANDBY;
    currentState = states.BEGIN_HOME;
    serial.write('D');

    break;

  case 's':
    previousState = states.STANDBY;
    currentState = states.STANDBY;
    serial.write('s');
    break;

  case 'd':
    previousState = states.STANDBY;
    currentState = states.GET_RAW_DATA;
    rawDataWriter = createWriter("rawData.txt");
    serial.write('D');
    break;

  case 'b':

    backgroundWriter = createWriter("background.txt");
    previousState = states.STANDBY;
    currentState = states.BEGIN_COLLECT_BACKGROUND;
    serial.write('D');
    break;

  case 'm':
    bgSampleNum = 0;
    backgroundWriter = createWriter("background.txt");
    previousState = states.STANDBY;
    currentState = states.MULTISCAN;
    serial.write('D');
    break;

  case '1':
    bgSampleNum = 0;
    backgroundWriter = createWriter("background.txt");
    previousState = states.STANDBY;
    currentState = states.BEGIN_SCAN;
    serial.write('D');
    break;

  case 'i':
    serial.write('I');
    break;

  case 'l':
    serial.write('L');
    break;

  case 'f':
    rawDataWriter.flush();
    break;
  case 'p':
    workupData();
    break;

  case 'c':
    workupDataC();
    break;

  case 't':
    testWriter = createWriter("Calibration Test.txt");
    testWriter.println("Screw Position (mm)\tDiameter (mm)\tAvg Dia (mm)");

    previousState = states.TEST_CALIBRATION;
    currentState = states.BEGIN_HOME;
    serial.write('D');
    break;
  }
}

void initSerial () {
  try {

    serial = new Serial(this, Serial.list()[0], 250000); 
    serial.bufferUntil(linefeed); 
    serialInited = true;
  } 
  catch (RuntimeException e) {
    if (e.getMessage().contains("<init>")) {
      System.out.println("port in use, trying again later...");
      serialInited = false;
    }
  }
}