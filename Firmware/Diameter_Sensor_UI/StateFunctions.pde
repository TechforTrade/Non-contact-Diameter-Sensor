void standby() {
  String[] valStrings = split(inString, ' ');
  for (int i=1; i<valStrings.length; i++) {
    if (normaliseIntensity)
    {
      //dataY[i-1] = float(valStrings[i])*bgCorrection[i-1][0]+bgCorrection[i-1][1];
      if (first)
      {
        firstYData[i-1] = float(valStrings[i]);
      }
      else
      {
        dataY[i-1] = float(valStrings[i])/firstYData[i-1]*500;
      }
    }
    else
    {
      dataY[i-1] = float(valStrings[i]);
      
    }
    if ( !(first) && (logDataY) )
    {
      if (i!=1)
      {
        dataYOut.print(",");
      }
      dataYOut.print(dataY[i-1]);
    }
    
    if (i == dataY.length-1) {
      break;
    }
  }
  if (first)
  {
    first=false;
  }
  else
  {
    if (logDataY)
    {
      dataYOut.println("");
      dataYOut.flush();
    }
  }
  //dataY[20] = dataY[20]*750/418-35;
  //if (!detectYJump()) {
  getDiameter();
  getAvgDia(actDia);

  //}
  serial.write('D');
}

void getRawData() {
  String[] valStrings = split(inString, ' ');

  for (int i=1; i<valStrings.length; i++) {
    dataY[i-1] = float(valStrings[i]);
     //dataY[i-1] = float(valStrings[i])*bgCorrection[i-1][0]+bgCorrection[i-1][1];

    //print(dataY[i-1]+"\t");
    if (i == dataY.length-1) {
      break;
    }
  }
  for (int i=0; i<256; i++) {
   print("("+i+")"+dataY[i]+"\t");
 }
 println();
 println();
 println();
  
  //if (detectYJump()) {
  //  serial.write('D');
  //} else {
  //  for (int i=0; i<25; i++) {
  //    print(dataY[i]+"\t");
  //  }
  //  println();
  //}

  serial.write('D');
}

void beginHome() {
  //There is just error in Logic here. Coming from the right works coming from the left has flawed logic.
println("Homming...");
  //beginHome does not modify the previous state
  float overshoot = .5;
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);
  int stps;

  switch (valStrings[0]) {
  case "F":
    stps = int((startPos+overshoot-leftX)/(.5/(200*16)));
    serial.write("M"+stps);
    break;

  case "R":
    currentState = states.HOME;
    stps = int((leftX-(startPos+overshoot))/(.5/(200*16)));

    serial.write("M"+stps);
    break;

  case "D": 
    for (int i=1; i<valStrings.length; i++) {
      dataY[i-1] = float(valStrings[i]);
      if (i == dataY.length-1) {
        break;
      }
    }
    //if (detectYJump()) {
    //  serial.write('D');
    //  return;
    //}
    getDiameter();

    if (leftX < startPos+overshoot && leftX > 0.0) {
      serial.write('F');
    } else if (leftX >startPos+overshoot) {
      serial.write('R');
    } else {

      currentState = states.STANDBY;
      serial.write('E');
    }
    break;

  case "M"://the filament has just moved forward and must now go into reverse
    currentState = states.HOME;
    serial.write('R');
    break;
  }

  inString = null;
  delay(100);
}

//to enter the home state, the position should b within 0.01mm of the overshoot position
void home() {
  float startSlowApproach = 0.03;

  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);

  switch (valStrings[0]) {
  case "R":
    serial.write('D');
    break;
  case "D":
    for (int i=1; i<valStrings.length; i++) {
      dataY[i-1] = float(valStrings[i]);
      if (i == dataY.length-1) {
        break;
      }
    }
    //if (detectYJump()) {
    //  serial.write('D');
    //  return;
    //}
    getDiameter();

    if (leftX > startPos+startSlowApproach) {
      serial.write("M"+int((leftX-(startPos+startSlowApproach))/(.5/(200*16))));
    } else if (lIndex > startLIndex) {
      delay(100);
      serial.write("h");
    } else {
      println("Homed");
      //turn control back to previous state
      
      currentState = previousState;
      serial.write('E');
    }
  case "M":
    serial.write('D');
    break;

  default:
    currentState = states.STANDBY;

    break;
  }
  delay(100);
}





void beginCollectBackground() {
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);

  switch (valStrings[0]) {
  case "B"://Just received command to collect Background
    //get location
    skipNum = 0;
    serial.write('D');
    break;

  case "D"://evaluate position and send command to direction for movement to background position
    for (int i=1; i<valStrings.length; i++) {
      dataY[i-1] = float(valStrings[i]);
      if (i == dataY.length-1) {
        break;
      }
    }
    //if (detectYJump()) {
    //  serial.write('D');
    //  return;
    //}
    skipNum = 0;
    if (leftX < backgroundPosition) {
      serial.write('F');
    } else if (leftX>= backgroundPosition) {
      serial.write('R');
    } else {
      //the filament isn't in front of the sensor, so just take the background
      backgroundDir = 'E';
      backgroundStepsTaken = 0;
      currentState = states.COLLECT_BACKGROUND;
      serial.write(backgroundDir);
    }
    break;

  case "F"://filament is to the left of the background position. sent the correct move command
    currentState = states.COLLECT_BACKGROUND;
    backgroundDir = 'R';//direction to move on exiting background collection
    backgroundStepsTaken = int((backgroundPosition-leftX)/(0.5/(200*16)));
    serial.write("M"+backgroundStepsTaken);
    break;

  case "R"://filament is to the right of the background position. sent the correct move command
    currentState = states.COLLECT_BACKGROUND;
    backgroundDir = 'F';//direction to move on exiting background collection
    backgroundStepsTaken = int((leftX-backgroundPosition)/(0.5/(200*16)));
    serial.write("M"+backgroundStepsTaken);
    break;
  }
  inString = null;
  delay(100);
}

void collectBackground() {
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);

  switch (valStrings[0]) {
  case "M"://Just received finished moving to position
  case "E"://filament not on sensor just collect background
    //zero out the background reading array
    for (int i=0; i<bgReading.length; i++) {
      bgReading[i] = 0;
      if (i == bgReading.length-1) {
        break;
      }
    }
    serial.write('D');
    break;

  case "D":
    if (skipNum < skipData) {//skip the first skipData samples to ensure that the jump pixel's average is well known
      skipNum++;
      serial.write('D');
      return;
    }
    for (int i=1; i<valStrings.length; i++) {
      dataY[i-1] = float(valStrings[i]);
      if (i == dataY.length-1) {
        break;
      }
    }
    //if (detectYJump()) {
    //  serial.write('D');
    //  return;
    //}

    for (int i=0; i<dataY.length; i++) {
      bgReading[i] += int(dataY[i]);
    }
    bgSampleNum++;
    if (bgSampleNum<bgNumSamples) {
      serial.write('D');//get more data because we still haven't reached the number of samples
    } else {//finished collecting background. Make average and then move on to exit Background Collection.
      println("finished Collecting background");
      for (int i=0; i<bgReading.length; i++) {
        bgReading[i] = bgReading[i]/bgNumSamples;
        backgroundWriter.print(bgReading[i]+"\t");
      }
      backgroundWriter.println();
      currentState = states.EXIT_COLLECT_BACKGROUND;

      serial.write(backgroundDir);
    }
    break;
  }
}

void exitCollectBackground() {
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);
  switch (valStrings[0]) {
  case "F"://Just received finished moving to position
  case "R"://Just received finished moving to position
  case "E"://filament not of sensor just collect background
    serial.write("M"+abs(backgroundPosition-(startPos+.55))/(0.5/(200*16)));
    break;

  case "M":
    if (!multiscan) {
      backgroundWriter.flush();
      backgroundWriter.close();
    }
    currentState = previousState;
    serial.write("W");
  }
}


void beginScan() {
  println("Beginning scan....");
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);

  switch (valStrings[0]) {
  case "N"://Just received command to do scan. create file to hold scan data then Home 

    break;

  case "E":
    currentState = states.BEGIN_COLLECT_BACKGROUND;
    serial.write('D');
    break;

  case "W"://just Finished collecting Background. Start Home
    previousState = states.SCAN;
    currentState = states.BEGIN_HOME;
    serial.write('D');
    break;

  case "D"://measurement received
    scanWriter = createWriter("scan"+scan+".txt"); 
    scanWriter.println("LPixel\tRPixel\tposition\tLVal\trVal");//add titles for graphing in excel
    previousState = states.BEGIN_SCAN;
    currentState = states.BEGIN_COLLECT_BACKGROUND;
    serial.write('D');
    break;

  case "M"://finished moving
    serial.write("D");
    break;
  }
}

void scan() {
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);

  switch (valStrings[0]) {
  case "E"://Just finished Homing. begin Scan
    position = leftX;
    serial.write('D');
    break;
  case "D":
    for (int i=1; i<valStrings.length; i++) {
      dataY[i-1] = float(valStrings[i]);
      if (i == dataY.length-1) {
        break;
      }
    }
    //println("Time to get data back from arduino = " + (millis()-now));
    //if (detectYJump()) {
    //  serial.write('D');
    //  return;
    //}
    //get the diameter and record the data in the relevent text files

    getDiameter();
    scanWriter.println(lIndex+"\t"+rIndex+"\t"+position+"\t"+lValue+"\t"+rValue);
    measurementNum--;
    position -=delta;
    if (measurementNum >=0 && leftX>endPos) {//the theoretical position doesn't match actual so both of these must be true.
      //println("Time to write to file = "+(millis()-now));
      serial.write("m");
      //now = millis();
      return;
    } else {
      measurementNum = numMeasurements - 1;//reset for the next data set
      println("finished scan "+scan);
      println("time taken = "+((millis() - now)/(1000*60)));
      now = millis();
      scanWriter.flush();
      scanWriter.close();
      if (multiscan) {
        currentState = states.MULTISCAN;
        serial.write('E');
      } else {
        backgroundWriter.flush();
        backgroundWriter.close();
        currentState = states.STANDBY;
        println("STANDBY");
        serial.write('D');
      }
    }
    break;

  case "M":
    serial.write('D');
    break;
  }
}
void multiscan() {
  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);
  switch (valStrings[0]) {
  case "D"://Just starged a multiscan. get set
    now = millis();
    startTime = now;
    multiscan = true;
    currentState = states.BEGIN_SCAN;
    serial.write('D');
    break;

  case "E"://just finished a scan, setup for the next scan, or, process data
    scan++;
    if (scan < scans) {//start the next scanning process
      bgSampleNum = 0;
      currentState = states.BEGIN_SCAN;
      serial.write('D');
    } else {//work up the data
      println("Scan Durration: "+((millis()-startTime)/(1000*60)));
      println("Multiscan Finished. Working up data...");
      backgroundWriter.flush();
      backgroundWriter.close();
      workupData();
    }
  }
}

void testCalibration() {

  String[] valStrings = split(inString, ' ');
  valStrings[0] = "" + valStrings[0].charAt(0);

  switch (valStrings[0]) {
  case "E"://Just finished Homing. begin Scan
    position = startPos;
    serial.write('D');
    break;
  case "D":
    for (int i=1; i<valStrings.length; i++) {
      dataY[i-1] = float(valStrings[i]);
      if (i == dataY.length-1) {
        break;
      }
    }

    getDiameter();
      getAvgDia(actDia);
    testWriter.println(position+"\t"+actDia+"\t"+averageDia);
    measurementNum--;
    position -=delta;
    if (measurementNum >=0 || leftX>endPos) {//the theoretical position doesn't match actual so both of these must be true.
      //println("Time to write to file = "+(millis()-now));
      serial.write("m");
      //now = millis();
      return;
    } else {
      measurementNum = numMeasurements - 1;//reset for the next data set
      println("finished test ");
      println("time taken = "+((millis() - now)/(1000*60)));
      now = millis();
      testWriter.flush();
      testWriter.close();
      currentState = states.STANDBY;
      println("STANDBY");
      serial.write('D');
    }
    break;

  case "M":
    serial.write('D');
    break;
  }
}
