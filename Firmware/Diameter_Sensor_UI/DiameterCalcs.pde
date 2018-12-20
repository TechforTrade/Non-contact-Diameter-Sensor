  boolean getDiameter() {

  for (int i = leftGap; i < rightGap; i++) {
    if (dataY[i]< minBackgroundValue) {
      lIndex = i;
      lValue =  (dataY[i]+dataY[i+1]+dataY[i-1])/3.0;
      break;
    }
  }
  if (lIndex == 0) {
    //return false;
  }
  leftX = (lIndex)*pixelPitch+((dataY[lIndex]-60)/(minBackgroundValue-60))*pixelPitch;


  for (int i = lIndex+1; i < rightGap; i++) {
    if (dataY[i+1]>minBackgroundValue) {
      rIndex = i;
      rValue = (dataY[i]+dataY[i+1]+dataY[i-1])/3.0;
      break;
    }
  }
  if (rIndex < 2 ) {
    return false;
  }

  //for the diameter graph
  diaDataX[0] = lIndex*pixelPitch;
  diaDataX[1] = rIndex*pixelPitch;
  diaDataY[0] = dataY[lIndex];
  diaDataY[1] = dataY[rIndex];

  //for homing and diameter estimate
  dia = ((rIndex+(dataY[rIndex]-60)/(minBackgroundValue-60))-(lIndex+(dataY[lIndex]-60)/(minBackgroundValue-60)))*pixelPitch;

  //use map to find location of left edge
  ArrayList<mapEntry> tempMap = new ArrayList<mapEntry>();//make tempMap and fill with all entries having the correct index
  for (mapEntry entry : leftMap) {
    if (entry.p == lIndex) {
      tempMap.add(entry);
    }
  }
  if (tempMap.size()<2) {
    actDia = -1;
    return false;
  }

  boolean found = false;
  for (mapEntry entry : tempMap) {//find correct interval and interpolate or extrapolate
    if (entry.i>lValue) {
      found = true;
      int i = tempMap.indexOf(entry);
      int j;//j is set so that if the value is below the first point, the first two points in the tempMap are used to extrapolate
      if (i > 0) {
        j = i-1;
      } else {
        j = i + 1;
      }
      lPos = (interpolate(tempMap.get(j).i, tempMap.get(j).s, entry.i, entry.s, lValue));
      break;
    }
  }
  if (!found) {//if not found, use the last two points to extrapolate.
    int i = tempMap.size()-1;
    lPos = (interpolate(tempMap.get(i-1).i, tempMap.get(i-1).s, tempMap.get(i).i, tempMap.get(i).s, lValue));
  }

  //println("lPos "+lPos);

  //use map to find location of right edge
  //reset tempMap and found flag
  tempMap.clear();
  found = false;

  for (mapEntry entry : rightMap) {//populate tempMap
    if (entry.p == rIndex) {
      tempMap.add(entry);
    }
  }
  if (tempMap.size()<2) {
    actDia = -1;
    return false;
  }
  
  for (mapEntry entry : tempMap) {//search
    if (entry.i < rValue) {
      found = true;
      int i = tempMap.indexOf(entry);
      int j; //j is set so that if the value is below the first point, the first two points in the tempMap are used to extrapolate
      if (i > 0) {
        j = i-1;
      } else {
        j = i + 1;
      }
      rPos = (interpolate(tempMap.get(j).i, tempMap.get(j).s, entry.i, entry.s, rValue));
      break;
    }
  }
  if (!found) { //if not found, use the last two points to extrapolate.
    int i = tempMap.size()-1;
    rPos = (interpolate(tempMap.get(i-1).i, tempMap.get(i-1).s, tempMap.get(i).i, tempMap.get(i).s, rValue));
  }  

  actDia = rPos-lPos;
  
 
  return true;
}

void testAlg() {
  println("Here in test Alg");
  float lValue = 240.0;
  float lIndex = 157;
  boolean found = false;
  for (mapEntry entry : leftMap) {
    if (entry.p == lIndex && entry.i>lValue) {// or equal?
      found = true;
      int i = leftMap.indexOf(entry);
      println(i);
      int j;
      if (i > 0) {
        j = i-1;
      } else {
        j = i + 1;
      }
      println(interpolate(leftMap.get(j).i, leftMap.get(j).s, entry.i, entry.s, lValue));
      break;
    }
  }
  if (!found) {
    int i = leftMap.size()-1;
    println(interpolate(leftMap.get(i-1).i, leftMap.get(i-1).s, leftMap.get(i).i, leftMap.get(i).s, lValue));
  }
}


public float[]  linReg(float[][] Vals) {
  //Variables
  int n = 0;
  float[] slopeAndInt = new float[2];
  float sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0, sumYY = 0.0;
  float sXY, sXX;

  //Determine n
  //Determine n
  for (int i = 0; i < 8; i++) {
    n = i;
    if (Vals[i][0] == 0) {
      break;
    }
  }  

  //Find Sums
  for (int i = 0; i < n; i++) {
    sumX = sumX + Vals[i][0];
    sumY = sumY + Vals[i][1];
    sumXY = sumXY + Vals[i][0] * Vals[i][1];
    sumXX = sumXX + Vals[i][0] * Vals[i][0];
    sumYY = sumYY + Vals[i][1] * Vals[i][1];
  }

  //Compute slope and intercept
  sXY = sumXY - sumX * sumY / n;
  sXX = sumXX - sumX * sumX / n;
  slopeAndInt[0] = sXY / sXX;
  slopeAndInt[1] = sumY / n - slopeAndInt[0] * (sumX / n);

  return slopeAndInt;
}

float average(int[] vals) {
  float sum = 0;
  for (float val : vals) sum+=val;
  float avg = sum/vals.length;
  return avg;
}

float average(float[] vals) {
  float sum = 0;
  for (float val : vals) sum+=val;
  float avg = sum/vals.length;
  return avg;
}

void getAverageJumpValue(int newValue) {
  setToAverage[averageIndex] = newValue;
  averageJumpValue = average(setToAverage);
  averageIndex++;
  if (averageIndex==setToAverage.length) {
    averageIndex = 0;
  }
}

int avgInd;
void getAvgDia(float newValue) {
  dias[avgInd] = newValue;
  averageDia = average(dias);
  avgInd++;
  if (avgInd==dias.length) {
    avgInd = 0;
  }
}

boolean detectYJump() {
  boolean hasJumped = false;
  //getAverageJumpValue(int(dataY[jumpIndex]));
  //if (dataY[jumpIndex] > averageJumpValue) {
  //  hasJumped = true;
  //} 
  return hasJumped;
}

float interpolate(float x0, float y0, float x1, float y1, float x) {
  return y0+(x-x0)*(y1-y0)/(x1-x0);
}



float interpolateL(int index, int intensity) {
  float y1 =0.0;
  float y0=0.0;
  int x = intensity;
  int x1 =0;
  int x0 =0;

  //int i = index;
  //int j = intensity;

  //while (y1 == 0.0 ) {

  //  if (leftMap[i][j] == 0.0) {
  //    j++;
  //  } else {
  //    y1 = leftMap[i][j];
  //    x1 = j;
  //  }
  //  if (j > 1023) {
  //    break;
  //  }
  //}


  //j = intensity;
  //while (y0 == 0.0 ) {
  //  if (leftMap[i][j] == 0.0) {
  //    j--;
  //  } else {
  //    y0 = leftMap[i][j];
  //    x0 = j;
  //  }
  //  if (j == 0) {
  //    break;
  //  }
  //}
  return y0+(x-x0)*(y1-y0)/(x1-x0);
}

float interpolateR(int index, int intensity) {
  float y1=0.0;
  float y0=0.0;
  int x = intensity;
  int x1=0;
  int x0=0;

  //int i = index;
  //int j = intensity;
  //while (y1 == 0.0 ) {
  //  if (rightMap[i][j] == 0.0) {
  //    j++;
  //  } else {
  //    y1 = rightMap[i][j];
  //    x1 = j;
  //  }
  //  if (j > 1023) {
  //    break;
  //  }
  //}
  //j = intensity;
  //while (y0 == 0.0 ) {
  //  if (rightMap[i][j] == 0.0) {
  //    j--;
  //  } else {
  //    y0 = rightMap[i][j];
  //    x0 = j;
  //  }
  //  if (j == 0) {
  //    break;
  //  }
  //}
  return y0+(x-x0)*(y1-y0)/(x1-x0);
}