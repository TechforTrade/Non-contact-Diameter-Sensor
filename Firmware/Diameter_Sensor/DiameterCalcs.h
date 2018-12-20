int adcVals[256];// The array where the readout of the photodiodes is stored, as integers

float interpolate(float x0, float y0, float x1, float y1, float x) {
//
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}


//note for right shadows use pixel + 256
float locateEdge(int pixel, int inten) {

  bool found = false;
  int x0;
  int x1;
  float y0;
  float y1;
  const mapPoint * pixelMap = (const mapPoint *) pgm_read_word (&mapPtrs [pixel]);

  	//locate the edge of the shadow
  	if ((int)pgm_read_word(pixelMap) > inten) {
  		x0 = pgm_read_word(pixelMap);
  		y0 = pgm_read_float((int*)(pixelMap)+1);
  		x1 = pgm_read_word(pixelMap+1);
  		y1 = pgm_read_float((int*)(pixelMap+1)+1);
  	}
  
  	for (int i = 1; i < pgm_read_word_near(&mapLengths[pixel]) ; i++) {
  		if ((int)pgm_read_word(pixelMap+i) > inten) {
  			found = true;
  			x0 = pgm_read_word(pixelMap+i-1);
  			y0 = pgm_read_float((int*)(pixelMap+i-1)+1);
  			x1 = pgm_read_word(pixelMap+i);
  			y1 = pgm_read_float((int*)(pixelMap+i)+1);
  
  			break;
  		}
  	}
  
  	if (!found) {
  		int index = pgm_read_word_near(&mapLengths[pixel]);
  
  		x0 = pgm_read_word(pixelMap+index-2);
  		y0 = pgm_read_float((int*)(pixelMap+index-2)+1);
  		x1 =pgm_read_word(pixelMap+index-1);
  		y1 = pgm_read_float((int*)(pixelMap+index-1)+1);
  	}

  return interpolate(x0, y0, x1, y1, inten);
}



float getDia() {
  const int minBackgroundValue = 450;
  const int leftGap = 5;
  const int rightGap = 245;
  float leftX;
  float rightX;
  int leftPixel;
  int leftIntensity;
  int rightPixel;
  int rightIntensity;
  float dia;
  
  //adjust data on certain pixels here. 
adcVals[129] = adcVals[129] * 1.76600441501104-53.6203090507726;
adcVals[130] = adcVals[130] * 1.64609053497942-45.2263374485597;
adcVals[131] = adcVals[131] * 1.51228733459357-35.8601134215501;
adcVals[132] = adcVals[132] * 1.78173719376392-54.7216035634744;
adcVals[133] = adcVals[133] * 1.67014613778706-46.9102296450939;
adcVals[134] = adcVals[134] * 1.78970917225951-55.2796420581655;
adcVals[138] = adcVals[138] * 1.66320166320166-46.4241164241164;
  
  	//locate left shadow pixel
  	for (int i = leftGap; i < rightGap; i++) {
  		if (adcVals[i] < minBackgroundValue) {
  			leftPixel = i;
  			leftIntensity =  (adcVals[i] + adcVals[i + 1] + adcVals[i - 1]) / 3.0;
  			break;
  		}
  	}

  	if (leftPixel == 0.0){
      return 0.0;
  	}
  	for (int i = leftPixel + 1; i < rightGap; i++) {
  		if (adcVals[i + 1] > minBackgroundValue) {
  			rightPixel = i;
  			rightIntensity = (adcVals[i] + adcVals[i + 1] + adcVals[i - 1]) / 3.0;
  			break;
  		}
  	}
  	if (rightPixel < 2.0 ) {
  		return 0.0;
  	}
//   Serial.print("leftPixel: ");
//   Serial.println(leftPixel);
//   Serial.print("RightPixel: ");
//   Serial.println(rightPixel);
    dia = locateEdge(rightPixel+256, rightIntensity) - locateEdge(leftPixel, leftIntensity);
    if (dia != dia) return 0;
    if (dia <0) return 0;
    return dia;
}





