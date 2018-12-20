class sensorReading {
  public int p;//pixel index
  public float s;//screw position
  public float i;//Intensity
  public int n;//the number of values accumulated

  sensorReading(int P, float S, float I, int N) {
    this.p = P;
    this.s = S;
    this.i = I;
    this.n = N;
  }
}

class sensorReadingComparator implements Comparator {  
  public int compare(Object o1, Object o2) {  
    sensorReading s1=(sensorReading)o1;  
    sensorReading s2=(sensorReading)o2;  

    if (s1.i==s2.i)  
      return 0;  
    else if (s1.i>s2.i)  
      return 1;  
    else  
    return -1;
  }
}  

/*

 create scans readers
 read first lines 
 set screw position for all lines
 
 if all lines have same are on same pixel, average intensities and  add it to the PML or PMR
 
 */

void workupData() {
  // in C create array of pointers to arrays for each pixel
  //then search the individual array for the interval needed.
  //PrintWriter mapWriter1;

  //mapWriter1 = createWriter("PMLexcel.txt");

  //String[] line = new String[scans];



  //BufferedReader[] reader = new BufferedReader[scans];

  ////create scans Readers
  //for (int i=0; i<scans; i++) {
  //  reader[i] = createReader("scan"+i+".txt");
  //  //Read and throw away first line. Just titles for graphing in excel
  //  try {
  //    line[i] = reader[i].readLine();
  //  } 
  //  catch (IOException e) {
  //    e.printStackTrace();
  //    line[i] = null;
  //  }
  //}
  //float screwPos = startPos;
  ////now for each measurement,  read each of the scans line and process the data
  //for (int i = 0; i<numMeasurements; i++) {
  //  int[] lPixel = new int[scans];
  //  float[] lVal =new float[scans];
  //  mapWriter1.print(screwPos+"\t");
  //  for (int j = 0; j<scans; j++) {
  //    try {
  //      line[j] = reader[j].readLine();
  //    } 
  //    catch (IOException e) {
  //      e.printStackTrace();
  //      line[j] = null;
  //    }
  //    //process line
  //    //break line into values
  //    String[] valStrings = split(line[j], TAB);

  //    //extract values from valStrings
  //    lPixel[j] = int(valStrings[0]);
  //    lVal[j] = float(valStrings[3]);

  //    mapWriter1.print(lVal[j]+"\t");
  //  }
  //  mapWriter1.println();
  //  screwPos-= delta;
  //}

  //mapWriter1.flush();
  //mapWriter1.close();











  ArrayList<sensorReading> PML = new ArrayList<sensorReading>();//Pixel Map Left
  ArrayList<sensorReading> PMR = new ArrayList<sensorReading>();//Pixel Map Right
  String[] line = new String[scans];
  println("Working up data!");

  BufferedReader[] reader = new BufferedReader[scans];

  //create scans Readers
  int selectedScan = 5;
  reader[selectedScan] = createReader("scan"+selectedScan+".txt");
  //Read and throw away first line. Just titles for graphing in excel
  try {
    line[selectedScan] = reader[selectedScan].readLine();
  } 
  catch (IOException e) {
    e.printStackTrace();
    line[selectedScan] = null;
  }

  float screwPos = startPos;
  //now for each measurement,  read each of the scans line and process the data
  for (int i = 0; i<numMeasurements; i++) {
    int[] lPixel = new int[scans];
    int[] rPixel = new int[scans];
    float[] lVal =new float[scans];
    float[] rVal =  new float[scans];

    //for (int j = 0; j<scans; j++) {

    try {
      line[selectedScan] = reader[selectedScan].readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line[selectedScan] = null;
    }
    //process line
    //break line into values
    String[] valStrings = split(line[selectedScan], TAB);

    //extract values from valStrings
    lPixel[selectedScan] = int(valStrings[0]);
    rPixel[selectedScan] = int(valStrings[1]);
    lVal[selectedScan] = float(valStrings[3]);
    rVal[selectedScan] =  float(valStrings[4]);


    PML.add(new sensorReading(lPixel[5], screwPos, lVal[selectedScan], scans));
    PMR.add(new sensorReading(rPixel[5], screwPos+filamentDia, rVal[selectedScan], scans));

    screwPos -= delta;
  }

  //Sort the arraylists based on screwLocation
  Collections.sort(PML, new sensorReadingComparator());  
  Collections.sort(PMR, new sensorReadingComparator());  

  //PrintWriter mapWriter1;

  //mapWriter1 = createWriter("PML.txt");
  //for (sensorReading mapReading : PML) {
  //  mapWriter1.println(mapReading.p+"\t"+mapReading.s+"\t"+mapReading.i+"\t"+mapReading.n);
  //}
  //mapWriter1.flush();
  //mapWriter1.close();

  //println("PML: "+PML.size());
  //println("PMR: "+PMR.size());

  ////create a map for each pixel
  PrintWriter mapWriter;
  mapWriter = createWriter("leftMap.txt");
  mapWriter.println("class mapEntry {\r\n\tpublic int p;//pixel index\r\n\tpublic float s;//screw position\r\n\tpublic float i;//Intensity\r\n\r\n"+
    "\tmapEntry(int P, float S, float I) {\r\n\t\tthis.p = P;\r\n\t\tthis.s = S;\r\n\t\tthis.i = I;\r\n\t\t}\r\n}\r\n\r\n"+
    "ArrayList<mapEntry> leftMap = new ArrayList<mapEntry>();\r\n");
  mapWriter.println("class mapEntryComparator implements Comparator {\r\n"+
    "  public int compare(Object o1, Object o2) {  \r\n"+
    "    mapEntry s1=(mapEntry)o1;\r\n"+
    "    mapEntry s2=(mapEntry)o2;\r\n"+
    "\r\n"+
    "    if (s1.i==s2.i)  \r\n"+
    "      return 0;  \r\n"+
    "    else if (s1.i>s2.i)  \r\n"+
    "      return 1;  \r\n"+
    "    else  \r\n"+
    "    return -1;\r\n"+
    "  }\r\n"+
    "}  \r\n");
  mapWriter.println("void initializeLeftMap(){");
  // Create an array list of the left shadow data an then reduce it.
  for (int i=0; i<256; i++) {
    List<Point> points = new ArrayList<Point>();
    int count = 0;
    for (sensorReading mapReading : PML) {
      if (mapReading.p == i) {
        //println(i+"\t"+mapReading.s+"\t"+mapReading.i);
        points.add(new Point(mapReading.s, mapReading.i));
        count++;
      }
    }
    if (count>3) { 
      //println(i);
      //println("trying to Reduce");
      List<Point> reduced = SeriesReducer.reduce(points, 0.002);

      for (int j = 0; j < reduced.size(); j++) {
        mapWriter.println("\tleftMap.add(new mapEntry("+i+","+reduced.get(j).x+","+(int)reduced.get(j).y+"));");
        //mapWriter.println("leftMap["+i+"]["+(int)reduced.get(j).y+"] = "+reduced.get(j).x+";");
        //println(reduced.get(j).x+"\t"+(int)reduced.get(j).y);
      }
    }
  }
  mapWriter.println("}");
  mapWriter.flush();
  mapWriter.close();


  //Create the Right map
  mapWriter = createWriter("rightMap.txt");
  mapWriter.println("ArrayList<mapEntry> rightMap = new ArrayList<mapEntry>();\r\n\r\n"+
    "void initializeRightMap(){");

  // Create an array list of the left shadow data an then reduce it.
  for (int i=0; i<256; i++) {
    List<Point> points = new ArrayList<Point>();
    int count = 0;
    for (sensorReading mapReading : PMR) {
      if (mapReading.p == i) {
        //println(i+"\t"+mapReading.s+"\t"+mapReading.i);
        points.add(new Point(mapReading.s, mapReading.i));
        count++;
      }
    }
    if (count>3) { 
      //println(i);
      //println("trying to Reduce");
      List<Point> reduced = SeriesReducer.reduce(points, 0.002);

      for (int j = 0; j < reduced.size(); j++) {
        mapWriter.println("\trightMap.add(new mapEntry("+i+","+reduced.get(j).x+","+(int)reduced.get(j).y+"));");
        //mapWriter.println("rightMap["+i+"]["+(int)reduced.get(j).y+"] = "+reduced.get(j).x+";");
        //println("leftMap["+i+"]["+(int)reduced.get(j).y+"] = "+reduced.get(j).x+";");
      }
    }
  }
  mapWriter.println("}");
  mapWriter.flush();
  mapWriter.close();
  println("Finished Working up Data. Yay!");
}










void workupDataC() {
  // in C create array of pointers to arrays for each pixel
  //then search the individual array for the interval needed.
  //PrintWriter mapWriter1;

  //mapWriter1 = createWriter("PMLexcel.txt");

  //String[] line = new String[scans];



  //BufferedReader[] reader = new BufferedReader[scans];

  ////create scans Readers
  //for (int i=0; i<scans; i++) {
  //  reader[i] = createReader("scan"+i+".txt");
  //  //Read and throw away first line. Just titles for graphing in excel
  //  try {
  //    line[i] = reader[i].readLine();
  //  } 
  //  catch (IOException e) {
  //    e.printStackTrace();
  //    line[i] = null;
  //  }
  //}
  //float screwPos = startPos;
  ////now for each measurement,  read each of the scans line and process the data
  //for (int i = 0; i<numMeasurements; i++) {
  //  int[] lPixel = new int[scans];
  //  float[] lVal =new float[scans];
  //  mapWriter1.print(screwPos+"\t");
  //  for (int j = 0; j<scans; j++) {
  //    try {
  //      line[j] = reader[j].readLine();
  //    } 
  //    catch (IOException e) {
  //      e.printStackTrace();
  //      line[j] = null;
  //    }
  //    //process line
  //    //break line into values
  //    String[] valStrings = split(line[j], TAB);

  //    //extract values from valStrings
  //    lPixel[j] = int(valStrings[0]);
  //    lVal[j] = float(valStrings[3]);

  //    mapWriter1.print(lVal[j]+"\t");
  //  }
  //  mapWriter1.println();
  //  screwPos-= delta;
  //}

  //mapWriter1.flush();
  //mapWriter1.close();











  ArrayList<sensorReading> PML = new ArrayList<sensorReading>();//Pixel Map Left
  ArrayList<sensorReading> PMR = new ArrayList<sensorReading>();//Pixel Map Right
  String[] line = new String[scans];
  println("Working up data!");

  BufferedReader[] reader = new BufferedReader[scans];

  //create scans Readers
  int selectedScan = 5;
  reader[selectedScan] = createReader("scan"+selectedScan+".txt");
  //Read and throw away first line. Just titles for graphing in excel
  try {
    line[selectedScan] = reader[selectedScan].readLine();
  } 
  catch (IOException e) {
    e.printStackTrace();
    line[selectedScan] = null;
  }
  //int scans = 1;
  float screwPos = startPos;
  //now for each measurement,  read each of the scans line and process the data
  for (int i = 0; i<numMeasurements; i++) {
    int[] lPixel = new int[scans];
    int[] rPixel = new int[scans];
    float[] lVal =new float[scans];
    float[] rVal =  new float[scans];

    //for (int j = 0; j<scans; j++) {

    try {
      line[selectedScan] = reader[selectedScan].readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line[selectedScan] = null;
    }
    //process line
    //break line into values
    String[] valStrings = split(line[selectedScan], TAB);

    //extract values from valStrings
    lPixel[selectedScan] = int(valStrings[0]);
    rPixel[selectedScan] = int(valStrings[1]);
    lVal[selectedScan] = float(valStrings[3]);
    rVal[selectedScan] =  float(valStrings[4]);


    PML.add(new sensorReading(lPixel[5], screwPos, lVal[selectedScan], scans));
    PMR.add(new sensorReading(rPixel[5], screwPos+filamentDia, rVal[selectedScan], scans));

    screwPos -= delta;
  }

  //Sort the arraylists based on screwLocation
  Collections.sort(PML, new sensorReadingComparator());  
  Collections.sort(PMR, new sensorReadingComparator());  


  ////create a map for each pixel
  PrintWriter mapWriter;
  int[] pixelMapLengths = new int[512];

  mapWriter = createWriter("leftMap.txt");
  mapWriter.println(" struct mapPoint {\r\n"+
    " \tint intensity;\r\n"+
    "  \tfloat position;\r\n"+
    "};\r\n\r\n");


  // Create an array list of the left shadow data an then reduce it.
  for (int i=0; i<256; i++) {
    List<Point> points = new ArrayList<Point>();
    int count = 0;
    for (sensorReading mapReading : PML) {
      if (mapReading.p == i) {
        points.add(new Point(mapReading.s, mapReading.i));
        count++;
      }
    }
    mapWriter.println("const mapPoint lPixel"+i+"[]  PROGMEM = {");
    if (count>3) { 

      List<Point> reduced = SeriesReducer.reduce(points, 0.002);

      for (int j = 0; j < reduced.size()-1; j++) {
        mapWriter.println("\t{"+(int)reduced.get(j).y+", "+reduced.get(j).x+"},");
      }
      mapWriter.println("\t{"+(int)reduced.get(reduced.size()-1).y+", "+reduced.get(reduced.size()-1).x+"}");
      pixelMapLengths[i] = reduced.size();
    }
    mapWriter.println("\t};\r\n");
  }

  mapWriter.flush();
  mapWriter.close();


  //Create the Right map
  mapWriter = createWriter("rightMap.txt");
  // Create an array list of the left shadow data an then reduce it.
  for (int i=0; i<256; i++) {
    List<Point> points = new ArrayList<Point>();
    int count = 0;
    for (sensorReading mapReading : PMR) {
      if (mapReading.p == i) {
        points.add(new Point(mapReading.s, mapReading.i));
        count++;
      }
    }
    mapWriter.println("const mapPoint rPixel"+i+"[]  PROGMEM = {");
    if (count>3) { 


      List<Point> reduced = SeriesReducer.reduce(points, 0.002);

      for (int j = 0; j < reduced.size()-1; j++) {
        mapWriter.println("\t{"+(int)reduced.get(j).y+", "+reduced.get(j).x+"},");
      }
      mapWriter.println("\t{"+(int)reduced.get(reduced.size()-1).y+", "+reduced.get(reduced.size()-1).x+"}");
      pixelMapLengths[i+256] = reduced.size();
    }
    mapWriter.println("\t};\r\n");
  }


  mapWriter.print("const PROGMEM int mapLengths[512] = {");
  for (int i=0; i<511; i++) {
    mapWriter.print(pixelMapLengths[i]+", ");
  }
  mapWriter.print(pixelMapLengths[511]+"};\r\n\r\n");

  mapWriter.print("const PROGMEM int mapPtrs[512] = {");
  for (int i =0; i<256; i++) {
    mapWriter.print("lPixel"+i+", ");
  }
  for (int i =0; i<255; i++) {
    mapWriter.print("rPixel"+i+", ");
  }
  mapWriter.println("rPixel255};\r\n");

  mapWriter.flush();
  mapWriter.close();
  println("Finished Working up Data. Yay!");
}