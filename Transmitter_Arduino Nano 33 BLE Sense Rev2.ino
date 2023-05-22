#include <Arduino_BMI270_BMM150.h>                    
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino_KNN.h>
#include "addOn.h"

const int serialRate = 115200;            // Serial monitor
const int transmissionRate = 115200;      // UART
const float accelerationThreshold = 2;    // threshold of significant in G's
const int noiseThreshold = 30;
const int sampleDelay = 10;
int samplesRead = samplePerGesture;
sampleData sample;
distanceDTW distance;

KNNClassifier myKNN(12);
int kValue = 5;

const char* TransmissionGesture[] = {
  "gesture1", 
  "gesture2",
  "gesture3",
  "gesture4"
};

void setup() {
  Serial.begin(serialRate);
  Serial1.begin(transmissionRate);
  IMU.begin();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  createModelKNN();
}

void loop() {
  float aX, aY, aZ, gX, gY, gZ;

  // wait for significant motion
  while (samplesRead == samplePerGesture) {
    if (IMU.accelerationAvailable()) {
      // read the acceleration data
      IMU.readAcceleration(aX, aY, aZ);

      // sum up the absolutes
      float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

      // check if it's above the threshold
      if (aSum >= accelerationThreshold) {
        // reset the sample read count
        samplesRead = 0;
        Serial.println("significant motion detected"); // print the header aX,aY,aZ,gX,gY,gZ
        break;
      }
    }
  }

  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < samplePerGesture) {
    digitalWrite(LED_BUILTIN, HIGH);
    // check if both new acceleration and gyroscope data is
    // available
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      // read the acceleration and gyroscope data
      IMU.readAcceleration(aX, aY, aZ);
      IMU.readGyroscope(gX, gY, gZ);

      samplesRead++;
      sample.aX[samplesRead-1] = aX;
      sample.aY[samplesRead-1] = aY;
      sample.aZ[samplesRead-1] = aZ;
      sample.gX[samplesRead-1] = gX;
      sample.gY[samplesRead-1] = gY;
      sample.gZ[samplesRead-1] = gZ;

      if (samplesRead == samplePerGesture) {
        digitalWrite(LED_BUILTIN, LOW);

        // serial print sample
        sample.printSampleData(1,"aX");
        sample.printSampleData(1,"aY");
        sample.printSampleData(1,"aZ");
        sample.printSampleData(1,"gX");
        sample.printSampleData(1,"gY");
        sample.printSampleData(1,"gZ");
        //--------------------------------------------------------------------------------------------------------------
        //testDTW();
        //--------------------------------------------------------------------------------------------------------------
        //collectKNNExample();
        //--------------------------------------------------------------------------------------------------------
        //printKNNExample(numberPerGesture*totalGesture, accAxis*totalGesture);
        //--------------------------------------------------------------------------------------------------------
        testModelKNN();
      }
    }
    delay(sampleDelay); // 200 for 10 sec, 100 for 5 sec
  }
}

void testDTW(){
  //test dtw
  for (int i=0; i<numberPerGesture; i++){
    distance.distanceArray[i][0] = dtw_distance(sample.aX, forward[i*allAxis]);
    distance.distanceArray[i][1] = dtw_distance(sample.aY, forward[i*allAxis + 1]);
    distance.distanceArray[i][2] = dtw_distance(sample.aZ, forward[i*allAxis + 2]);
    distance.distanceArray[i][3] = dtw_distance(sample.gX, forward[i*allAxis + 3]);
    distance.distanceArray[i][4] = dtw_distance(sample.gY, forward[i*allAxis + 4]);
    distance.distanceArray[i][5] = dtw_distance(sample.gZ, forward[i*allAxis + 5]);
  }
  Serial.println("compare to forward");
  distance.printDistanceArray();

  for (int i=0; i<numberPerGesture; i++){
    distance.distanceArray[i][0] = dtw_distance(sample.aX, backward[i*allAxis]);
    distance.distanceArray[i][1] = dtw_distance(sample.aY, backward[i*allAxis + 1]);
    distance.distanceArray[i][2] = dtw_distance(sample.aZ, backward[i*allAxis + 2]);
    distance.distanceArray[i][3] = dtw_distance(sample.gX, backward[i*allAxis + 3]);
    distance.distanceArray[i][4] = dtw_distance(sample.gY, backward[i*allAxis + 4]);
    distance.distanceArray[i][5] = dtw_distance(sample.gZ, backward[i*allAxis + 5]);
  }
  Serial.println("compare to backward");
  distance.printDistanceArray();

  for (int i=0; i<numberPerGesture; i++){
    distance.distanceArray[i][0] = dtw_distance(sample.aX, left[i*allAxis]);
    distance.distanceArray[i][1] = dtw_distance(sample.aY, left[i*allAxis + 1]);
    distance.distanceArray[i][2] = dtw_distance(sample.aZ, left[i*allAxis + 2]);
    distance.distanceArray[i][3] = dtw_distance(sample.gX, left[i*allAxis + 3]);
    distance.distanceArray[i][4] = dtw_distance(sample.gY, left[i*allAxis + 4]);
    distance.distanceArray[i][5] = dtw_distance(sample.gZ, left[i*allAxis + 5]);
  }
  Serial.println("compare to left");
  distance.printDistanceArray();

  for (int i=0; i<numberPerGesture; i++){
    distance.distanceArray[i][0] = dtw_distance(sample.aX, right[i*allAxis]);
    distance.distanceArray[i][1] = dtw_distance(sample.aY, right[i*allAxis + 1]);
    distance.distanceArray[i][2] = dtw_distance(sample.aZ, right[i*allAxis + 2]);
    distance.distanceArray[i][3] = dtw_distance(sample.gX, right[i*allAxis + 3]);
    distance.distanceArray[i][4] = dtw_distance(sample.gY, right[i*allAxis + 4]);
    distance.distanceArray[i][5] = dtw_distance(sample.gZ, right[i*allAxis + 5]);
  }
  Serial.println("compare to right");
  distance.printDistanceArray();
}

void collectKNNExample(){
  for(int i = 0; i < numberPerGesture; i++){
    knnExample[i][0] = dtw_distance(forward_Ref[0], forward[i*6+0]);
    knnExample[i][1] = dtw_distance(forward_Ref[1], forward[i*6+1]);
    knnExample[i][2] = dtw_distance(forward_Ref[2], forward[i*6+2]);
    knnExample[i][3] = dtw_distance(backward_Ref[0], forward[i*6+0]);
    knnExample[i][4] = dtw_distance(backward_Ref[1], forward[i*6+1]);
    knnExample[i][5] = dtw_distance(backward_Ref[2], forward[i*6+2]);
    knnExample[i][6] = dtw_distance(left_Ref[0], forward[i*6+0]);
    knnExample[i][7] = dtw_distance(left_Ref[1], forward[i*6+1]);
    knnExample[i][8] = dtw_distance(left_Ref[2], forward[i*6+2]);
    knnExample[i][9] = dtw_distance(right_Ref[0], forward[i*6+0]);
    knnExample[i][10] = dtw_distance(right_Ref[1], forward[i*6+1]);
    knnExample[i][11] = dtw_distance(right_Ref[2], forward[i*6+2]);
  }

  for(int i = 0; i < numberPerGesture; i++){
    knnExample[i+10][0] = dtw_distance(forward_Ref[0], backward[i*6+0]);
    knnExample[i+10][1] = dtw_distance(forward_Ref[1], backward[i*6+1]);
    knnExample[i+10][2] = dtw_distance(forward_Ref[2], backward[i*6+2]);
    knnExample[i+10][3] = dtw_distance(backward_Ref[0], backward[i*6+0]);
    knnExample[i+10][4] = dtw_distance(backward_Ref[1], backward[i*6+1]);
    knnExample[i+10][5] = dtw_distance(backward_Ref[2], backward[i*6+2]);
    knnExample[i+10][6] = dtw_distance(left_Ref[0], backward[i*6+0]);
    knnExample[i+10][7] = dtw_distance(left_Ref[1], backward[i*6+1]);
    knnExample[i+10][8] = dtw_distance(left_Ref[2], backward[i*6+2]);
    knnExample[i+10][9] = dtw_distance(right_Ref[0], backward[i*6+0]);
    knnExample[i+10][10] = dtw_distance(right_Ref[1], backward[i*6+1]);
    knnExample[i+10][11] = dtw_distance(right_Ref[2], backward[i*6+2]);
  }

  for(int i = 0; i < numberPerGesture; i++){
    knnExample[i+20][0] = dtw_distance(forward_Ref[0], left[i*6+0]);
    knnExample[i+20][1] = dtw_distance(forward_Ref[1], left[i*6+1]);
    knnExample[i+20][2] = dtw_distance(forward_Ref[2], left[i*6+2]);
    knnExample[i+20][3] = dtw_distance(backward_Ref[0], left[i*6+0]);
    knnExample[i+20][4] = dtw_distance(backward_Ref[1], left[i*6+1]);
    knnExample[i+20][5] = dtw_distance(backward_Ref[2], left[i*6+2]);
    knnExample[i+20][6] = dtw_distance(left_Ref[0], left[i*6+0]);
    knnExample[i+20][7] = dtw_distance(left_Ref[1], left[i*6+1]);
    knnExample[i+20][8] = dtw_distance(left_Ref[2], left[i*6+2]);
    knnExample[i+20][9] = dtw_distance(right_Ref[0], left[i*6+0]);
    knnExample[i+20][10] = dtw_distance(right_Ref[1], left[i*6+1]);
    knnExample[i+20][11] = dtw_distance(right_Ref[2], left[i*6+2]);
  }

  for(int i = 0; i < numberPerGesture; i++){
    knnExample[i+30][0] = dtw_distance(forward_Ref[0], right[i*6+0]);
    knnExample[i+30][1] = dtw_distance(forward_Ref[1], right[i*6+1]);
    knnExample[i+30][2] = dtw_distance(forward_Ref[2], right[i*6+2]);
    knnExample[i+30][3] = dtw_distance(backward_Ref[0], right[i*6+0]);
    knnExample[i+30][4] = dtw_distance(backward_Ref[1], right[i*6+1]);
    knnExample[i+30][5] = dtw_distance(backward_Ref[2], right[i*6+2]);
    knnExample[i+30][6] = dtw_distance(left_Ref[0], right[i*6+0]);
    knnExample[i+30][7] = dtw_distance(left_Ref[1], right[i*6+1]);
    knnExample[i+30][8] = dtw_distance(left_Ref[2], right[i*6+2]);
    knnExample[i+30][9] = dtw_distance(right_Ref[0], right[i*6+0]);
    knnExample[i+30][10] = dtw_distance(right_Ref[1], right[i*6+1]);
    knnExample[i+30][11] = dtw_distance(right_Ref[2], right[i*6+2]);
  }
}

void printKNNExample(int KNNRowIndex, int KNNColumnIndex){
  Serial.println("KNN example = {");
  for(int i = 0; i < KNNRowIndex; i++) {
    Serial.print("{");
    for(int j = 0; j < KNNColumnIndex; j++)  {
      if (j!=KNNColumnIndex-1){
        Serial.print(String(knnExample[i][j])+", ");
      }
      else{
        Serial.print(String(knnExample[i][j]));
      }
    }
    Serial.print("},");
    Serial.println("");
  }
  Serial.println("}");
}

void createModelKNN(){
  for(int i = 0; i < (totalGesture*numberPerGesture)+addNumberPerGesture_1+addNumberPerGesture_2+addNumberPerGesture_3+addNumberPerGesture_4+addNumberPerGesture_5+addNumberPerGesture_6; i++){
    if (i<numberPerGesture+addNumberPerGesture_1){
      myKNN.addExample(knnExample[i], 1);
    }
    else if (i<numberPerGesture*2+addNumberPerGesture_1+addNumberPerGesture_2){
      myKNN.addExample(knnExample[i], 2);
    }
    else if (i<numberPerGesture*3+addNumberPerGesture_1+addNumberPerGesture_2+addNumberPerGesture_3){
      myKNN.addExample(knnExample[i], 3);
    }
    else if (i<numberPerGesture*4+addNumberPerGesture_1+addNumberPerGesture_2+addNumberPerGesture_3+addNumberPerGesture_4){
      myKNN.addExample(knnExample[i], 4);
    }
  }

  Serial.print("\tmyKNN.getCount() = ");
  Serial.println(myKNN.getCount());
  Serial.println();

  Serial.print("\tmyKNN.getCountByClass(1) = ");
  Serial.println(myKNN.getCountByClass(1));

  Serial.print("\tmyKNN.getCountByClass(2) = ");
  Serial.println(myKNN.getCountByClass(2));

  Serial.print("\tmyKNN.getCountByClass(3) = ");
  Serial.println(myKNN.getCountByClass(3));

  Serial.print("\tmyKNN.getCountByClass(4) = ");
  Serial.println(myKNN.getCountByClass(4)); 
}

void testModelKNN(){
  float input[12];          
  input[0] = dtw_distance(sample.aX, forward_Ref[0]);
  input[1] = dtw_distance(sample.aY, forward_Ref[1]);
  input[2] = dtw_distance(sample.aZ, forward_Ref[2]);
  input[3] = dtw_distance(sample.aX, backward_Ref[0]);
  input[4] = dtw_distance(sample.aY, backward_Ref[1]);
  input[5] = dtw_distance(sample.aZ, backward_Ref[2]);
  input[6] = dtw_distance(sample.aX, left_Ref[0]);
  input[7] = dtw_distance(sample.aY, left_Ref[1]);
  input[8] = dtw_distance(sample.aZ, left_Ref[2]);
  input[9] = dtw_distance(sample.aX, right_Ref[0]);
  input[10] = dtw_distance(sample.aY, right_Ref[1]);
  input[11] = dtw_distance(sample.aZ, right_Ref[2]);

  //noise condition
  if (input[0]+input[1]+input[2] <= noiseThreshold || input[3]+input[4]+input[5]<= noiseThreshold || input[6]+input[7]+input[8]<= noiseThreshold || input[9]+input[10]+input[11]<= noiseThreshold){
    Serial.println("input = ");
    Serial.print("{");
    for(int i = 0; i < 12; i++)  {
      if (i!=11){
        Serial.print(String(input[i])+", ");
      }
      else{
        Serial.print(String(input[i]));
      }
    }
    Serial.print("}");
    Serial.println("");

    int classification = myKNN.classify(input, kValue); // classify input with K=18
    float confidence = myKNN.confidence();

    // print the classification and confidence
    Serial.print("\tclassification = ");
    Serial.println(classification);

    // since there are 2 examples close to the input and K = 3,
    // expect the confidence to be: 2/3 = ~0.67
    Serial.print("\tconfidence     = ");
    Serial.println(confidence);

    Serial1.write(TransmissionGesture[classification-1]);
  }
  else{
    Serial.println("noise gesture");
    Serial1.write("RndNoise");
  }
}