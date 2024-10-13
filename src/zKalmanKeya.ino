#include <TinyGPSPlus.h>
#include "CircularBuffer.hpp"
#include "Correlation.h"

// Create a circular buffer with a capacity of 10
const int bufferSize = 10;
CircularBuffer<float, bufferSize> buffer;

double latOld=0;
double lonOld=0;
double courseTo=0;
float tinyWheelAngle=0;
double courseOld=0;

#define MAXBUFFERLEN 500      //consider 20Hz    500->25 sec
#define MINBUFFERLEN 300      //consider 20Hz    300->15 sec
#define MAXANGLECALC 15
#define MINSPEEDCAL 0.8       //  m/s

float dualWheelAngleBuffer[MAXBUFFERLEN];
float steerAngleActualBuffer[MAXBUFFERLEN];
float WTWheelAngleBuffer[MAXBUFFERLEN];
int bufferLen = MINBUFFERLEN;

//KALMAN
double R   = 1.5;              //varianza del rumore sulla misura dell'angolo stimato
double Q   = 1e-03;          // varianza del disturbo sul processo
double Pp  = 0.0;            // P(t|t-1) varianza dell'errore di predizione
double K   = 0.0;            // Kalman gain
double P   = 1.0;            // P(t|t) varianza dell'errore di filtraggio
double Xp  = 0.0;            // x_^(t|t-1) predizione dello stato precedente

float steerAngleActualOld = 0;
float kalmanErrorSum = 0;
int counter = 0;

Correlation dualC;
Correlation WTC;
Correlation KalmanC;


void TinyGPSloop(){
    double rate;
    char* ptr; // just a dummy value needed for strtod
    double lon = strtod(longitude, &ptr);
    double lat = strtod(latitude, &ptr);
    double distance = TinyGPSPlus::distanceBetween(lat, lon, latOld, lonOld);
    // Serial.print("distance:");
    // Serial.println(distance);
    if(distance>0.5){ //ho fatto mezzo metro
      courseTo = TinyGPSPlus::courseTo(latOld, lonOld, lat, lon);
      if(debugState == EXPERIMENT){
        Serial.print("course:");
        Serial.println(courseTo);
      }
      rate = (courseOld-courseTo);
      if(rate>300)
        rate-=360;
      if(rate<-300)
        rate+=360;
      tinyWheelAngle = atan(rate/RAD_TO_DEG*wheelBase/distance) * RAD_TO_DEG * workingDir;
      courseOld=courseTo;
      latOld=lat;
      lonOld=lon;
    }
}

void angleStimeUpdate(){
  dualWheelAngle = atan(headingDualRate/RAD_TO_DEG*wheelBase/speedCorrect) * RAD_TO_DEG * workingDir;
  dualWheelAngleWT61 = atan(headingRateWT/RAD_TO_DEG*wheelBase/speedCorrect*-1) * RAD_TO_DEG * workingDir;

  if(abs(dualWheelAngle-steerAngleActual)<10 && abs(steerAngleActual)<MAXANGLECALC && speed>MINSPEEDCAL){ //se non c'è troppo differenza tra stima e reale (escludo manovre e retromarce) e non sto sterzando troppo e se andiamo almeno a 0.5*3.6=1.8 Km/h
    dualWheelAngleBuffer[indexBuffer] = dualWheelAngle; 
    steerAngleActualBuffer[indexBuffer] = steerAngleActual;
    WTWheelAngleBuffer[indexBuffer] = dualWheelAngleWT61;
    indexBuffer++;
  }
  else{                                   //altrimenti scarto precedenti misurazioni
    indexBuffer = 0;
    if(abs(steerAngleActual)>MAXANGLECALC)  //shorter time only if i have turned back
      bufferLen = MINBUFFERLEN;
  }

  if(debugState == EXPERIMENT){
    Serial.print("tinyAn:");
    Serial.print(tinyWheelAngle);
    Serial.print(",DualAn:");
    Serial.print(dualWheelAngle);
    Serial.print(",WTAn:");
    Serial.println(dualWheelAngleWT61);
  }

  if(indexBuffer==bufferLen){   //buffer full -> calculate offset
    float sumDual=0;
    float sumActual=0;
    float sumWT=0;
    for(int i=0; i<bufferLen; i++){
      sumDual += dualWheelAngleBuffer[i];
      sumActual += steerAngleActualBuffer[i];
      sumWT += WTWheelAngleBuffer[i];
    }
    float meanDual = sumDual/(float)bufferLen;
    float meanActual = sumActual/(float)bufferLen;
    float meanWT = sumWT/(float)bufferLen;

    //if (!steerConfig.InvertWAS) ////////////////////
      //keyaEncoderOffsetNew = keyaEncoderOffset + ((meanWT-meanActual) * steerSettings.steerSensorCounts); //steerSettings.steerSensorCounts;
    keyaEncoderOffsetWT += ((meanWT-meanActual) * steerSettings.steerSensorCounts);
    indexBuffer = 0;
    bufferLen = MAXBUFFERLEN;
  }
  //if(guidanceStatus && speed>MINSPEEDCAL)
  if(abs(dualWheelAngle-steerAngleActual)<10 && abs(steerAngleActual)<MAXANGLECALC && speed>MINSPEEDCAL){ //se non c'è troppo differenza tra stima e reale (escludo manovre e retromarce) e non sto sterzando troppo e se andiamo almeno a 0.5*3.6=1.8 Km/h
    KalmanUpdate();
    correlationLoop();
  }
}

void KalmanUpdate(){
  float angleDiff = ((float)(keyaEncoderValue + keyaEncoderOffset) /  steerSettings.steerSensorCounts) - steerAngleActualOld;              //how the wheelAngle changed according to encoder from last kalman update

  // buffer.unshift(angleDiff);  //add to the head
  // if(buffer.isFull()) 
  //   angleDiff = buffer.pop(); //retrive from tail

  // --- Kalman process ---
  Pp = P + Q;                                 // (PREDICTION) predizione della varianza dell'errore al prossimo step
  Xp = X + angleDiff;                         // (PREDICTION) predizione dello stato al prossimo step
  K = Pp/(Pp + R);                            // (CORRECTION) Kalman gain
  P = (1-K)*Pp;                               // (CORRECTION) aggiornamento della varianza dell'errore di filtraggio
  X = Xp + K*(dualWheelAngleWT61-Xp);             // (CORRECTION) stima di Kalman dell'output del sensore

  // if (steerConfig.InvertWAS){ ///////////////////////////
  //   kalmanErrorSum += K*(dualWheelAngleWT61-Xp);
  //   counter++;
  //   if(counter==100){
  //     keyaEncoderOffsetNew = keyaEncoderOffset + (kalmanErrorSum/counter * steerSettings.steerSensorCounts);
  //     kalmanErrorSum=0;
  //     counter=0;
  //   }
  //   keyaEncoderOffsetNew=0; /////////////////////////////////////////
  // }

  steerAngleActualOld = (float)(keyaEncoderValue) /  steerSettings.steerSensorCounts;

  if(debugState == EXPERIMENT){
    Serial.print("KalmanState:");
    Serial.println(X);
  }
}

void correlationSetup(){
  dualC.clear();
  dualC.setRunningCorrelation(true);

  WTC.clear();
  WTC.setRunningCorrelation(true);

  KalmanC.clear();
  KalmanC.setRunningCorrelation(true);
}

void correlationLoop(){
  if(debugState == CORRELATION){
    dualC.add(dualWheelAngle, steerAngleSens);
    WTC.add(dualWheelAngleWT61, steerAngleSens);
    KalmanC.add(X, steerAngleSens);

    // Serial.print("dualR");
    // Serial.print(dualC.getR());
    // Serial.print("dualA");
    // Serial.print(dualC.getA());
    // Serial.print("dualB");
    // Serial.print(dualC.getB());
    // Serial.print("WTR");
    // Serial.print(WTC.getR());
    // Serial.print("WTA");
    // Serial.print(WTC.getA());
    // Serial.print("WTB");
    // Serial.print(WTC.getB());
    // Serial.print("KalmanR");
    // Serial.print(KalmanC.getR());
    // Serial.print("KalmanA");
    // Serial.print(KalmanC.getA());
    // Serial.print("KalmanB");
    // Serial.print(KalmanC.getB());
  }
}