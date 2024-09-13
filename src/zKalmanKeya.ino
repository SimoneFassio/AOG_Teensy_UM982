#include <TinyGPSPlus.h>
#include "CircularBuffer.hpp"

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
#define MINSPEEDCAL 0.5       //  m/s

float dualWheelAngleBuffer[MAXBUFFERLEN];
float steerAngleActualBuffer[MAXBUFFERLEN];
float WTWheelAngleBuffer[MAXBUFFERLEN];
int bufferLen = MINBUFFERLEN;

//KALMAN
double R   = 0.1;            //varianza del rumore sulla misura dell'angolo stimato
double Q   = 1e-02;          // varianza del disturbo sul processo
double Pp  = 0.0;            // P(t|t-1) varianza dell'errore di predizione
double K   = 0.0;            // Kalman gain
double P   = 1.0;            // P(t|t) varianza dell'errore di filtraggio
double Xp  = 0.0;            // x_^(t|t-1) predizione dello stato precedente
double X   = 0.0;            // x_^(t|t) stato filtrato

float steerAngleActualOld=0;


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
  dualWheelAngleWT61 = atan(fGyro[2]/RAD_TO_DEG*wheelBase/speedCorrect*-1) * RAD_TO_DEG * workingDir;

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

    keyaEncoderOffsetNew = keyaEncoderOffset + ((meanDual-meanActual) * steerSettings.steerSensorCounts); //steerSettings.steerSensorCounts;
    keyaEncoderOffsetWT += ((meanWT-meanActual) * steerSettings.steerSensorCounts);
    indexBuffer = 0;
    bufferLen = MAXBUFFERLEN;
  }

  KalmanUpdate();
}

void KalmanUpdate(){
  float angleDiff = steerAngleActual - steerAngleActualOld;              //how the wheelAngle changed according to encoder from last kalman update

  // buffer.unshift(angleDiff);  //add to the head
  // if(buffer.isFull()) 
  //   angleDiff = buffer.pop(); //retrive from tail

  // --- Kalman process ---
  Pp = P + Q;                                 // (PREDICTION) predizione della varianza dell'errore al prossimo step
  Xp = X + angleDiff;                         // (PREDICTION) predizione dello stato al prossimo step
  K = Pp/(Pp + R);                            // (CORRECTION) Kalman gain
  P = (1-K)*Pp;                               // (CORRECTION) aggiornamento della varianza dell'errore di filtraggio
  X = Xp + K*(dualWheelAngle-Xp);             // (CORRECTION) stima di Kalman dell'output del sensore

  steerAngleActualOld = steerAngleActual;

  Serial.print("KalmanState:");
  Serial.println(X);
}