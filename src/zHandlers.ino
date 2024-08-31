// Conversion to Hexidecimal
const char *asciiHex = "0123456789ABCDEF";

#define MAXBUFFERLEN 500      //consider 20Hz    500->25 sec
#define MINBUFFERLEN 300      //consider 20Hz    300->15 sec
#define MAXANGLECALC 15
#define MINSPEEDCAL 0.5       //  m/s

float dualWheelAngleBuffer[MAXBUFFERLEN];
float steerAngleActualBuffer[MAXBUFFERLEN];
float WTWheelAngleBuffer[MAXBUFFERLEN];
int bufferLen = MINBUFFERLEN;

#include <SimpleKalmanFilter.h>

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter rollKF(0.1, 0.1, 0.1);


// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = {};
char speedKnots[10] = {};

// HPR
char umHeading[8];
char umRoll[8];
int solQuality;



// If odd characters showed up.
void errorHandler()
{
  // nothing at the moment
}

void GGA_Handler() // Rec'd GGA
{
  // fix time
  parser.getArg(0, fixTime);

  // latitude
  parser.getArg(1, latitude);
  parser.getArg(2, latNS);

  // longitude
  parser.getArg(3, longitude);
  parser.getArg(4, lonEW);

  // fix quality
  parser.getArg(5, fixQuality);

  // satellite #
  parser.getArg(6, numSats);

  // HDOP
  parser.getArg(7, HDOP);

  // altitude
  parser.getArg(8, altitude);

  // time of last DGPS update
  parser.getArg(12, ageDGPS);

  if (blink)
  {
    digitalWrite(GGAReceivedLED, HIGH);
  }
  else
  {
    digitalWrite(GGAReceivedLED, LOW);
  }

  blink = !blink;

  dualReadyGGA = true;

  gpsReadyTime = systick_millis_count; // Used for GGA timeout (LED's ETC)
}

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);  //i can use for reverse detection (180° from dualHeading)
  headingVTG = atof(vtgHeading);

  if(abs((int)(headingVTG-headingDual)%360)>120 && speed>MINSPEEDCAL)  //reverse
    workingDir=-1;
  else
    workingDir=1;


  // vtg Speed knots
  parser.getArg(4, speedKnots);
  speed = atof(speedKnots) * 1852 / 3600; // m/s
}

// UM982 Support
void HPR_Handler()
{
  digitalWrite(GPSRED_LED, LOW); // Turn red GPS LED OFF (we are now in dual mode so green LED)

  // HPR Heading
  parser.getArg(1, umHeading);

  headingDualOld = headingDual;
  headingDual = atof(umHeading);

  // Solution quality factor
  parser.getArg(4, solQuality);

  if (solQuality == 4){  //RTK
    float rate = (headingDual - headingDualOld);
    if(rate>300)
      rate-=360;
    if(rate<-300)
      rate+=360;
    
    headingDualRate = rate / RAD_TO_DEG * 20;  //20Hz update rate

    dualWheelAngle = atan(headingDualRate*wheelBase/speed) * RAD_TO_DEG * workingDir;
    dualWheelAngleWT61 = atan(fGyro[2]/RAD_TO_DEG*wheelBase/speed*-1) * RAD_TO_DEG * workingDir;

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
      keyaEncoderOffsetWT += ((meanWT-meanActual) * steerSettings.steerSensorCounts); //steerSettings.steerSensorCounts;
      indexBuffer = 0;
      bufferLen = MAXBUFFERLEN;
    }
  }




  // HPR Substitute pitch for roll
  if (parser.getArg(2, umRoll))
  {
    rollDual = atof(umRoll)- 0.2;
    digitalWrite(GPSGREEN_LED, HIGH); // Turn green GPS LED ON
    Serial.print("rollDual:");
    Serial.print(rollDual);

    rollDual = rollKF.updateEstimate(rollDual);

    //plotter
    Serial.print(",rollKF:");
    Serial.print(rollDual);
    Serial.print(",angleWT:");
    Serial.println(rollWT, 3);
    //Serial.print(",");
    //Serial.print("accx:");
    //Serial.println(accXWT);
  }
  else
  {
    digitalWrite(GPSGREEN_LED, blink); // Flash the green GPS LED
  }
  
  dualReadyHPR = true; // RelPos ready is true so PAOGI will send when the GGA is also ready
}

void BuildNmea(void)
{
  strcpy(nmea, "");

  if (makeOGI)
    strcat(nmea, "$PAOGI,");
  else
    strcat(nmea, "$PANDA,");

  strcat(nmea, fixTime);
  strcat(nmea, ",");

  strcat(nmea, latitude);
  strcat(nmea, ",");

  strcat(nmea, latNS);
  strcat(nmea, ",");

  strcat(nmea, longitude);
  strcat(nmea, ",");

  strcat(nmea, lonEW);
  strcat(nmea, ",");

  // 6
  strcat(nmea, fixQuality);
  strcat(nmea, ",");

  strcat(nmea, numSats);
  strcat(nmea, ",");

  strcat(nmea, HDOP);
  strcat(nmea, ",");

  strcat(nmea, altitude);
  strcat(nmea, ",");

  // 10
  strcat(nmea, ageDGPS);
  strcat(nmea, ",");

  // 11
  strcat(nmea, speedKnots);
  strcat(nmea, ",");

  // 12
  strcat(nmea, headingPanda);
  strcat(nmea, ",");

  // 13
  strcat(nmea, rollPanda);
  strcat(nmea, ",");

  // 14
  strcat(nmea, pitchPanda);
  strcat(nmea, ",");

  // 15
  strcat(nmea, yawRatePanda);

  strcat(nmea, "*");

  CalculateChecksum();

  strcat(nmea, "\r\n");

  if (sendUSB)
  {
    SerialAOG.write(nmea);
  } // Send USB GPS data if enabled in user settings

  if (Ethernet_running) // If ethernet running send the GPS there
  {
    int len = strlen(nmea);
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(nmea, len);
    Eth_udpPAOGI.endPacket();
  }
  if(debugSerial){
    Serial.print("GPS-");
    Serial.print(millis());
    Serial.print(",");
    Serial.println(nmea);
  }
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp; // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = {asciiHex[chk], 0};
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = {asciiHex[chk], 0};
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/
