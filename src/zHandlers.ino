/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// new KSXT buffer
char ksxt[150];

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

//SXT
char fixTime[18];
char longitude[13];
char latitude[12];
char height[11];
//char sxtHeading[7];
float sxtHeading;
//char sxtPitch[7];
float sxtPitch;
char track[7];
char velocity[8];
char sxtRoll[7];
char posQual[2];
char headQual[2];
char hSats[3];
char mSats[3];
char east[8];
char north[8];
char up[8];
char eastVel[8];
char northVel[8];
char upVel[8];

String double2string(double n, int ndec) {
    String r = "";

    int v = n;
    r += v;     // whole number part
    r += '.';   // decimal point
    int i;
    for (i=0;i<ndec;i++) {
        // iterate through each decimal digit for 0..ndec 
        n -= v;
        n *= 10; 
        v = n;
        r += v;
    }

    return r;
}

// If odd characters showed up.
void errorHandler()
{
  //nothing at the moment
}

void SXT_Handler()
{
  // Fix time
  parser.getArg(0, fixTime);
  // Longitude
  parser.getArg(1, longitude);
  // Latitude
  parser.getArg(2, latitude);
  parser.getArg(3, height);
  parser.getArg(4, sxtHeading);
  parser.getArg(5, sxtPitch);
  parser.getArg(6, track);
  parser.getArg(7, velocity);
  parser.getArg(8, sxtRoll);
  parser.getArg(9, posQual);
  parser.getArg(10, headQual);
  parser.getArg(11, hSats);
  parser.getArg(12, mSats);
  parser.getArg(13, east);
  parser.getArg(14, north);
  parser.getArg(15, up);
  parser.getArg(16, eastVel);
  parser.getArg(17, northVel);
  parser.getArg(18, upVel);

  imuDualDelta();
  imuHandler();
  BuildKsxt();

  if (blink)
  {
      digitalWrite(GGAReceivedLED, HIGH);
  }
  else
  {
      digitalWrite(GGAReceivedLED, LOW);
  }
  blink = !blink;
  digitalWrite(GPSGREEN_LED, HIGH);   //Turn green GPS LED ON
  // GGA_Available = true;

  // dualReadyGGA = true;
  
  gpsReadyTime = systick_millis_count;    //Used for GGA timeout (LED's ETC) 
}

void readBNO()
{
          if (bno08x.dataAvailable() == true)
        {
            float dqx, dqy, dqz, dqw, dacr;
            uint8_t dac;

            //get quaternion
            bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
/*            
            while (bno08x.dataAvailable() == true)
            {
                //get quaternion
                bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
                //Serial.println("Whiling");
                //Serial.print(dqx, 4);
                //Serial.print(F(","));
                //Serial.print(dqy, 4);
                //Serial.print(F(","));
                //Serial.print(dqz, 4);
                //Serial.print(F(","));
                //Serial.println(dqw, 4);
            }
            //Serial.println("End of while");
*/            
            float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
            dqw = dqw / norm;
            dqx = dqx / norm;
            dqy = dqy / norm;
            dqz = dqz / norm;

            float ysqr = dqy * dqy;

            // yaw (z-axis rotation)
            float t3 = +2.0 * (dqw * dqz + dqx * dqy);
            float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
            yaw = atan2(t3, t4);

            // Convert yaw to degrees x10
            correctionHeading = -yaw;
            yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
            if (yaw < 0) yaw += 3600;

            // pitch (y-axis rotation)
            float t2 = +2.0 * (dqw * dqy - dqz * dqx);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
//            pitch = asin(t2) * RAD_TO_DEG_X_10;

            // roll (x-axis rotation)
            float t0 = +2.0 * (dqw * dqx + dqy * dqz);
            float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
//            roll = atan2(t0, t1) * RAD_TO_DEG_X_10;

            if(steerConfig.IsUseY_Axis)
            {
              roll = asin(t2) * RAD_TO_DEG_X_10;
              pitch = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
            else
            {
              pitch = asin(t2) * RAD_TO_DEG_X_10;
              roll = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
            
            if(invertRoll)
            {
              roll *= -1;
            }
        }
}

void imuHandler()
{
  int16_t temp = 0;

  if (useBNO08x)
  {
      //BNO is reading in its own timer    
      // Fill rest of Panda Sentence - Heading
      temp = yaw;
      itoa(temp, imuHeading, 10);

      // the pitch x10
      temp = (int16_t)pitch;
      itoa(temp, imuPitch, 10);

      // the roll x10
      temp = (int16_t)roll;
      itoa(temp, imuRoll, 10);

      // YawRate - 0 for now
      itoa(0, imuYawRate, 10);
  }

  // No else, because we want to use dual heading and IMU roll when both connected
  // We have a IMU so apply the dual/IMU roll/heading error to the IMU data.
  if ( useBNO08x && baseLineCheck)
  {
      float dualTemp;   //To convert IMU data (x10) to a float for the PAOGI so we have the decamal point
              
      // the IMU heading raw
      // dualTemp = yaw * 0.1;
      // dtostrf(dualTemp, 3, 1, imuHeading);          

      // the IMU heading fused to the dual heading
      fuseIMU();
      dtostrf(imuCorrected, 3, 1, imuHeading);
    
      // the pitch
      dualTemp = (int16_t)pitch * 0.1;
      dtostrf(dualTemp, 3, 1, imuPitch);

      // the roll
      dualTemp = (int16_t)roll * 0.1;
      //If dual heading correction is 90deg (antennas left/right) correct the IMU roll
      if(headingcorr == 900)
      {
        dualTemp += rollDeltaSmooth;
      }
      dtostrf(dualTemp, 3, 1, imuRoll);

  }
  else  //Not using IMU so put dual Heading & Roll in direct.
  {
      // the roll
      dtostrf(sxtPitch, 4, 2, imuRoll);

      // the Dual heading raw
      dtostrf(sxtHeading, 4, 2, imuHeading);
  }
}

void imuDualDelta()
{
                                       //correctionHeading is IMU heading in radians
   gpsHeading = sxtHeading * DEG_TO_RAD;  //gpsHeading is Dual heading in radians

   //Difference between the IMU heading and the GPS heading
   gyroDelta = (correctionHeading + imuGPS_Offset) - gpsHeading;
   if (gyroDelta < 0) gyroDelta += twoPI;

   //calculate delta based on circular data problem 0 to 360 to 0, clamp to +- 2 Pi
   if (gyroDelta >= -PIBy2 && gyroDelta <= PIBy2) gyroDelta *= -1.0;
   else
   {
       if (gyroDelta > PIBy2) { gyroDelta = twoPI - gyroDelta; }
       else { gyroDelta = (twoPI + gyroDelta) * -1.0; }
   }
   if (gyroDelta > twoPI) gyroDelta -= twoPI;
   if (gyroDelta < -twoPI) gyroDelta += twoPI;

   //if the gyro and last corrected fix is < 10 degrees or 0.18 radians, super low pass for gps
   if (abs(gyroDelta) < 0.18)
   {
       //a bit of delta and add to correction to current gyro
       imuGPS_Offset += (gyroDelta * (0.1));
       if (imuGPS_Offset > twoPI) imuGPS_Offset -= twoPI;
       if (imuGPS_Offset < -twoPI) imuGPS_Offset += twoPI;
   }
   else
   {
       //a bit of delta and add to correction to current gyro
       imuGPS_Offset += (gyroDelta * (0.2));
       if (imuGPS_Offset > twoPI) imuGPS_Offset -= twoPI;
       if (imuGPS_Offset < -twoPI) imuGPS_Offset += twoPI;
   }

   //So here how we have the difference between the IMU heading and the Dual GPS heading
   //This "imuGPS_Offset" will be used in imuHandler() when the GGA arrives 

   //Calculate the diffrence between dual and imu roll
   float imuRoll;
   imuRoll = (int16_t)roll * 0.1;
   rollDelta = sxtPitch - imuRoll;
   rollDeltaSmooth = (rollDeltaSmooth * 0.7) + (rollDelta * 0.3);

  //  Serial.println("imuDualDelta");
  //  Serial.println(imuGPS_Offset);
  //  Serial.println(rollDeltaSmooth);
}

void fuseIMU()
{     
   //determine the Corrected heading based on gyro and GPS
   imuCorrected = correctionHeading + imuGPS_Offset;
   if (imuCorrected > twoPI) imuCorrected -= twoPI;
   if (imuCorrected < 0) imuCorrected += twoPI;

   imuCorrected = imuCorrected * RAD_TO_DEG; 
}

void BuildKsxt(void) {

  strcpy(ksxt, "");

  strcat(ksxt, "$KSXT,");

  strcat(ksxt, fixTime);
  strcat(ksxt, ",");

  strcat(ksxt, longitude);
  strcat(ksxt, ",");

  strcat(ksxt, latitude);
  strcat(ksxt, ",");

  strcat(ksxt, height);
  strcat(ksxt, ",");

  strcat(ksxt, imuHeading);
  strcat(ksxt, ",");

  strcat(ksxt, imuPitch);
  strcat(ksxt, ",");

  strcat(ksxt, track);
  strcat(ksxt, ",");

  strcat(ksxt, velocity);
  strcat(ksxt, ",");
  
  strcat(ksxt, imuRoll);
  strcat(ksxt, ",");

  strcat(ksxt, posQual);
  strcat(ksxt, ",");

  strcat(ksxt, headQual);
  strcat(ksxt, ",");

  strcat(ksxt, hSats);
  strcat(ksxt, ",");

  strcat(ksxt, mSats);
  strcat(ksxt, ",");

  strcat(ksxt, east);
  strcat(ksxt, ",");

  strcat(ksxt, north);
  strcat(ksxt, ",");

  strcat(ksxt, up);
  strcat(ksxt, ",");

  strcat(ksxt, eastVel);
  strcat(ksxt, ",");

  strcat(ksxt, northVel);
  strcat(ksxt, ",");

  strcat(ksxt, upVel);
  strcat(ksxt, ",,");

  strcat(ksxt, "*");

  CalculateChecksum();

  strcat(ksxt, "\r\n");

  if (sendUSB) { SerialAOG.write(ksxt); } // Send USB GPS data if enabled in user settings
    
  if (Ethernet_running)   //If ethernet running send the GPS there
  {
      int len = strlen(ksxt);
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
      Eth_udpPAOGI.write(ksxt, len);
      Eth_udpPAOGI.endPacket();
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
    tmp = ksxt[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(ksxt, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(ksxt, hex2);
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
