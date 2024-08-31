/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/
/*
"Barrowed" Keya code from Matt Elias @ https://github.com/m-elias/AgOpenGPS_Boards/tree/575R-Keya/TeensyModules/V4.1"
*/

uint8_t keyaEncoderQuery[] = {0x40, 0x04, 0x21, 0x01};
uint8_t keyaEncoderSpeedQuery[] = {0x40, 0x03, 0x21, 0x01};

/// to move
int indexBuffer=0;


////////////////// User Settings /////////////////////////

// How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

#define KEYA_OFFSET_INCREMENT 2 // 1->about 0.5°/s

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2400

//   ***********  Motor drive connections  **************888
// Connect ground only for cytron, Connect Ground and +5v for IBT2

// Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE 4

// PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM 2

// Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM 3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37 //reboot teensy  //debug

// Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS); // Use this for the 16-bit version ADS1115

#include <IPAddress.h>
#include "BNO08x_AOG.h"

#ifdef ARDUINO_TEENSY41
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#ifdef ARDUINO_TEENSY41
// uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE]; // Buffer For Receiving UDP Data
#endif

// loop time variables in microseconds
const uint16_t LOOP_TIME = 25; // 40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

// Heart beat hello AgIO
uint8_t helloFromIMU[] = {128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71};
uint8_t helloFromAutoSteer[] = {0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71};
int16_t helloSteerPosition = 0;

// fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = {0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

// fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = {0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading = 0;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

// EEPROM
int16_t EEread = 0;

// Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

// Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

// On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

// speed sent as *10
float gpsSpeed = 0;

// steering variables
float steerAngleActual = 0;
float steerAngleSens = 0;
float steerAngleSetPoint = 0; // the desired angle from AgOpen
int16_t steeringPosition = 0; // from steering sensor
float steerAngleError = 0;    // setpoint - actual
int32_t keyaEncoderValue = 0;
float keyaEncoderSpeed = 0;
float dualWheelAngle = 0;
int8_t workingDir = 1; // 1 forward, -1 reverse
float dualWheelAngleIMU = 0;
float dualWheelAngleWT61 = 0;

float dualWheelAngleV[400];
float dualWheelAngleWT61V[400];
float dualWheelAngleAverage=0;
int indexV=0;
int indexWT61V=0;

int32_t keyaEncoderOffset = 0;
int32_t keyaEncoderOffsetNew = 0;
int32_t keyaEncoderOffsetWT = 0;
int32_t keyaEncoderOffsetTiny = 0;
int32_t keyaEncoderOffsetNewWT = 0;


// pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

// Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;

// Variables for settings
struct Storage
{
  uint8_t Kp = 40;     // proportional gain
  uint8_t lowPWM = 10; // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60; // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1; // sent as percent
};
Storage steerSettings; // 11 bytes

// Variables for settings - 0 is false
struct Setup
{
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0; // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0; // 1 if switch selected
  uint8_t SteerButton = 0; // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0; // Set to 0 to use X Axis, 1 to use Y avis
};
Setup steerConfig; // 9 bytes

void steerConfigInit()
{
  if (steerConfig.CytronDriver)
  {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

void steerSettingsInit()
{
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup()
{
  // PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_LPWM, 490);
    analogWriteFrequency(PWM2_RPWM, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_LPWM, 122);
    analogWriteFrequency(PWM2_RPWM, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_LPWM, 3921);
    analogWriteFrequency(PWM2_RPWM, 3921);
  }

  // keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Disable digital inputs for analog input pins
  pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  // set up communication
  Wire1.end();
  Wire1.begin();

  // Check ADC
  if (adc.testConnection())
  {
    Serial.println("ADC Connecton OK");
  }
  else
  {
    Serial.println("ADC Connecton FAILED!");
    Autosteer_running = false;
  }

  // 50Khz I2C
  // TWBR = 144;   //Is this needed?

  EEPROM.get(0, EEread); // read identifier

  if (EEread != EEP_Ident) // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
    // EEPROM.put(70, analogWork);
  }
  else
  {
    EEPROM.get(10, steerSettings); // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress);
    // EEPROM.get(70, analogWork);
  }

  steerSettingsInit();
  steerConfigInit();

  if (Autosteer_running)
  {
    Serial.println("Autosteer running, waiting for AgOpenGPS");
    // Autosteer Led goes Red if ADS1115 is found
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    digitalWrite(AUTOSTEER_STANDBY_LED, 1);
  }
  else
  {
    Autosteer_running = false; // Turn off auto steer if no ethernet (Maybe running T4.0)
    //    if(!Ethernet_running)Serial.println("Ethernet not available");
    Serial.println("Autosteer disabled, GPS only mode");
    return;
  }

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); // 128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

  pinMode(A12, INPUT_PULLUP);

} // End of Setup

void autosteerLoop()
{
#ifdef ARDUINO_TEENSY41
  ReceiveUdp();
#endif
  // Serial.println("AutoSteer loop");

  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = systick_millis_count;

  if (currentTime - autsteerLastTime >= LOOP_TIME)
  {
    autsteerLastTime = currentTime;

    // If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250)
      watchdogTimer = WATCHDOG_FORCE_VALUE;

    // read section inputs
    // readSectionInputs();

    // read all the switches
    workSwitch = digitalRead(WORKSW_PIN); // read work switch

    if (steerConfig.SteerSwitch == 1) // steer switch on - off
    {
      // steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off

      // new code for steer "Switch" mode that keeps AutoSteer OFF after current/pressure kickout until switch is cycled
      reading = digitalRead(STEERSW_PIN);
      if (reading == HIGH) // switching "OFF"
      {
        steerSwitch = reading;
      }
      else if (reading == LOW && previous == HIGH)
      {
        steerSwitch = reading;
      }
      previous = reading;
      // end new code
    }
    else if (steerConfig.SteerButton == 1) // steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    }
    else // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
      }
    }

    // if (steerConfig.ShaftEncoder)  //when active no offest for initial calibration
    // {
    //   //keyaEncoderOffset=0;
    // }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
      sensorSample *= 0.25;
      sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
      if (keyaDetected) // means Keya HB was detected
      {
        sensorReading =  KeyaCurrentRapportSmooth; // then use keya current data
        sensorReading = min(sensorReading, 255);
      }
      else // otherwise continue using analog input on PCB
      {
        sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
        sensorSample = (abs(775 - sensorSample)) * 0.5;
        sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
        sensorReading = min(sensorReading, 255);
      }

      if (sensorReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
    }

    remoteSwitch = digitalRead(REMOTE_PIN);
    if (!remoteSwitch)
    {
      SCB_AIRCR = 0x05FA0004; // Teensy Reboot
    }
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); // put remote in bit 2
    switchByte |= (steerSwitch << 1);  // put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
      #if Relay_Type == 1
        SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
      #endif
    */

    // get steering position
    if (steerConfig.SingleInputWAS) // Single Input ADS
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
      steeringPosition = adc.getConversion();
      adc.triggerConversion(); // ADS1115 Single Mode

      steeringPosition = (steeringPosition >> 1); // bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }
    else // ADS1115 Differential Mode
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();

      steeringPosition = (steeringPosition >> 1); // bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }

    // DETERMINE ACTUAL STEERING POSITION

    keyaCommand(keyaEncoderQuery);
    delay(2);
    KeyaBus_Receive();

    // update keyaOffset about at 0.5° per second
    if(keyaEncoderOffsetNew>keyaEncoderOffset){
      if(keyaEncoderOffset+KEYA_OFFSET_INCREMENT > keyaEncoderOffsetNew)
        keyaEncoderOffset=keyaEncoderOffsetNew;
      else
        keyaEncoderOffset+=KEYA_OFFSET_INCREMENT;
    }
    else if(keyaEncoderOffsetNew<keyaEncoderOffset){
      if(keyaEncoderOffset-KEYA_OFFSET_INCREMENT < keyaEncoderOffsetNew)
        keyaEncoderOffset=keyaEncoderOffsetNew;
      else
        keyaEncoderOffset-=KEYA_OFFSET_INCREMENT;
    }

    steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset); // 1/2 of full scale

    steerAngleSens = (float)(steeringPosition) / 76.0f; // steerSettings.steerSensorCounts;
      // Ackerman fix
      if (steerAngleSens < 0)
        steerAngleSens = (steerAngleSens * 91);


    //   ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
      steerAngleActual = (float)(keyaEncoderValue + keyaEncoderOffset) /  steerSettings.steerSensorCounts;   //steerSettings.steerSensorCounts;
    }
    else
    {
      steerAngleActual = (float)(keyaEncoderValue) /  steerSettings.steerSensorCounts;
    }

    

    if(debugSerial){
      Serial.print("SENS-");
      Serial.print(millis());
      Serial.print(",");
      Serial.println(steerAngleActual);


      keyaCommand(keyaEncoderQuery);
      delay(2);
      KeyaBus_Receive();

      Serial.print("KEYAV-");
      Serial.print(millis());
      Serial.print(",");
      Serial.println(keyaEncoderValue);

      keyaCommand(keyaEncoderSpeedQuery);
      delay(2);
      KeyaBus_Receive();

      Serial.print("KEYAS-");
      Serial.print(millis());
      Serial.print(",");
      Serial.println(keyaEncoderSpeed);
    }

    dualWheelAngleV[indexV++]=dualWheelAngle;
    indexV=indexV%400;
    dualWheelAngleWT61V[indexV++]=steerAngleActual;
    float sumDual =0;
    float sumActual =0;
    for(int k=0; k<400; k++){
      sumDual+=dualWheelAngleV[k];
      sumActual+=dualWheelAngleWT61V[k];
    }
    float meanDual=sumDual/400.0f;
    float meanActual=sumActual/400.0f;


    Serial.print(indexBuffer);
    Serial.print("\t");

    Serial.print("sens:");
    Serial.print(steerAngleSens);
    Serial.print(",");
    // Serial.print("dualEst:");
    // Serial.print(dualWheelAngle);
    // Serial.print(",");
    // Serial.print("meanDual:");
    // Serial.print(meanDual);
    // Serial.print(",");
    // Serial.print("meanActual:");
    // Serial.print(meanActual);
    // Serial.print(",");
    Serial.print("Keya:");
    Serial.println(steerAngleActual);
    // Serial.print(",");
    // Serial.print("KeyaWT:");
    // Serial.print((float)(keyaEncoderValue + keyaEncoderOffsetWT) / 86.0f);

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      // Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 0);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 1);
        }
      }
      else
        digitalWrite(DIR1_RL_ENABLE, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint; // calculate the steering error
      // if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID(); // do the pid
      motorDrive();      // out to motors the pwm value
      // Autosteer Led goes GREEN if autosteering

      digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
      digitalWrite(AUTOSTEER_STANDBY_LED, 0);
    }
    else
    {
      // we've lost the comm to AgOpenGPS, or just stop request
      // Disable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 1);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 0);
        }
      }
      else
        digitalWrite(DIR1_RL_ENABLE, 0); // IBT2

      pwmDrive = 0; // turn off steering motor
      motorDrive(); // out to motors the pwm value
      // Autosteer Led goes back to RED when autosteering is stopped
      digitalWrite(AUTOSTEER_STANDBY_LED, 1);
      digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    }
  } // end of timed loop

  // This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  // delay(1);
} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  uint16_t len = Eth_udpAutoSteer.parsePacket();

  // if (len > 0)
  // {
  //  Serial.print("ReceiveUdp: ");
  //  Serial.println(len);
  // }

  // Check for len > 4, because we check byte 0, 1, 3 and 3
  if (len > 4)
  {
    Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

    if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) // Data
    {
      if (autoSteerUdpData[3] == 0xFE && Autosteer_running) // 254
      {
        gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;  //is negative when in reverse? No!
        gpsSpeedUpdateTimer = 0;

        prevGuidanceStatus = guidanceStatus;

        guidanceStatus = autoSteerUdpData[7];
        guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

        // Bit 8,9    set point steer angle * 100 is sent
        steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; // high low bytes

        // Serial.print("steerAngleSetPoint: ");
        // Serial.println(steerAngleSetPoint);

        // Serial.println(gpsSpeed);
        // Serial.println(millis());

        if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
        {
          watchdogTimer = WATCHDOG_FORCE_VALUE; // turn off steering motor
        }
        else // valid conditions to turn on autosteer
        {
          watchdogTimer = 0; // reset watchdog
        }

        // Bit 10 Tram
        tram = autoSteerUdpData[10];

        // Bit 11
        relay = autoSteerUdpData[11];

        // Bit 12
        relayHi = autoSteerUdpData[12];

        //----------------------------------------------------------------------------
        // Serial Send to agopenGPS

        int16_t sa = (int16_t)(steerAngleActual * 100);

        PGN_253[5] = (uint8_t)sa;
        PGN_253[6] = sa >> 8;

        // heading
        PGN_253[7] = (uint8_t)9999;
        PGN_253[8] = 9999 >> 8;

        // roll
        PGN_253[9] = (uint8_t)8888;
        PGN_253[10] = 8888 >> 8;

        PGN_253[11] = switchByte;
        PGN_253[12] = (uint8_t)pwmDisplay;

        // checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < PGN_253_Size; i++)
          CK_A = (CK_A + PGN_253[i]);

        PGN_253[PGN_253_Size] = CK_A;

        // off to AOG
        SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

        // Steer Data 2 -------------------------------------------------
        if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
        {
          if (aog2Count++ > 2)
          {
            // Send fromAutosteer2
            PGN_250[5] = (byte)sensorReading;

            // add the checksum for AOG2
            CK_A = 0;

            for (uint8_t i = 2; i < PGN_250_Size; i++)
            {
              CK_A = (CK_A + PGN_250[i]);
            }

            PGN_250[PGN_250_Size] = CK_A;

            // off to AOG
            SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
            aog2Count = 0;
          }
        }

        //Serial.println(steerAngleActual);
        //--------------------------------------------------------------------------
      }

      // steer settings
      else if (autoSteerUdpData[3] == 0xFC && Autosteer_running) // 252
      {
        // PID values
        steerSettings.Kp = ((float)autoSteerUdpData[5]); // read Kp from AgOpenGPS

        steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

        steerSettings.lowPWM = (float)autoSteerUdpData[7]; // read lowPWM from AgOpenGPS

        steerSettings.minPWM = autoSteerUdpData[8]; // read the minimum amount of PWM for instant on

        float temp = (float)steerSettings.minPWM * 1.2;
        steerSettings.lowPWM = (byte)temp;

        steerSettings.steerSensorCounts = autoSteerUdpData[9]; // sent as setting displayed in AOG

        steerSettings.wasOffset = (autoSteerUdpData[10]); // read was zero offset Lo

        steerSettings.wasOffset |= (autoSteerUdpData[11] << 8); // read was zero offset Hi

        steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

        // crc
        // autoSteerUdpData[13];

        // store in EEPROM
        EEPROM.put(10, steerSettings);

        // Re-Init steer settings
        steerSettingsInit();
      }

      else if (autoSteerUdpData[3] == 0xFB) // 251 FB - SteerConfig
      {
        uint8_t sett = autoSteerUdpData[5]; // setting0

        if (bitRead(sett, 0))
          steerConfig.InvertWAS = 1;
        else
          steerConfig.InvertWAS = 0;
        if (bitRead(sett, 1))
          steerConfig.IsRelayActiveHigh = 1;
        else
          steerConfig.IsRelayActiveHigh = 0;
        if (bitRead(sett, 2))
          steerConfig.MotorDriveDirection = 1;
        else
          steerConfig.MotorDriveDirection = 0;
        if (bitRead(sett, 3))
          steerConfig.SingleInputWAS = 1;
        else
          steerConfig.SingleInputWAS = 0;
        if (bitRead(sett, 4))
          steerConfig.CytronDriver = 1;
        else
          steerConfig.CytronDriver = 0;
        if (bitRead(sett, 5))
          steerConfig.SteerSwitch = 1;
        else
          steerConfig.SteerSwitch = 0;
        if (bitRead(sett, 6))
          steerConfig.SteerButton = 1;
        else
          steerConfig.SteerButton = 0;
        if (bitRead(sett, 7))
          steerConfig.ShaftEncoder = 1;
        else
          steerConfig.ShaftEncoder = 0;

        steerConfig.PulseCountMax = autoSteerUdpData[6];

        // was speed
        // autoSteerUdpData[7];

        sett = autoSteerUdpData[8]; // setting1 - Danfoss valve etc

        if (bitRead(sett, 0))
          steerConfig.IsDanfoss = 1;
        else
          steerConfig.IsDanfoss = 0;
        if (bitRead(sett, 1))
          steerConfig.PressureSensor = 1;
        else
          steerConfig.PressureSensor = 0;
        if (bitRead(sett, 2))
          steerConfig.CurrentSensor = 1;
        else
          steerConfig.CurrentSensor = 0;
        if (bitRead(sett, 3))
          steerConfig.IsUseY_Axis = 1;
        else
          steerConfig.IsUseY_Axis = 0;

        // crc
        // autoSteerUdpData[13];

        EEPROM.put(40, steerConfig);

        // Re-Init
        steerConfigInit();

      }                                    // end FB
      else if (autoSteerUdpData[3] == 200) // Hello from AgIO
      {
        if (Autosteer_running)
        {
          int16_t sa = (int16_t)(steerAngleActual * 100);

          helloFromAutoSteer[5] = (uint8_t)sa;
          helloFromAutoSteer[6] = sa >> 8;

          helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
          helloFromAutoSteer[8] = helloSteerPosition >> 8;
          helloFromAutoSteer[9] = switchByte;

          SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
        }
        if (useBNO08x)
        {
          SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
        }
      }

      else if (autoSteerUdpData[3] == 201)
      {
        // make really sure this is the subnet pgn
        if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
        {
          networkAddress.ipOne = autoSteerUdpData[7];
          networkAddress.ipTwo = autoSteerUdpData[8];
          networkAddress.ipThree = autoSteerUdpData[9];

          // save in EEPROM and restart
          EEPROM.put(60, networkAddress);
          SCB_AIRCR = 0x05FA0004; // Teensy Reset
        }
      } // end 201

      // whoami
      else if (autoSteerUdpData[3] == 202)
      {
        // make really sure this is the reply pgn
        if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
        {
          IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

          // hello from AgIO
          uint8_t scanReply[] = {128, 129, Eth_myip[3], 203, 7,
                                 Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
                                 rem_ip[0], rem_ip[1], rem_ip[2], 23};

          // checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
          {
            CK_A = (CK_A + scanReply[i]);
          }
          scanReply[sizeof(scanReply) - 1] = CK_A;

          static uint8_t ipDest[] = {255, 255, 255, 255};
          uint16_t portDest = 9999; // AOG port that listens

          // off to AOG
          SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
        }
      }
      else if (autoSteerUdpData[3] == 238) // machine config 0xEE
      {
        Serial.println("0xEE - Machine Config");

        // analogWork.thresh = autoSteerUdpData[9];
        // analogWork.hyst = autoSteerUdpData[10];
        // pressSensor.hiTriggerLevel = autoSteerUdpData[11];
        // pressSensor.loTriggerLevel = autoSteerUdpData[12];
        // EEPROM.put(70, analogWork);

        // Serial << "user1: " << analogWork.thresh << " user2: " << analogWork.hyst << " user3: " << autoSteerUdpData[11] << " user4: " << autoSteerUdpData[12] << "\r\n";
      }

    } // end if 80 81 7F
  }
}
#endif

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}
#endif