// UM982 Connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
// Keya bits from https://github.com/lansalot/AgOpenGPS_Boards/tree/KeyaV2/TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"
#include <FlexCAN_T4.h>
#include <REG.h>
#include <wit_c_sdk.h>
// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif // ARDUINO_TEENSY41

/************************* User Settings *************************/
bool udpPassthrough = false;  // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool makeOGI = true;          // Set to true to make PAOGI messages. Else PANDA message will be made.
const bool invertRoll = true; // Used for IMU with dual antenna
bool baseLineCheck = false;   // Set to true to use IMU fusion with UM982
float headingOffset = 90;
// #define baseLineLimit 5       // Max CM differance in baseline for fusion

// Roll correction can be entered in the AOG GUI. If not enter roll correction here.
// Roll correction. Negative number = left; positive number = right.
// double rollcorr = 50;

// Serial Ports
#define SerialAOG Serial              // AgIO USB conection
#define SerialRTK Serial2             // RTK radio
#define SerialWT61 Serial3             // IMU

HardwareSerial *SerialGPS = &Serial7; // Main postion receiver (GGA)
const int32_t baudAOG = 115200;       // USB connection speed
const int32_t baudGPS = 460800;       // UM982 connection speed
const int32_t baudRTK = 9600;         // most are using Xbee radios with default of 115200

// Send data to AgIO via usb
bool sendUSB = false;

struct ConfigIP
{
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};

float wheelBase = 2.38;   //of the tractor (meters)
float distanceFromCenterRearAxis = 0.7; // meters
/************************* End User Settings *********************/

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;

#define ImuWire Wire // SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG 57.295779513082320876798154814105 //572.95779513082320876798154814105

// Keya Support
// CRX1/CTX1 on Teensy are CAN1 on Tony's board
// CRX2/CTX2 on Teensy are CAN2 on AIO board, CAN2 on Tony's board
// CRX3/CTX3 on Teensy are CAN1 on AIO board, CAN3 on Tony's board
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;
float KeyaCurrentSensorReading = 0;
float KeyaCurrentRapport = 0;
float KeyaCurrentRapportSmooth = 0;

bool keyaDetected = false; //****************************************************************************************** */

#define REPORT_INTERVAL 20  // BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0; // Used stop BNO data pile up (This version is without resetting BNO everytime)

// Status LED's   
#define GGAReceivedLED 12        // blink if GGA received, ON if RTK, OFF no GGA,     blue
#define DEBUG_LED 13             // ON if debugState > SETUP                          red on board
#define AUTOSTEER_ACTIVE_LED 10  // blink if hello from AOG, ON if steering,          red
#define CAN_ACTIVE_LED 9         // ON if keya heartbeat,                             yellow
#define DEBUG_PIN 37             //button

uint32_t gpsReadyTime = 0;       // Used for GGA timeout      
uint32_t KeyaBeatTime = 0;       // Used for Keya timeout

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void autosteerSetup();
void EthernetStart();
void udpNtrip();
void BuildNmea();
void readBNO();
void autosteerLoop();
void ReceiveUdp();
void checkUM982();
void configureUM982();
void imuSetup();
void passthroughSerial();
void LedSetup();
void getKeyaInfo();

ConfigIP networkAddress; // 3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;           // port of this module
unsigned int AOGNtripPort = 2233;     // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888; // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;  // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];     // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     // Out port 5544
EthernetUDP Eth_udpNtrip;     // In port 2233
EthernetUDP Eth_udpAutoSteer; // In & Out Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;


// Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool dualReadyGGA = false;
bool dualReadyHPR = false;

// booleans to see if we are using BNO08x
bool useBNO08x = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = {0x4A, 0x4B};
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 2040;
uint8_t GPSrxbuffer[serial_buffer_size];  // Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];  // Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];  // Extra serial rx buffer
const int tmp_serial_buffer_size = 2048;
uint8_t tmpGPSrxbuffer[tmp_serial_buffer_size]; // Temp serial rx buffer for detecting / configuring the UM982
uint8_t tmpGPStxbuffer[tmp_serial_buffer_size]; // Temp serial tx buffer for detecting / configuring the UM982

// Baudrates for detecting UM982 receiver
uint32_t baudrates[]{
    460800,
    115200};
const uint32_t nrBaudrates = sizeof(baudrates) / sizeof(baudrates[0]);

// Bools for detecting / configuring the UM982
bool gotUM982 = false;
bool setUM982 = false;

/* A parser is declared with 4 handlers at most */
NMEAParser<4> parser;

bool blink = false;

bool Autosteer_running = false; // Auto set off in autosteer setup
bool Ethernet_running = false; // Auto set on in ethernet setup

float WT61[3];

int32_t keyaEncoderOffset = 0;
int32_t keyaEncoderOffsetNew = 0;

enum debugList {
  SETUP,
  EXPERIMENT,
  ROLL,
  WAS,
  GPS,
  KEYA,
  SWITCH,
  UDP,
  STATE_INFO
};

enum debugList debugState = SETUP;

int8_t debugButton = 1;
uint32_t debugTime = 0;


// Setup procedure ---------------------------------------------------------------------------------------------------------------
void setup()
{
  delay(500); // Small delay so serial can monitor start up

  LedSetup();

  Serial.println("Start setup");
  Serial.println();

  setupWT61();

  checkUM982();

  if (gotUM982)
    configureUM982();

  // the dash means wildcard

  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  parser.addHandler("G-HPR", HPR_Handler);

  delay(10);
  Serial.begin(baudAOG);
  delay(10);

  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  Serial.println("SerialAOG, SerialRTK, SerialGPS initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();

  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  //Serial.println("\r\nStarting BNO085...");
  imuSetup();

  useBNO08x=false; //*********************************

  delay(100);
  //Serial.print("useBNO08x = ");
  //Serial.println(useBNO08x);

  // Keya support
  CAN_Setup();

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");

}

void loop()
{
  // Keya support
  KeyaBus_Receive();

  // WT61 IMU receive
  loopWT61();

  // Read incoming nmea from GPS
  if (SerialGPS->available())
  {
    if (udpPassthrough)
    {
      passthroughSerial();
    }
    else
    {
      parser << SerialGPS->read();
    }
  }

  udpNtrip();

  // Check for RTK Radio
  if (SerialRTK.available())
    SerialGPS->write(SerialRTK.read());

  // If both dual messages are ready, send to AgOpen
  // Serial.print(dualReadyGGA);
  // Serial.println(dualReadyHPR);
  // delay(10);
  if (dualReadyGGA == true && dualReadyHPR == true)
  {
    imuHandler();
    BuildNmea();
    dualReadyGGA = false;
    dualReadyHPR = false;
  }

  // Read BNO
  if ((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
  {
    READ_BNO_TIME = systick_millis_count;
    readBNO();
  }

  //turn off LED if GGa timeout 3 sec
  if (systick_millis_count - gpsReadyTime > 3000)
    digitalWrite(GGAReceivedLED, LOW);

  if(systick_millis_count - KeyaBeatTime > 5000){
    digitalWrite(CAN_ACTIVE_LED, LOW);
    keyaEncoderOffset = 0;
    keyaEncoderOffsetNew = 0;
  }
  if (Autosteer_running)
    autosteerLoop();
  else{
    ReceiveUdp();
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
  }

  debugLoop();

} // End Loop
//**************************************************************************

void checkUM982(){
  uint32_t baudrate = 0;
  for (uint32_t i = 0; i < nrBaudrates; i++)
  {
    baudrate = baudrates[i];
    Serial.print(F("Checking for UM982 at baudrate: "));
    Serial.println(baudrate);
    SerialGPS->begin(baudrate);
    // Increase the size of the serial buffer to hold longer UM982 config messages
    SerialGPS->addMemoryForRead(tmpGPSrxbuffer, tmp_serial_buffer_size);
    SerialGPS->addMemoryForWrite(tmpGPStxbuffer, tmp_serial_buffer_size);
    delay(500);
    SerialGPS->write("VERSION\r\n");
    delay(100);
    while (SerialGPS->available())
    {
      char incoming[100];
      SerialGPS->readBytesUntil('\n', incoming, 100);
      //Serial.print("UM982 VERSION: ");
      //Serial.println(incoming);
      if (strstr(incoming, "UM982") != NULL)
      {
        if (baudrate != 460800)
        {
          Serial.println("UM982 baudrate wrong for AOG. Setting to 460800 bps for AOG");
          SerialGPS->write("CONFIG COM1 460800\r\n");
          delay(100);
          SerialGPS->begin(baudGPS);
        }
        gotUM982 = true;
        break;
      }
      if (gotUM982)
      {
        break;
      }
    }
    if (gotUM982)
    {
      break;
    }
  }
}

void configureUM982(){
  SerialGPS->write("CONFIG\r\n"); // Request the UM982 Configuration
  delay(200);

  while (SerialGPS->available() && !setUM982)
  {
    char incoming[100];
    SerialGPS->readBytesUntil('\n', incoming, 100);

    // Check the "UM982 configured" flag.
    if (strstr(incoming, "CONFIG ANTENNADELTAHEN") != NULL)
    {
      Serial.println("Got the config line");
      if (strstr(incoming, "ANTENNADELTAHEN 0.0099 0.0099 0.0091") != NULL)
      {
        Serial.println("And it is already configured");
        Serial.println();

        // Reset serial buffer size
        SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
        SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
        setUM982 = true;
      }
      else
      {
        Serial.println("And it is not already configured");

        // Clear out the serial channel
        SerialGPS->write("UNLOGALL\r\n");
        while ((SerialGPS->available()))
        {
          SerialGPS->read();
        }
        // Set UM982 operating mode
        Serial.println("Setting rover mode");
        SerialGPS->write("MODE ROVER SURVEY\r\n");
        delay(100);

        // Set heading to variablelength
        Serial.println("Setting heading to variablelength");
        //SerialGPS->write("CONFIG HEADING VARIABLELENGTH\r\n");
        SerialGPS->write("CONFIG HEADING FIXLENGTH\r\n");
        delay(100);

        SerialGPS->write("CONFIG HEADING REALIABILITY 3\r\n");
        delay(100);

        SerialGPS->write("CONFIG HEADING LENGHT 104 2\r\n");
        delay(100);

        SerialGPS->write("CONFIG PPP ENABLE E6-HAS\r\n");
        delay(100);

        SerialGPS->write("CONFIG PPP DATUM WGS84\r\n");
        delay(100);

        SerialGPS->write("CONFIG PPP CONVERGE 30 50\r\n");
        delay(100);

        // Set heading smoothing
        Serial.println("Setting heading smoothing");
        SerialGPS->write("CONFIG SMOOTH HEADING 1\r\n");
        delay(100);

        // Set rtk height smoothing
        Serial.println("Setting rtkheight smoothing");
        SerialGPS->write("CONFIG SMOOTH RTKHEIGHT 10\r\n");
        delay(100);

        // Set COM1 to 460800
        Serial.println("Setting COM1 to 460800 bps");
        SerialGPS->write("CONFIG COM1 460800\r\n");
        delay(100);

        // Set COM2 to 460800
        Serial.println("Setting COM2 to 460800 bps");
        SerialGPS->write("CONFIG COM2 460800\r\n");
        delay(100);

        // Set COM3 to 460800
        Serial.println("Setting COM3 to 460800 bps");
        SerialGPS->write("CONFIG COM3 460800\r\n");
        delay(100);

        // Set GGA message and rate
        Serial.println("Setting GGA");
        SerialGPS->write("GPGGA COM1 0.1\r\n");
        delay(100);

        // Set VTG message and rate
        Serial.println("Setting VTG");
        SerialGPS->write("GPVTG COM1 0.1\r\n");
        delay(100);

        // Set HPR message and rate
        Serial.println("Setting HPR");
        SerialGPS->write("GPHPR COM1 0.05\r\n");
        delay(100);

        // Setting the flag to signal UM982 is configured for AOG
        Serial.println("Setting UM982 configured flag");
        SerialGPS->write("CONFIG ANTENNADELTAHEN 0.0099 0.0099 0.0091\r\n");
        delay(100);

        // Saving the configuration in the UM982
        Serial.println("Saving the configuration");
        SerialGPS->write("SAVECONFIG\r\n");
        Serial.println();
        delay(100);

        // Reset the serial buffer size
        SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
        SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
      }
    }
  }
}

void passthroughSerial(){
  char incoming = SerialGPS->read();
  // Serial.println(incoming);
  switch (incoming)
  {
  case '$':
    msgBuf[msgBufLen] = incoming;
    msgBufLen++;
    gotDollar = true;
    break;
  case '\r':
    msgBuf[msgBufLen] = incoming;
    msgBufLen++;
    gotCR = true;
    gotDollar = false;
    break;
  case '\n':
    msgBuf[msgBufLen] = incoming;
    msgBufLen++;
    gotLF = true;
    gotDollar = false;
    break;
  default:
    if (gotDollar)
    {
      msgBuf[msgBufLen] = incoming;
      msgBufLen++;
    }
    break;
  }
  if (gotCR && gotLF)
  {
    // Serial.print(msgBuf);
    // Serial.println(msgBufLen);
    if (sendUSB)
    {
      SerialAOG.write(msgBuf);
    } // Send USB GPS data if enabled in user settings
    if (Ethernet_running)
    {
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
      Eth_udpPAOGI.write(msgBuf, msgBufLen);
      Eth_udpPAOGI.endPacket();
    }
    gotCR = false;
    gotLF = false;
    gotDollar = false;
    memset(msgBuf, 0, 254);
    msgBufLen = 0;
    if (blink)
    {
      digitalWrite(GGAReceivedLED, HIGH);
    }
    else
    {
      digitalWrite(GGAReceivedLED, LOW);
    }

    blink = !blink;
  }
}

void LedSetup(){
  pinMode(GGAReceivedLED, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
  pinMode(CAN_ACTIVE_LED, OUTPUT);

  digitalWrite(GGAReceivedLED, 1);
  delay(300);
  digitalWrite(GGAReceivedLED, 0);
  delay(300);
  digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
  delay(300);
  digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
  delay(300);
  digitalWrite(CAN_ACTIVE_LED, 1);
  delay(300);
  digitalWrite(CAN_ACTIVE_LED, 0);
  delay(300);
  digitalWrite(GGAReceivedLED, 1);
  digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
  digitalWrite(CAN_ACTIVE_LED, 1);
  delay(800);
  digitalWrite(GGAReceivedLED, 0);
  digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
  digitalWrite(CAN_ACTIVE_LED, 0);
}

void debugLoop(){
  int8_t read=digitalRead(DEBUG_PIN);
  //debug button handler
  if(read == LOW && debugButton == 1){  //just pressed
    if(debugState == STATE_INFO)
      debugState = SETUP;
    else
      debugState=debugState+1;

    switch(debugState){
    case SETUP:
      Serial.println("DEBUG: SETUP");
      break;
    case EXPERIMENT:
      Serial.println("DEBUG: EXPERIMENT");
      break;
    case ROLL:
      Serial.println("DEBUG: ROLL");
      break;
    case WAS:
      Serial.println("DEBUG: WAS");
      break;
    case GPS:
      Serial.println("DEBUG: GPS");
      break;
    case KEYA:
      Serial.println("DEBUG: KEYA");
      break;
    case SWITCH:
      Serial.println("DEBUG: SWITCH");
      break;
    case UDP:
      Serial.println("DEBUG: UDP");
      break;
    case STATE_INFO:
      Serial.println("DEBUG: STATE_INFO");
      break;
    }

    if(debugState == SETUP)
      digitalWrite(DEBUG_LED, LOW);
    else
      digitalWrite(DEBUG_LED, HIGH);

    delay(200);
  }
  debugButton=read;

  if(debugState == STATE_INFO && systick_millis_count - debugTime > 10000){
    getKeyaInfo();
    debugTime = systick_millis_count;
  }
}