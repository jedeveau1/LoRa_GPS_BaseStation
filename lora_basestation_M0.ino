// [v1.1] - first fielded version
// [v1.2] - Changed SF = 10
// [v1.3] - 1/31/22 - Changed for SN#2 and beyond - fixed color bitmap
// [v1.35] - 1/4/23 - add version info and fix bug for basestation GPS not updating without remote GPS update
// [v1.5] - 7/10/24 - expand # of channels to 15
// [v2.0] - 6/21/25 - Updates for larger screen, saving LKG, etc
// [v2.01] - 8/15/25 - Fix "black" color for background for display 

#include <FlashAsEEPROM.h>

#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ILI9341.h> // Hardware-specific library for ILI9341 graphics chip
#include <SPI.h>
#include <RH_RF95.h>
#include <QMC5883LCompass.h>
#include <Button.h>
#include <Adafruit_FT6206.h>

// BS_VERSION - Basestation Version string
static const char* BS_VERSION = "v2.01";

#define LOCAL_GPS_BAUD  38400 // set to either 9600 for BN series GPS module, or 38400 for BE series

#define MAX_DISTANCE  100000

// for Feather32u4 RFM9x
//#define RFM95_CS 8
//#define RFM95_RST 4
//#define RFM95_INT 7

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LCD_DISPLAY 1
//#define SERIAL_CONSOLE 1
#define COMPASS 1

#define LOCAL_GPS 1

#define SN_NO_1 0 // SN #1 has different display bitmap

// The TinyGPS++ object
TinyGPSPlus gps;
TinyGPSPlus gpsBS;   //basestation local GPS object

// last known good Remote Lat/long [v1.4]
typedef struct
{
  double Lat;
  double Long;
} GPS_Pos_struct;

GPS_Pos_struct  LKG_Position;

typedef struct {
  uint8_t x0;
  uint8_t y0;
  uint8_t x1;
  uint8_t y1;
  uint8_t x2;
  uint8_t y2;
} Tracker_struct;

#define NUM_POINTS 18
#define INCREMENT_DEGS	20

/*
// an arrow for each 20 degs
static const Tracker_struct Tracker_icon[NUM_POINTS] = {{80,1,76,13,85,13},  //0
{102,5,92,15,102,18}, // 20
{120,16,109,22,117,29},
{135,33,121,34,127,43},
{142,53,129,50,131,60},
{142,75,131,68,129,78},
{135,96,127,85,121,94},
{120,112,117,99,109,106},
{102,123,102,110,92,113},
{80,127,85,115,75,115},
{58,123,68,113,58,110},
{40,112,51,106,43,99},
{25,96,39,94,33,85},
{18,75,31,78,29,68},
{18,53,29,60,31,50},
{25,33,33,43,39,34},
{40,16,43,29,51,22},
{58,5,58,18,68,15}};  // 340
*/

static const Tracker_struct Tracker_icon[NUM_POINTS] = {
  {120,   0, 110,  21, 130,  21},  //   0°
  {161,   7, 145,  23, 163,  30},  //  20°
  {197,  28, 176,  37, 192,  50},  //  40°
  {224,  60, 201,  62, 211,  79},  //  60°
  {238,  99, 216,  93, 220, 113},  //  80°
  {238, 141, 220, 127, 216, 147},  // 100°
  {224, 180, 211, 161, 201, 178},  // 120°
  {197, 212, 192, 190, 176, 203},  // 140°
  {161, 233, 163, 210, 145, 217},  // 160°
  {120, 240, 130, 219, 110, 219},  // 180°
  { 79, 233,  95, 217,  77, 210},  // 200°
  { 43, 212,  64, 203,  48, 190},  // 220°
  { 16, 180,  39, 178,  29, 161},  // 240°
  {  2, 141,  24, 147,  20, 127},  // 260°
  {  2,  99,  20, 113,  24,  93},  // 280°
  { 16,  60,  29,  79,  39,  62},  // 300°
  { 43,  28,  48,  50,  64,  37},  // 320°
  { 79,   7,  77,  30,  95,  23}   // 340°
};


    
int displayIndex = 0;
static bool usingLKG; // indicates using Last-Known-Good and waiting for actual update
bool updatedGPSFix = false;

// These pins will also work for the 1.8" TFT shield.
#define TFT_CS        6 // Adafruit Feather
#define TFT_RST        5 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         10
#define TFT_MOSI 11  // Data out
#define TFT_SCLK 12 // Clock out
#define TFT_MISO 9  // not connected but needed for the constructor
#define TOUCH_RESET A5

// For 1.44" and 1.8" TFT with ST7735 use:
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);   // SW SPI
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ts = Adafruit_FT6206();

#ifdef COMPASS

QMC5883LCompass compass;
#endif

// Button object = Pin 11
//Button button = Button(A5,PULLUP);

// Menu definitions
enum{TRACKER_MENU=0, RADIO_GPS_MENU,  LOCAL_GPS_MENU };

// Distance conversion
#define METERS_2_FEET 3.291

// Frequency Channels:  First channel of each subband for North America
#define RF95_FREQ_0 902.3
#define RF95_FREQ_1 903.9
#define RF95_FREQ_2 905.5
#define RF95_FREQ_3 907.1
#define RF95_FREQ_4 908.7
#define RF95_FREQ_5 910.3
#define RF95_FREQ_6 911.9
#define RF95_FREQ_7 913.5
#define RF95_FREQ_8 903.1
#define RF95_FREQ_9 904.7
#define RF95_FREQ_10 906.3
#define RF95_FREQ_11 907.9
#define RF95_FREQ_12 909.5
#define RF95_FREQ_13 911.1
#define RF95_FREQ_14 912.7

float RF95_FREQ;

#define MAX_OP_CHAN 14

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Flash location definition struct [v1.4]
typedef struct {
  uint8_t ChanID;
  double lkg_Lat;
  double lkg_Long;
} Flash_struct;

Flash_struct  gFlashData;

// Blinky on receipt
#define LED 13

// Battery voltage input
#define VBATPIN A7

// Last RF channel
int radio_channel;

//max altitude indicator
int max_altitude = 0;

// Button definitions
#define LEFTBUTTON_X 20
#define LEFTBUTTON_Y 260
#define LEFTBUTTON_W 80
#define LEFTBUTTON_H 50

#define RIGHTBUTTON_X (LEFTBUTTON_X + LEFTBUTTON_W + 40)
#define RIGHTBUTTON_Y LEFTBUTTON_Y
#define RIGHTBUTTON_W LEFTBUTTON_W
#define RIGHTBUTTON_H LEFTBUTTON_H

//
//  Display Clearing function
//
// Background Colors:
//  Dark Gray 1	- 0x0861	Barely above black — looks black indoors.
//  Dark Gray 2	- 0x10A2	A bit lighter; safer for long static screens.
//  Dark Blue-Gray - 0x0842	Very dark, slightly blue tint (reduces eye strain).
//  Dark Green-Gray - 0x0862	Dark with subtle green — great in daylight.
//  Dark Warm Gray - 0x0861	Slightly brown tone, hides ghosting well.
//
void ClearDisplay(void)
{

  //tft.fillScreen(0x0861);  
  tft.fillScreen(ILI9341_BLACK);

}


/////////////////////////////////////
//
//  SETUP ROUTINE
//
/////////////////////////////////////

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(TOUCH_RESET, OUTPUT);
  digitalWrite(TOUCH_RESET, HIGH);

#ifdef SERIAL_CONSOLE
  // Open serial communications to console
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);
#endif

#ifdef LOCAL_GPS
  // Initial GPS Port
  Serial1.begin(LOCAL_GPS_BAUD);
#endif

#ifdef COMPASS
  //init compass
  compass.init();
  //compass.setCalibration(-336, 1190, -1160, 352, -1322, 0);   // SN #1 (me)
  //compass.setCalibration(-686, 797, -1171, 267, -1237, 0);    // SN #2 (Paul)
  //compass.setCalibration(-498, 1013, -1135, 355, -1242, 0);    // SN #3 (Dave)
  //compass.setCalibration(-713, 2007, -1826, 968, -1460, 0);    // SN #4 (Val)
  //compass.setCalibration(-942, 377, -1101, 280, -1227, 0);     // SN #5 (Mike M)  
  //compass.setCalibration(-797, 603, -1041, 372, -1155, 0);    // SN #6 (Ron R)
  //compass.setCalibration(-1073, 352, -892, 536, -1133, 0);    // SN #7 (Jim M)
  //compass.setCalibration(-722, 782, -861, 851, -1307, 0);       // SN #9 (me- 2)
  compass.setCalibration(-1027, 1211, -405, 1931, -2328, 0);  // SN #9 (me- 2)
  
#endif

#ifdef SERIAL_CONSOLE
  Serial.println("Start Listening for GPS\n");
#endif

#ifdef LCD_DISPLAY
 // Use this initializer if using a 1.8" TFT screen:
//  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
 // tft.invertDisplay(true);  // include this if the display is inverted
  tft.begin();
  tft.invertDisplay(true);  // include this if the display is inverted
    
  if (!ts.begin(40, &Wire)) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }
  // read diagnostics (optional but can help debug problems)
 /* uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); */ 
#endif

  // get the data stored in flash [v1.4]
  getFlashData(&gFlashData);

  LKG_Position.Lat = gFlashData.lkg_Lat;
  LKG_Position.Long = gFlashData.lkg_Long;
  usingLKG = true;

  // Provide user opportunity to change the radio channel
  radio_channel = getRadioChannel();

  Serial.print("Read Radio Chan: ");
  Serial.println(radio_channel);
  
  // v1.5 - Expand number of channels
  switch(radio_channel){
    case 0:
	    RF95_FREQ = RF95_FREQ_0;
	    break;
    case 1:
	    RF95_FREQ = RF95_FREQ_1;
	    break;
    case 2:
	    RF95_FREQ = RF95_FREQ_2;
	    break;
    case 3:
	    RF95_FREQ = RF95_FREQ_3;
	    break;
    case 4:
	    RF95_FREQ = RF95_FREQ_4;
	    break;
    case 5:
	    RF95_FREQ = RF95_FREQ_5;
	    break;
    case 6:
	    RF95_FREQ = RF95_FREQ_6;
	    break;
    case 7:
	    RF95_FREQ = RF95_FREQ_7;
	    break;
    case 8:
      RF95_FREQ = RF95_FREQ_8;
      break;
    case 9:
      RF95_FREQ = RF95_FREQ_9;
      break;
    case 10:
      RF95_FREQ = RF95_FREQ_10;
      break;
    case 11:
      RF95_FREQ = RF95_FREQ_11;
      break;
    case 12:
      RF95_FREQ = RF95_FREQ_12;
      break;
    case 13:
      RF95_FREQ = RF95_FREQ_13;
      break;
    case 14:
      RF95_FREQ = RF95_FREQ_14;
      break;
    default:
	    RF95_FREQ = RF95_FREQ_0;
	    break;
    }


  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
#ifdef SERIAL_CONSOLE
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
#endif
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {  
#ifdef SERIAL_CONSOLE
    Serial.println("setFrequency failed");
#endif
    while (1);
  }
#ifdef SERIAL_CONSOLE
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
#endif

 // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(23, false);   // set defaults in RH_RF95.cpp

#ifdef LCD_DISPLAY
  //init display

  ClearDisplay();   // clear display

  // Display initial screen - waiting for GPS from Radio
  tft.setRotation(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(0, 30);
  tft.print("Version ");
  tft.print(BS_VERSION);
  tft.setCursor(0, 55);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("Waiting for GPS...");
  tft.print("...on Channel ");
  tft.print(radio_channel);
  //updateButtons(NULL, "NEXT");
  updatedGPSFix = true;
#endif

  delay(2000);
 

}

uint8_t gps_len; 
uint8_t radioBuf[RH_RF95_MAX_MESSAGE_LEN];
bool first_fix = true;
int display_mode = TRACKER_MENU;
int lastDisplayMode = -1;
int heading = 0;

int updateDisplayCnt = 0;
static int heading_test = 0;

// MAIN PROCESSING LOOP
//  - Check button press
//  - Check for messages from remote GPS
//  - Check for message from local GPS
//  - Read the compass
//  - Call display update accordingly
void loop() {

  ////////////////////////////////////////////
  //  check the button
  // and see if we should switch screens
/*  if(button.isPressed()){
     if(display_mode++ >= LOCAL_GPS_MENU){
       display_mode = RADIO_GPS_MENU;
     }
*/

  if(rightButtonPressed()){
     if(display_mode++ >= LOCAL_GPS_MENU){
       display_mode = TRACKER_MENU;
     }

#ifdef LCD_DISPLAY
     ClearDisplay();   // clear display
#endif
     updatedGPSFix = true;  // trick later display update so that we can get a new display even if GPS doesn't update
  }

  // check if LKG Save button was hit, and if so, write it to flash
  if(leftButtonPressed())
  {
    /****TEST
    LKG_Position.Lat = gpsBS.location.lat();
    LKG_Position.Long = gpsBS.location.lng();
      */
    gFlashData.lkg_Lat = LKG_Position.Lat;
    gFlashData.lkg_Long = LKG_Position.Long;

    UpdateFlashData(&gFlashData);
    usingLKG = true;
    showLKGSaved();
    ClearDisplay();   // clear display
    updateButtons("SAVE", "NEXT");
    updatedGPSFix = true;
  }

  //////////////////////////////////////////////
  //  Monitor the remote GPS
  // Wait for message over radio
  if (rf95.available())
  {
    // Should be a message for us now

    gps_len = sizeof(radioBuf);

    if (rf95.recv(radioBuf, &gps_len))
    {
      digitalWrite(LED, HIGH);

#ifdef SERIAL_CONSOLE
      Serial.print("Got: ");
      Serial.write((char*)radioBuf, gps_len);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // listen to Lora Radio and push out to the console
      printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
      printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      printInt(gps.location.age(), gps.location.isValid(), 5);
      printDateTime(gps.date, gps.time);
      printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    
      printInt(gps.charsProcessed(), true, 6);
      printInt(gps.sentencesWithFix(), true, 10);
      printInt(gps.failedChecksum(), true, 9);
      Serial.println();
#endif
    
      smartDelay(500, 0);

#ifdef SERIAL_CONSOLE
      if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
#endif

    }

    if(gps.sentencesWithFix()){
      updatedGPSFix = true;   // update loop indicator that we've had a new GPS fix

            // update LKG position [v1.4]
      LKG_Position.Lat = gps.location.lat();
      LKG_Position.Long = gps.location.lng();
      usingLKG = false;
      digitalWrite(LED, LOW);
    } 
  } 

#ifdef COMPASS
  //////////////////////////////////////////////////////////////////////////
  //Check compass when not waiting for message from radio
  // Read compass values
  compass.read();
  heading = compass.getAzimuth();

  // subtract mounting error of 90 degs but add the 11 deg inclination angle = -79 degs
  heading -= 79;
  if(heading < 0)
    heading = 360 + heading;


#endif

#ifdef LOCAL_GPS
  //////////////////////////////////////////////////////////////////////////
  // Then get fix from basestation GPS

  unsigned long timeout = millis() + 1000;
  uint8_t inIndex = 0;

// check if anything is in the serial buffer from the GPS
  if (Serial1.available() > 0) {
    while ( ((int32_t)(millis() - timeout) < 0) && (inIndex < (sizeof(radioBuf)/sizeof(radioBuf[0])))) {
        if (Serial1.available() > 0) {
            // read the incoming byte:
            radioBuf[inIndex] = Serial1.read();
            //Serial.write(radioBuf[inIndex]);
            if ((radioBuf[inIndex] == '\n') || (radioBuf[inIndex] == '\r')) {
                inIndex++;
                break;
            }
            inIndex++;
        }
    }
    //radioBuf[inIndex] = 0; // put a null delimiter on string
    gps_len = inIndex;
    if(!strncmp((const char *) &radioBuf[3], "GGA",3)){
#ifdef SERIAL_CONSOLE
      Serial.write(radioBuf,inIndex);
      Serial.print(" ");
      Serial.println(display_mode);
#endif
      // Now update display
      smartDelay(500, 1);
      if(gpsBS.sentencesWithFix()){ //[v1.35]
         updatedGPSFix = true;   // update loop indicator that we've had a new local GPS fix
      }
    }
    inIndex = 0;
  }
#endif

  ////////////////////////////
  //  // Lastly, update the display if its time - for either remote or local GPS fix

  if(((++updateDisplayCnt % 2) == 0) && updatedGPSFix)// only update display on every other instance of update GPS fix
  {
    // Check if we need to clear the screen
    if(first_fix)
    {    
#ifdef LCD_DISPLAY
      ClearDisplay();   // clear display
#endif
      first_fix = false;
    }
    //delay(500);
#ifdef LCD_DISPLAY
    updateDisplay(display_mode);  // new remote GPS update, update display
#endif
    updatedGPSFix = false;
  }
 
}


/* 
 *  
 *  Display the appropriate menu or display
 *  
 */

static void updateDisplay(int display)
{
  TinyGPSPlus *pGPS; 
  int distanceInMeters, courseToTarget;
  float measuredvbat;
  bool displayChanged = false;

  // first, check if this is a new display
  if(display != lastDisplayMode)
  {
    displayChanged = true;
    lastDisplayMode = display;
  }

    if(display != TRACKER_MENU)
    {
      tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
      tft.setTextSize(2);

      if(display == RADIO_GPS_MENU)
      {
        tft.setCursor(60, 0);
        tft.print("Remote GPS");
        pGPS = &gps;
       
        if(usingLKG)
          tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
        else
          tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(0,30);   // how far to move cursor?
        tft.print("N: ");
        tft.print(LKG_Position.Lat,6);   // print or println?
        tft.setCursor(0,50);   // how far to move cursor?
        tft.print("E: ");
        tft.print(LKG_Position.Long,6);
        tft.setCursor(0,70);   // how far to move cursor?
      }
      else {
        tft.setCursor(35, 0);  
        tft.print("Basestation Data");
        pGPS = &gpsBS;
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(0,30);   // how far to move cursor?
        tft.print("N: ");
        tft.print(pGPS->location.lat(),6);   // print or println?
        tft.setCursor(0,50);   // how far to move cursor?
        tft.print("E: ");
        tft.print(pGPS->location.lng(),6);
        tft.setCursor(0,70);   // how far to move cursor?
      }

      if(display == RADIO_GPS_MENU)
      {
        tft.print("ALT: ");
        tft.print(pGPS->altitude.meters()*METERS_2_FEET,0);
      }
      else {  
        tft.print("Compass: ");
        tft.print(heading);
      }
      tft.print("    ");
      tft.setCursor(0,90);   // how far to move cursor?
      tft.print("#Sats: ");
      tft.print(pGPS->satellites.value());
      tft.print(" ");
      tft.setCursor(0,110);   // how far to move cursor?
      tft.print("Time: ");
//      tft.print(pGPS->time.hour()); 
//      tft.print(":");
//      tft.print(pGPS->time.minute()); 
//      tft.print(":");
      tft.print(pGPS->time.second()); 
      
      if(display == RADIO_GPS_MENU)
      {
        tft.setCursor(0,130);   // how far to move cursor?
        tft.print("RSSI: ");
        tft.print(rf95.lastRssi(), DEC);
        tft.print("    ");
      }
	    else	// compute and display battery voltage
	    {
	      tft.setCursor(0,130);   // how far to move cursor?
        tft.print("VBat: ");
	      measuredvbat = analogRead(VBATPIN);
	      measuredvbat *= 2;    // we divided by 2, so multiply back
	      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	      measuredvbat /= 1024; // convert to voltage
        tft.print(measuredvbat, 2);
        tft.print("    ");
      }
    } 
    else    // If we got here, display must be tracker.  Process the tracker mode
    {
      //distanceInMeters = (int) gps.distanceBetween(gpsBS.location.lat(), gpsBS.location.lng(), gps.location.lat(), gps.location.lng());
      //courseToTarget = (int) gps.courseTo(gpsBS.location.lat(), gpsBS.location.lng(),gps.location.lat(), gps.location.lng());
	    distanceInMeters = (int) gps.distanceBetween(gpsBS.location.lat(), gpsBS.location.lng(), LKG_Position.Lat, LKG_Position.Long);
      courseToTarget = (int) gps.courseTo(gpsBS.location.lat(), gpsBS.location.lng(),LKG_Position.Lat, LKG_Position.Long);
	
	// check for uninitialized/out of bounds distances
	if(distanceInMeters > 99999.0)
	  distanceInMeters = 99999;
      tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
      tft.setTextSize(2);
      tft.setCursor(50, 80);
      tft.print("DIST: ");
      if(usingLKG)
        tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
      if((distanceInMeters*METERS_2_FEET) > MAX_DISTANCE)
      {
        tft.print("XXXXXX");
      }
      else
      {
        tft.print(distanceInMeters*METERS_2_FEET, 0);  // print distance
        tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
        tft.print("    ");
      }
      tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
      tft.setCursor(50, 120);
      tft.print("ALT: ");
      if(usingLKG)
        tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
      tft.print(gps.altitude.meters()*METERS_2_FEET,0);
      tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
      tft.print("    ");
      // Put update of arrows here
      // first clear the last pointer icon
      //tft.drawTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_BLACK);
      tft.fillTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ILI9341_BLACK);
//     
//      tft.setRotation(1);
 //     tft.setTextSize(2);
      //tft.setCursor(60, 90);
      //tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      // adjust heading based on compass
      courseToTarget -= heading;
      if(courseToTarget < 0)
        courseToTarget = 360 + courseToTarget;
      //tft.print(courseToTarget);
      //tft.print("  ");
      displayIndex = courseToTarget / INCREMENT_DEGS;
      if((courseToTarget % INCREMENT_DEGS) > (INCREMENT_DEGS/2))
        displayIndex++;
       /***TEST
      heading_test++;
      if(heading_test >= NUM_POINTS)
        heading_test = 0;

      displayIndex = heading_test;
*/

      if(displayIndex >= NUM_POINTS){
        displayIndex = 0;
      }

      tft.drawCircle(120, 120, 119, ILI9341_WHITE);
  
      //tft.drawTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_GREEN);
      tft.fillTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ILI9341_GREEN);
      
    }

    if(displayChanged){
      // Update buttons at the bottom
      updateButtons("SAVE", "NEXT");
    
    }
}
/**********************************************************
 * 
 * The following routines are to support the GPS processing
 * from the TinyGPS++ examples
 * 
 */
 
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms, int local_gps)
{
  uint8_t msg_len = 0; 
  unsigned long start = millis();

  do 
  {
    while (msg_len < gps_len){
      if(local_gps)
        gpsBS.encode(radioBuf[msg_len++]);   // local GPS object
      else
        gps.encode(radioBuf[msg_len++]);   // radio GPS object
    }  
  } while (millis() - start < ms);
}

/*
 * Compute
 */

#ifdef SERIAL CONSOLE
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0, -1);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0, -1);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0, -1);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0, -1);
}
#endif

// initial loop to allow selection of receiver channel
static int getRadioChannel(void)
{
  int TimerCnt = 10; // 10 times with 500 msec delay = 5 secs
  bool buttonPressed = false;

  // First, get radio_channel from flash
  radio_channel = (int) gFlashData.ChanID; // read first location for channel
  if(radio_channel > MAX_OP_CHAN)
    radio_channel = 0;  // only 15 channels, if we read higher, set to zero

  // Loop for 5 secs and wait for button hit.  For each button hit
  
  ClearDisplay();   // clear display
  tft.setRotation(2);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(0, 20);
  tft.println("CONFIG CHANNEL: ");
  tft.setCursor(0, 45);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Current Chan = ");
  tft.println(radio_channel);
  tft.println("To Change, hit");
  tft.println("UP or DOWN in 5 secs");
  updateButtons("DOWN", " UP ");
  while(TimerCnt--)
  {
      if(rightButtonPressed()){
        if(radio_channel++ >= MAX_OP_CHAN)
          radio_channel = 0;
        buttonPressed = true;
      }
      else if(leftButtonPressed())
      {
        if(--radio_channel < 0)
          radio_channel = MAX_OP_CHAN;
        buttonPressed = true;
      }
      if(buttonPressed)
      {
        ClearDisplay();   // clear display
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(2);
        tft.setCursor(0, 30);
        tft.print("Channel = ");
        tft.println(radio_channel);
        TimerCnt = 10;
        // Write the new value into flash
        gFlashData.ChanID = (uint8_t) radio_channel; // [v.14]
        UpdateFlashData(&gFlashData);
        tft.println("Channel Updated");
        buttonPressed = false;
        updateButtons("DOWN", " UP ");
      }
      delay(500);
  }
  return radio_channel;
}


static void getFlashData(Flash_struct *FlashData)
{
  uint8_t   *ptr;
  int i;
  
  if (EEPROM.isValid()) {
    // read whole struct out in serial fashion
    ptr = (uint8_t *) FlashData;
    for(i=0;i<sizeof(Flash_struct);i++)
    {
      *ptr++ = (int) EEPROM.read(i); // read each byte
    
    }
    Serial.println("EEPROM Data Read");
  }
  else  // if flash isn't valid, then zero everything out
  {
    Serial.println("EEPROM IS INVALID");
    FlashData->ChanID = 0;
    FlashData->lkg_Lat = 0.0;
    FlashData->lkg_Long = 0.0;
  }
}

static void UpdateFlashData(Flash_struct *FlashData)
{
  uint8_t  *ptr;
  int i;
 

    // write whole struct out in serial fashion
    ptr = (uint8_t *) FlashData;
    for(i=0;i<sizeof(Flash_struct);i++)
    {
      EEPROM.write(i, *ptr++);
      Serial.printf("EEPROM %d\n", i);
    }
    EEPROM.commit();

}


void updateButtons(char *LeftLabel, char *RightLabel)
{

  tft.setTextSize(2);

  // check if the button is not null before displaying it
  if(LeftLabel != NULL)
  {
    tft.fillRect(LEFTBUTTON_X, LEFTBUTTON_Y, LEFTBUTTON_W, LEFTBUTTON_H, ILI9341_GREEN);
    tft.drawRect(LEFTBUTTON_X, LEFTBUTTON_Y, LEFTBUTTON_W, LEFTBUTTON_H, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE, ILI9341_GREEN);
    tft.setCursor(LEFTBUTTON_X + (LEFTBUTTON_W/2) - 20 , LEFTBUTTON_Y + (LEFTBUTTON_H/2) - 5);
    tft.print(LeftLabel);
  }

  tft.fillRect(RIGHTBUTTON_X, RIGHTBUTTON_Y, RIGHTBUTTON_W, RIGHTBUTTON_H, ILI9341_BLUE);
  tft.drawRect(RIGHTBUTTON_X, RIGHTBUTTON_Y, RIGHTBUTTON_W, RIGHTBUTTON_H, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.setCursor(RIGHTBUTTON_X + (RIGHTBUTTON_W/2) - 20, RIGHTBUTTON_Y + (RIGHTBUTTON_H/2) - 5);
  tft.print(RightLabel);


}



bool rightButtonPressed(void)
{
 // See if there's any  touch data for us
  if (ts.touched())
  {   
    // Retrieve a point  
    TS_Point p = ts.getPoint(); 
    // rotate coordinate system
    // flip it around to match the screen.
    p.x = map(p.x, 0, 240, 240, 0);
    p.y = map(p.y, 0, 320, 320, 0);
   // int y = tft.height() - p.x;
   // int x = p.y;
    int x = p.x;
    int y = p.y;

    if((x > RIGHTBUTTON_X) && (x < (RIGHTBUTTON_X + RIGHTBUTTON_W))) { 
        if ((y > RIGHTBUTTON_Y) && (y <= (RIGHTBUTTON_Y + RIGHTBUTTON_H))) {
          tft.fillRect(RIGHTBUTTON_X, RIGHTBUTTON_Y, RIGHTBUTTON_W, RIGHTBUTTON_H, ILI9341_WHITE); 
          delay(100);
          return(true);
        }
    }
  }
  return(false);
}


bool leftButtonPressed(void)
{
 // See if there's any  touch data for us
  if (ts.touched())
  {   
    // Retrieve a point  
    TS_Point p = ts.getPoint(); 
    // rotate coordinate system
    // flip it around to match the screen.
    p.x = map(p.x, 0, 240, 240, 0);
    p.y = map(p.y, 0, 320, 320, 0);
   // int y = tft.height() - p.x;
   // int x = p.y;
    int x = p.x;
    int y = p.y;

    if((x > LEFTBUTTON_X) && (x < (LEFTBUTTON_X + LEFTBUTTON_W))) { 
        if ((y > LEFTBUTTON_Y) && (y <= (LEFTBUTTON_Y + LEFTBUTTON_H))) {
          tft.fillRect(LEFTBUTTON_X, LEFTBUTTON_Y, LEFTBUTTON_W, LEFTBUTTON_H, ILI9341_WHITE); 
          delay(100);
          return(true);
        }
    }
  }
  return(false);
}

void showLKGSaved(void)
{

  //tft.fillRect(LEFTBUTTON_X, LEFTBUTTON_Y-80, LEFTBUTTON_W+RIGHTBUTTON_W, RIGHTBUTTON_H, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
  tft.setCursor(LEFTBUTTON_X+20, LEFTBUTTON_Y-80);
  tft.print("POSITION SAVED!");
  delay(2000);

}