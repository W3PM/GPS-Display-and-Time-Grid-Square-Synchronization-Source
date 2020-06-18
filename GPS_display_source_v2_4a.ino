/*
  Arduino Nano/Uno sketch to display GPS data from s software serial port
  and send time and Maidenhead grid square updates via hardware serial port.

  The sketch is based upon an inexpensive NEO-6M GPS receiver board.

  Note: Reguires TinyGPS++ and SoftwareSerial libraries. SoftwareSerial is
        included in the Arduino library cache, but TinyGPS++ is found on
        the web.

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Copyright (C) 2020,  Gene Marcus W3PM GM4YRE

  7 May, 2020


  18 May, 2020  v2_3  Added mutliple locations
  22 May. 2020  v2_4  Improve location selection

  Successfully compiled using Arduino 1.8.13


  ------------------------------------------------------------------------
  Nano Digital Pin Allocations follow:
  ------------------------------------------------------------------------
  D0/TX 9600 baud time/location serial out
  D1
  D2
  D3
  D4 GPS 9600 baud data input from NEO-6M
  D5
  D6
  D7
  D8
  D9
  D10
  D11
  D12
  D13
  A0/D14
  A1/D15 Location change pushbutton
  A2/D16
  A3/D17
  A4/D18 OLED SDA
  A5/D19 OLED SCL

*/

#include <EEPROM.h>

struct DFCoordinateData {
  char* name;
  float Lat;
  float Lon;
}

//________________________________________________________________________________________
// Geographical Location Data  
//
// Add addional locations using format: {"Name", 0.00000, 0.00000},
// Note: limit location name to 8 characters
//________________________________________________________________________________________
dfCoordinateData[] = {
  {"HOME:", 34.733089, -86.793525},
  {"NYC:",  40.730610, -73.935242},
  {"London:", 51.509865, -0.118092},
  {"Sydney, Aus:", -33.865143, 151.209900},
 
};



#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#define pushButton1           15 // Location change button, This is Nano pin A1/D15 but may be changed to any unused pin.

// Define OLED address
#define I2C_ADDRESS           0x3C

// initialize oled display
SSD1306AsciiAvrI2c oled;

static const int RXPin = 4, TXPin = 3; // Input from NEO-6M GPS nodule
static const uint32_t GPSBaud = 9600;
char locator[8];
int count, count2;
unsigned long timer, timer2, currentTime;
static double Bearing_LAT = 34.733089, Bearing_LON = -86.793525; //Madison, AL
static char Location[13];
byte locationNumber;
bool changeLocationFlag = false;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Wire.begin(4); // join i2c buss

  // Set up pushbutton for location change
  pinMode(pushButton1, INPUT);
  digitalWrite(pushButton1, HIGH);       // internal pull-up enabled

  // Retrieve stored coordinate data
  EEPROM.get (10, locationNumber);                 // Get stored VFO band
  if (locationNumber > 20 | locationNumber < 0) locationNumber = 0;    //Ensures valid EEPROM data - if invalid will default to band 0

  // Set oled font size and type
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);

  // Setup OLED initial display
  oled.clear();
}

void loop()
{
  Serial.write(gps.time.hour());
  Serial.write(gps.time.minute());
  Serial.write(gps.time.second());
  for (int i = 0; i < 7; i ++) Serial.print(locator[i]);
  Serial.print("#");

  if (digitalRead(pushButton1) == LOW)
  {
    changeLocationFlag = true;
    changeLocation();
  }

  unsigned long distanceKmToHome =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      dfCoordinateData[locationNumber].Lat,
      dfCoordinateData[locationNumber].Lon) / 1000;

  double courseToHome =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      dfCoordinateData[locationNumber].Lat,
      dfCoordinateData[locationNumber].Lon);

  const char *cardinalToHome = TinyGPSPlus::cardinal(courseToHome);

  printTime(gps.time);

  if ((millis()) > timer)
  {
    timer = millis() + 4000;
    count ++;
    if (count > 2) count = 0;
    oled.setCursor(0, 2);

    switch (count)
    {
      case 0: // Display grid square
        calcGridSquare();
        oled.print (F("Grid:"));
        oled.print(locator);
        break;

      case 1: // Display satellite count
        oled.print (F("Sats: "));
        oled.print(gps.satellites.value());
        oled.print(F("       "));
        break;

      case 2: //Display date
        oled.setCursor(0, 2);
        printDate(gps.date);
        break;
    }
  }

  if ((millis()) > timer2)
  {
    timer2 = millis() + 2000;
    count2 ++;
    if (count2 > 9) count2 = 0;
    switch (count2)
    {
      case 0: // Display latitude
        oled.setCursor(0, 4);
        oled.print (F("Latitude:   "));
        oled.setCursor(0, 6);
        oled.print(gps.location.lat(), 6);
        break;

      case 1: // Display longitude
        oled.setCursor(0, 4);
        oled.print (F("Longitude:  "));
        oled.setCursor(0, 6);
        oled.print(gps.location.lng(), 6);
        break;

      case 2: // Display distance to home km
        oled.setCursor(0, 4);
        oled.print(dfCoordinateData[locationNumber].name);
        oled.print(F("           "));
        //dfCoordinateData[locationNumber].Lon;
        oled.setCursor(0, 6);
        oled.print(distanceKmToHome);
        oled.print(F(" km         "));
        break;

      case 3: // Display distance to home miles
        oled.setCursor(0, 6);
        oled.print(int(distanceKmToHome * 0.62137119));
        oled.print(F(" miles      "));
        break;

      case 4: // Course to home
        oled.setCursor(0, 6);
        oled.print(courseToHome);
        oled.print(F(" "));
        oled.print(cardinalToHome);
        oled.print(F("    "));
        break;

      case 5: // Speed in knots
        oled.setCursor(0, 4);
        oled.print (F("Speed:      "));
        oled.setCursor(0, 6);
        oled.print(gps.speed.knots(), 1);
        oled.print(F(" knots   "));
        break;

      case 6: // Speed in km
        oled.setCursor(0, 6);
        oled.print(gps.speed.kmph(), 1);
        oled.print(F(" kmph    "));
        break;

      case 7: // Speed in km
        oled.setCursor(0, 6);
        oled.print(gps.speed.mph(), 1);
        oled.print(F(" mph     "));
        break;

      case 8: // Altitude in meters
        oled.setCursor(0, 4);
        oled.print (F("Altitude:"));
        oled.setCursor(0, 6);
        oled.print(gps.altitude.meters(), 0);
        oled.print(F(" m         "));
        break;

      case 9: // Altitude in feet
        oled.setCursor(0, 6);
        oled.print((gps.altitude.meters() *  3.2808399), 0);
        oled.print(F(" ft        "));
        break;
    }
  }

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


static void printDate(TinyGPSDate &d)
{
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
  oled.print(sz);
}


static void printTime(TinyGPSTime &t)
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
  oled.setCursor(24, 0);
  oled.print(sz);
}


void changeLocation()
{
  bool displayFlag = true;
  long displayTime = millis() + 2000;
  while (changeLocationFlag == true)
  {
    long tempTime = millis();
    if (tempTime > displayTime)
    {
      oled.clear();
      changeLocationFlag = false; 
    }
    if (displayFlag == true)
    {
      oled.clear();
      oled.setCursor(0, 2);
      oled.print(locationNumber);
      oled.setCursor(0, 4);
      oled.print(dfCoordinateData[locationNumber].name);
      displayFlag = false;
    }
    smartDelay(400);
    if (digitalRead(pushButton1) == LOW)
    {
      locationNumber++;
      if (locationNumber > (sizeof(dfCoordinateData) / sizeof(DFCoordinateData) - 1))locationNumber = 0;
      EEPROM.put(10, locationNumber);
      displayTime = millis() + 2000;
      displayFlag = true;
    }
  }
}



//******************************************************************
// Calculate the 6 digit Maidenhead Grid Square location
//******************************************************************
void calcGridSquare()
{
  int o1, o2, o3;
  int a1, a2, a3;
  double remainder;
  // longitude
  remainder = gps.location.lng() + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = gps.location.lat() + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  locator[0] = (char)o1 + 'A';
  locator[1] = (char)a1 + 'A';
  locator[2] = (char)o2 + '0';
  locator[3] = (char)a2 + '0';
  locator[4] = (char)o3 + 'A';
  locator[5] = (char)a3 + 'A';
  locator[6] = (char)0;
}
