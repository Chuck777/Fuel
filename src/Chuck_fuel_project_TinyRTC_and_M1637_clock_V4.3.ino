//////////////////////////////////////////////////////////
// CGUOU Fuel Management System by Chuck Caron
// Objective, create a UTC clock & display GPS data
// Menu based interface via 2 interupt push buttons.
//------------------------------------------------------
// Arduino YUN, 5-12V input
// GPS GP-20u7, 3v
// Display 4 Digits: TM1637, 5.0V, on DIO pin
// 128x64 Graphic Display: U8glib, 5.0V on I2C
///////////////////////////////////////////////////////////
#include <Adafruit_GPS.h>
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define GPSECHO true // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console // Set to 'true' if you want to debug and listen to the raw GPS sentences
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
//////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include "U8glib.h" // 128x64 display on I2C, 5V
#include <TM1637Display.h>  //4-Digits Display on 5.0V
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send ACK

bool OUT_hour_set = false;
bool OFF_hour_set = false;
bool ON_hour_set = false;
bool IN_hour_set = false;

float ET_minute = 0;
int MAG_heading = 0;
int OUT_hour = 0; // le définir une seule fois pour affichage, OUT, OFF, ON & IN hour and minute
int OUT_minute = 0;

int OFF_hour = 0;
int OFF_minute = 0;

int ON_hour = 0;
int ON_minute = 0;

int IN_hour = 0;
int IN_minute = 0;

///////////////4-Digits Dislay on 5.0V///////////////////////////////////
// Module connection pins (Digital Pins)
#define CLK 8
#define DIO 9
TM1637Display display(CLK, DIO);  /// 4-Digit display on pins 8&9
uint8_t data[] = {0x0, 0x0, 0x0, 0x0};
/////////////////////////////////////////////////////////////////////////////////////

// Push Button UP and Down setup
// Push Button Pin 5 = UP
// Push Button Pin 6 = Down
const int buttonUPPin = 5;     // the pin number for the pushbutton UP
const int buttonDNPin = 6;     // the pin number for the pushbutton Down
int buttonUPState = 0;         // variable for reading the pushbutton status
int buttonDNState = 0;         // variable for reading the pushbutton status
uint8_t draw_state = 0;        // debug draw_state (x)

//////////////////////////////////////////////////////////////////////////////////////

void setup ()
{
  Wire.begin();
  ///////////////Push Button//////////////////////////
  // initialize the pushbutton pin as an input:
  pinMode(buttonUPPin, INPUT);
  pinMode(buttonDNPin, INPUT);
  ///////////////////////////////////////////////////
  display.setBrightness(0x0a); // Brightness for 4-Digits Display

  /////////////////GPS Begin//////////////////////////
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  GPS.begin(9600); // 9600 NMEA is the default baud rate
  mySerial.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // // Set the update rate, 1 Hz update rate

#ifdef __arm__
  usingInterrupt = false;
#else
  useInterrupt(true);
#endif
  delay(500);
}

#ifdef __AVR__

SIGNAL(TIMER0_COMPA_vect) // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
{
  char c = GPS.read(); // if you want to debug, this is a good time to do it!

#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__
uint32_t timer = millis();
/////////////////END of Setup////////////////////////////////////////////////////

void u8g_prepare(void) {
  u8g.setFont(u8g_font_6x10);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
}

void draw_All_Time() { // CASE 0
  u8g.setFont(u8g_font_unifont);
  u8g.undoScale();  // scale normal
  //u8g.drawFrame(112,48,17,15); ////Draw BOX and indicate number of Satellite Right-lower corner of screen///
  //u8g.setPrintPos( 113, 61);
  //u8g.print(GPS.satellites);

  if (GPS.fix == 0)
  {
    u8g.drawStr( 120, 60, "X");
  }
  else
  {
    u8g.drawStr( 1, 20, "OUT:");

    if (!OUT_hour_set && GPS.speed >=0) {
      OUT_hour_set = true;
      OUT_hour = GPS.hour;
      OUT_minute = GPS.minute;
    }
    if (OUT_hour >= 10) {
      u8g.setPrintPos( 40, 20);
      u8g.print((OUT_hour));
    }
    else {
      u8g.drawStr( 40, 20, "0");
      u8g.setPrintPos( 49, 20);
      u8g.print((OUT_hour));
    }

    if (OUT_minute >= 10) {
     u8g.setPrintPos( 59, 20);
     u8g.print(OUT_minute);
    }

    else
    {
      u8g.drawStr( 59, 20, "0");
      u8g.setPrintPos( 64, 20);
      u8g.print(OUT_minute);
    }

 ////////////////////OFF Time SETTING /////////////////////////////////////////
    u8g.drawStr( 1, 34, "OFF:");
 if (!OFF_hour_set && GPS.speed >=0)
 {
  OFF_hour_set = true;
  OFF_hour = GPS.hour;
  OFF_minute = GPS.minute;

    if (OFF_hour >= 10)
    {
      u8g.setPrintPos( 40, 34);
      u8g.print((OFF_hour));
    }
    else
    {
      u8g.drawStr( 40, 34, "0");
      u8g.setPrintPos( 49, 34);
      u8g.print((OFF_hour));
    }

    if (OUT_minute >= 10)
    {
     u8g.setPrintPos( 59, 34);
     u8g.print(OFF_minute);
    }
    else
    {
    u8g.drawStr( 59, 34, "0");
    u8g.setPrintPos( 64, 34);
    u8g.print(OFF_minute);
    }
  }
////////////////////ON Time SETTING /////////////////////////////////////////
    u8g.drawStr( 1, 49, "ON :");
if (!ON_hour_set && OFF_hour_set && GPS.speed >= 0)
{
    ON_hour_set = true;
    ON_hour = GPS.hour;
    ON_minute = GPS.minute;
}
if (ON_hour >= 10)
{
  u8g.setPrintPos( 40, 49);
  u8g.print((ON_hour));
}
else
{
  u8g.drawStr( 40, 49, "0");
  u8g.setPrintPos( 49, 49);
  u8g.print((ON_hour));
}

if (ON_minute >= 10)
{
 u8g.setPrintPos( 59, 49);
 u8g.print(ON_minute);
}
else
{
u8g.drawStr( 59, 49, "0");
u8g.setPrintPos( 64, 49);
u8g.print(ON_minute);
}

////////////////////IN Time SETTING ///////////////////////////////////////////
    u8g.drawStr( 1, 64, "IN :");


////////////////////////////////////////////////////////////////////////////////
  }
}

void draw_GMT()
{ // CASE-1 64*128 OLED display
  u8g.setFont(u8g_font_unifont);
  u8g.setFontRefHeightExtendedText();
  u8g.setScale2x2(); //double scale

  u8g.drawStr( 1, 15, "GS");
  u8g.setPrintPos( 30, 15);
  u8g.print((int)GPS.speed);

  u8g.undoScale();  // scale normal
  u8g.drawStr( 0, 40, "----------------");

/////////////////////////////// Elapsed Time routine here //////////////////////

  u8g.setScale2x2(); //double scale
  u8g.drawStr( 1, 30, "ET");  // Now need to calculate and display Elapsed Time (ET) since startup (OUT time)

  if (GPS.minute >= OUT_minute)
  {
     ET_minute = (GPS.minute - OUT_minute);
     u8g.setPrintPos( 30, 30);
     u8g.print("0:0");
     //u8g.setPrintPos( 55, 30);
     u8g.print ((int)(ET_minute));
  }
  if (ET_minute >= 10)
     {
     u8g.drawStr( 30, 30, "0:  ");
     u8g.setPrintPos( 45, 30);
     u8g.print ((int)(ET_minute));
     }


  if (GPS.minute < OUT_minute)
  {
     GPS.minute = (GPS.minute + 60);
     GPS.hour = (GPS.hour - 1);
     ET_minute = (GPS.minute - OUT_minute);
     u8g.setPrintPos( 30, 30);
     u8g.print ((int)(ET_minute));
  }
}

void draw(void)
{
  u8g_prepare();
  switch (draw_state)
  {
    case 0: draw_All_Time(); break;
    case 1: draw_GMT(); break;
  }
}

/////////////////////////////////////////////////////////////////////////////////

void loop()
{

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
    return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (timer > millis())  timer = millis();  // if millis() or timer wraps around, we'll just reset it


  if (millis() - timer > 1200) // approximately every 2 seconds or so, print out the current stats
  {
    timer = millis(); // reset the timer
  }

  ////////////////////TESTING USING SERIAL MONITOR//////////////////////////////

    //Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.println(':');
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    //if (GPS.fix)
     //Serial.print("Location: ");
     Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
     Serial.print(", ");
     Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
     Serial.print("Speed (knots): "); Serial.println(GPS.speed);
     Serial.print("Angle: "); Serial.println(GPS.angle);
     //Serial.print("MAG: "); Serial.println(GPS.magvariation);
     //Serial.print("Altitude: "); Serial.println(GPS.altitude);
     Serial.print("Satellites: "); Serial.println((int)GPS.satellites);


  ////////////////////////END OF SERIAL MONITOR TESTING///////////////////////////////////////

  // 128x64 display picture loop
  u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage() );

  // check if the pushbutton is pressed, if yes increase the state value
  // digitalRead(buttonUPPin); // read the state of the UP pushbutton value:
  // digitalRead(buttonDNPin); // read the state of the Down pushbutton value:

  if (digitalRead(buttonUPPin) == HIGH) {
    while (digitalRead(buttonUPPin) == HIGH);
    draw_state++;
    delay(300);
  }
  if (digitalRead(buttonDNPin) == HIGH) {
    while (digitalRead(buttonDNPin) == HIGH);
    draw_state == 0;
    delay(300);
  }

  if ( draw_state > 1 ) {
    draw_state = 0;
  }

  ///////////////////// GPS GMT Time Case-0 ////////////////////////////////////
  if (draw_state == 0 ) {
    display.setColon(true);
    display.showNumberDec((GPS.hour), true, 2, 0);
    display.showNumberDec(GPS.minute, true, 2, 2);  // (integer, leading zeros, # of digits to be displ, Px (0=leftmost,,3- rightmost))
    delay(50);
  }

  /////////////////////// MAG HDG Case-1 //////////////////////////////////////
  MAG_heading = ((int) GPS.angle - 16); // removing 16 deg for West Variation in CYHU @ 7325W, CYOW @ W7566 13 deg, CYQB 7123W @ 17

  if (draw_state == 1 ) {
    display.setColon(false);

    if (GPS.angle >= 100) {
      data[0] = display.encodeDigit((int) MAG_heading / 100);
      data[1] = display.encodeDigit((int) MAG_heading % 100 / 10 );
      data[2] = display.encodeDigit((int) MAG_heading % 10 );
      data[3] = display.encodeDigit(11);
      display.setSegments(data);
    }
    else if (GPS.angle >= 10) {
      data[0] = display.encodeDigit((int) 0);
      data[1] = display.encodeDigit((int) MAG_heading / 10 );
      data[2] = display.encodeDigit((int) MAG_heading % 10 );
      data[3] = display.encodeDigit(11);
      display.setSegments(data);

    } else {
      data[0] = display.encodeDigit((int) 0);
      data[1] = display.encodeDigit((int) 0);
      data[2] = display.encodeDigit((int) MAG_heading);
      data[3] = display.encodeDigit(11);
      display.setSegments(data);
    }
    delay(50);
  }

}
