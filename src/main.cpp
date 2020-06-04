#include <Arduino.h>
#include <Wire.h>

#include <GxEPD.h>
#include <GxGDEW042Z15/GxGDEW042Z15.h>    // 4.2" b/w/r
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include GxEPD_BitmapExamples

GxIO_Class io(SPI, /*CS=D8*/ SS, /*DC=D3*/ 0, /*RST=D4*/ 16);
GxEPD_Class display(io, /*RST=D4*/ 16, /*BUSY=D6*/ 12);
#define HAS_RED_COLOR

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

int frameCounter = 0;

// run once on startup
void setup()
{
  // Setup serial connection for debugging
  Serial.begin(115200);
  display.init(115200);
  display.setTextColor(GxEPD_BLACK);  
  display.setFont(&FreeMonoBold24pt7b); 
  display.eraseDisplay();
  Serial.println("\nSetup done!");
}

void showFont(const char name[], const GFXfont* f)
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f);
  display.setCursor(0, 0);
  display.println();
  display.println(name);
  display.println(" !\"#$%&'()*+,-./");
  display.println("0123456789:;<=>?");
  display.setTextColor(GxEPD_DARKGREY);
  display.println("@ABCDEFGHIJKLMNO");
  display.setTextColor(GxEPD_LIGHTGREY);
  display.println("PQRSTUVWXYZ[\\]^_");
#if defined(HAS_RED_COLOR)
  display.setTextColor(GxEPD_RED);
#endif
  display.println("`abcdefghijklmno");
  display.println("pqrstuvwxyz{|}~ ");
  display.update();
  delay(5000);
}

// run forever
void loop()
{
  frameCounter++;
  showFont("FreeMonoBold9pt7b", &FreeMonoBold9pt7b);
  showFont("FreeMonoBold12pt7b", &FreeMonoBold12pt7b);
  showFont("FreeMonoBold18pt7b", &FreeMonoBold18pt7b);
  showFont("FreeMonoBold24pt7b", &FreeMonoBold24pt7b);

  // display.drawPicture(BitmapExample3, BitmapExample4, sizeof(BitmapExample3), sizeof(BitmapExample4));
  // delay(5000);
  // display.drawPicture(BitmapExample1, BitmapExample2, sizeof(BitmapExample1), sizeof(BitmapExample2));
  // delay(5000);
}


