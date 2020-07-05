using namespace std;

#include <Arduino.h>

#define LED_BUILTIN (13) // LED is connected to IO13

// Environment sensor includes and defines
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define BME280_PIN_I2C_SCL (32)
#define BME280_PIN_I2C_SDA (33)
#define BME280_PIN_VCC (25)
#define BME280_ADDR (0x76)
#define PRESSURE_MEASUREMENT_CALIBRATION (6000)
#define SEALEVEL_PRESSURE (1013.25)
#define SEALEVEL_PRESSURE_CALIBRATION (9.65)

TwoWire I2CBME = TwoWire(0); // set up a new Wire-Instance for BME280 Environment Sensor
Adafruit_BME280 bme;         // use I2C
bool environmentSensorAvailable = false;

// Modem pinning
#define MODEM_RST (5)
#define MODEM_PWKEY (4)
#define MODEM_POWER_ON (23)
#define MODEM_TX (27)
#define MODEM_RX (26)

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800     // Modem is SIM800
#define TINY_GSM_RX_BUFFER (1024) // Set RX buffer to 1Kb
#include <TinyGsmClient.h>

// SIM800L includes, defines and variables
#define SerialAT Serial1
// const char simPIN[]   = "1234"; // SIM card PIN code, if any
TinyGsm modem(SerialAT);

// Display stuff
#include <GxEPD.h>
#include <GxGDEW042Z15/GxGDEW042Z15.h> // 4.2" b/w/r
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

// platformIO board definition: MOSI=15 SCK=14
GxIO_Class io(SPI, /*CS*/ 19, /*DC*/ 18, /*RST*/ 0);
GxEPD_Class display(io, /*RST*/ 0, /*BUSY*/ 2);
#define BLACK (0x0000)
#define WHITE (0xFFFF)
#define COLOR (0xF800)
#define HAS_RED_COLOR

// screen positions and text formatting
#define SMS_X (0)
#define SMS_Y (67)
#define SMS_LINELENGTH (55)
#define SMS_LINES (2)
#define SMS_LINEHEIGHT (16)

#define WISDOM_X (0)
#define WISDOM_Y (118)
#define WISDOM_LINELENGTH (55)
#define WISDOM_LINES (2)
#define WISDOM_LINEHEIGHT (16)

#define SIGBAR_X (345)
#define SIGBAR_Y (10)
#define SIGBAR_NUMBARS (5)
#define SIGBAR_BARWIDTH (7)
#define SIGBAR_BARHEIGHT (40)
#define SIGBAR_HEIGHTDELTA (5)
#define SIGBAR_GAP (3)

#include <Fonts/Org_01.h>

// Custom 8-bit Fonts including character codes 128-255 (e.g. öäü)
#include <FreeSans7pt8b.h>
#include <FreeSansBold7pt8b.h>
#include <FreeSans9pt8b.h>
#include <FreeSansBold9pt8b.h>
#include <FreeSans12pt8b.h>
#include <FreeSansBold12pt8b.h>
#include <LiberationSansNarrow9pt8b.h>
#include <LiberationSansNarrowBold9pt8b.h>

// images
#include <images.h>
#include <vector>
vector<Image> slideshow;

// texts for wisdom of the day
#include <wisdom.h>

// Statistics Helper-Class
#include <timeseries.h>
Timeseries tempStats(5000U);
Timeseries humStats(5000U);
Timeseries pressStats(5000U);

// uptime calculation
#include <uptime.h>
Uptime uptime;

// charts
#include <chart.h>
Chart chart;

// Global Variables
float currentTemperatureCelsius = 0;
float currentHumidityPercent = 0;
float currentPressurePascal = 0;

uint8_t currentSignalStrength = 0;
int currentYear = 0;
int currentMonth = 0;
int currentDay = 0;
int currentHour = 0;
int currentMin = 0;
int currentSec = 0;
float currentTimezone = 0.0;

String smsText = "";
String smsNumber = "";
String smsTime = "";

int wisdomText = 0;

// Track initialisation
uint32_t initStage = 0;

// Flow control, basic task scheduler
#define SCHEDULER_MAIN_LOOP_MS (10) // ms
uint32_t counterBase = 0;
uint32_t counter300s = 0;
uint32_t counter1h = 0;
bool enableDisplay = true; // display output can be disabled for testing purposes with this flag

/**
 * helper function to send AT Command to modem and write command and result to Monitor
 * 
 ******************************************************/
String sendATcommand(const char *toSend, unsigned long milliseconds)
{
  String result;

  Serial.print("Sending AT command: ");
  Serial.println(toSend);
  SerialAT.println(toSend);

  unsigned long startTime = millis();
  Serial.print("Received AT result: ");

  while (millis() - startTime < milliseconds)
  {
    if (SerialAT.available())
    {
      char c = SerialAT.read();
      Serial.write(c);
      result += c; // append to the result string
    }
  }

  return result;
}

/**
 * Compare the timestamps from two SMS messages. Return true if second is newer than first, false otherwise
 * 
 ******************************************************/
boolean smsTimeCompare(String first, String second)
{
  uint8_t firstYear, firstMonth, firstDay, firstHour, firstMinute, firstSecond;
  uint8_t secondYear, secondMonth, secondDay, secondHour, secondMinute, secondSecond;
  boolean result;

  firstYear = first.substring(0, 2).toInt();
  firstMonth = first.substring(3, 5).toInt();
  firstDay = first.substring(6, 8).toInt();
  firstHour = first.substring(9, 11).toInt();
  firstMinute = first.substring(12, 14).toInt();
  firstSecond = first.substring(15, 17).toInt();

  secondYear = second.substring(0, 2).toInt();
  secondMonth = second.substring(3, 5).toInt();
  secondDay = second.substring(6, 8).toInt();
  secondHour = second.substring(9, 11).toInt();
  secondMinute = second.substring(12, 14).toInt();
  secondSecond = second.substring(15, 17).toInt();

  result = false;
  if (first == "")
    result = true;
  else if (second == "")
    result = false;
  else if (secondYear > firstYear)
    result = true;
  else if (secondMonth > firstMonth)
    result = true;
  else if (secondDay > firstDay)
    result = true;
  else if (secondHour > firstHour)
    result = true;
  else if (secondMinute > firstMinute)
    result = true;
  else if (secondSecond > firstSecond)
    result = true;
  else
    result = false;

  return result;
}

/**
 * Insert newlines in string after first space before width and limit to maxlines
 * 
 ******************************************************/
String textWrap(String input, int width, int maxlines)
{
  int lastspace = -1;
  int lines = 0;
  int pos = 0;
  String result, tmp;

  tmp = input; // copy input String
  result = "";

  while (lines < maxlines)
  {

    lastspace = -1;

    // look for poition of last space in one line
    // also seems to work if remainder of input text in tmp is less than width :-)
    for (pos = 0; pos < width; pos++)
      if (tmp.substring(pos, pos + 1) == " ")
        lastspace = pos;

    // if no space in line OR we are in the last line OR there is no space in the last third of the line
    // insert newline after width characters
    if (lastspace == -1 || (lines == maxlines - 1) || lastspace < (width * 2 / 3))
    {
      result = result + tmp.substring(0, width) + "\n";
      tmp = tmp.substring(width, tmp.length());
    }
    // otherwise insert newline at the last space in the line
    else
    {
      result = result + tmp.substring(0, lastspace) + "\n";
      tmp = tmp.substring(lastspace + 1, tmp.length());
    }

    lines++;
  }

  return result;
}

/**
 * Check for SMS and update global variables if SMS received.
 * 
 ******************************************************/
void checkSMS(void)
{
  String tmpNumber, tmpTime, tmpText;

  Serial.println("[ MODEM  ] Checking for SMS ... ");

  // request unread received SMS mesages
  // sendATcommand("AT+CMGL=\"REC UNREAD\"" , 1000);

  // request all SMS mesages
  String buffer = sendATcommand("AT+CMGL=\"ALL\"", 2000);

  // cycle through all SMS in memory and use newest one
  while ((buffer.indexOf("+CMGL:")) != -1)
  {
    // example result. more messages with +CMGL before OK
    // +CMGL: 1,"REC READ","0190696969","","20/06/23,19:10:24+00"
    // Ruf! Mich! An!
    // OK

    buffer = buffer.substring(buffer.indexOf(",\"") + 2, buffer.length()); // skip eerything until after the first ,"
    buffer = buffer.substring(buffer.indexOf(",\"") + 2, buffer.length()); // skip eerything until after the second ,"
    tmpNumber = buffer.substring(0, buffer.indexOf("\""));                 // read sender number until "
    Serial.print("[ MODEM  ] SMS sender number: ");
    Serial.print(tmpNumber);
    Serial.print(" ,");

    buffer = buffer.substring(buffer.indexOf(",\"") + 2, buffer.length()); // skip eerything until after the third ,"
    buffer = buffer.substring(buffer.indexOf(",\"") + 2, buffer.length()); // skip eerything until after the fourth ,"
    tmpTime = buffer.substring(0, buffer.indexOf("+"));                    // read timestamp until + (leave out timezone Information)
    Serial.print("SMS timestamp: ");
    Serial.print(tmpTime);
    Serial.print(" ,");

    if (buffer.indexOf("+CMGL:") != -1)
    {
      tmpText = buffer.substring(buffer.indexOf("\"") + 1, buffer.indexOf("+CMGL:") - 1); // SMS text is now after " and before OK
      buffer = buffer.substring(buffer.indexOf("+CMGL:"), buffer.length());               // skip eerything until next SMS
    }
    else
    {
      tmpText = buffer.substring(buffer.indexOf("\"") + 1, buffer.indexOf("OK") - 1); // SMS text is now after " and before OK
    }

    tmpText.trim(); // get rid of leading and trailing whtespaces, newlines, ...
    Serial.print("SMS text: ");
    Serial.println(tmpText);

    if (smsTimeCompare(smsTime, tmpTime)) // if current SMS has a more up to date timestamp, use that one
    {
      smsNumber = tmpNumber;
      smsTime = tmpTime;
      smsText = tmpText;
    }

    // Delete all SMS messages except for unread ones
    sendATcommand("AT+CMGD=1,3", 1000);
  }
}

/**
 * Writes multiline text to the display in such a way that x position is correct for following lines
 * The function looks for \n to identify newline.
 * current color and font is used, y distance between two lines has to be given as an argument
 * 
 ******************************************************/
void writeText(String text, int xPos, int yPos, int yDelta)
{
  int yPosNext, indexNL;
  String tmpText;

  yPosNext = yPos;
  tmpText = text;

  do
  {
    // find next Newline "\n"
    indexNL = tmpText.indexOf("\n");
    // if none found --> take remainder of input text
    if (indexNL == -1)
      indexNL = tmpText.length();

    // print current line
    display.setCursor(xPos, yPosNext);
    display.print(tmpText.substring(0, indexNL));

    // remove text that has already been printed
    tmpText = tmpText.substring(indexNL + 1, tmpText.length());

    yPosNext += yDelta;
  } while (tmpText.length() > 0);
}

/**
 * Update all information from modem.
 * 
 ******************************************************/
void updateModemInfo(void)
{
  // read current data
  currentSignalStrength = modem.getSignalQuality();
  Serial.print("[ MODEM  ] Sigal Quality [0-31]: ");
  Serial.println(currentSignalStrength);

  if (modem.isNetworkConnected())
  {
    modem.getNetworkTime(&currentYear, &currentMonth, &currentDay, &currentHour, &currentMin, &currentSec, &currentTimezone);
    Serial.printf("[ MODEM  ] Current Network Time (Values) - Year: %d, Month: %02d, Day: %02d, Hour: %02d, Minute: %02d, Second: %02d, Timezone: %.1f\n",
                  currentYear, currentMonth, currentDay, currentHour, currentMin, currentSec, currentTimezone);
  }
  else
  {
    Serial.println("[  WARN  ] Network not connected.");
  }
}

/**
 * Sample measurements from environment sensor.
 * 
 ******************************************************/
void doMeasurement(void)
{
  if (environmentSensorAvailable)
  {
    // read current measurements
    currentTemperatureCelsius = bme.readTemperature();
    currentHumidityPercent = bme.readHumidity();
    currentPressurePascal = bme.readPressure() + PRESSURE_MEASUREMENT_CALIBRATION;

    // update statistics for each measurement
    uint32_t timestamp = uptime.getSeconds();
    tempStats.push(timestamp, currentTemperatureCelsius);
    humStats.push(timestamp, currentHumidityPercent);
    pressStats.push(timestamp, currentPressurePascal / 100.); // use hPa
  }
  else
  {
    Serial.println("[  WARN  ] Environment sensor not available.");
  }
}

/**
 * Update the screen
 * 
 ******************************************************/
void updateScreen()
{
  // Temperature Demo
  display.fillScreen(WHITE);

  display.drawBitmap(images.logo.color, 0, 0, 50, 50, COLOR, display.bm_invert);
  display.drawBitmap(images.logo.black, 0, 0, 50, 50, BLACK, display.bm_invert | display.bm_transparent);

  // Date and Update time
  display.setFont(&FreeSans12pt8b);
  display.setTextColor(COLOR);
  display.setCursor(155, 25);
  if (currentYear == 0)
    display.printf("--.--.----");
  else
    display.printf("%02d.%02d.%04d", currentDay, currentMonth, currentYear);

  // Udate Time
  display.setFont(&FreeSans7pt8b);
  display.setTextColor(BLACK);
  display.setCursor(140, 42);
  if (currentYear == 0)
    display.printf(" Last update: --:--:--");
  else
    display.printf(" Last update: %02d:%02d:%02d", currentHour, currentMin, currentSec);

  // Signal strength
  chart.signalBars(&display, currentSignalStrength,
                   SIGBAR_X, SIGBAR_Y, SIGBAR_NUMBARS, SIGBAR_BARWIDTH, SIGBAR_BARHEIGHT, SIGBAR_HEIGHTDELTA, SIGBAR_GAP, BLACK, COLOR);

  // SMS display
  display.setFont(&LiberationSansNarrow_Bold9pt8b);
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(SMS_X, SMS_Y);
  display.print("OTA Message ");

  display.setFont(&LiberationSansNarrow_Regular9pt8b);
  display.print(" from: ");
  if (smsText == "")
    display.print("-------");
  else if (smsNumber == "")
    display.print("unknown");
  else
    display.print(smsNumber);

  /* because of space restrictions, currently don't print timestamp
  display.print("  at ");
  if (smsText == "")
    display.print("-----------");
  else
    display.print(smsTime);
  */

  display.setFont(&LiberationSansNarrow_Regular9pt8b);
  display.setTextColor(GxEPD_BLACK);
  if (smsText == "")
    writeText(textWrap("Your ex coleagues have forgotten about you. No new OTA Message has been sent to you!", SMS_LINELENGTH, SMS_LINES), SMS_X, SMS_Y + SMS_LINEHEIGHT, SMS_LINEHEIGHT);
  else
    writeText(textWrap(smsText, SMS_LINELENGTH, SMS_LINES), SMS_X, SMS_Y + SMS_LINEHEIGHT, SMS_LINEHEIGHT);

  // Wisdom of the day
  display.setFont(&LiberationSansNarrow_Bold9pt8b);
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(WISDOM_X, WISDOM_Y);
  display.print("Wisdom of the day:");

  display.setFont(&LiberationSansNarrow_Regular9pt8b);
  writeText(textWrap(wisdomTexts[wisdomText], WISDOM_LINELENGTH, WISDOM_LINES), WISDOM_X, WISDOM_Y + WISDOM_LINEHEIGHT, WISDOM_LINEHEIGHT);

  // Uptime and Memory stats
  display.setFont(&Org_01);
  display.setTextColor(BLACK);
  display.setCursor(0, 298);
  display.printf("Free: %u KiB (%u KiB)  Temp: %u (%u B)  Hum: %u (%u B) Press: %u (%u B)",
                 ESP.getFreeHeap() / 1024,
                 ESP.getMaxAllocHeap() / 1024,
                 tempStats.size(),
                 sizeof(tempStats.data) + sizeof(Point) * tempStats.data.capacity(),
                 humStats.size(),
                 sizeof(humStats.data) + sizeof(Point) * humStats.data.capacity(),
                 pressStats.size(),
                 sizeof(pressStats.data) + sizeof(Point) * pressStats.data.capacity());

  // current values
  display.fillRect(0, 260, 400, 30, COLOR);
  display.setFont(&FreeSansBold12pt8b);
  display.setTextColor(BLACK);
  display.setCursor(10, 282);
  display.printf("%.1f\xb0"
                 "C",
                 currentTemperatureCelsius);
  display.setCursor(145, 282);
  display.printf("%.0f%%", currentHumidityPercent);
  display.setCursor(280, 282);
  display.printf("%.0fhPa", currentPressurePascal / 100);

  // Linecharts
  // Y-Axis Labels
  display.setFont(&Org_01);
  display.setTextColor(BLACK);
  display.setCursor(2, 165);
  display.printf("%.1f", tempStats.max);
  display.setCursor(2, 254);
  display.printf("%.1f", tempStats.min);

  display.setCursor(135, 165);
  display.printf("%.0f", humStats.max);
  display.setCursor(135, 254);
  display.printf("%.0f", humStats.min);

  display.setCursor(268, 165);
  display.printf("%.1f", pressStats.max);
  display.setCursor(268, 254);
  display.printf("%.1f", pressStats.min);

  // Frame
  display.drawFastHLine(0, 159, 400, BLACK);
  display.drawFastHLine(0, 256, 400, BLACK);
  display.drawFastVLine(133, 159, 97, BLACK);
  display.drawFastVLine(266, 159, 97, BLACK);

  // Charts
  chart.lineChart(&display, &tempStats, 0, 160, 130, 95, BLACK);
  chart.lineChart(&display, &humStats, 135, 160, 130, 95, BLACK);
  chart.lineChart(&display, &pressStats, 270, 160, 130, 95, BLACK);

  // Slideshow every 1h
  if (!((counter300s + 1) % 12))
  {
    display.drawBitmap(slideshow.at(counter300s % slideshow.size()).color, 0, 0, 400, 400, COLOR, display.bm_invert);
    display.drawBitmap(slideshow.at(counter300s % slideshow.size()).black, 0, 0, 400, 300, BLACK, display.bm_invert | display.bm_transparent);
  }

  display.update();
}

/**
 * This is run once on startup
 * 
 ******************************************************/
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  initStage++;

  // Setup serial connection for debugging
  Serial.begin(115200U);
  Serial.println();
  Serial.println("[  INIT  ] Begin");
  initStage++;

  Serial.printf("[  INIT  ] ChipRevision: 0x%02X    CpuFreq: %uMHz   FlashChipSize: %uKiB   HeapSize: %uKiB   MAC: %" PRIX64 "   SdkVersion: %s\n",
                ESP.getChipRevision(),
                ESP.getCpuFreqMHz(),
                ESP.getFlashChipSize() / 1024,
                ESP.getHeapSize() / 1024,
                __builtin_bswap64(ESP.getEfuseMac()) >> 16, // swap endianess and rshift for better readability
                ESP.getSdkVersion());
  initStage++;

  // Initialize Environment Sensor
  pinMode(BME280_PIN_VCC, OUTPUT);
  digitalWrite(BME280_PIN_VCC, HIGH); // Power-Supply via GPIO (BME280 uses less than 1mA).
  delay(5);                           // wait for BMW280 to power up. Takes around 2ms.
  initStage++;

  if (I2CBME.begin(BME280_PIN_I2C_SDA, BME280_PIN_I2C_SCL, 250000U)) // set I2C-Clock to 250kHz
  {
    initStage++;
    if (bme.begin(BME280_ADDR, &I2CBME)) // use custom Wire-Instance to avoid interference with other libraries.
    {
      initStage++;
      environmentSensorAvailable = true;
      Serial.println("[  INIT  ] found BME280 environment sensor");
    }
  }

  if (environmentSensorAvailable)
  {
    initStage++;
    // Setup Environment Sensor
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BME280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BME280::SAMPLING_X16,    /* Hum. oversampling */
                    Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BME280::FILTER_X16,      /* Filtering. */
                    Adafruit_BME280::STANDBY_MS_500); /* Standby time. */
    Serial.println("[  INIT  ] BME280 setup done");
  }
  else
  {
    Serial.println("[ ERROR  ] Could not find a BME280 sensor, check wiring!");
  }
  initStage++;

  Serial.println("[  INIT  ] setup ePaper display");
  delay(100); // wait a bit, before display-class starts writing to serial out

  display.init(/*115200*/); // uncomment serial speed definition for debug output
  io.setFrequency(500000L); // set to 500kHz; the default setting (4MHz) could be unreliable with active modem and unshielded wiring
  delay(100);
  initStage++;

  // Initialize SIM800L Module
  // Set-up modem reset, enable, power pins
  Serial.println("[  INIT  ] modem power-on");
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  Serial.println("[  INIT  ] Initializing serial interface to modem...");
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  initStage++;

  // Restart takes quite some time
  // use modem.init() if you don't need the complete restart
  Serial.println("[  INIT  ] Initializing modem...");
  modem.restart();

  String name = modem.getModemName();
  Serial.print("[  INIT  ] Modem Name: ");
  Serial.print(name);
  String modemInfo = modem.getModemInfo();
  Serial.print(", Modem Info: ");
  Serial.println(modemInfo);
  initStage++;

  // set SMS text format
  sendATcommand("AT+CMGF=1", 1000);
  initStage++;

  // setup slideshow content
  Serial.println("[  INIT  ] Creating slideshow...");
  slideshow.push_back(images.beer);
  slideshow.push_back(images.bmw);
  slideshow.push_back(images.bruce);
  slideshow.push_back(images.yellowScreen);
  slideshow.push_back(images.coffin);
  slideshow.push_back(images.parking);
  slideshow.push_back(images.unittest);
  slideshow.push_back(images.fixing);
  initStage++;

  // initialize random number generator, timer should be ok for that purpose
  Serial.println("[  INIT  ] Setting up RNG...");
  randomSeed(uptime.getMicros());
  initStage++;

  Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);
  digitalWrite(LED_BUILTIN, HIGH); // turn on LED to indicate normal operation;

  // delay to allow modem connect to network
  delay(4000);
}

/**
 * This runs forever
 * 
 ******************************************************/
void loop()
{
  // 100ms Tasks
  if (!(counterBase % (100L / SCHEDULER_MAIN_LOOP_MS)))
  {
    digitalWrite(LED_BUILTIN, HIGH); // regularly turn on LED
  }

  // 500ms Tasks
  if (!(counterBase % (500L / SCHEDULER_MAIN_LOOP_MS)))
  {
  }

  // 2s Tasks
  if (!(counterBase % (2000L / SCHEDULER_MAIN_LOOP_MS)))
  {
    // indicate alive
    digitalWrite(LED_BUILTIN, LOW);

    // Sequence to attach to GPRS:
    // AT+CGATT=1
    // AT+SAPBR=3,1,"Contype","GPRS"
    // AT+SAPBR=3,1,"APN","iot.1nce.net"
    // AT+SAPBR=1,1

    // Check with; will return IP-address
    // AT+SAPBR=2,1
    //  +SAPBR: 1,1,"100.85.212.1"
    //  OK

    // FIXME: time sync over GPRS connection required. GPRS connection must be established before.
    // AT+CIPGSMLOC=2,1
    //  +CIPGSMLOC: 0,2020/07/04,12:23:20
    //  OK
  }

  // 30s Tasks
  if (!(counterBase % (30000L / SCHEDULER_MAIN_LOOP_MS)))
  {
    checkSMS();

    doMeasurement();

    // memory state
    Serial.printf("[ MEMORY ] Free: %u KiB (%u KiB)  Temp: %u (%u B)  Hum: %u (%u B) Press: %u (%u B) Uptime: %u s\n",
                  ESP.getFreeHeap() / 1024,
                  ESP.getMaxAllocHeap() / 1024,
                  tempStats.size(),
                  sizeof(tempStats.data) + sizeof(Point) * tempStats.data.capacity(),
                  humStats.size(),
                  sizeof(humStats.data) + sizeof(Point) * humStats.data.capacity(),
                  pressStats.size(),
                  sizeof(pressStats.data) + sizeof(Point) * pressStats.data.capacity(),
                  uptime.getSeconds());
  }

  // 300s Tasks
  // e-Paper Display MUST not be updated more often than every 180s to ensure lifetime function
  if (!(counterBase % (300000L / SCHEDULER_MAIN_LOOP_MS)))
  {
    if (counter300s > 0) // don't trim/compact on startup
    {
      // RAM is limited so we cut off the timeseries after x days
      tempStats.trim(uptime.getSeconds(), 7 * 24 * 3600);
      humStats.trim(uptime.getSeconds(), 7 * 24 * 3600);
      pressStats.trim(uptime.getSeconds(), 7 * 24 * 3600);

      // FIXME: Filter high-frequency noise somehow

      // apply compression (Ramer-Douglas-Peucker)
      tempStats.compact(0.05);
      humStats.compact(0.2);
      pressStats.compact(0.05);
    }

    // update modem Information every time display is updated, to get current timestamp
    updateModemInfo();

    if (enableDisplay)
    {
      Serial.print("[  DISP  ] Updating... ");
      updateScreen();
      Serial.println("ok");
    }

    counter300s++;
  }

  // 1h Tasks
  if (!(counterBase % (3600000L / SCHEDULER_MAIN_LOOP_MS)))
  {
    if (!(counter1h % 7))
    {
      const int wisdomNumberOfTexts = static_cast<int>(wisdomTexts.size());
      wisdomText = random(wisdomNumberOfTexts);
    }
    counter1h++;
  }

  delay(SCHEDULER_MAIN_LOOP_MS);
  counterBase++;
}
