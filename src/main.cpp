#include <Arduino.h>

// Environment sensor includes and defines
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define BME280_ADDR (0x76)
#define PRESSURE_MEASUREMENT_CALIBRATION (6000)
#define SEALEVEL_PRESSURE (1013.25)
#define SEALEVEL_PRESSURE_CALIBRATION (9.65)

Adafruit_BME280 bme; // I2C

// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21  // Original example sets "KeepOn Mode" via I2C in IP5306, looks like only needed when powered from battery?
#define I2C_SCL              22  // curretly not implemented

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#include <TinyGsmClient.h>

// SIM800L includes, defines and variables
#define SerialAT  Serial1
#define SMS_TARGET  "+491719552455"
const char simPIN[]   = "1234"; // SIM card PIN code, if any

TinyGsm modem(SerialAT);



#include <statistics.h>
Statistics tempStats;
Statistics humStats;
Statistics pressStats;

// Global Variables
float currentTemperatureCelsius = 0;
float currentHumidityPercent = 0;
float currentPressurePascal = 0;
int frameCounter = 0;




// helper function to send AT Command to modem and write command and result to Monitor
String sendATcommand(const char *toSend, unsigned long milliseconds)
{
  String result;

  Serial.print("Sending AT command: ");
  Serial.println(toSend);
  SerialAT.println(toSend);

  unsigned long startTime = millis();
  Serial.print("Received AT result: ");

  while (millis() - startTime < milliseconds) {
    if (SerialAT.available()) {
      char c = SerialAT.read();
      Serial.write(c);
      result += c;  // append to the result string
    }
  }

Serial.println();  // new line after timeout.

return result;
}



// run once on startup
void setup()
{
  // Setup serial connection for debugging
  Serial.begin(115200);

  Serial.println("--------------- Startup -----------------");

  // Initialize Environment Sensor
  if (!bme.begin(BME280_ADDR, &Wire))
  {
    Serial.println("[ ERROR ] - Could not find a BME280 sensor, check wiring!");
    while (1)
      ;
  }

  // Setup Environment Sensor
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BME280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BME280::SAMPLING_X16,    /* Hum. oversampling */
                  Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BME280::FILTER_X16,      /* Filtering. */
                  Adafruit_BME280::STANDBY_MS_500); /* Standby time. */


  // Initialize SIM800L Module
  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart takes quite some time
  // use modem.init() if you don't need the complete restart
  Serial.println("Initializing modem...");
  modem.restart();
  
  String name = modem.getModemName();
  Serial.print("Modem Name: ");
  Serial.println(name);

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    Serial.println("Unlocking SIM...");
    modem.simUnlock(simPIN);
  }

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    Serial.println(" fail");
    Serial.println("[ ERROR ] - Could not connect to mobile network, check antenna!");
    while (1);
  }
  Serial.println(" OK");

  // maybe not really needed, since we just connected to the network, but maybe useful in the loop part
  if (modem.isNetworkConnected()) {
    Serial.println("Network connected!");
  }

  int csq = modem.getSignalQuality();
  Serial.print("Sigal Quality [0-31]: ");
  Serial.println(csq);

  // Network time is not set properly if requested too early after network connection
  delay(4000);

}

// run forever
void loop()
{
  frameCounter++;

  // read current measurements
  currentTemperatureCelsius = bme.readTemperature();
  currentHumidityPercent = bme.readHumidity();
  currentPressurePascal = bme.readPressure() + PRESSURE_MEASUREMENT_CALIBRATION;

  // update statistics for each measurement
  tempStats.update(currentTemperatureCelsius);
  humStats.update(currentHumidityPercent);
  pressStats.update(currentPressurePascal / 100.);

  Serial.print("\nTemperature = ");
  Serial.print(currentTemperatureCelsius);
  Serial.print(" °C (▼");
  Serial.print(tempStats.min);
  Serial.print(" ▲");
  Serial.print(tempStats.max);
  Serial.print(")");

  Serial.print("    Humidity = ");
  Serial.print(currentHumidityPercent);
  Serial.print(" % (▼");
  Serial.print(humStats.min);
  Serial.print(" ▲");
  Serial.print(humStats.max);
  Serial.print(")");

  Serial.print("    Pressure = ");
  Serial.print(currentPressurePascal / 100.); // convert to hPa
  Serial.print(" hPa (▼");
  Serial.print(pressStats.min);
  Serial.print(" ▲");
  Serial.print(pressStats.max);
  Serial.print(")");

  Serial.print("    Altitude = ");
  Serial.print(bme.readAltitude(SEALEVEL_PRESSURE + SEALEVEL_PRESSURE_CALIBRATION));
  Serial.print(" m");


  //
  // Get network time as a string
  String time = modem.getGSMDateTime(DATE_FULL);
  Serial.print("Current Network Time [String]: ");
  Serial.println(time);


 // Get network time in individual values
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  float timezone = 0;

  if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec,
                                 &timezone)) {
    Serial.println("Current Network Time (Values): ");
    Serial.print("Year: ");
    Serial.print(year);
    Serial.print("  - Month: ");
    Serial.print(month);
    Serial.print("  - Day: ");
    Serial.print(day);
    Serial.print("  - Hour: ");
    Serial.print(hour);
    Serial.print("  - Minute: ");
    Serial.print(min);
    Serial.print("  - Second: ");
    Serial.print(sec);
    Serial.print("  - Timezone: ");
    Serial.println(timezone);
  }
  else
    Serial.println("[ERROR] Couldn't get network time.");
 
  // Send a SMS message
//  bool res = modem.sendSMS(SMS_TARGET, String("Hier ist das ESP32 Board!"));
//  if (res) { 
//    Serial.print("SMS sent to ");
//    Serial.println(SMS_TARGET);
//  }
//  else
//    Serial.println("[ERROR] SMS failed!");


  // set SMS text format
  sendATcommand("AT+CMGF=1" , 2000);
  // request unread received SMS mesages
//  sendATcommand("AT+CMGL=\"REC UNREAD\"" , 2000);
  // request all SMS mesages
  sendATcommand("AT+CMGL=\"ALL\"" , 2000);

  // Delete all SMS messages except for unread ones
//  sendATcommand("AT+CMGD=1,3" , 2000);

  delay(2000);
}



