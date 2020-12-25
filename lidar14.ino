/*This IoT sketch is for a LIDAR Lite v3HP sensor instrusion alarm with an Arduino MKR WIFI 1010 microcontroller with MKR SD 
 * PROTO SHIELD. The purpose of the PROTO SHIELD is to house the 680 uF capacitor and two 4.7K pullup resistors needed for 
 * stable I2C serial communications with the LIDAR sensor. The PROTO SHIELD also contains an SD memory card which is used to
 * log readings for analysis.
 * 
 * The LIDAR sensor emits a narrow beam of infrared light with a range of up to 40 meters each time the loop is 
 * executed. The returned reflection is converted to the distance in centimeters when a new reading is available over I2C interface.The sketch
 * compares the new distance to the previous distance and generates a notification through www.pushsafer.com to an iPhone XS
 * with the Pushsafer iOS app installed and signed into the Pushsafer website. The microcontroller must be registered on the
 * website account and the access key must be stored in the arduino_secrets.h include file along with the wifi network SSID
 * and password.
 * 
 * A pan/tilt unit with two servos is used for scanning the LIDAR sensor; however, false alarms tend to be generated due
 * to movement of the sensor. The sketch is designed to activate or deactivate the servos for use as a stationary beam-
 * interrupter or a scanning sensor.
 */
#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>
#include <WiFiNINA.h>
#include <WiFiClient.h>
#include <RTCZero.h>
#include <Servo.h>
#include <Pushsafer.h>
#include <SD.h>
#include "arduino_secrets.h"

// Wifi network login credentials
char ssid[] = SECRET_SSID;     // wifi network SSID (name) from arduino_secrets.h
char password[] = SECRET_PASS; // wifi network key from arduino_secrets.h

// Pushsafer private or alias key
char PushsaferKey[] = SECRET_KEY; // pushsafer private key from arduin_secrets.h

//Object declarations
WiFiClient client; // The wifi client object is used for pushsafer notification
Pushsafer pushsafer(PushsaferKey, client); // Pushsafer object to send event
RTCZero rtc; // Real time clock object on Arduino MKR board
LIDARLite_v3HP myLidarLite; // LIDAR sensor object
Servo panservo,tiltservo; // Servo objects used for pan/tilt servos when activated

#define FAST_I2C // Operate the LIDAR Lite v3HP in fast serial communications mode

//Variables
float cmtofeet = 0.0328084; // Conversion factor from cm to feet
uint16_t threshld = 10; // Threshold value for change in LIDAR distance reading to activate alarm
String dtstg = String(__DATE__); // Compiler date for setting RTC
String tmstg = String(__TIME__); // Compiler time for setting RTC
uint16_t distance,d1,d2; // LIDAR sensor distance
long t; // Time variable to prevent multiple alarm reporting on each beam interruption
int panservopin = 9; // pan servo attach pin when used
int tiltservopin = 10; // tilt servo attach pin when used
int servodelay = 15; // servo positioning delay in milliseconds when used
int alarmdelay = 6000; // delay after alarm to prevent multiple alarms
int panpos; // pan servo position when used
int tiltpos; // tilt servo position when used
int panservoX1 = 0,panservoX2 = 90; // pan servo start and end positions when used
int tiltservoY1 = 150,tiltservoY2 = 170; // tilt servo start and end positions when used
bool push=false; // flag to prevent push notifications during testing
const int chipSelect = 4; // SD card select pin
String lineout; // SD card write line buffer
File logfile; // SD card file handle
String fn = "LOGGER00.CSV"; // SD file name

// Executes once on startup
void setup()
{
  Serial.begin(115200); // Start serial communications
  while (!Serial);
  Wire.begin(); // Wait for serial port to initialize
  panservo.attach(panservopin); // Initialize the pan servo object
  pinMode(chipSelect, OUTPUT);
  
  // Attempt to connect to Wifi network:
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // Wait until wifi connects
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  // Wifi is connected. Display IP address
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Real time clock is used to timestamp alarms
  rtc.begin(); // Start the real time clock
  rtcSet(); // Set the RTC with the compile date and time
  Serial.print("RTC started at: ");
  if (rtc.getMonth() < 10) Serial.print("0"); // Display leading month zero if < 10
  Serial.print(rtc.getMonth());
  Serial.print("-");
  if (rtc.getDay() < 10) Serial.print("0"); // Display leading day zero if < 10
  Serial.print(rtc.getDay());
  Serial.print("-");
  Serial.print(rtc.getYear());
  Serial.print(" ");
  if (rtc.getHours() < 10) Serial.print("0"); // Display leading hours zero if < 10
  Serial.print(rtc.getHours());
  Serial.print(":");
  if (rtc.getMinutes() < 10) Serial.print("0"); // Display leading minutes zero if < 10
  Serial.print(rtc.getMinutes());
  Serial.print(":");
  if (rtc.getSeconds() < 10) Serial.print("0"); // Display leading seconds zero if < 10
  Serial.println(rtc.getSeconds());

  myLidarLite.configure(3); // Configure the LIDAR sensor for maximum range
  delay(100); // Wait for lidar to stabilize
  panservo.write(panservoX1); // position the pan servo at the initial position

  // Initialize the SD card and create a log file
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
    }
    Serial.println("card initialized.");

    SD.remove("LOGGER00.CSV");
    // create a new file
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }
    
    if (! logfile) {
      error("couldnt create file");
    }
    
    Serial.print("Logging to: ");
    Serial.println(filename);

  t = millis(); // take a time stamp before starting the main loop for later use in closing the log file
}

// Main loop executes continuously
void loop()
{
  uint8_t  newDistance = 0; // Flag set by distanceSingle function to signal new data
  
  for (panpos = panservoX1;panpos <= panservoX2;panpos++)
  {
    panservo.write(panpos);
    delay(servodelay);
    newDistance = distanceSingle(&distance);
    if (newDistance)
    {
      printValues();
    }
  }

  for (panpos = panservoX2;panpos >= panservoX1;panpos--)
  {
    panservo.write(panpos);
    delay(servodelay);
    newDistance = distanceSingle(&distance);
    if (newDistance)
    {
      printValues();
    }
  }

  /* Every 10 milliseconds flush the file buffer and write it to the SD card so that no data is lost when power
     is removed.*/
  if ((millis() - t) > 10)
  {
    logfile.flush();
  }
}

// This function sends a single LIDAR pulse and returns a flag to indicate if there
// is new data to the calling instruction in the main loop. The distance is read as a
// pointer to the distance variable. This function comes from the LIDARLite_v3HP library
// v3HP_I2C example sketch.
uint8_t distanceSingle(uint16_t * distance)
{
    // 1. Wait for busyFlag to indicate device is idle. This must be
    //    done before triggering a range measurement.
    myLidarLite.waitForBusy();

    // 2. Trigger range measurement.
    myLidarLite.takeRange();

    // 3. Wait for busyFlag to indicate device is idle. This should be
    //    done before reading the distance data that was triggered above.
    myLidarLite.waitForBusy();

    // 4. Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    return 1; // Return a boolean value that indicates there is new data to be read
}

// Display a timestamp from the RTC and distance on the serial monitor and generate a
// Pushsafer notification when an alarm is triggered
void printValues()
{
  String yy,mm,dd,hr,mn,se; // String variables to hold the elements of the RTC timestamp
  yy = String(rtc.getYear()); // Get the timestamp year from the RTC
  if (rtc.getMonth() < 10) // Add a leading month zero if < 10
  {
    mm = String("0")+String(rtc.getMonth());
  } else {
    mm = String(rtc.getMonth());
  }
  if (rtc.getDay() < 10) // Add a leading day zero if < 10
  {
    dd = String("0")+String(rtc.getDay());
  } else {
    dd = String(rtc.getDay());
  }
  if (rtc.getHours() < 10) // Add a leading hour zero if < 10
  {
    hr = String("0")+String(rtc.getHours());
  } else {
    hr = String(rtc.getHours());
  }
  if (rtc.getMinutes() < 10) // Add a leading minute zero if < 10
  {
    mn = String("0")+String(rtc.getMinutes());
  } else {
    mn = String(rtc.getMinutes());
  }
  if (rtc.getSeconds() < 10) // Add a leading second zero if < 10
  {
    se = String("0")+String(rtc.getSeconds());
  } else {
    se = String(rtc.getSeconds());
  }
  // Combine the RTC timestamp elements into a single string
  String timestr = mm+"-"+dd+"-"+yy+" "+hr + ":" + mn + ":" + se;
    
  // This is the structure of the pushsafer input object
  // Change the values for different notifications
  struct PushSaferInput input;
  input.message = "LiDAR sensed movement at " + String(timestr);
  input.title = "LiDAR sensor tripped!";
  input.sound = "8";
  input.vibration = "1";
  input.icon = "1";
  input.iconcolor = "#FFCCCC";
  input.priority = "1";
  input.device = "a";
  input.url = "https://www.pushsafer.com";
  input.urlTitle = "Open Pushsafer.com";
  input.picture = "";
  input.picture2 = "";
  input.picture3 = "";
  input.time2live = "";
  input.retry = "";
  input.expire = "";
  input.answer = "";

  // Sends the push notification to the phone if the push switch is true
  if (push) Serial.print(pushsafer.sendEvent(input));

  // Displays the alarm timestamp and distance in feet
  lineout = String("\"")+timestr+String("\",")+String(distance);
  logfile.println(lineout);
  Serial.println(lineout);
  //Serial.print(" pos: ");
  //Serial.print(panpos);
  //Serial.print(" distance: ");
  //Serial.println(distance);
}

/* Sets the MKR WIFI 1010 real time clock (RTC) to the sketch compile time. The RTC will keep time if a battery
 *  is connected to the 1010 but the RTC must be set if no battery is connected.
 */
void rtcSet()
{
  String months = "JanFebMarAprMayJunJulAugSepOctNovDec"; // Month identifier array

  // Parses the compiler DATE string into separate substrings needed to set the RTC
  String mm = dtstg.substring(0,3);
  String dd = dtstg.substring(4,7);
  String yy = dtstg.substring(8,11);

  // Convert the date strings to integers needed to set the RTC
  int ddnbr = dd.toInt();
  int mmnbr = int((months.indexOf(mm)/3)+1);
  int yynbr = yy.toInt();

  // Parses the compiler TIME string into separate substrings needed to set the RTC
  String hh = tmstg.substring(0,2);
  String mn = tmstg.substring(3,5);
  String ss = tmstg.substring(6,8);

  // Convert the time strings to integers needed to set the RTC
  int hhnbr = hh.toInt();
  int mnnbr = mn.toInt();
  int ssnbr = mn.toInt();

  // Set the RTC date and time
  rtc.setDate(ddnbr,mmnbr,yynbr);
  rtc.setTime(hhnbr,mnnbr,ssnbr);
}

/* This function displays an error on the serial monitor if the SD card cannot be initialized.*/
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  while(1);
}
