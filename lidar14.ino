/*This IoT sketch is for a LIDAR Lite v3HP sensor instrusion alarm with an Arduino MKR WIFI 1010 microcontroller. The LIDAR 
 * sensor emits a narrow beam of infrared light with a range of up to 40 meters each time the loop is executed. The returned 
 * reflection is converted to the distance in centimeters when a new reading is available over I2C interface.The sketch
 * compares the new distance to the previous distance and generates a notification through www.pushsafer.com to an iPhone XS
 * with the Pushsafer iOS app installed and signed into the Pushsafer website. The microcontroller must be registered on the
 * website account and the access key must be stored in the arduino_secrets.h include file along with the wifi network SSID
 * and password.
*/
#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>
#include <WiFiNINA.h>
#include <WiFiClient.h>
#include <RTCZero.h>
#include <Servo.h>
#include <Pushsafer.h>
#include "arduino_secrets.h"

// Wifi network login credentials
char ssid[] = SECRET_SSID;     // wifi network SSID (name) from arduino_secrets.h
char password[] = SECRET_PASS; // wifi network key from arduino_secrets.h

// Pushsafer private or alias key
char PushsaferKey[] = SECRET_KEY; // pushsafer private key from arduin_secrets.h

//Object declarations
WiFiClient client;
Pushsafer pushsafer(PushsaferKey, client); // Pushsafer object to send event
RTCZero rtc; // Real time clock object on Arduino MKR board
LIDARLite_v3HP myLidarLite; // LIDAR sensor object
Servo panservo,tiltservo; // Servo objects

#define FAST_I2C // Operate the LIDAR Lite v3HP in fast serial communications mode

//Variables
float cmtofeet = 0.0328084; // Conversion factor from cm to feet
uint16_t threshld = 10; // Threshold value for change in LIDAR distance reading to activate alarm
String dtstg = String(__DATE__); // Compiler date for setting RTC
String tmstg = String(__TIME__); // Compiler time for setting RTC
uint16_t distance,d1,d2; // LIDAR sensor distance
long t; // Time variable to prevent multiple alarm reporting on each beam interruption
int panservopin = 9; // servo attach pin when used
int servodelay = 100; // servo positioning delay when used
int pulsedelay = 1; // delay between lidar pulses
int alarmdelay = 6000; // delay after alarm to prevent multiple alarms
int panpos; // pan servo position when used
int tiltpos; // tilt servo position when used
int panservoX1 = 0,panservoX2 = 45; // pan servo start and end positions when used
int tiltservoY1 = 45,tiltservoY2 = 55; // tilt servo start and end positions when used
int redledpin = 3; // visual two-color led indicates alarm when serial monitor unavailable
int greenledpin = 2; // visual two-color led indicates alarm when serial monitor unavailable
bool push=false; // flag to prevent push notifications during testing


// Executes once on startup
void setup()
{
  Serial.begin(115200); // Start serial communications
  Wire.begin(); // Wait for serial port to initialize
  panservo.attach(panservopin); // Initialize the pan servo object

  pinMode(redledpin, OUTPUT); // visual alarm pin
  pinMode(greenledpin, OUTPUT); // visual alarm pin
  
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
  delay(50); // Wait for lidar to stabilize
  //Serial.println("Sweep\tPos\tRef\tDist\tDiff"); // headings for serial output if desired
}

// Main loop executes continuously
void loop()
{
  uint8_t  newDistance = 0; // Flag set by distanceSingle function to signal new data
  
  newDistance = distanceSingle(&distance); // take the first distance measurement
  if (newDistance) // if the measurement is new, then store it in d1
  {
    d1 = distance;
  }
  delay(pulsedelay); // wait before taking the second distance measurement
  
  newDistance = distanceSingle(&distance); // take the second distance measurement
  if (newDistance) // if the measurement is new, then store it in d2
  {
    d2 = distance;
  }

   // if the absolute difference between d1 and d2 > threshld and the last alarm was more than 2 seconds
   // ago, generate an alarm.
  if (abs(d1 - d2) > threshld & millis() - t > 2000)
  {    
    PrintValues();
    delay(alarmdelay);
    t = millis(); 
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

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (myLidarLite.getBusyFlag() == 0)
    {
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}

// Display a timestamp from the RTC and distance on the serial monitor and generate a
// Pushsafer notification when an alarm is triggered
void PrintValues()
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
  Serial.print("MOVEMENT at ");
  Serial.print(timestr);
  Serial.print(" d1(ft): ");
  Serial.print(d1*cmtofeet);
  Serial.print(" d2(ft): ");
  Serial.println(d2*cmtofeet);

  // Flash red-green led for seconds passed in parameter
  RedGreenLED(3);
}

// Sets the MKR WIFI 1010 RTC to the sketch compile time
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

// function flashes the red-green alarm led for the number of seconds in the parameter
void RedGreenLED(int t)
{
  for (int i = 0;i <= t;i++)
  {
    // alternately flash red and green leds
    analogWrite(redledpin, 0);
    analogWrite(greenledpin, 255);
    delay(500);
    analogWrite(redledpin, 255);
    analogWrite(greenledpin, 0);
    delay(500);    
  }
  // turn both leds off
  analogWrite(redledpin, 0);
  analogWrite(greenledpin, 0);
}
