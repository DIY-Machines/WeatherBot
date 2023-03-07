#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include "Images-diym.h"

JSONVar myObject;

#include <GxEPD2_BW.h> 
#include <GxEPD2_3C.h> 
#include "GxEPD2_display_selection_new_style.h"
#include <Fonts/FreeSans12pt7b.h>
#include <ESP32Servo.h>  // Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// -----------------------------------------------------------------------------------------------------------------------------------------------------/////
///                                                                                                                                                     /////
///                                              VARIABLES AND CONSTANTS FOR YOU TO UPDATE BELOW                                                        /////
///                                                                                                                                                     /////
///                                                                                                                                                     /////

const char* ssid = "YOUR-SSID";         //  YOUR WIFI SSID / NAME
const char* password = "YOUR-WIFI-PASSWORD";   // YOUR WIFI PASSWORD

const int servoSpeed = 23;                // Controls the speed of rotation of the servos (0 to 90 with 90 being the fastest) Default = 23

String serverPath = "http://api.openweathermap.org/data/3.0/onecall?lat={latitude}&lon={longitude}&units=metric&exclude=minutely&appid={Your-API-Key}";  // Your API call made up as follows: http://api.openweathermap.org/data/2.5/onecall?lat={latitude}}&lon=-{longitude}}&units=metric&exclude=minutely&appid={Your-API-Key}

unsigned long timerDelay = 3600000;  //20 seconds = 20000.   60 second = 60000.  5 minutes. =  300000.  Hourly = 3600000. Daily = 86400000. Check the API call limits per hour/minute to avoid getting blocked/banned

// Temperature disc:
const int temperatureBelowWhichToShowColdScene = 8;     // in degrees celcius
const int temperatureAboveWhichToShowHotWeather = 20;   // in degrees celcius

// Wind Speed disc:
const int windSpeedBelowWhichToShowStillScene = 4;         // in mph
const int windSpeedAboveWhichToShowVeryWindyScene = 30;   // in mph

// Cloud Cover disc:
const int cloudCoverBelowWhichToShowClearSky = 4;        // in %
const int cloudCoverAboveWhichToShowVeryCloudySky = 70;  // in %

// Precipitation disc:
const int rainfallBelowWhichToShowDrySky = 1;        // in mm of rainfall
const int rainfallAboveWhichToShowVeryWetSky = 5;  // in mm of rainfall


///                                                                                                                                                      /////
///                                                                                                                                                      /////
///                                        END OF VARIABLE AND CONSTANTS THAT ARE EXPECTED TO BE EDITED                                                  /////
///                                                                                                                                                      /////
/// ---------------------------------------------------------------------------------------------------------------------------------------------------- /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// -----------------------------------------------------------------------------------------------------------------------------------------------------/////
///                                                                                                                                                      /////
///                                       VARIABLES AND CONSTANTS TO BE EDITED ONLY IF YOU HAVE PROBLEMS                                                 /////
///                                                                                                                                                      /////
///                                                                                                                                                      /////

const int currentNotchClearanceDelay = 250; //this delay ensures we leave the notch before we check the limit switch too soon and mistake the notch we were already in as the next notch! This can be adjusted only if you have issues with the discs not turning enough before the limit switched value is read. Default = 250

///                                                                                                                                                      /////
///                                                                                                                                                      /////
///                             END OF VARIABLE AND CONSTANTS TO BE EDITED ONLY IF YOU HAVE PROBLEMS                                                     /////
///                                                                                                                                                      /////
/// ---------------------------------------------------------------------------------------------------------------------------------------------------- /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




long lastTime = (timerDelay - (timerDelay*2)) + 1000;  //causes the API to request the data one second after start up and then fall back to the reguar interval as set in the timerDelay


String jsonBuffer;

// --- Servo setup --

  Servo discServo;  // create servo object to control a servo

// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33 
  const int servo1Pin = 25;      // GPIO pin used to connect the servo control (digital out)
  const int servo2Pin = 19;      // GPIO pin used to connect the servo control (digital out)
  const int servo3Pin = 21;      // GPIO pin used to connect the servo control (digital out)
  const int servo4Pin = 22;      // GPIO pin used to connect the servo control (digital out)

// Published values for SG90 servos; adjust if needed
  const int minUs = 1000;
  const int maxUs = 2000;

const int  discOneSwitch = 26;    // the pin the front most limit switch is connected to for detecting disc ones position
const int  discTwoSwitch = 27;    // the pin the second most limit switch from the front is connected to for detecting disc twos position
const int  discThreeSwitch = 32;    // the pin the third limit switch from the front is connected to for detecting disc threes position
const int  discFourSwitch = 33;    // the pin the fourth limit switch from the front is connected to for detecting disc fours position

//somewhere to store the correct scene disc info for the scene disc we are currently working with
int currentDiscSwitch;
int currentDiscRotationSpeed;
int currentDiscServoPin;
int currentDiscNumber;
int *currentDiscPosition;
int *currentDesiredDiscPosition;

//somewhere to store each discs current position. (The values they have been initialised with are wildy incorrect and are used to spot errors in the code if they are printed later in the programme).
int discOneCurrentPosition = 101;
int discTwoCurrentPosition = 102;
int discThreeCurrentPosition = 103;
int discFourCurrentPosition = 104;

int discOneDesiredPosition = 101;
int discTwoDesiredPosition = 102;
int discThreeDesiredPosition = 103;
int discFourDesiredPosition = 104;

bool weatherUpdateWaitingToBeShownOnDiscs = false;
bool sceneDiscsHomed = false;

String weatherTodaysDescription;
  double weatherTodaysTempFeel;
  double weatherTodaysPrecipitationProbability;
  double weatherTodaysWindSpeed;
  double weatherTodaysCloudCover;
  double weatherTodaysPrecipitationQty;

String weatherTomorrowsDescription;
  double weatherTomorrowsTempFeel;
  double weatherTomorrowsPrecipitationProbability;
  double weatherTomorrowsWindSpeed;
  double weatherTomorrowsCloudCover;
  double weatherTomorrowsPrecipitationQty;

//timers for servos
  unsigned long startMillis;
  unsigned long currentMillis;
  unsigned long elapsedMillis;

// Arrays for positioning each disc during startup
int discPositionTimingArray[4];

TaskHandle_t Task1;

void setup() {
  Serial.begin(115200);
  screenOutputDIYMWelcome();

  Serial.print("Current value for 'lastTime' is: ");
Serial.println(lastTime);


//create a task that will be executed in the Task1code() function, with priority 0 and executed on core 0. /* Task function. *//* name of task. *//* Stack size of task *//* parameter of the task *//* priority of the task *//* Task handle to keep track of created task */
xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,0,&Task1,0);
 

  // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    discServo.setPeriodHertz(50);// Standard 50hz servo


  // Contact switches setup

    pinMode(discOneSwitch, INPUT_PULLDOWN);
    pinMode(discTwoSwitch, INPUT_PULLDOWN);
    pinMode(discThreeSwitch, INPUT_PULLDOWN);
    pinMode(discFourSwitch, INPUT_PULLDOWN);
  
  //maintaining wifi conneciton
    WiFi.begin(ssid, password);
    Serial.println("Connecting");

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
  // Send the HTTP GET request and then process the JSON data
  if ((millis() - lastTime) > timerDelay) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {


      jsonBuffer = httpGETRequest(serverPath.c_str());
      //Serial.println(jsonBuffer);
      JSONVar myObject = JSON.parse(jsonBuffer);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }

      // Storing todays weather
      weatherTodaysDescription = myObject["daily"][0]["weather"][0]["description"];
      weatherTodaysTempFeel = myObject["daily"][0]["feels_like"]["day"];
      weatherTodaysPrecipitationProbability = myObject["daily"][0]["pop"];
      weatherTodaysWindSpeed = myObject["daily"][0]["wind_speed"];
      weatherTodaysCloudCover = myObject["daily"][0]["clouds"];
      weatherTodaysPrecipitationQty  = myObject["daily"][0]["rain"];
      Serial.print("Amount of rain today: ");
      Serial.println(weatherTodaysPrecipitationQty);
      Serial.print("Amount of clouds today: ");
      Serial.println(weatherTodaysCloudCover);


      // Storing tomorrows weather
      weatherTomorrowsDescription = myObject["daily"][1]["weather"][0]["description"];
      weatherTomorrowsTempFeel = myObject["daily"][1]["feels_like"]["day"];
      weatherTomorrowsPrecipitationProbability = myObject["daily"][1]["pop"];
      weatherTomorrowsWindSpeed = myObject["daily"][1]["wind_speed"];
      weatherTomorrowsCloudCover = myObject["daily"][1]["clouds"];
      weatherTomorrowsPrecipitationQty  = myObject["daily"][1]["rain"];

      // send weather data to the screen
      screenOutput();

      //Let second core know that we have a weather updte for it to show using the mechanical discs
      weatherUpdateWaitingToBeShownOnDiscs = true;

    } else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

//Second core tasks - controlling scene discs
void Task1code( void * pvParameters ) {
  for(;;) {
  //Serial.print("This function task1 isrunning on core ");
  //Serial.println(xPortGetCoreID());

  if (sceneDiscsHomed == false){
    Serial.println("Trying to home all four scene discs...");
    homeAllSceneDiscs();
    Serial.println("...homing of scene discs completed.");
  }

  Serial.print("The value of 'weatherUpdateWaitingToBeShownOnDiscs' is: "); //Check for an updated weather report which we have not diplayed on the scene discs yet
  Serial.print(weatherUpdateWaitingToBeShownOnDiscs);
  Serial.println(". This means we are still waiting for a weather forecast update via WiFi......");
  delay(5000);

  if (weatherUpdateWaitingToBeShownOnDiscs == true) {
    Serial.println("Weather update received. Converting to scene discs locations...");
    convertWeatherToSceneDiscsSceneLocations(); // look at the weather forecast from OpenWeatherMaps and figure out which segmnet of each discs we need to display
    Serial.println("... scene disc positions decided.");
    Serial.println("Moving discs to show weather forecast received from OpenWeatherMaps....");
    showWeatherOnSceneDiscs();
    weatherUpdateWaitingToBeShownOnDiscs = false; //set the value to false as we the scene discs currently represent the same weather that the e-ink display is reporting.
    Serial.println("Weather update for the scene discs is complete. Waiting for the next update to display.");
  }
  }
}

// request data from web server
String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void screenOutput()
  {
  //display.init();
  display.init(115200, true, 2, false);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setRotation(1);
  display.setTextSize(1);
  display.firstPage();
  display.fillScreen(GxEPD_WHITE);

  // --------- TODAYS WEATHER --------------
  //first line of todays weather
  display.setCursor(5, 20);
  display.print("Dress for ");
  display.println(weatherTodaysDescription);
  
  display.drawBitmap(0, 27, epd_bitmap_Temp_32, 32, 32, GxEPD_BLACK);
  display.setCursor(33, 50);
  display.print(weatherTodaysTempFeel,1);
  display.println("'C");

  //second line of todays weather
  display.drawBitmap(100, 27, epd_bitmap_Precip_32, 32, 32, GxEPD_BLACK);
  display.setCursor(130, 50);
  display.print((weatherTodaysPrecipitationProbability)*100,0);
  display.println("%");

  display.drawBitmap(190, 27, epd_bitmap_Wind_32, 32, 32, GxEPD_BLACK);
  display.setCursor(222, 50);
  display.print(weatherTodaysWindSpeed * 2.2369,0);
  display.println("mph");


  // --------- TOMORROWS WEATHER --------------
  //first line of tomorrows weather
  display.setCursor(5, 91);
  display.print("It'll be ");
  display.println(weatherTomorrowsDescription);

  //second line of tomorrows weather

  display.drawBitmap(0, 97, epd_bitmap_Temp_32, 32, 32, GxEPD_BLACK);
  display.setCursor(33, 120);
  display.print(weatherTomorrowsTempFeel,1);
  display.println("'C");
  
  display.drawBitmap(100, 97, epd_bitmap_Precip_32, 32, 32, GxEPD_BLACK);
  display.setCursor(130, 120);
  display.print((weatherTomorrowsPrecipitationProbability)*100,0);
  display.println("%");

  display.drawBitmap(190, 97, epd_bitmap_Wind_32, 32, 32, GxEPD_BLACK);
  display.setCursor(222, 120);
  display.print(weatherTomorrowsWindSpeed * 2.2369,0);
  display.println("mph");

  //update the display
  display.display();

  }

void screenOutputDIYMWelcome()
  {
    //display.init();
    display.init(115200, true, 2, false);
    display.setRotation(1);
    display.firstPage();
    display.drawBitmap(0, 0, epd_bitmap_diym_welcome, 296, 128, GxEPD_BLACK);
    display.display();
    delay(3000);
  }
void setDiscVariables(int servoToMove){
  if (servoToMove == 1){
    currentDiscSwitch = discOneSwitch;
    currentDiscRotationSpeed = 90 + servoSpeed;
    currentDiscServoPin = servo1Pin;
    currentDiscNumber = 1;
    currentDiscPosition = &discOneCurrentPosition;
    currentDesiredDiscPosition = &discOneDesiredPosition;
  } 
  else if (servoToMove == 2){
      currentDiscSwitch = discTwoSwitch;
      currentDiscRotationSpeed = 90 - servoSpeed;
      currentDiscServoPin = servo2Pin;
      currentDiscNumber = 2;
      currentDiscPosition = &discTwoCurrentPosition;
      currentDesiredDiscPosition = &discTwoDesiredPosition;
  }
  else if (servoToMove == 3){
    currentDiscSwitch = discThreeSwitch;
    currentDiscRotationSpeed = 90 + servoSpeed;
    currentDiscServoPin = servo3Pin;
    currentDiscNumber = 3;
    currentDiscPosition = &discThreeCurrentPosition;
    currentDesiredDiscPosition = &discThreeDesiredPosition;
  } 
  else if (servoToMove == 4){
      currentDiscSwitch = discFourSwitch;
      currentDiscRotationSpeed = 90 - servoSpeed;
      currentDiscServoPin = servo4Pin;
      currentDiscNumber = 4;
      currentDiscPosition = &discFourCurrentPosition;
      currentDesiredDiscPosition = &discFourDesiredPosition;
  }  
  }

void homeAllSceneDiscs(){
  Serial.println("<Function 'homeAllSceneDiscs'> Calculating positioning of scene discs and then setting them to the correct weather.....");
  for (byte d = 1; d<5; d = d+1){
    //select each disc in turn
      if (d == 1){
        setDiscVariables(1);
      }
      else if (d == 2){
        setDiscVariables(2);
      }
      else if (d == 3){
        setDiscVariables(3);
      }
      else if (d == 4){
        setDiscVariables(4);
      }
    Serial.print("Counting the time to travel between the notches on disc");
    Serial.print(currentDiscNumber);
    //Find any notch to start our timings from
    discServo.attach(currentDiscServoPin, minUs, maxUs);   // attaches the servo to the servo object
    discServo.write(currentDiscRotationSpeed);                  // start the servo moving
    while (digitalRead(currentDiscSwitch) == 1) {
      delay(1);
    }
    discServo.write(90); //stops the servo
    delay(500);

    //find the next five notches whilst timing the distance between them
      for (byte n = 0; n<5; n = n+1){
      startMillis = millis();
      discServo.write(currentDiscRotationSpeed);                  // start the servo moving
      delay(currentNotchClearanceDelay); //this delay ensures we leave the notch before we check the limit switch too soon and mistake the notch we were already in as the next notch!
      while (digitalRead(currentDiscSwitch) == 1) {
        delay(1);
      }
      discServo.write(90); //stops the servo
      currentMillis = millis();
      delay(500);    
      discPositionTimingArray[n] = (currentMillis - startMillis);
    }
    
  printDiscTimingArray();
  discServo.detach();

  //find the 'double notch' marker by looking for the smallest distance between two notches
    int smallestTimingInArray = 99999999;
    int smallestTimingInPosition = 10;
    Serial.print("Checking value in position:");
    for(int i = 0; i<5; i = i+1)
    {
      Serial.print(" ");
      Serial.print(i);
      Serial.print(",");
      if(discPositionTimingArray[i] < smallestTimingInArray)
      {
          smallestTimingInArray = discPositionTimingArray[i];
          smallestTimingInPosition = i;        
      }
    }
    Serial.println("....");
    Serial.print("Smallest value found was: ");
    Serial.print(smallestTimingInArray);
    Serial.print(" which is found in the following position in the timings array: ");
    Serial.println(smallestTimingInPosition);
    if (smallestTimingInPosition == 4){smallestTimingInPosition = 0;}

    *currentDiscPosition = smallestTimingInPosition;
    Serial.print("The position of disc ");
    Serial.print(currentDiscNumber);
    Serial.print(" is currently scene ");
    Serial.print(*currentDiscPosition);
    Serial.println(". (If scene is reported as 0 then we are currently in the sceneless 'timing' notch.)");


  Serial.println("Moving to home position....");
  //now we know where the two close notches are on the disc we can turn the disc so that the first/home scene is viewable on top
  if (smallestTimingInPosition == 0){
    moveSpecifiedServo(d,1);
  }
  else if (smallestTimingInPosition == 1){
    moveSpecifiedServo(d,2);
  }
  else if (smallestTimingInPosition == 2){
    moveSpecifiedServo(d,3);
  }
  else if (smallestTimingInPosition == 3){
    //Do nothing, this disc has already found itself in the homed position - woop woop!
  }

  *currentDiscPosition = 1;
  Serial.print("The position of disc ");
  Serial.print(currentDiscNumber);
  Serial.print(" is now currently scene ");
  Serial.println(*currentDiscPosition);
  Serial.println("");
  }
  sceneDiscsHomed = true;
}

void convertWeatherToSceneDiscsSceneLocations(){
  if (weatherTodaysTempFeel <= temperatureBelowWhichToShowColdScene){
    discOneDesiredPosition = 3;
    }
  else if (weatherTodaysTempFeel >= temperatureAboveWhichToShowHotWeather){
    discOneDesiredPosition = 2;
    }
  else {
    discOneDesiredPosition = 1;
    }

  if (weatherTodaysWindSpeed <= windSpeedBelowWhichToShowStillScene){
    discTwoDesiredPosition = 2;
    }
  else if (weatherTodaysWindSpeed >= windSpeedAboveWhichToShowVeryWindyScene){
    discTwoDesiredPosition = 1;
    }
  else {
    discTwoDesiredPosition = 3;
    }

  if (weatherTodaysCloudCover <= cloudCoverBelowWhichToShowClearSky){
    discThreeDesiredPosition = 3;
    }
  else if (weatherTodaysCloudCover >= cloudCoverAboveWhichToShowVeryCloudySky){
    discThreeDesiredPosition = 1;
    }
  else {
    discThreeDesiredPosition = 2;
    }

  if (weatherTodaysPrecipitationQty <= rainfallBelowWhichToShowDrySky){
    discFourDesiredPosition = 3;
    }
  else if (weatherTodaysPrecipitationQty >= rainfallAboveWhichToShowVeryWetSky){
    discFourDesiredPosition = 1;
    }
  else {
    discFourDesiredPosition = 2;
    }
}

void showWeatherOnSceneDiscs(){

  for (byte d = 1; d<5; d = d+1){
    //select each disc in turn
      if (d == 1){
        setDiscVariables(1);        
      }
      else if (d == 2){
        setDiscVariables(2);
      }
      else if (d == 3){
        setDiscVariables(3);
      }
      else if (d == 4){
        setDiscVariables(4);
      }

      if (*currentDiscPosition == 1 && *currentDesiredDiscPosition == 1) {
        //do nothing
      }
      else if (*currentDiscPosition == 1 && *currentDesiredDiscPosition == 2){
        moveSpecifiedServo(d,1);
      }
      else if (*currentDiscPosition == 1 && *currentDesiredDiscPosition == 3){
        moveSpecifiedServo(d,2);
      }
      else if (*currentDiscPosition == 2 && *currentDesiredDiscPosition == 1) {
        moveSpecifiedServo(d,3);
      }
      else if (*currentDiscPosition == 2 && *currentDesiredDiscPosition == 2){
        //do nothing
      }
      else if (*currentDiscPosition == 2 && *currentDesiredDiscPosition == 3){
        moveSpecifiedServo(d,1);
      }
      else if (*currentDiscPosition == 3 && *currentDesiredDiscPosition == 1) {
        moveSpecifiedServo(d,2);
      }
      else if (*currentDiscPosition == 3 && *currentDesiredDiscPosition == 2){
        moveSpecifiedServo(d,3);
      }
      else if (*currentDiscPosition == 3 && *currentDesiredDiscPosition == 3){
        //do nothing
      }
      *currentDiscPosition = *currentDesiredDiscPosition;
  }

}


void printDiscTimingArray(){
  Serial.print("<Function 'printDiscTimingArray'>  Printing array of timings found for disc ");
  Serial.println(currentDiscNumber);
  for (byte i = 0; i<5; i = i+1) {
    Serial.println(discPositionTimingArray[i]);
  }
}

void moveSpecifiedServo(int servoBeingMoved, int positionsToMove){
  Serial.println("");
  setDiscVariables(servoBeingMoved);
  Serial.print("Function <moveSpecifiedServo>:  Moving specified servo for disc ");
  Serial.println(currentDiscNumber);
  Serial.print("Moving disc and then checking disc switch. We need to do this ");
  Serial.print(positionsToMove);
  Serial.println(" times.");
  for (byte r = positionsToMove; r != 0; r = r-1){
    discServo.attach(currentDiscServoPin, minUs, maxUs);   // attaches the servo to the servo object
    discServo.write(currentDiscRotationSpeed);                  // start the servo moving
    delay(150);
    while (digitalRead(currentDiscSwitch) == 1) {
    delay(1);
    }
    discServo.write(90); //stops the servo
    discServo.detach();
    Serial.print("We still need to move ");
    Serial.print(r-1);
    Serial.println(" time(s) more around to get to where we want to be.");
    delay(1000);
  }
  Serial.println("We're there.");
  Serial.println("");
}
