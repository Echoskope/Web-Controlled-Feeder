/**********************************************************/
//  This code was written mostly by Microsoft CoPilot. 
//
//   F9:18 <-- Don't worry about this, used for development
//
//  General Operation:
//    Upon power up, the ESP32 will boot into
//    Wifi AP mode for 30 seconds (blue on board LED 
//    will be steadily illuminated). During this time
//    you can connect to the broadcasted network
//    to configure the local wifi network's credentials.
//
//    If nothing connects after 30 seconds, the board
//    continues to boot (blue on board LED will start
//    to blink), and will join the local wifi network
//    with the stored credentials. After fully booted
//    you can go to the http://<IP Address>/ URL to
//    change the Wifi credentials. If you go to the
//    http://<IP Address>/feed URL it will cause the
//    latching relay to cycle. Local browser caching
//    can cause issues, so ensure it does a fresh
//    load, for example by using CURL.
//
//  This code was designed to run on:
//    Unexpected Maker FeatherS3 ESP32-S3 Board
//    Adafruit Latching Relay FeatherWing PID:2923
//  
//  The FeatherWing wiring:
//    Set wired to pin 17
//    Reset wired to 18
//
//  The relay COM and NO are wired to
//  the Quick Feed Automatic Feeder
//  using wires soldered to the manual
//  feed button on the PCB.
//
//  This code won't work if the feeder
//  controller has been power cycled,
//  as it requires the user to set
//  the serving size upon power up on
//  first manual feed trigger. After
//  the serving size is set, it works.
//
//  The future:
//    Add in sonar/ultrasonic sensor for
//    detecting height of food in tank.
//    (this is currently simulated via
//    the on board ambient light sensor)
//
//    Add a beam break sensor in the
//    discharge tube to sense when food
//    was successfully delivered.
//
//  Known Issues:
//    Because this uses HTTP GET, web
//    browser cache can cause it to fail.
//    It is better to use a tool like
//    CURL to trigger the cycle.
/**********************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_task_wdt.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <UMS3.h>

#define RelaySet 17 // For the Latching Relay Set
#define RelayReset 18 // For the Latching Relay Reset
#define ALS 4 // Built-in Ambient Light Sensor on the FeatherS3 board
#define RX_PIN 6 // Connect to the TX pin of the MB7092
#define TX_PIN 5 // Optional, for MB7092 sensor feedback if needed

//*********************************//
// These two #define are used to
// tell the sensor reading code
// which sensor to read!
//********************************//
//#define UseMB7092
#define UseALS

#define BeamBreakSensorActive

HardwareSerial mySerial(1); // Use hardware Serial1
Preferences preferences;
QueueHandle_t taskQueue;
WebServer server(80);
DNSServer dnsServer;
UMS3 ums3;

//**********************************************//
// the apSSID and apPassword is for when the ESP32
// starts up in Wifi AP mode, with IP address of
// apIP.
//**********************************************//
const char* apSSID = "ESP32_Config";
const char* apPassword = "12345678";
IPAddress apIP(192,168,4,1);

#ifdef BeamBreakSensorActive
struct BeamBreakSensor {
  const uint8_t PIN;
  bool beamBreakISRdisabled; 
};

volatile BeamBreakSensor beamBreakSensor1 = {11, true};

void IRAM_ATTR isr(){
  detachInterrupt(beamBreakSensor1.PIN); // Turn off the interrupt so it isn't trying to call the ISR when not needed
  beamBreakSensor1.beamBreakISRdisabled = true;
  //Serial.println("Beam Break Sensor Activated!");
}
#endif

void readTankLevel(bool callSource){//, BeamBreakSensor& beamBreakSensor1){
  char webResponse[200];
  char sensorData[30];
  char feedJamIndicator[45];

  if(callSource){ // If we're coming from the feeder handle
    strcpy(webResponse, "Feeder Activated!\nCurrent Tank Level: ");
    #ifdef BeamBreakSensorActive
      Serial.print("Beam Break Before: ");
      Serial.println(beamBreakSensor1.beamBreakISRdisabled);
      if(beamBreakSensor1.beamBreakISRdisabled == true){ //We need to enable the interrupt
        beamBreakSensor1.beamBreakISRdisabled = false; // Reset before we check further down after the delay to allow the feed to drop
        Serial.print("Beam Break False?: ");
        Serial.println(beamBreakSensor1.beamBreakISRdisabled);
        attachInterrupt(beamBreakSensor1.PIN, isr, FALLING); // Set the interrupt so that anytime the beam breaks the function is called
      }
      //Serial.println("callSource is 1"); vTaskDelay(pdMS_TO_TICKS(100));
      vTaskDelay(pdMS_TO_TICKS(5000)); //Wait 5 seconds for the grain to break the sensor
      Serial.print("Beam Break After: ");
      Serial.println(beamBreakSensor1.beamBreakISRdisabled);    
      if(beamBreakSensor1.beamBreakISRdisabled == true){ //The ISR was called, the interrupt remains disabled
        strcpy(feedJamIndicator, "Beam Break Sensor Activated!\n");
      } else {
        detachInterrupt(beamBreakSensor1.PIN); // If the ISR wasn't called, we need to still disable the interrupt
        beamBreakSensor1.beamBreakISRdisabled = true; // Reset so we go back through the ISR enable process next time
        strcpy(feedJamIndicator, "ERROR: Beam Break Sensor Not Activated!\n");
      }
    #endif
  } else {
    strcpy(webResponse, "Current Tank Level: "); // Otherwise we're coming from the tank handle, so just the sensor response.
    //Serial.println("callSource is 0."); vTaskDelay(pdMS_TO_TICKS(100));
  }

  #ifdef UseALS // Use the Ambient Light Sensor as our data source (left here for troubleshooting).
    Serial.println("Starting ALS Sensor Reading...");
    int lightLevel = 0;
    lightLevel = analogRead(ALS);
    Serial.println(lightLevel);
    itoa(lightLevel, sensorData, 10);
  #endif

  #ifdef UseMB7092
    //Serial.println("Clearing the serial buffer..."); vTaskDelay(pdMS_TO_TICKS(100));
    while(mySerial.available()>0){mySerial.read();} // Clear out all the old data first
    vTaskDelay(pdMS_TO_TICKS(250)); // Give the sensor some time to give us some more fresh values
    
    if(mySerial.available()>0){
      while(mySerial.read() != '\r'){vTaskDelay(pdMS_TO_TICKS(5));} //Now look for the end of the last data read so we get a fresh start
    } else {
      //Serial.println("Serial Data Not Available."); // No serial data was available :(
      strcpy(sensorData, "Serial Data Not Available.");
    }
    
    //Serial.println("Starting MB7092 Sensor Reading..."); vTaskDelay(pdMS_TO_TICKS(100));

    if (mySerial.available()) { // Check if data is available from the sensor
      String sensorDataObject = mySerial.readStringUntil('\r'); // Read until the carriage return (ASCII 13)
      
      if (sensorDataObject.startsWith("R")) { // Validate the format (starts with 'R')
        sensorDataObject = sensorDataObject.substring(1); // Extract the range value (after 'R')
        //Serial.print("Range in cm: ");
        //Serial.println(sensorDataObject); vTaskDelay(pdMS_TO_TICKS(100));
        strcpy(sensorData, sensorDataObject.c_str()); // Convert the String object into a char array
      } else {
        //Serial.println("Invalid data received."); // String didn't start with 'R'
        strcpy(sensorData, "Invalid data received.");
      }
    }

  #endif

  //Serial.println("Building Web Request data..."); vTaskDelay(pdMS_TO_TICKS(100));
  strcat(webResponse, sensorData); // Append the sensor reading to the string we built based on handler source.
  strcat(webResponse, "\n"); // Make the response look pretty.
  strcat(webResponse, feedJamIndicator);
  //Serial.println("Sending back HTML to browser..."); vTaskDelay(pdMS_TO_TICKS(100));
  server.send(200, "text/html", webResponse);
}

void handleRoot() {
  String html = "<form action='/save' method='POST'><input type='text' name='ssid' placeholder='SSID'><br><input type='text' name='password' placeholder='Password'><br><input type='submit' value='Save'></form>";
  server.send(200, "text/html", html);
}

void handleSave() { // If you open the web root page, you can change the wifi credentials 
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    server.send(200, "text/html", "Credentials saved. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
  } else {
    server.send(400, "text/html", "Missing SSID or Password");
  }
}

void handleTank(){
  readTankLevel(false);//, beamBreakSensor1);
  }

void handleFeed() { // the /feed GET URL occurred, we need to trigger he task to cycle the relay
  int taskTrigger = 1;
  xQueueSend(taskQueue, &taskTrigger, portMAX_DELAY);
  //vTaskDelay(pdMS_TO_TICKS(3000));
  readTankLevel(true);//, beamBreakSensor1); // This is to print back some information to the user
}

/*
void handlePost() {
  if (server.hasArg("value")) {
    int taskTrigger = server.arg("value").toInt();
    if (taskTrigger == 1) {
      xQueueSend(taskQueue, &taskTrigger, portMAX_DELAY);
      server.send(200, "application/json", "{\"status\": \"Task1 triggered\"}");
    } else {
      server.send(400, "application/json", "{\"status\": \"Invalid value\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\": \"No value provided\"}");
  }
}
*/

//**********************************************************************//
//
// The startAPMode() function is executed upon boot of the ESP32.
// The ESP32 will broadcast a wifi network for the user to connect
// to and configure the local network wifi credentials. If no user
// connects after 30 seconds it will continue to boot into the normal
// operation. 
//
//**********************************************************************//

void startAPMode() { 
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSSID, apPassword);
  dnsServer.start(53, "*", apIP);
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
}

void connectToWiFi() { // This function connects the ESP32 to the local wifi network
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");

  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  ums3.setPixelBrightness(0);
}

void taskWebServer(void *parameter) { // This task handles the web page stuff (runs on core 0)
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void taskRunManualFeed(void *parameter) { // This task cycles the latching relay to trigger the manual feed operation (runs on core 1)
  int taskTrigger;
  while (true) {
    if (xQueueReceive(taskQueue, &taskTrigger, portMAX_DELAY) == pdTRUE) {
      if (taskTrigger == 1) {
        // Turn on RelaySet for 20 ms
        digitalWrite(RelaySet, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(RelaySet, LOW);
        vTaskDelay(pdMS_TO_TICKS(250));

        // Turn on RelayReset for 20 ms
        digitalWrite(RelayReset, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(RelayReset, LOW);
        vTaskDelay(pdMS_TO_TICKS(250));
      }
    }
  }
}

void wifiMonitorTask(void *parameter){ // This task blinks the on board blue LED as a heartbeat (runs on core 0)
  while(true) {
    if (WiFi.status() != WL_CONNECTED) {
      ums3.setPixelBrightness(255);
      connectToWiFi();
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void taskHeartbeatLED(void *parameter){ // This task blinks the on board blue LED as a heartbeat (runs on core 0)
  while(true) {
  digitalWrite(BUILTIN_LED, HIGH);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(BUILTIN_LED, LOW);
  vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup() { // Standard setup function for Arduino framework
  Serial.begin(115200);

  ums3.begin();
  ums3.setPixelBrightness(255);
  ums3.setPixelPower(true);
  ums3.setPixelColor(UMS3::colorWheel(240));

  pinMode(LED_BUILTIN, OUTPUT); //Set GPIO as output for built-in Blue LED
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in Blue LED to indicate board is booting up
  pinMode(39, OUTPUT); // Set the 3.3V LDO power control pin to output
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(39, HIGH); // Turn on the 2nd 3.3V LDO power supply for the ultrasonic sensor
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN, 1); // Initialize Serial1 for the MB7092

  pinMode(11, INPUT_PULLUP); //Beam break sensor requires a pull up! 

  // Initialize Relay pins
  pinMode(RelaySet, OUTPUT);
  pinMode(RelayReset, OUTPUT);

  preferences.begin("wifi-config", false);

  // Start AP mode for 30 seconds, if no client connects then proceed with booting.
  startAPMode();

  unsigned long startTime = millis();
  while ((millis() - startTime) < 30000) { 
    dnsServer.processNextRequest();
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));

    if (WiFi.softAPgetStationNum() > 0) {
      // Client connected, wait for configuration
      while (WiFi.softAPgetStationNum() > 0) {
        dnsServer.processNextRequest();
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      return;
    }
  }

  digitalWrite(LED_BUILTIN, LOW); //Turn off built-in Blue LED now that the AP Config time period has elapsed. 
  
  // No client connected, continue with known credentials
  WiFi.softAPdisconnect(true);
  server.stop();
  connectToWiFi();

 

  // Initialize the server
  server.on("/", handleRoot);
  server.on("/feed", handleFeed);
  //server.on("/feed", HTTP_POST, handlePost);
  server.on("/tank", handleTank);
  server.begin();
  Serial.println("HTTP server started");

  // Create the task queue
  taskQueue = xQueueCreate(10, sizeof(int));
  if (taskQueue == NULL) {
    Serial.println("Failed to create queue");
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("Creating Tasks...");
  vTaskDelay(pdMS_TO_TICKS(10));
  // Create tasks on core 0
  xTaskCreatePinnedToCore(taskWebServer, "taskWebServer", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskHeartbeatLED, "taskHeartbeatLED", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(wifiMonitorTask, "wifimonitorTask", 4096, NULL, 2, NULL, 0);

  // Create task on core 1
  xTaskCreatePinnedToCore(taskRunManualFeed, "taskRunManualFeed", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Keep the main loop empty as tasks are handled in separate cores
}
