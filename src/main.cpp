/**********************************************************/
//  This code was written mostly by Microsoft CoPilot. 
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

HardwareSerial mySerial(1); // Use hardware Serial1

#define PIN_1 17 // For the Latching Relay Set
#define PIN_2 18 // For the Latching Relay Reset
#define ALS 4 // Built-in Ambient Light Sensor on the FeatherS3 board

// Define FeatherS3 UART pins
#define RX_PIN 6 // Connect to the TX pin of the MB7092
//#define TX_PIN 5 // Optional, for sensor feedback if needed
//#define analogInputPin 17 // 4

Preferences preferences;
QueueHandle_t taskQueue;
WebServer server(80);
DNSServer dnsServer;

//**********************************************//
// the apSSID and apPassword is for when the ESP32
// starts up in Wifi AP mode, with IP address of
// apIP.
//**********************************************//
const char* apSSID = "ESP32_Config";
const char* apPassword = "12345678";
IPAddress apIP(192,168,4,1);

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
    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "text/html", "Missing SSID or Password");
  }
}

void handleTank(){
  /*
  int lightLevel = 0;
  char string[30] = "Current Tank Level: ";
  char lightLevelStr [8];
  lightLevel = analogRead(ALS);
  itoa( lightLevel, lightLevelStr, 10 );
  strcat(lightLevelStr, "\n");
  strcat(string, lightLevelStr);
  server.send(200, "text/html", string);
  */
 if (mySerial.available()) { // Check if data is available from the sensor
  String sensorData = mySerial.readStringUntil('\r'); // Read until the carriage return (ASCII 13)
  
  if (sensorData.startsWith("R")) { // Validate the format (starts with 'R')
    //String range = sensorData.substring(1); // Extract the range value (after 'R')
    Serial.print("Range in cm: ");
    //Serial.println(range);
    Serial.println(sensorData);
  } else {
    Serial.println("Invalid data received.");
  }
} else {
  Serial.println("Serial Data Not Available.");
}
while(mySerial.available()>0){mySerial.read();}
}

void handleTank2(){
  int lightLevel = 0;
  char string[100] = "Feeder Activated.\nCurrent Tank Level: ";
  char lightLevelStr [8];
  lightLevel = analogRead(ALS);
  itoa( lightLevel, lightLevelStr, 10 );
  strcat(lightLevelStr, "\n");
  strcat(string, lightLevelStr);
  server.send(200, "text/html", string);
}

void handleFeed() { // the /feed GET URL occurred, we need to trigger he task to cycle the relay
  int taskTrigger = 1;
  xQueueSend(taskQueue, &taskTrigger, portMAX_DELAY);
  //erver.send(200, "text/html", "Feeder triggered\n");
  handleTank2(); // This is to print back some information to the user
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
  //server.on("/trigger", handleTrigger);
  //server.on("/trigger", HTTP_POST, handlePost);
  server.begin();
}

void connectToWiFi() { // This function connects the ESP32 to the local wifi network
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");

  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void task0(void *parameter) { // This task handles the web page stuff (runs on core 0)
  while (true) {
    server.handleClient();
    delay(10);
  }
}

void task1(void *parameter) { // This task cycles the latching relay to trigger the manual feed operation (runs on core 1)
  int taskTrigger;
  while (true) {
    if (xQueueReceive(taskQueue, &taskTrigger, portMAX_DELAY) == pdTRUE) {
      if (taskTrigger == 1) {
        // Turn on PIN_1 for 20 ms
        digitalWrite(PIN_1, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_1, LOW);
        vTaskDelay(pdMS_TO_TICKS(250));

        // Turn on PIN_2 for 20 ms
        digitalWrite(PIN_2, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_2, LOW);
        vTaskDelay(pdMS_TO_TICKS(250));
      }
    }
  }
}

void task2(void *parameter){ // This task blinks the on board blue LED as a heartbeat (runs on core 0)
  while (true){
    digitalWrite(BUILTIN_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(250));
    digitalWrite(BUILTIN_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void setup() { // Standard setup function for Arduino framework
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(39, OUTPUT);
  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(39, HIGH);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN, 1); // Initialize Serial1 for the MB7092
  //delay(10000);
  Serial.println("FeatherS3 reading MB7092 range sensor data...");
  preferences.begin("wifi-config", false);

  // Start AP mode for 30 seconds
  startAPMode();

  unsigned long startTime = millis();
  while ((millis() - startTime) < 30000) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(10);

    if (WiFi.softAPgetStationNum() > 0) {
      // Client connected, wait for configuration
      while (WiFi.softAPgetStationNum() > 0) {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
      }
      return;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  // No client connected, continue with known credentials
  WiFi.softAPdisconnect(true);
  server.stop();
  connectToWiFi();

  // Initialize pins
  pinMode(PIN_1, OUTPUT);
  pinMode(PIN_2, OUTPUT);

  // Initialize the server
  server.on("/", handleRoot);
  server.on("/feed", handleFeed);
  //server.on("/trigger", HTTP_POST, handlePost);
  server.on("/tank", handleTank);
  server.begin();
  Serial.println("HTTP server started");

  // Create the task queue
  taskQueue = xQueueCreate(10, sizeof(int));
  if (taskQueue == NULL) {
    Serial.println("Failed to create queue");
    return;
  }

  // Create task0 on core 0
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 2, NULL, 0);

  // Create task1 on core 1
  xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1);

  xTaskCreatePinnedToCore(task2, "Task2", 4096, NULL, 1, NULL, 0);
}



void loop() {
  // Keep the main loop empty as tasks are handled in separate cores
}
