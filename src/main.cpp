#include <WiFi.h>
#include <WebServer.h>
#include <esp_task_wdt.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Arduino.h>
/**************************************/
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
/**************************************/
#define PIN_1 17
#define PIN_2 18
#define ALS 4

Preferences preferences;
QueueHandle_t taskQueue;
WebServer server(80);
DNSServer dnsServer;

const char* apSSID = "ESP32_Config";
const char* apPassword = "12345678";
IPAddress apIP(192,168,4,1);

void handleRoot() {
  String html = "<form action='/save' method='POST'><input type='text' name='ssid' placeholder='SSID'><br><input type='text' name='password' placeholder='Password'><br><input type='submit' value='Save'></form>";
  server.send(200, "text/html", html);
}

void handleSave() {
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
  int lightLevel = 0;
  char string[30] = "Current Tank Level: ";
  char lightLevelStr [8];
  lightLevel = analogRead(ALS);
  itoa( lightLevel, lightLevelStr, 10 );
  strcat(lightLevelStr, "\n");
  strcat(string, lightLevelStr);
  server.send(200, "text/html", string);
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

void handleFeed() {
  int taskTrigger = 1;
  xQueueSend(taskQueue, &taskTrigger, portMAX_DELAY);
  //erver.send(200, "text/html", "Feeder triggered\n");
  handleTank2();
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

void connectToWiFi() {
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");

  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void task0(void *parameter) {
  while (true) {
    server.handleClient();
    delay(10);
  }
}

void task1(void *parameter) {
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

void task2(void *parameter){
  while (true){
    digitalWrite(BUILTIN_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(250));
    digitalWrite(BUILTIN_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
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
