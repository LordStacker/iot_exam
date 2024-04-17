#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>

// Setup network and MQTT
const char* ssid = "IphoneNico";
const char* password =  "nico1234";
const char* mqttServer = "mqtt.flespi.io";
const int mqttPort = 1883;
const char* mqttUser = "CtheHXh2THq0Pf3JhObAMNlLn5bkx4klSAGiWdJY0R53tzL5vM6yZh2euJDgC01e";
const char* mqttPassword = "*";

WiFiClient espClient;
PubSubClient client(espClient);

// Setup sound sensor
const int sensorMinValue = 0;
const int sensorMaxValue = 1680;
const int dbMinValue = 30;
const int dbMaxValue = 130;

float temperature;
float humidity;
int db;

Adafruit_BME280 bme;

unsigned long delayTime;

void setup() {
  Serial.begin(9600);

  setupWiFiConnection();
  setupSoundSensor();

  Serial.println(F("BME280 test"));
  bool status;

  // Default settings
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  configTime(0, 0, "pool.ntp.org"); // Configure the NTP client

  Serial.println();
}

void loop() {
  client.loop();
  printValues();
  measureSound();
  delay(delayTime);
}

void setupWiFiConnection() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connected to the WiFi network");
  }

  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  client.publish("getTemperatureAndHumidity/", "Hello World!");
}

void setupSoundSensor() {
  pinMode(A0, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
}

void measureSound() {
  unsigned int sample;
  unsigned long startMillis = millis();
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // Collect data for 50 mS
  while (millis() - startMillis < 50) {
    sample = analogRead(A0); // Get reading from sound sensor
    if (sample < 1024) { // Discard spurious readings
      if (sample > signalMax) {
        signalMax = sample; // Save max level
      } else if (sample < signalMin) {
        signalMin = sample; // Save min level
      }
    }
  }

  // Calculate peak-to-peak amplitude
  float peakToPeak = signalMax - signalMin;

  // Map peak-to-peak amplitude to dB using calibration data
  db = map(peakToPeak, sensorMinValue, sensorMaxValue, dbMinValue, dbMaxValue);

  Serial.print("Loudness: ");
  Serial.print(db);
  Serial.println(" dB");

  // Output level indication based on sound level
  if (db <= 60) {
    Serial.println("Level: Quiet");
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
  } else if (db > 60 && db < 85) {
    Serial.println("Level: Moderate");
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
  } else if (db >= 85) {
    Serial.println("Level: High");
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
  }

  delay(200);
}

void printValues() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();

  Serial.println("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  // Get current time
  time_t now = time(nullptr);
  struct tm* timeinfo;
  timeinfo = localtime(&now);
  
  // Format: DD/MM/YYYY and HOUR:MINUTE
  char formattedDateTime[20];
  snprintf(formattedDateTime, 20, "%d/%d/%d and %d:%s%d", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, (timeinfo->tm_min < 10 ? "0" : ""), timeinfo->tm_min);
  String message = "{\"sensor_id\": \"Pending\", \"device_id\": \"Pending\", \"sound_level\": " + String(db) + ", \"temperature\": " + String(temperature) + ", \"humidity\": " + String(humidity) + ", \"date\": \"" + String(formattedDateTime) + "\"}";
  client.publish("getTemperatureAndHumidity/", message.c_str()); // Publish the message

  Serial.println("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();
}
