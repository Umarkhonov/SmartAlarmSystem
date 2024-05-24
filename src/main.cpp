#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Sensor pins
const int flameSensorPin = 34;
const int moistureSensorPin = 32;
const int gasSensorPin = 33;
const int tempSensorPin = 25;

// LCD initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Initialize LCD with I2C address 0x27

// WiFi and MQTT
const char* ssid = "iPhone";
const char* password = "00000000";
const char* mqttServer = "ec2-16-171-233-102.eu-north-1.compute.amazonaws.com";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

WiFiClient espClient;
PubSubClient client(espClient);

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup_wifi();
void reconnect();
void publishMessage(const char* topic, const char* payload);
void Task1code(void * pvParameters);
void Task2code(void * pvParameters);

// Timer and temperature variables
unsigned long previousMillis = 0;
const long interval = 10000;
const long tempInterval = 30000;
float temperature = 0;

void setup() {
  // Initialize the I2C communication for the LCD
  Wire.begin(21, 22);

  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();

  pinMode(flameSensorPin, INPUT);
  pinMode(moistureSensorPin, INPUT);
  pinMode(gasSensorPin, INPUT);
  pinMode(tempSensorPin, INPUT);
  pinMode(2, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  analogReadResolution(12); // Set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db);

  Serial.begin(115200);

  // Initialize LEDC
  ledcAttachPin(2, 0);
  ledcSetup(0, 5000, 8);

  // Create tasks
  xTaskCreatePinnedToCore(
    Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(
    Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);
}

void loop() {
  // Empty loop as tasks are now handling the operations
}

void Task1code(void * pvParameters) {
  setup_wifi();
  client.setServer(mqttServer, mqttPort);

  for (;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Allow other tasks to run
  }
}

void Task2code(void * pvParameters) {
  unsigned long currentMillis = millis();

  for (;;) {
    currentMillis = millis();

    int flameSensorValue = digitalRead(flameSensorPin);
    int moistureSensorValue = digitalRead(moistureSensorPin);
    int gasSensorValue = analogRead(gasSensorPin);

    // Update temperature reading every 10 seconds
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      int tempSensorValue = analogRead(tempSensorPin);
      float voltage = tempSensorValue * (3.3 / 4095.0);
      temperature = voltage * 100.0;
      temperature -= 10;

      Serial.print("Analog Read Value: ");
      Serial.println(tempSensorValue);
      Serial.print("Voltage: ");
      Serial.println(voltage);
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" C");

      static unsigned long lastTempPublish = 0;
      if (currentMillis - lastTempPublish >= tempInterval) {
        lastTempPublish = currentMillis;
        char tempStr[8];
        dtostrf(temperature, 1, 2, tempStr);
        publishRetainedMessage("home/temperature", tempStr);
      }
    }

    if (flameSensorValue == HIGH) {
      lcd.setCursor(0, 0);
      lcd.print("    WARNING!    ");
      lcd.setCursor(0, 1);
      lcd.print(" FIRE DETECTED! ");
      digitalWrite(19, HIGH);
      digitalWrite(18, LOW);
      ledcWriteTone(0, 523);
      publishMessage("home/fire", "FIRE DETECTED");
    } else if (moistureSensorValue == HIGH) {
      lcd.setCursor(0, 0);
      lcd.print("    WARNING!    ");
      lcd.setCursor(0, 1);
      lcd.print(" FLOOD DETECTED ");
      digitalWrite(19, HIGH);
      digitalWrite(18, LOW);
      ledcWriteTone(0, 784);
      publishMessage("home/flood", "FLOOD DETECTED");
    } else if (gasSensorValue > 1400) {
      lcd.setCursor(0, 0);
      lcd.print("    WARNING!    ");
      lcd.setCursor(0, 1);
      lcd.print(" GAS DETECTED!  ");
      digitalWrite(19, HIGH);
      digitalWrite(18, LOW);
      ledcWriteTone(0, 659);
      publishMessage("home/gas", "GAS DETECTED");
    } else {
      lcd.setCursor(0, 0);
      lcd.print(" Temp: ");
      lcd.print(temperature);
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      digitalWrite(18, HIGH);
      digitalWrite(19, LOW);
      ledcWriteTone(0, 0);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 0;
  const int maxRetries = 30;

  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishMessage(const char* topic, const char* payload) {
  if (!client.connected()) {
    reconnect();
  }
  client.publish(topic, payload);
}

void publishRetainedMessage(const char* topic, const char* payload) {
  if (!client.connected()) {
    reconnect();
  }
  client.publish(topic, payload,true);
}
