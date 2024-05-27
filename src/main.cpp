#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define GAS_MIN_VAL 1150
#define GAS_MAX_VAL 3630
#define TEMP_INTERVAL 60000 // 1 minute in milliseconds
#define GAS_INTERVAL 60000  // 1 minute in milliseconds
#define SENSOR_INTERVAL 10000 // 10 seconds in milliseconds
#define ALERT_GAS_THRESHOLD 150
#define BUZZER_DURATION 5000 // 5 seconds in milliseconds

// Sensor pins
const int flameSensorPin = 35;
const int moistureSensorPin = 32;
const int gasSensorPin = 33;
const int tempSensorPin = 34;
const int buzzerPin = 2; // Buzzer pin

// LCD initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WiFi and MQTT
const char* ssid = "IoTPrivate";
const char* password = "iotprivate303";
const char *mqttServer = "ec2-16-171-233-102.eu-north-1.compute.amazonaws.com";
const int mqttPort = 1883;
const char *mqttUser = "";
const char *mqttPassword = "";

WiFiClient espClient;
PubSubClient client(espClient);

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup_wifi();
void reconnect();
void publishMessage(const char *topic, const char *payload);
void publishRetainedMessage(const char *topic, const char *payload);
void Task1code(void *pvParameters);
void Task2code(void *pvParameters);
void sendTemperature();
void sendGasPPM();
int gas_ppm(void);
void callback(char* topic, byte* message, unsigned int length);

// Timer and temperature variables
unsigned long previousMillis = 0;
unsigned long lastTempPublish = 0;
unsigned long lastGasPublish = 0;
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;
float temperature = 0;

// Alert state variables
bool fireAlert = false;
bool floodAlert = false;
bool gasAlert = false;

void setup()
{
  Wire.begin(21, 22);

  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();

  pinMode(flameSensorPin, INPUT);
  pinMode(moistureSensorPin, INPUT);
  pinMode(gasSensorPin, INPUT);
  pinMode(tempSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.begin(115200);

  // Initialize LEDC
  ledcAttachPin(buzzerPin, 0);
  ledcSetup(0, 5000, 8);

  // Create tasks
  xTaskCreatePinnedToCore(
      Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(
      Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);
}

void loop()
{
  // Empty loop as tasks are now handling the operations
}

void Task1code(void *pvParameters)
{
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  for (;;)
  {
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Allow other tasks to run
  }
}

void Task2code(void *pvParameters)
{
  unsigned long currentMillis = millis();

  for (;;)
  {
    currentMillis = millis();

    int flameSensorValue = digitalRead(flameSensorPin);
    int moistureSensorValue = digitalRead(moistureSensorPin);
    int gasSensorValue = analogRead(gasSensorPin);

    // Update temperature reading every SENSOR_INTERVAL milliseconds
    if (currentMillis - previousMillis >= SENSOR_INTERVAL)
    {
      previousMillis = currentMillis;
      int tempSensorValue = analogRead(tempSensorPin);
      // Convert the ADC value to voltage
      float voltage = tempSensorValue * (3.3 / 4095.0);

      // Convert voltage to temperature in Celsius for TMP36
      temperature = voltage * 100.0;
      
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" C");

      if (currentMillis - lastTempPublish >= TEMP_INTERVAL)
      {
        lastTempPublish = currentMillis;
        sendTemperature();
      }
    }

    // Send gas sensor value every GAS_INTERVAL milliseconds
    if (currentMillis - lastGasPublish >= GAS_INTERVAL)
    {
      lastGasPublish = currentMillis;
      sendGasPPM();
    }

    // Handle fire alert
    if (flameSensorValue == HIGH)
    {
      fireAlert = true;
      lcd.setCursor(0, 0);
      lcd.print("    WARNING!    ");
      lcd.setCursor(0, 1);
      lcd.print(" FIRE DETECTED! ");
      digitalWrite(19, HIGH);
      digitalWrite(18, LOW);
      ledcWriteTone(0, 523);
      publishMessage("home/fire", "FIRE DETECTED");
    }
    else if (fireAlert)
    {
      fireAlert = false;
      lcd.clear();
      publishMessage("home/fire", "FIRE CLEARED");
      sendTemperature();
    }

    // Handle flood alert
    if (moistureSensorValue == HIGH)
    {
      floodAlert = true;
      lcd.setCursor(0, 0);
      lcd.print("    WARNING!    ");
      lcd.setCursor(0, 1);
      lcd.print(" FLOOD DETECTED ");
      digitalWrite(19, HIGH);
      digitalWrite(18, LOW);
      ledcWriteTone(0, 784);
      publishMessage("home/flood", "FLOOD DETECTED");
    }
    else if (floodAlert)
    {
      floodAlert = false;
      lcd.clear();
      publishMessage("home/flood", "FLOOD CLEARED");
      sendTemperature();
    }
    
    int gasval = gas_ppm();
    Serial.printf("Gas Concentration: %d ppm\n", gasval);

    // Handle gas alert
    if (gasval > ALERT_GAS_THRESHOLD)
    {
      gasAlert = true;
      lcd.setCursor(0, 0);
      lcd.print("    WARNING!    ");
      lcd.setCursor(0, 1);
      lcd.print(" GAS DETECTED!  ");
      digitalWrite(19, HIGH);
      digitalWrite(18, LOW);
      ledcWriteTone(0, 659);
      publishMessage("home/gas", "GAS DETECTED");
    }
    else if (gasAlert)
    {
      gasAlert = false;
      lcd.clear();
      publishMessage("home/gas", "GAS CLEARED");
      sendTemperature();
    }

    // Display temperature when no alerts are active
    if (!fireAlert && !floodAlert && !gasAlert)
    {
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

    // Handle buzzer activation
    if (buzzerActive && (currentMillis - buzzerStartTime >= BUZZER_DURATION))
    {
      ledcWriteTone(0, 0);
      buzzerActive = false;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void sendTemperature()
{
  char tempStr[8];
  dtostrf(temperature, 1, 2, tempStr);
  Serial.println("Temperature");
  Serial.print(temperature);
  publishMessage("home/temperature", tempStr);
}

void sendGasPPM()
{
  int gasVal = gas_ppm();
  char gasStr[8];
  itoa(gasVal, gasStr, 10);
  Serial.println("Gas PPM");
  Serial.print(gasVal);
  publishRetainedMessage("home/gasppm", gasStr);
}

int gas_ppm(void)
{
  int gasVal = analogRead(gasSensorPin);

  gasVal = gasVal - GAS_MIN_VAL;

  if (gasVal < 0){
    gasVal = 0;
  }

  float coeff = (float)(GAS_MAX_VAL - GAS_MIN_VAL) / 490;

  gasVal = gasVal / (int)coeff;

  gasVal = gasVal + 10;

  return gasVal;
}

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 0;
  const int maxRetries = 30;

  while (WiFi.status() != WL_CONNECTED && retries < maxRetries)
  {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("");
    Serial.println("Failed to connect to WiFi");
  }
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("connected");
      client.subscribe("home/buzzer"); // Subscribe to the buzzer topic
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishMessage(const char *topic, const char *payload)
{
  if (!client.connected())
  {
    reconnect();
  }
  client.publish(topic, payload);
}

void publishRetainedMessage(const char *topic, const char *payload)
{
  if (!client.connected())
  {
    reconnect();
  }
  client.publish(topic, payload, true);
}

void callback(char* topic, byte* message, unsigned int length)
{
  String receivedMessage;
  for (unsigned int i = 0; i < length; i++)
  {
    receivedMessage += (char)message[i];
  }
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(receivedMessage);

  if (String(topic) == "home/buzzer")
  {
    buzzerStartTime = millis();
    buzzerActive = true;
    ledcWriteTone(0, 1000); // Activate buzzer at 1 kHz
    Serial.println("Buzzer activated for 2 seconds");
  }
}
