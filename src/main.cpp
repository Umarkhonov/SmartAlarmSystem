#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int flameSensorPin = 34;       // Adjust pin number as needed
const int moistureSensorPin = 32;    // Adjust pin number as needed
const int gasSensorPin = 33;         // Adjust pin number as needed
const int tempSensorPin = 25;        // Pin for LM35 temperature sensor

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Initialize LCD with I2C address 0x27

void setup() {
  // Initialize the I2C communication for the LCD
  Wire.begin(21, 22); // SDA, SCL pins
  
  lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows
  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight
  
  pinMode(flameSensorPin, INPUT);
  pinMode(moistureSensorPin, INPUT);
  pinMode(gasSensorPin, INPUT);
  pinMode(tempSensorPin, INPUT);
  pinMode(2, OUTPUT);  // Buzzer
  pinMode(18, OUTPUT); // Green LED
  pinMode(19, OUTPUT); // Red LED

  Serial.begin(115200); // Initialize serial communication for debugging

  // Initialize LEDC
  ledcAttachPin(2, 0);  // Attach pin 2 to channel 0
  ledcSetup(0, 5000, 8); // 5 kHz frequency, 8-bit resolution
}

void loop() {
  int flameSensorValue = digitalRead(flameSensorPin);
  int moistureSensorValue = digitalRead(moistureSensorPin);
  int gasSensorValue = analogRead(gasSensorPin);
  int tempSensorValue = analogRead(tempSensorPin);

  // Convert the analog reading (which goes from 0 - 4095) to a voltage (0 - 3.3V):
  float voltage = tempSensorValue * (3.3 / 4095.0);
  // Convert the voltage to a temperature in Celsius:
  float temperature = voltage * 100.0;  // LM35 has a 10mV/°C scale factor
  temperature -= 10;  // Adjust for 10°C offset

  // Print the gas sensor value to the serial monitor
  Serial.print("Gas Sensor Value: ");
  Serial.println(gasSensorValue);

  if (flameSensorValue == HIGH) {
    lcd.setCursor(0, 0);
    lcd.print("    WARNING!    ");
    lcd.setCursor(0, 1);
    lcd.print(" FIRE DETECTED! ");
    digitalWrite(19, HIGH); // Turn on Red LED
    digitalWrite(18, LOW);  // Turn off Green LED
    ledcWriteTone(0, 523);  // Produce a tone on channel 0 at 523 Hz (C5 note)
  } else if (moistureSensorValue == HIGH) {
    lcd.setCursor(0, 0);
    lcd.print("    WARNING!    ");
    lcd.setCursor(0, 1);
    lcd.print(" FLOOD DETECTED ");
    digitalWrite(19, HIGH); // Turn on Red LED
    digitalWrite(18, LOW);  // Turn off Green LED
    ledcWriteTone(0, 784);  // Produce a tone on channel 0 at 784 Hz (G5 note)
  } else if (gasSensorValue > 2700) { // Adjust threshold as needed
    lcd.setCursor(0, 0);
    lcd.print("    WARNING!    ");
    lcd.setCursor(0, 1);
    lcd.print(" GAS DETECTED!  ");
    digitalWrite(19, HIGH); // Turn on Red LED
    digitalWrite(18, LOW);  // Turn off Green LED
    ledcWriteTone(0, 659);  // Produce a tone on channel 0 at 659 Hz (E5 note)
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("                "); // Clear second row
    digitalWrite(18, HIGH); // Turn on Green LED
    digitalWrite(19, LOW);  // Turn off Red LED
    ledcWriteTone(0, 0);    // Turn off Buzzer
  }

  delay(2000); // Delay for 2 seconds to allow sensor readings to update
}
