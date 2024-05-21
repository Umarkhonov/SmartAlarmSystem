#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int flameSensorPin = 34;       // Adjust pin number as needed
const int moistureSensorPin = 32;    // Adjust pin number as needed

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Initialize LCD with I2C address 0x27

void setup() {
  // Initialize the I2C communication for the LCD
  Wire.begin(21, 22); // SDA, SCL pins
  
  lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows
  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight
  
  pinMode(flameSensorPin, INPUT);
  pinMode(moistureSensorPin, INPUT);
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
  
  if (flameSensorValue == HIGH) {
    lcd.setCursor(0, 0);
    lcd.print("    WARNING!    ");
    lcd.setCursor(0, 1);
    lcd.print(" FIRE DETECTED! ");
    digitalWrite(19, HIGH); // Turn on Red LED
    digitalWrite(18, LOW);  // Turn off Green LED
    tone(2, 523);           // Produce a tone on pin 2 at 523 Hz (C5 note)
  } else if (moistureSensorValue == HIGH) {
    lcd.setCursor(0, 0);
    lcd.print("    WARNING!    ");
    lcd.setCursor(0, 1);
    lcd.print(" FLOOD DETECTED ");
    digitalWrite(19, HIGH); // Turn on Red LED
    digitalWrite(18, LOW);  // Turn off Green LED
    tone(2, 784);           // Produce a tone on pin 2 at 523 Hz (C5 note)
  } else {
    lcd.setCursor(0, 0);
    lcd.print("    ALL GOOD    ");
    lcd.setCursor(0, 1);
    lcd.print("                "); // Clear second row
    digitalWrite(18, HIGH); // Turn on Green LED
    digitalWrite(19, LOW);  // Turn off Red LED
    noTone(2);              // Turn off Buzzer
  }
  
  delay(100);
}