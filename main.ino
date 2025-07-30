#include <Wire.h>
#include <LiquidCrystal.h>

#define ENCODER_DT 3  // DT pin connected to pin 3
#define PULSES_PER_REV 20  // Adjust based on encoder specs
#define WHEEL_DIAMETER_CM 10.0  // Adjust based on your wheel
#define WHEEL_CIRCUMFERENCE (3.1416 * WHEEL_DIAMETER_CM / 100.0)  // Convert to meters
#define MODE_SELECTOR 8

volatile int pulseCount = 0;  // Pulse counter
unsigned long lastTime = 0;   // Last time check
const unsigned long interval = 200; // 1-second interval for speed calculation
float totalDistance = 0.0; // Total distance covered (meters)

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int Display_Mode = 0;
bool headlight = false;

void countPulse() {
    pulseCount++;  // Increment pulse count on every pulse
}

void setup() {
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(MODE_SELECTOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_DT), countPulse, RISING); // Interrupt on rising edge
    
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("    Welcome    ");
    delay(1000);
    lcd.clear();
    lcd.print("     Speed     ");
    lastTime = millis();
    Serial.begin(9600);
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime >= interval) {  // Update every second
        int speed = pulseCount;  // Pulses per second
        float rpm = (speed / (float)PULSES_PER_REV) * 60; // Convert pulses to RPM
        float distanceCovered = (speed / (float)PULSES_PER_REV) * WHEEL_CIRCUMFERENCE; // Distance in meters
        totalDistance += distanceCovered; // Accumulate total distance

      lcd.setCursor(0, 1);
      lcd.print("     "); // Clear old RPM value
      lcd.setCursor(0, 1);
      switch (Display_Mode) {
            case 0:
              // Print to LCD
              lcd.print((int)rpm);  // Show integer RPM
              break;
            case 1:
              lcd.print(totalDistance, 1); // Show distance with 1 decimal
              break;
            default:
              lcd.print("Welcome");
              break;
      }
      if (digitalRead(MODE_SELECTOR) == LOW) {
      Serial.println(Display_Mode);
      lcd.clear();
      lcd.setCursor(0, 0);
      if (Display_Mode < 1) {
        lcd.print("    Distance    ");
        Display_Mode++;
      }
      else {
        lcd.print("     Speed     ");
        Display_Mode = 0;
      }
    }


        pulseCount = 0;  // Reset pulse count
        lastTime = currentTime;  // Update time
    }
}
