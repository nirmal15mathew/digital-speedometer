#include <Wire.h>
#include <LiquidCrystal.h>

#define ENCODER_DT 3  // DT pin connected to pin 3
#define PULSES_PER_REV 20  // Adjust based on encoder specs
#define WHEEL_DIAMETER_CM 10.0  // Adjust based on your wheel
#define WHEEL_CIRCUMFERENCE (3.1416 * WHEEL_DIAMETER_CM / 100.0)  // Convert to meters

volatile int pulseCount = 0;  // Pulse counter
unsigned long lastTime = 0;   // Last time check
const unsigned long interval = 1000; // 1-second interval for speed calculation
float totalDistance = 0.0; // Total distance covered (meters)

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void countPulse() {
    pulseCount++;  // Increment pulse count on every pulse
}

void setup() {
    pinMode(ENCODER_DT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_DT), countPulse, RISING); // Interrupt on rising edge
    
    lcd.begin(16, 2);
    
    lcd.setCursor(0, 0);
    lcd.print("    Welcome    ");
    delay(1000);
    lcd.clear();
    lcd.print("RPM:      Dist:");
    lastTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime >= interval) {  // Update every second
        int speed = pulseCount;  // Pulses per second
        float rpm = (speed / (float)PULSES_PER_REV) * 60; // Convert pulses to RPM
        float distanceCovered = (speed / (float)PULSES_PER_REV) * WHEEL_CIRCUMFERENCE; // Distance in meters
        totalDistance += distanceCovered; // Accumulate total distance
        
        // Print to LCD
        lcd.setCursor(4, 1);
        lcd.print("     "); // Clear old RPM value
        lcd.setCursor(4, 1);
        lcd.print((int)rpm);  // Show integer RPM

        lcd.setCursor(11, 1);
        lcd.print("     "); // Clear old distance
        lcd.setCursor(11, 1);
        lcd.print(totalDistance, 1); // Show distance with 1 decimal

        pulseCount = 0;  // Reset pulse count
        lastTime = currentTime;  // Update time
    }
}
