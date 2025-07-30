#include <Wire.h>
#include <LiquidCrystal.h>

#define ENCODER_DT 3  // DT pin connected to pin 3
#define PULSES_PER_REV 20  // Adjust based on encoder specs
#define WHEEL_DIAMETER_CM 10.0  // Adjust based on your wheel
#define WHEEL_CIRCUMFERENCE (3.1416 * WHEEL_DIAMETER_CM / 100.0)  // Convert to meters
#define MODE_SELECTOR 8
#define TRIG_PIN 6  // Ultrasonic sensor trigger pin
#define ECHO_PIN 2  // Ultrasonic sensor echo pin
#define ALERT_PIN 13 // Alert output pin
#define FLASH_BTN 7
#define FLASH_PIN A5

volatile int pulseCount = 0;  // Pulse counter
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool echoReceived = false;

unsigned long lastTime = 0;   // Last time check
const unsigned long interval = 200; // 1-second interval for speed calculation
float totalDistance = 0.0; // Total distance covered (meters)

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

byte flash[8] = {
  0b11111,
  0b10001,
  0b01010,
  0b01110,
  0b01010,
  0b01010,
  0b01010,
  0b01110,
};

int Display_Mode = 0;
bool headlight = false;

void countPulse() {
    pulseCount++;  // Increment pulse count on every pulse
}

void echoISR() {
    if (digitalRead(ECHO_PIN) == HIGH) {
        echoStartTime = micros(); // Rising edge detected
    } else {
        echoEndTime = micros();   // Falling edge detected
        echoReceived = true;
    }
}

void setup() {
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(MODE_SELECTOR, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(ALERT_PIN, OUTPUT);
    pinMode(FLASH_BTN, INPUT_PULLUP);
    pinMode(FLASH_PIN, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_DT), countPulse, RISING); // Interrupt on rising edge
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE); // Interrupt on both edges
    
    lcd.begin(16, 2);
    lcd.createChar(0, flash);
    lcd.setCursor(0, 0);
    lcd.print("    Welcome    ");
    delay(1000);
    lcd.clear();
    lcd.print("     Speed     ");
    lastTime = millis();
    Serial.begin(9600);
}

float measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    while (!echoReceived); // Wait for the echo interrupt to complete
    echoReceived = false; // Reset flag
    
    long duration = echoEndTime - echoStartTime;
    float distance = (duration * 0.0343) / 2; // Convert to cm
    return distance;
}

void loop() {
    unsigned long currentTime = millis();
    
    // Ultrasonic sensor measurement
    float obstacleDistance = measureDistance();
    if (obstacleDistance > 0 && obstacleDistance < 30) {
        digitalWrite(ALERT_PIN, HIGH); // Turn on alert
        lcd.setCursor(0, 1);
        lcd.print("   ALERT!    ");
    } else {
        lcd.setCursor(3, 1);
        lcd.print("       ");
        digitalWrite(ALERT_PIN, LOW); // Turn off alert
        if (currentTime - lastTime >= interval) {  // Update every second
        int speed = pulseCount;  // Pulses per second
        float rpm = (speed / (float)PULSES_PER_REV) * 60; // Convert pulses to RPM
        float distanceCovered = (speed / (float)PULSES_PER_REV) * WHEEL_CIRCUMFERENCE; // Distance in meters
        totalDistance += distanceCovered; // Accumulate total distance

        lcd.setCursor(0, 1);
        lcd.print("     "); // Clear old value
        lcd.setCursor(0, 1);
        switch (Display_Mode) {
            case 0:
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
            } else {
                lcd.print("     Speed     ");
                Display_Mode = 0;
            }
        }
        if (digitalRead(FLASH_BTN) == LOW) {
          digitalWrite(FLASH_PIN, !headlight);
          Serial.print("Headlight");
          Serial.println(headlight);
          headlight = !headlight;
          lcd.setCursor(15, 0);
          if (headlight)  {
            lcd.write((byte)0); 
          }
          else {
            lcd.write(" ");
          }
        }

        pulseCount = 0;  // Reset pulse count
        lastTime = currentTime;  // Update time
    }
    }
}
