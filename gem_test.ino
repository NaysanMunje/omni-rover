#include <Arduino.h>

// --- Pin Assignments (ESP32 GPIOs) ---
const int PIN_ENCODER_A = 1;  // Yellow Wire
const int PIN_ENCODER_B = 2;  // Green Wire
const int PIN_PWMA      = 48; // Speed Pin
const int PIN_AIN2      = 42; // Direction Pin 2
const int PIN_AIN1      = 41; // Direction Pin 1
const int PIN_STBY      = 3;  // Standby Pin

// --- PWM Setup for ESP32 ---
const int PWM_FREQ = 5000;    // 5 kHz frequency
const int PWM_RES  = 8;       // 8-bit resolution (0-255)

// --- Global Variables ---
volatile long encoderTicks = 0;

// Function Prototypes
void setMotor(int speed, bool forward);
void printTicksFor(unsigned long duration);
void IRAM_ATTR readEncoder();

void setup() {
  Serial.begin(115200);

  // Control Pin Modes
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Configure ESP32 Hardware PWM on PWMA pin
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);

  // Encoder Pin Modes
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);

  // Enable the TB6612 Driver
  digitalWrite(PIN_STBY, HIGH);

  // Attach Interrupt for Encoder (Yellow Wire on GPIO 1)
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), readEncoder, RISING);
}

void loop() {
  Serial.println("Moving FORWARD...");
  setMotor(180, true);   // ~70% Speed Forward
  printTicksFor(2000);   // Run for 2 seconds while printing ticks

  Serial.println("STOPPING...");
  setMotor(0, true);     // Stop motor
  delay(1000);

  Serial.println("Moving REVERSE...");
  setMotor(180, false);  // ~70% Speed Reverse
  printTicksFor(2000);   // Run for 2 seconds while printing ticks

  Serial.println("STOPPING...");
  setMotor(0, true);     // Stop motor
  delay(1000);
}

// Controls motor speed (0-255) and direction
void setMotor(int speed, bool forward) {
  // Drive PWM signal using ESP32 ledc write
  ledcWrite(PIN_PWMA, speed);

  if (forward) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }
}

// Prints encoder readings safely outside the ISR
void printTicksFor(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    // Print current position
    Serial.print("Encoder Ticks: ");
    Serial.println(encoderTicks);
    delay(100);
  }
}

// Interrupt Service Routine (ISR) stored in RAM for fast execution
void IRAM_ATTR readEncoder() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}