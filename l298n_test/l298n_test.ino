#include <Arduino.h>

// L298N only. Same loop as l982n_test / gem_test.
// Pins from three_motor_test motor 3.

const int PIN_ENA = 15;        // PWM speed
const int PIN_IN1 = 47;
const int PIN_IN2 = 21;
const int PIN_ENCODER_A = 13;  // Yellow
const int PIN_ENCODER_B = 14;  // Green

const int PWM_FREQ = 5000;
const int PWM_RES = 8;

volatile long encoderTicks = 0;

void setMotor(int speed, bool forward);
void printTicksFor(unsigned long duration);
void IRAM_ATTR readEncoder();

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), readEncoder, RISING);
}

void loop() {
  Serial.println("Moving FORWARD...");
  setMotor(180, true);
  printTicksFor(2000);

  Serial.println("STOPPING...");
  setMotor(0, true);
  delay(1000);

  Serial.println("Moving REVERSE...");
  setMotor(180, false);
  printTicksFor(2000);

  Serial.println("STOPPING...");
  setMotor(0, true);
  delay(1000);
}

void setMotor(int speed, bool forward) {
  ledcWrite(PIN_ENA, speed);

  if (forward) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
}

void printTicksFor(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    Serial.print("Encoder Ticks: ");
    Serial.println(encoderTicks);
    delay(100);
  }
}

void IRAM_ATTR readEncoder() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}
