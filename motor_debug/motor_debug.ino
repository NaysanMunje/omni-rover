// UART debug for TB6612 on Freenove ESP32-S3 (CH343 / UART USB).
// Same wiring: PWMA=47, AIN1=41, AIN2=42, STBY=21
// Open Serial Monitor 115200 on COM6. No commands — it only prints.

#include "driver/gpio.h"

static const int PIN_PWMA = 47;
static const int PIN_AIN1 = 41;
static const int PIN_AIN2 = 42;
static const int PIN_STBY = 21;

static const int PWM_FREQ_HZ = 20000;
static const int PWM_RES_BITS = 8;
static const int SPEED = 255;
static const unsigned long INTERVAL_MS = 3000;

static bool clockwise = true;
static unsigned long lastSwitchMs = 0;
static bool pwmOk = false;

static void claimPin(int pin) {
  gpio_reset_pin((gpio_num_t)pin);
  pinMode(pin, OUTPUT);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
}

static void applyDrive() {
  if (clockwise) {
    gpio_set_level((gpio_num_t)PIN_AIN1, 1);
    gpio_set_level((gpio_num_t)PIN_AIN2, 0);
  } else {
    gpio_set_level((gpio_num_t)PIN_AIN1, 0);
    gpio_set_level((gpio_num_t)PIN_AIN2, 1);
  }
  gpio_set_level((gpio_num_t)PIN_STBY, 1);
  if (pwmOk) {
    ledcWrite(PIN_PWMA, SPEED);
  }
}

static void printStatus(const char* event) {
  Serial.printf(
      "%s t=%lu dir=%s STBY_w=%d STBY_r=%d AIN1_w=%d AIN1_r=%d AIN2_w=%d AIN2_r=%d pwmOk=%d duty=%d\n",
      event,
      millis(),
      clockwise ? "CW" : "CCW",
      1,
      gpio_get_level((gpio_num_t)PIN_STBY),
      clockwise ? 1 : 0,
      gpio_get_level((gpio_num_t)PIN_AIN1),
      clockwise ? 0 : 1,
      gpio_get_level((gpio_num_t)PIN_AIN2),
      pwmOk ? 1 : 0,
      SPEED);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("motor_debug start");
  Serial.printf("psramFound=%d\n", psramFound() ? 1 : 0);

  claimPin(PIN_STBY);
  claimPin(PIN_AIN1);
  claimPin(PIN_AIN2);
  gpio_reset_pin((gpio_num_t)PIN_PWMA);

  pwmOk = ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_RES_BITS);
  Serial.printf("ledcAttach GPIO%d -> %s\n", PIN_PWMA, pwmOk ? "ok" : "FAIL");

  gpio_set_level((gpio_num_t)PIN_STBY, 1);
  clockwise = true;
  applyDrive();
  lastSwitchMs = millis();
  printStatus("boot");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSwitchMs >= INTERVAL_MS) {
    lastSwitchMs = now;
    clockwise = !clockwise;
    applyDrive();
    printStatus("switch");
  }

  static unsigned long lastBeat = 0;
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    printStatus("beat");
  }
}
