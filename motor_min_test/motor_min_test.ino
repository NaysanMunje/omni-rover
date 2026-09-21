// GPIO2 left free (your AO pin).
// AIN1=GPIO38  AIN2=GPIO3  PWMA=GPIO48  STBY jumper to 3.3V

#include "driver/gpio.h"

static const int PIN_AIN1 = 38;
static const int PIN_AIN2 = 3;
static const int PIN_PWMA = 48;
static const unsigned long INTERVAL_MS = 3000;

static bool clockwise = true;
static unsigned long lastSwitchMs = 0;

static void claimOut(int pin) {
  gpio_reset_pin((gpio_num_t)pin);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_level((gpio_num_t)pin, 0);
}

static void applyDrive() {
  gpio_set_level((gpio_num_t)PIN_PWMA, 1);
  if (clockwise) {
    gpio_set_level((gpio_num_t)PIN_AIN1, 1);
    gpio_set_level((gpio_num_t)PIN_AIN2, 0);
  } else {
    gpio_set_level((gpio_num_t)PIN_AIN1, 0);
    gpio_set_level((gpio_num_t)PIN_AIN2, 1);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("motor_min_test GPIO2 free");
  Serial.println("AIN1=GPIO38 AIN2=GPIO3 PWMA=GPIO48 STBY jumper to 3V3 (GPIO2 unused)");

  claimOut(PIN_AIN1);
  claimOut(PIN_AIN2);
  claimOut(PIN_PWMA);

  clockwise = true;
  applyDrive();
  lastSwitchMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSwitchMs >= INTERVAL_MS) {
    lastSwitchMs = now;
    clockwise = !clockwise;
    applyDrive();
  }

  static unsigned long lastBeat = 0;
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    Serial.printf("dir=%s AIN1=%d AIN2=%d PWMA=%d\n",
                  clockwise ? "CW" : "CCW",
                  gpio_get_level((gpio_num_t)PIN_AIN1),
                  gpio_get_level((gpio_num_t)PIN_AIN2),
                  gpio_get_level((gpio_num_t)PIN_PWMA));
  }
}
