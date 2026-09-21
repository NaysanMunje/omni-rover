// One-shot: drive each candidate HIGH and print pad level. UART 115200.
#include "driver/gpio.h"

static const int kPins[] = {3, 8, 14, 21, 38, 39, 40, 41, 42, 45, 46, 47, 48};
static const int kCount = sizeof(kPins) / sizeof(kPins[0]);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("gpio_drive_scan");
  for (int i = 0; i < kCount; i++) {
    int p = kPins[i];
    gpio_reset_pin((gpio_num_t)p);
    gpio_set_direction((gpio_num_t)p, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level((gpio_num_t)p, 1);
    delay(5);
    int hi = gpio_get_level((gpio_num_t)p);
    gpio_set_level((gpio_num_t)p, 0);
    delay(5);
    int lo = gpio_get_level((gpio_num_t)p);
    Serial.printf("GPIO%02d high=%d low=%d %s\n", p, hi, lo,
                  (hi == 1 && lo == 0) ? "OK" : "FAIL");
    gpio_reset_pin((gpio_num_t)p);
  }
  Serial.println("done");
}

void loop() {}
