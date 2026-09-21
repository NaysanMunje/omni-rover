// TB6612 channel A: reverse direction every 3 s. No serial control.
// Wiring: PWMA=47, AIN1=41, AIN2=42, STBY=21

static const int PIN_PWMA = 47;
static const int PIN_AIN1 = 41;
static const int PIN_AIN2 = 42;
static const int PIN_STBY = 21;

static const int PWM_FREQ_HZ = 20000;
static const int PWM_RES_BITS = 8;
static const int SPEED = 180;  // 0–255
static const unsigned long INTERVAL_MS = 3000;

static bool clockwise = true;
static unsigned long lastSwitchMs = 0;

static void driveCw() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(PIN_PWMA, SPEED);
}

static void driveCcw() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, HIGH);
  ledcWrite(PIN_PWMA, SPEED);
}

void setup() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_RES_BITS);

  driveCw();
  lastSwitchMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSwitchMs >= INTERVAL_MS) {
    lastSwitchMs = now;
    clockwise = !clockwise;
    if (clockwise) {
      driveCw();
    } else {
      driveCcw();
    }
  }
}
