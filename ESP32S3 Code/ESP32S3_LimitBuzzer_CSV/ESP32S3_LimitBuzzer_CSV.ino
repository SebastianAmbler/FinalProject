// ESP32-S3 limit-switch + buzzer board - CSV serial version.
// Same architecture as ESP32S3_SensorBoard_U8_CSV: plain Arduino sketch,
// machine-readable CSV over USB serial for a Python script on the Pi, with
// '#' lines for comments/status.
//
// Serial OUT (115200 baud, every 50 ms):
//   - Header printed once after boot:
//       #CSV,ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer,motor1,motor2,motor3,motor4
//   - Data lines, 10 comma-separated fields, no spaces:
//       ms              millis() since boot
//       limit_switch0-3 raw digitalRead of GPIO 18, 21, 47, 48 (1 = HIGH, 0 = LOW)
//       buzzer          1 = buzzer sounding, 0 = silent
//       motor1-4        currently applied command, -255..255 (sign = direction)
//
// Serial IN (one command per line, '\n' or '\r\n'):
//   BUZZER,1      buzzer on (until turned off)
//   BUZZER,0      buzzer off
//   BUZZER,<ms>   beep for <ms> milliseconds (2..60000), then auto-off
//   MOTOR,<m1>,<m2>,<m3>,<m4>   each -255..255 (sign = direction, 0 = stop)
//   STOP          all motors off
//   TIMEOUT,<ms>  set motor watchdog (0 = disabled, bench debug only; default 500)
//   Motors stop automatically if no MOTOR command arrives within
//   MOTOR_TIMEOUT_MS (watchdog: Pi crash / cable pull / node killed).
//   Replies are '#' comment lines ("# OK ..." / "# ERROR ...").
//
// Motors: Cytron MDD10A in DIR+PWM mode. Pin pairs are {DIR, PWM}:
//   Motor1 = GPIO 1/2, Motor2 = 5/4, Motor3 = 7/6, Motor4 = 16/15.
//
// Buzzer wiring: the buzzer's + side is tied to supply, GPIO38 sinks the
// ground side. So GPIO38 LOW = loud, HIGH (or idle) = silent.
//
// Limit switches: pins use INPUT_PULLUP (switch to GND -> reads 0 when pressed).
// If your switches are wired the other way (external pull-down), change
// SWITCH_PIN_MODE to INPUT.

// ---------------- Pin Definitions ----------------
#define BUZZER_PIN 38
const uint8_t SWITCH_PINS[4] = {18, 21, 47, 48};
#define SWITCH_PIN_MODE INPUT_PULLUP

// Buzzer + is always on supply, - is on GPIO38, so LOW sinks current = loud.
#define BUZZER_ON  LOW
#define BUZZER_OFF HIGH

// Motor pins {DIR, PWM}. Index 0 = Motor1 ... index 3 = Motor4.
const uint8_t MOTOR_DIR_PINS[4] = {1, 5, 7, 16};
const uint8_t MOTOR_PWM_PINS[4] = {2, 4, 6, 15};
// Flip a motor's direction in firmware if it's wired/mounted reversed.
const bool MOTOR_INVERT[4] = {false, false, false, false};
#define MOTOR_PWM_FREQ 20000          // 20 kHz: silent, within MDD10A limit
#define MOTOR_PWM_BITS 8              // duty 0..255
#define MOTOR_TIMEOUT_MS 500          // default watchdog: stop if no MOTOR cmd this long
#define MOTOR_MAX_STEP 255            // max duty change per loop (ramp); lower = softer accel

// ---------------- State ----------------
int motorTarget[4] = {0, 0, 0, 0};    // last commanded
int motorApplied[4] = {0, 0, 0, 0};   // actually output (ramped)
unsigned long lastMotorCmd = 0;
unsigned long motorTimeoutMs = MOTOR_TIMEOUT_MS;  // 0 = watchdog disabled (bench debug only; resets on reboot)

bool buzzerOn = false;
unsigned long buzzerOffAt = 0;   // 0 = no auto-off pending

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 50; // ms

String cmdBuf;

// ---------------- Helpers ----------------
void setBuzzer(bool on) {
  buzzerOn = on;
  digitalWrite(BUZZER_PIN, on ? BUZZER_ON : BUZZER_OFF);
  if (!on) buzzerOffAt = 0;
}

void writeMotor(uint8_t i, int v) {
  motorApplied[i] = v;
  if (MOTOR_INVERT[i]) v = -v;
  digitalWrite(MOTOR_DIR_PINS[i], v >= 0 ? HIGH : LOW);
  int duty = abs(v);
  if (duty > 255) duty = 255;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM_PINS[i], duty);
#else
  ledcWrite(i, duty);
#endif
}

void stopMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    motorTarget[i] = 0;
    writeMotor(i, 0);
  }
}

// Ramp applied output toward target (limits acceleration; robot is top-heavy).
void updateMotors() {
  if (motorTarget[0] || motorTarget[1] || motorTarget[2] || motorTarget[3]) {
    if (motorTimeoutMs != 0 && millis() - lastMotorCmd > motorTimeoutMs) {
      stopMotors();
      Serial.println("# WARN motor watchdog timeout, stopped");
      return;
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    int d = motorTarget[i] - motorApplied[i];
    if (d > MOTOR_MAX_STEP) d = MOTOR_MAX_STEP;
    if (d < -MOTOR_MAX_STEP) d = -MOTOR_MAX_STEP;
    if (d != 0) writeMotor(i, motorApplied[i] + d);
  }
}

void handleMotorCommand(const String &line) {
  // line = "MOTOR,a,b,c,d"
  int vals[4];
  int start = line.indexOf(',');
  for (uint8_t i = 0; i < 4; i++) {
    if (start < 0) {
      Serial.print("# ERROR MOTOR needs 4 values: ");
      Serial.println(line);
      return;
    }
    int next = line.indexOf(',', start + 1);
    String tok = (next < 0) ? line.substring(start + 1) : line.substring(start + 1, next);
    tok.trim();
    long v = tok.toInt();
    bool numeric = (tok == "0") || (v != 0);
    if (!numeric || v < -255 || v > 255) {
      Serial.print("# ERROR bad motor value: ");
      Serial.println(tok);
      return;
    }
    vals[i] = (int)v;
    start = next;
  }
  if (start >= 0) {
    Serial.print("# ERROR MOTOR has extra values: ");
    Serial.println(line);
    return;
  }
  for (uint8_t i = 0; i < 4; i++) motorTarget[i] = vals[i];
  lastMotorCmd = millis();
  if (motorTimeoutMs == 0) {  // debug mode: echo what was parsed
    Serial.printf("# OK motor target %d,%d,%d,%d", vals[0], vals[1], vals[2], vals[3]);
    Serial.println();
  }
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  int comma = line.indexOf(',');
  String name = (comma < 0) ? line : line.substring(0, comma);
  name.trim();
  name.toUpperCase();

  if (name == "MOTOR") {
    handleMotorCommand(line);
    return;
  }
  if (name == "TIMEOUT") {
    // TIMEOUT,<ms>  watchdog length; 0 disables it (bench debugging only!)
    long v = line.substring(comma + 1).toInt();
    if (comma < 0 || v < 0 || v > 600000) {
      Serial.println("# ERROR usage: TIMEOUT,<0..600000 ms>");
      return;
    }
    motorTimeoutMs = (unsigned long)v;
    Serial.print("# OK motor watchdog ms=");
    Serial.println(v);
    return;
  }
  if (name == "STOP") {
    stopMotors();
    Serial.println("# OK motors stopped");
    return;
  }

  if (name != "BUZZER" || comma < 0) {
    Serial.print("# ERROR unknown command: ");
    Serial.println(line);
    return;
  }

  String arg = line.substring(comma + 1);
  arg.trim();
  long v = arg.toInt();
  bool numeric = (arg == "0") || (v != 0);
  if (!numeric || v < 0 || v > 60000) {
    Serial.print("# ERROR bad buzzer argument: ");
    Serial.println(arg);
    return;
  }

  if (v == 0) {
    setBuzzer(false);
    Serial.println("# OK buzzer off");
  } else if (v == 1) {
    setBuzzer(true);
    buzzerOffAt = 0;
    Serial.println("# OK buzzer on");
  } else {
    setBuzzer(true);
    buzzerOffAt = millis() + (unsigned long)v;
    if (buzzerOffAt == 0) buzzerOffAt = 1;
    Serial.print("# OK buzzer beep ms=");
    Serial.println(v);
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length()) handleCommand(cmdBuf);
      cmdBuf = "";
    } else if (cmdBuf.length() < 64) {
      cmdBuf += c;
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  // Set buzzer silent before anything else
  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  // Motors: DIR low, PWM 0 before anything else
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
    digitalWrite(MOTOR_DIR_PINS[i], LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PWM_PINS[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
    ledcSetup(i, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(MOTOR_PWM_PINS[i], i);
#endif
  }
  stopMotors();

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  Serial.println("# ESP32-S3 Limit Switch + Buzzer (CSV) booting");
  // 1=power-on 3=software 4=panic 6/7/8=watchdog 9=BROWNOUT (supply sag)
  Serial.print("# reset reason code: ");
  Serial.println((int)esp_reset_reason());

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(SWITCH_PINS[i], SWITCH_PIN_MODE);
  }

  Serial.println("# Setup complete");
  Serial.println("#CSV,ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer,motor1,motor2,motor3,motor4");
}

// ---------------- Loop ----------------
void loop() {
  readSerialCommands();
  updateMotors();

  // Auto-off for timed beeps
  if (buzzerOn && buzzerOffAt != 0 && (long)(millis() - buzzerOffAt) >= 0) {
    setBuzzer(false);
  }

  if (millis() - lastPrint < PRINT_INTERVAL) return;
  lastPrint = millis();

  Serial.print(lastPrint);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(',');
    Serial.print(digitalRead(SWITCH_PINS[i]));
  }
  Serial.print(',');
  Serial.print(buzzerOn ? 1 : 0);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(',');
    Serial.print(motorApplied[i]);
  }
  Serial.println();
}
