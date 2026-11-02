// ESP32-S3 limit-switch + buzzer board - CSV serial version.
// Same architecture as ESP32S3_SensorBoard_U8_CSV: plain Arduino sketch,
// machine-readable CSV over USB serial for a Python script on the Pi, with
// '#' lines for comments/status.
//
// Serial OUT (115200 baud, every 50 ms):
//   - Header printed once after boot:
//       #CSV,ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer
//   - Data lines, 6 comma-separated fields, no spaces:
//       ms              millis() since boot
//       limit_switch0-3 raw digitalRead of GPIO 18, 21, 47, 48 (1 = HIGH, 0 = LOW)
//       buzzer          1 = buzzer sounding, 0 = silent
//
// Serial IN (one command per line, '\n' or '\r\n'):
//   BUZZER,1      buzzer on (until turned off)
//   BUZZER,0      buzzer off
//   BUZZER,<ms>   beep for <ms> milliseconds (2..60000), then auto-off
//   Replies are '#' comment lines ("# OK ..." / "# ERROR ...").
//
// Buzzer wiring: the buzzer's + side is tied to supply, GPIO39 sinks the
// ground side. So GPIO39 LOW = loud, HIGH (or idle) = silent.
//
// Limit switches: pins use INPUT_PULLUP (switch to GND -> reads 0 when pressed).
// If your switches are wired the other way (external pull-down), change
// SWITCH_PIN_MODE to INPUT.

// ---------------- Pin Definitions ----------------
#define BUZZER_PIN 39
const uint8_t SWITCH_PINS[4] = {18, 21, 47, 48};
#define SWITCH_PIN_MODE INPUT_PULLUP

#define BUZZER_ON  LOW   // sink ground = loud
#define BUZZER_OFF HIGH

// ---------------- State ----------------
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

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  int comma = line.indexOf(',');
  String name = (comma < 0) ? line : line.substring(0, comma);
  name.trim();
  name.toUpperCase();

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

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  Serial.println("# ESP32-S3 Limit Switch + Buzzer (CSV) booting");

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(SWITCH_PINS[i], SWITCH_PIN_MODE);
  }

  Serial.println("# Setup complete");
  Serial.println("#CSV,ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer");
}

// ---------------- Loop ----------------
void loop() {
  readSerialCommands();

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
  Serial.println(buzzerOn ? 1 : 0);
}
