// ESP32-S3 POD board - CSV serial output version.
// Same sensors/pins as ESP32S3_SensorBoard_U8, but every data sample is ONE
// CSV line so a Python script on the Raspberry Pi can parse it easily.
//
// Serial format (115200 baud):
//   - Lines starting with '#' are comments/status (boot messages, errors,
//     the column header). Skip them when parsing.
//   - The column header is printed once at boot, right after "# Setup complete":
//       #CSV,ms,batt_v,heading_deg,roll_deg,pitch_deg,tof0_mm,tof1_mm,tof2_mm,tof3_mm,
//            gyro_x_rads,gyro_y_rads,gyro_z_rads,lin_acc_x_ms2,lin_acc_y_ms2,lin_acc_z_ms2,
//            cal_sys,cal_gyro,cal_accel,cal_mag
//   - Every data line has exactly 19 comma-separated fields, no spaces:
//       ms          millis() since boot
//       batt_v      battery voltage, volts
//       heading_deg BNO055 Euler x (heading/yaw), degrees
//       roll_deg    BNO055 Euler y, degrees
//       pitch_deg   BNO055 Euler z, degrees
//       tof0..3_mm  VL53L0X range in mm
//                   -1 = out of range, -2 = sensor failed to init
//       gyro_*_rads     BNO055 angular velocity, rad/s (sensor axes; z up = CCW positive)
//       lin_acc_*_ms2   BNO055 linear acceleration, GRAVITY REMOVED, m/s^2
//       cal_*           BNO055 calibration status, 0 (uncalibrated) .. 3 (fully calibrated)
//     If the BNO055 failed to init, its Euler/gyro/accel fields are "nan" and
//     the four cal_* fields are -1.
//     New columns are always appended at the END so older parsers keep working.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_VL53L0X.h>

// ---------------- Pin Definitions ----------------
#define battCheck 6
#define odom1 1
#define odom2 2
#define odom3 4
#define SCL 9
#define SDA 8

// ---------------- Battery Divider ----------------
// Vbat = Vadc * (R1 + R2) / R2
const float R1 = 75000.0;   // ohms, top resistor (battery side)
const float R2 = 19500.0;   // ohms, bottom resistor (GND side)
const float DIVIDER_RATIO = (R1 + R2) / R2;

// ESP32-S3 ADC: 12-bit default (0-4095), reference ~3.3V with default attenuation.
// Using ADC_11db attenuation gives full 0-3.3V range on the pin.
const float ADC_MAX = 4095.0;
const float ADC_VREF = 3.3;

// ---------------- PCA9548A / TCA9548A Multiplexer ----------------
#define TCA9548A_ADDR 0x70

// Channel assignment
#define TOF0_CH 0
#define TOF1_CH 1
#define TOF2_CH 2
#define TOF3_CH 3

// CSV sentinel values for ToF
// 1 = reject any reading with RangeStatus != 0 (cleaner but short range);
// 0 = only reject out-of-range/hardware-fail (longer range, a bit noisier).
#define TOF_STRICT_STATUS 0
#define TOF_OUT_OF_RANGE -1
#define TOF_NOT_INIT     -2

// ---------------- Sensor Objects ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_VL53L0X tof[4] = {
  Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X()
};
bool tof_ok[4] = {false, false, false, false};
uint8_t tof_miss[4] = {0, 0, 0, 0};          // consecutive failed presence checks
unsigned long tof_lastRetry[4] = {0, 0, 0, 0};
const unsigned long TOF_RETRY_MS = 1000;     // re-init attempt period while a sensor is down
const uint8_t TOF_MISS_LIMIT = 2;            // failed pings before a sensor is declared lost
bool bno_ok = false;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 50; // ms

// ---------------- Helper: select mux channel ----------------
bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

// ---------------- Helper: is a device ACKing on the selected mux channel? ----------------
bool tofPresent(uint8_t i) {
  if (!tcaSelect(i)) return false;
  Wire.beginTransmission(VL53L0X_I2C_ADDR);
  return Wire.endTransmission() == 0;
}

// ---------------- Helper: (re)initialise one ToF behind the mux ----------------
bool tofInit(uint8_t i) {
  if (!tofPresent(i)) return false;
  // LONG_RANGE profile (~2 m in good conditions); the default profile is
  // tuned for ~1.2 m and loses range quickly in bright/ambient-IR rooms.
  return tof[i].begin(VL53L0X_I2C_ADDR, false, &Wire,
                      Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
}

// ---------------- Helper: read battery voltage ----------------
float readBatteryVoltage() {
  int raw = analogRead(battCheck);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  return vAdc * DIVIDER_RATIO;
}

// ---------------- Helper: read one ToF in mm (or sentinel) ----------------
int readTof(uint8_t i) {
  if (!tof_ok[i]) {
    // Down (never started, or lost): retry periodically so a replugged or
    // re-powered sensor comes back without rebooting the board.
    if (millis() - tof_lastRetry[i] >= TOF_RETRY_MS) {
      tof_lastRetry[i] = millis();
      if (tofInit(i)) {
        tof_ok[i] = true;
        tof_miss[i] = 0;
        Serial.print("# ToF"); Serial.print(i); Serial.println(" reconnected");
      }
    }
    if (!tof_ok[i]) return TOF_NOT_INIT;
  }

  // Presence check: an unplugged sensor otherwise just ranges as "out of range".
  if (!tofPresent(i)) {
    if (++tof_miss[i] >= TOF_MISS_LIMIT) {
      tof_ok[i] = false;
      tof_lastRetry[i] = millis();
      Serial.print("# ERROR: ToF"); Serial.print(i); Serial.println(" lost, will retry");
      return TOF_NOT_INIT;
    }
    return TOF_OUT_OF_RANGE;   // single glitch: don't flap yet
  }
  tof_miss[i] = 0;

  VL53L0X_RangingMeasurementData_t measure;
  tof[i].rangingTest(&measure, false);
#if TOF_STRICT_STATUS
  if (measure.RangeStatus != 0) return TOF_OUT_OF_RANGE;
#else
  // Status 4 = phase/out-of-range, 5 = hardware fail; 8190+ is the sensor's
  // "nothing seen" value. Weak-signal statuses (1/2) still return a usable range.
  if (measure.RangeStatus == 4 || measure.RangeStatus == 5 ||
      measure.RangeMilliMeter >= 8190) return TOF_OUT_OF_RANGE;
#endif
  return measure.RangeMilliMeter;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  Serial.println("# ESP32-S3 Sensor Suite (CSV) booting");

  // Battery ADC pin
  pinMode(battCheck, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(battCheck, ADC_11db);

  // Odometry pins (AS5600 PWM capture not implemented yet)
  pinMode(odom1, INPUT);
  pinMode(odom2, INPUT);
  pinMode(odom3, INPUT);

  // I2C bus init on custom pins
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);

  // ---- BNO055 init (directly on main bus, NOT behind mux) ----
  if (!bno.begin()) {
    Serial.println("# ERROR: BNO055 not detected. Check wiring/address.");
  } else {
    bno_ok = true;
    Serial.println("# BNO055 OK");
    delay(1000);
    bno.setExtCrystalUse(true);
  }

  // ---- Mux + VL53L0X init ----
  for (uint8_t i = 0; i < 4; i++) {
    tof_lastRetry[i] = millis();
    if (tofInit(i)) {
      tof_ok[i] = true;
      Serial.print("# ToF"); Serial.print(i); Serial.println(" OK");
    } else {
      Serial.print("# ERROR: ToF"); Serial.print(i); Serial.println(" failed to init");
    }
  }

  Serial.println("# Setup complete");
  Serial.println("#CSV,ms,batt_v,heading_deg,roll_deg,pitch_deg,tof0_mm,tof1_mm,tof2_mm,tof3_mm,"
                 "gyro_x_rads,gyro_y_rads,gyro_z_rads,lin_acc_x_ms2,lin_acc_y_ms2,lin_acc_z_ms2,"
                 "cal_sys,cal_gyro,cal_accel,cal_mag");
}

// ---------------- Loop ----------------
void loop() {
  if (millis() - lastPrint < PRINT_INTERVAL) return;
  lastPrint = millis();

  float vBat = readBatteryVoltage();

  float heading = NAN, roll = NAN, pitch = NAN;
  float gx = NAN, gy = NAN, gz = NAN, ax = NAN, ay = NAN, az = NAN;
  uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
  if (bno_ok) {
    sensors_event_t ev;
    bno.getEvent(&ev, Adafruit_BNO055::VECTOR_EULER);
    heading = ev.orientation.x;
    roll    = ev.orientation.y;
    pitch   = ev.orientation.z;

    bno.getEvent(&ev, Adafruit_BNO055::VECTOR_GYROSCOPE);      // rad/s
    gx = ev.gyro.x; gy = ev.gyro.y; gz = ev.gyro.z;

    bno.getEvent(&ev, Adafruit_BNO055::VECTOR_LINEARACCEL);    // m/s^2, no gravity
    ax = ev.acceleration.x; ay = ev.acceleration.y; az = ev.acceleration.z;

    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
  }

  int r0 = readTof(0);
  int r1 = readTof(1);
  int r2 = readTof(2);
  int r3 = readTof(3);

  // One CSV line per sample
  Serial.print(lastPrint);  Serial.print(',');
  Serial.print(vBat, 3);    Serial.print(',');
  Serial.print(heading, 2); Serial.print(',');
  Serial.print(roll, 2);    Serial.print(',');
  Serial.print(pitch, 2);   Serial.print(',');
  Serial.print(r0);         Serial.print(',');
  Serial.print(r1);         Serial.print(',');
  Serial.print(r2);         Serial.print(',');
  Serial.print(r3);         Serial.print(',');
  Serial.print(gx, 4);      Serial.print(',');
  Serial.print(gy, 4);      Serial.print(',');
  Serial.print(gz, 4);      Serial.print(',');
  Serial.print(ax, 3);      Serial.print(',');
  Serial.print(ay, 3);      Serial.print(',');
  Serial.print(az, 3);      Serial.print(',');
  if (bno_ok) {
    Serial.print(calSys);   Serial.print(',');
    Serial.print(calGyro);  Serial.print(',');
    Serial.print(calAccel); Serial.print(',');
    Serial.println(calMag);
  } else {
    Serial.println("-1,-1,-1,-1");
  }
}
