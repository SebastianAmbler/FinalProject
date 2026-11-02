

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

// ---------------- TCA9548A Multiplexer ----------------
#define TCA9548A_ADDR 0x70

// Channel assignment
#define TOF0_CH 0
#define TOF1_CH 1
#define TOF2_CH 2
#define TOF3_CH 3

// ---------------- Sensor Objects ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_VL53L0X tof0 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

bool tof0_ok = false;
bool tof1_ok = false;
bool tof2_ok = false;
bool tof3_ok = false;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 500; // ms

// ---------------- Helper: select TCA9548A channel ----------------
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ---------------- Helper: read battery voltage ----------------
float readBatteryVoltage() {
  int raw = analogRead(battCheck);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  float vBat = vAdc * DIVIDER_RATIO;
  return vBat;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  Serial.println("=== ESP32-S3 Sensor Suite Booting ===");

  // Battery ADC pin
  pinMode(battCheck, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(battCheck, ADC_11db);

  // Odometry pins (add your encoder setup here if needed)
  pinMode(odom1, INPUT);
  pinMode(odom2, INPUT);
  pinMode(odom3, INPUT);

  // I2C bus init on custom pins
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);

  // ---- BNO055 init (directly on main bus, NOT behind mux) ----
  Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected. Check wiring/address.");
  } else {
    Serial.println("BNO055 initialized OK.");
    delay(1000);
    bno.setExtCrystalUse(true);
  }

  // ---- TCA9548A + VL53L0X init ----
  Serial.println("Initializing VL53L0X sensors via TCA9548A...");

  tcaSelect(TOF0_CH);
  if (tof0.begin()) {
    tof0_ok = true;
    Serial.println("ToF0 (ch0) initialized OK.");
  } else {
    Serial.println("ERROR: ToF0 (ch0) failed to initialize.");
  }

  tcaSelect(TOF1_CH);
  if (tof1.begin()) {
    tof1_ok = true;
    Serial.println("ToF1 (ch1) initialized OK.");
  } else {
    Serial.println("ERROR: ToF1 (ch1) failed to initialize.");
  }

  tcaSelect(TOF2_CH);
  if (tof2.begin()) {
    tof2_ok = true;
    Serial.println("ToF2 (ch2) initialized OK.");
  } else {
    Serial.println("ERROR: ToF2 (ch2) failed to initialize.");
  }

  tcaSelect(TOF3_CH);
  if (tof3.begin()) {
    tof3_ok = true;
    Serial.println("ToF3 (ch3) initialized OK.");
  } else {
    Serial.println("ERROR: ToF3 (ch3) failed to initialize.");
  }

  Serial.println("=== Setup complete ===\n");
}

// ---------------- Loop ----------------
void loop() {
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    // ---- Battery ----
    float vBat = readBatteryVoltage();
    Serial.print("Battery Voltage: ");
    Serial.print(vBat, 3);
    Serial.println(" V");

    // ---- BNO055 (orientation only) ----
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    Serial.print("BNO055 Orientation (x,y,z): ");
    Serial.print(orientationData.orientation.x, 2); Serial.print(", ");
    Serial.print(orientationData.orientation.y, 2); Serial.print(", ");
    Serial.println(orientationData.orientation.z, 2);

    // ---- ToF sensors ----
    VL53L0X_RangingMeasurementData_t measure;

    if (tof0_ok) {
      tcaSelect(TOF0_CH);
      tof0.rangingTest(&measure, false);
      if (measure.RangeStatus == 0) {
        Serial.print("ToF0: "); Serial.print(measure.RangeMilliMeter); Serial.println(" mm");
      } else {
        Serial.println("ToF0: out of range");
      }
    }

    if (tof1_ok) {
      tcaSelect(TOF1_CH);
      tof1.rangingTest(&measure, false);
      if (measure.RangeStatus == 0) {
        Serial.print("ToF1: "); Serial.print(measure.RangeMilliMeter); Serial.println(" mm");
      } else {
        Serial.println("ToF1: out of range");
      }
    }

    if (tof2_ok) {
      tcaSelect(TOF2_CH);
      tof2.rangingTest(&measure, false);
      if (measure.RangeStatus == 0) {
        Serial.print("ToF2: "); Serial.print(measure.RangeMilliMeter); Serial.println(" mm");
      } else {
        Serial.println("ToF2: out of range");
      }
    }

    if (tof3_ok) {
      tcaSelect(TOF3_CH);
      tof3.rangingTest(&measure, false);
      if (measure.RangeStatus == 0) {
        Serial.print("ToF3: "); Serial.print(measure.RangeMilliMeter); Serial.println(" mm");
      } else {
        Serial.println("ToF3: out of range");
      }
    }

    // ---- Odometry (placeholder) ----
    // If odom1/odom2/odom3 are encoder pulse pins, add interrupt-based
    // pulse counting elsewhere and print counts here, e.g.:
    // Serial.print("Odom1: "); Serial.println(odomCount1);

    Serial.println("-----------------------------------");
  }
}