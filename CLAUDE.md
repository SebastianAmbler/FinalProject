# CLAUDE.md — Autonomous Indoor Tour-Guide Robot


## How to use this file
- This is a living project brief. When decisions change, update the relevant section.
- Items marked **OPEN** are unresolved and need action.
- Prices are approximate (2026).

---

## Robot physical spec
- Footprint: **50 x 50 cm** · Height: **110 cm** · Weight: **up to 17 kg**
- Tall and top-heavy -> tipping is a live concern. Keep battery low (bottom plate), cap accel/decel hard in Nav2.
- Drive: **4WD mecanum** (holonomic).
  - NOTE: mecanum was flagged as heavy added complexity (worse wheel odometry, more current, 2x motor tuning) for strafing a waypoint-following tour robot rarely needs. User chose it anyway and compensates with odometry pods (below).
- Speed: capped low (~0.3 m/s) for safety around people.

## Environment (drives many decisions)
- Indoor, **lots of windows/glass** — defeats 2D LiDAR. Handled mainly by hand-marked **keepout zones** on the saved map (free, most important), plus ToF + bump backup.
- **Furniture/desks moved frequently** — localize against a STATIC map of walls/glass only; let Nav2 live local costmap handle furniture. Re-map only on structural layout change.
- **Shallow ~3 cm floor gaps** at doorways — cross square-on and slow.

## Drivetrain
- Motors (ALREADY OWNED): 4x DC geared motor — **12 V, 200 RPM, 15.5 kg.cm, 500 P/R quadrature encoder, ratio 1:27, ~3 A, 8 mm D-shaft**. Encoder wiring: Red=VCC, Yellow=A, Black=GND, White=B. 200 RPM is fast -> cap in software.
- Motor driver: Cytron MDD10A. User confirmed **DIR + PWM** control (-> Cytron MDD-style). NOT DRV8871 (3.6 A, trips on stall at 17 kg) or TB6612 (1.2 A).

## Odometry pods (compensate for mecanum wheel slip)
- 3x dead-wheel odometry pods: 2 parallel (forward + rotation) + 1 perpendicular (strafe) for full X/Y/theta.
- Pod wheels: aluminum omni with RUBBER rollers, ~60 mm. MUST be spring-loaded/suspended to keep constant floor contact (pod accuracy depends on it).
- Pod encoders: **AS5600 magnetic** (ALREADY OWNED). Each needs a **6 mm x 2.5 mm diametrically-magnetized** cylinder magnet, mounted concentric on the shaft end, ~1-2 mm air gap.
- AS5600 gotcha: all share fixed I2C address 0x36. CHOSEN SOLUTION: use **PWM output mode** (not I2C) — one OUT pin per pod, parallel reads, no mux needed.
  - Requires one-time I2C config at boot to set PWM mode.
  - Requires angle-UNWRAPPING in software (accumulate revolutions across the 360->0 wrap).
  - Check magnet status (AGC/MD register) at setup to confirm magnet placement.

## Sensors
- 2D LiDAR: **RPLIDAR C1** (DTOF, 12 m) — primary nav. Alt: LDROBOT LD19.
- IMU: **BNO055** 
- ToF obstacle array: **4x VL53L0X** — primary front obstacle sensing (best on angled glass + thin desk legs). Via **PCA9548A** I2C mux (all keep 0x29). 3.3 V, no level shifter.
- Bump switches: **4x separate**, low front bumper skirt — last-resort contact stop.
- Depth camera: SKIPPED (cost). Glass handled by map keepouts + ToF.
- Buzzer: passive, via PWM + small transistor — movement/status alerts.

## Compute
- Main computer: **Raspberry Pi 4** Runs ROS 2.
- Two microcontrollers, BOTH **ESP32-S3-WROOM-1 N16R8**, each a micro-ROS node to the Pi:
  - MAIN S3: 4 motors (DIR+PWM) + 4 drive encoders + 4 bump switches + buzzer (~21 pins).
  - POD S3: 3 AS5600 pods (PWM) + I2C sensor bus (ToF mux + IMU) + boot-config I2C (~7 pins).
- Two MCUs because one couldn't fit 7 total encoders + everything else.
- 3.3 V load is fine on each board's regulator (POD ~105 mA, MAIN ~60 mA vs ~300-400 mA budget).

## Software stack (target)
- **ROS 2 Jazzy** + **Nav2** (holonomic config for mecanum) + **slam_toolbox** (map once) -> **AMCL** (localize on static map) + **robot_localization EKF** (fuses drive odom + pod odom + IMU).
- Tour routes via Nav2 waypoints; **Piper TTS** voice; AprilTags for precise stop positioning.

## Power
- POD S3 board monitors a **3S Li-ion/LiPo pack (~11.1-12.6 V nominal)** via a resistive divider (R1=75 kΩ top, R2=19.5 kΩ bottom, ratio 4.846) into an ADC pin (GPIO6, 12-bit, ADC_11db) — comfortably covers the 0-12.6 V range within the ESP32-S3's 3.3 V ADC.
- **OPEN**: confirm whether this is the SAME pack that drives the motors (via Cytron MDD10A) or a separate logic-only battery for the POD board. If separate packs, MAIN S3 may need its own voltage-divider monitoring too.

## Build status (as of 2026-11-02)
- **Physical build has not started** — no chassis, drivetrain, or wiring assembled yet. Firmware is being prototyped on the bench ahead of the mechanical build.
- Original POD S3 sketch (`ESP32S3 Code/ESP32S3_SensorBoard_U8/ESP32S3_SensorBoard_U8.ino`) is plain Arduino with `Serial.print` debug output only — NOT a ROS node. Kept as-is as the bench-test version (prints battery voltage, BNO055 Euler angles, and the 4 ToF ranges over USB serial every 500 ms).
- Converted POD S3 firmware now lives in a separate sketch, `ESP32S3 Code/ESP32S3_SensorBoard_U8_microROS/ESP32S3_SensorBoard_U8_microROS.ino`, as a **micro-ROS node** (`pod_s3_node`, serial transport, waits for agent at boot + time-syncs before creating entities). Publishes at 10 Hz: `pod/imu` (`sensor_msgs/Imu`, quaternion + gyro + linear accel from the BNO055), `pod/battery` (`sensor_msgs/BatteryState`, voltage only, rest of fields NaN/unknown), `pod/tof_0..3` (`sensor_msgs/Range`, meters, out-of-range reported as +Inf per REP 117, through the I2C mux). Mux chip is referred to correctly as PCA9548A (addr 0x70). Not yet build-tested on real hardware (no agent/board available during conversion).
- CSV bench version of the POD S3 sketch: `ESP32S3 Code/ESP32S3_SensorBoard_U8_CSV/ESP32S3_SensorBoard_U8_CSV.ino` — same sensors/pins as the U8 sketch, but serial output (115200 baud, every 50 ms) is machine-readable CSV, intended to be read by a Python script on the Pi that parses and processes it. Not ROS; a plain serial alternative to micro-ROS.
  - Lines starting with `#` are comments/status (boot messages, errors, header) — parser must skip them.
  - Header (printed once after boot): `#CSV,ms,batt_v,heading_deg,roll_deg,pitch_deg,tof0_mm,tof1_mm,tof2_mm,tof3_mm`
  - Data lines: 9 comma-separated fields, no spaces. `ms` = millis since boot; `batt_v` volts; heading/roll/pitch = BNO055 Euler x/y/z in degrees (`nan` if BNO055 failed init); `tofN_mm` = range in mm, `-1` = out of range, `-2` = sensor failed init.
  - AS5600 pod odometry is not in the CSV yet (same OPEN item below); add columns at the END when implemented so existing parsers keep working.
- Limit-switch + buzzer S3 sketch (new, separate ESP32-S3 board): `ESP32S3 Code/ESP32S3_LimitBuzzer_CSV/ESP32S3_LimitBuzzer_CSV.ino` — same architecture as the CSV POD sketch (plain Arduino, 115200 baud, CSV every 50 ms, `#` lines = comments). Not ROS, not bench-tested on hardware.
  - Limit switches on GPIO **18, 21, 47, 48** = `limit_switch0..3`, raw `digitalRead` (1/0), pins set `INPUT_PULLUP` (switch to GND reads 0 when pressed — change `SWITCH_PIN_MODE` if wired otherwise; polarity/NO-vs-NC not yet confirmed).
  - Buzzer on **GPIO39**, sink-type (pull-down): **LOW = loud**, HIGH = silent; driven silent first thing in `setup()`.
  - Header: `#CSV,ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer` (6 fields; `buzzer` = 1 if sounding).
  - Serial commands IN (newline-terminated): `BUZZER,1` on, `BUZZER,0` off, `BUZZER,<ms>` timed beep (2..60000 ms). Replies are `# OK ...` / `# ERROR ...` lines.
  - Likely the role of the MAIN S3's bump switches + buzzer (4 switches, 1 buzzer); MAIN S3 motor/encoder firmware still not started. Pi-side reader: node `limit_buzzer` in the `raspberrypi4/pod_serial_reader` package (`limit_buzzer_node.py`) — publishes `main/limit_switch_0..3` (Bool, True = pressed, param `pressed_level` default 0), `main/buzzer_state` (Bool), subscribes `main/buzzer_cmd` (Int32: 0 off, 1 on, 2..60000 ms beep → sends `BUZZER,<n>`). Default port `/dev/ttyACM1`. Run both boards concurrently with `ros2 launch pod_serial_reader both_boards.launch.py pod_port:=... limit_port:=...` (launch file in `raspberrypi4/pod_serial_reader/launch/`; use `/dev/serial/by-id/` paths). Not yet built/tested on a Pi.
- Pi side: `raspberrypi4/pod_serial_reader/` is a ROS 2 `ament_python` package (node `serial_reader`) that reads the CSV above from the serial port, shows a live terminal table, and publishes `pod/battery`, `pod/euler_deg` (Vector3Stamped), `pod/tof_0..3` (Range). Params: `port`, `baud`, `display`, `display_hz`, `publish`. Not yet built/tested on a Pi. Build steps in `raspberrypi4/README.md`.
- **OPEN**: AS5600 pod reading is NOT yet implemented. `odom1`/`odom2`/`odom3` are declared as plain `INPUT` pins with only a comment — still need PWM duty-cycle capture (`pulseIn` or RMT), angle-unwrapping across the 360->0 wrap, and the one-time I2C boot config to put each AS5600 into PWM mode. Magnet-placement check via the AGC/MD register is also not implemented yet. No odometry topic is published until this lands.
- **OPEN**: MAIN S3 firmware (motors, drive encoders, bump switches, buzzer) has not been started — not converted to micro-ROS either, since it doesn't exist yet.
- **OPEN**: On the Pi, needs a micro-ROS agent running against the POD S3's serial port (`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSBx -b 115200`) and the Arduino build needs the `micro_ros_arduino` library installed + a "Minimum SPIFFS (Large APP)"-style large partition scheme selected for the sketch to fit.

