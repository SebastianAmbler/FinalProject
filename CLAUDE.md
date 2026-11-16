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
- Indoor, **lots of windows/glass** — defeats 2D LiDAR. Handled mainly by hand-marked **keepout zones** on the saved map (free, most important), plus ToF backup.
- **Furniture/desks moved frequently** — localize against a STATIC map of walls/glass only; let Nav2 live local costmap handle furniture. Re-map only on structural layout change.
- **Shallow ~3 cm floor gaps** at doorways — cross square-on and slow.

## Drivetrain
- Motors (ALREADY OWNED): 4x DC geared motor — **12 V, 200 RPM, 15.5 kg.cm, 500 P/R quadrature encoder, ratio 1:27, ~3 A, 8 mm D-shaft**. Encoder wiring: Red=VCC, Yellow=A, Black=GND, White=B. 200 RPM is fast -> cap in software.
- **Drive encoder pins (MAIN S3, user-specified; firmware not yet written):** Encoder 1 = A GPIO13 / B GPIO12 · Encoder 2 = A GPIO9 / B GPIO8 · Encoder 3 = A GPIO11 / B GPIO10 · Encoder 4 = A GPIO14 / B GPIO17. Encoder N pairs with Motor N (M1=GPIO 1/2, M2=5/4, M3=7/6, M4=16/15 for DIR/PWM).
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
- Depth camera: SKIPPED (cost). Glass handled by map keepouts + ToF.
- Buzzer: passive, via PWM + small transistor — movement/status alerts.

## Compute
- Main computer: **Raspberry Pi 4** Runs ROS 2.
- **Architecture: plain-Arduino ESP32-S3 sketches stream CSV over USB serial (115200 baud); Python ROS 2 nodes on the Pi parse the CSV and publish topics.** micro-ROS was tried and DROPPED — no micro-ROS agent, `micro_ros_arduino` library, or special partition scheme is needed.
- ESP32-S3 boards (**ESP32-S3-WROOM-1 N16R8**), each on its own USB serial port to the Pi (use `/dev/serial/by-id/` paths so ports don't swap):
  - POD S3: 3 AS5600 pods (PWM) + I2C sensor bus (ToF mux + BNO055 IMU) + battery ADC + boot-config I2C (~7 pins). Sketch exists (sensors only; AS5600 not yet implemented).
  - BUZZER/MOTOR S3 (sketch folder still named `ESP32S3_LimitBuzzer_CSV`): buzzer + motor drive. Sketch exists. In the plan this is the buzzer/motor part of the MAIN S3.
  - MAIN S3 (planned): 4 motors (DIR+PWM) + 4 drive encoders (+ buzzer, currently prototyped on the BUZZER/MOTOR board) (~21 pins). Motor/encoder firmware NOT started.
- Two MCUs because one couldn't fit 7 total encoders + everything else.
- 3.3 V load is fine on each board's regulator (POD ~105 mA, MAIN ~60 mA vs ~300-400 mA budget).

## Software stack (target)
- **ROS 2 Jazzy** + **Nav2** (holonomic config for mecanum) + **slam_toolbox** (map once) -> **AMCL** (localize on static map) + **robot_localization EKF** (fuses drive odom + pod odom + IMU).
- Tour routes via Nav2 waypoints; **Piper TTS** voice; AprilTags for precise stop positioning.

## Web interface (PLANNED — not started)
The Pi joins WiFi and serves a web UI; the user opens it from a phone/laptop on the same network to configure and monitor the robot.
- **Networking**: Pi connects to WiFi (NetworkManager/`nmcli`). **OPEN**: fallback if the known WiFi is unavailable — Pi starts its own hotspot (AP mode) so the UI is still reachable; also use a fixed hostname (e.g. `robot.local` via mDNS) so no IP lookup is needed.
- **Pages**:
  1. **Settings / map page**: displays the saved map streamed from the Pi (`nav_msgs/OccupancyGrid`, rendered on a canvas) with live robot pose. User can **mark tour spots** (click on map to place a waypoint, set name, heading, order, dwell time) and also **draw keepout zones** (glass/windows — the most important map edit, see Environment). Spots/zones are saved on the Pi and fed to Nav2 as waypoints / keepout mask.
  2. **Speech page**: user types the text the robot says at each tour spot (or ad-hoc "say this now"); text is sent to the Pi and spoken via **Piper TTS**. Per-spot scripts stored with the spot.
  3. **Status page**: live robot status — battery voltage (`pod/battery`), pose/localization state, current tour/waypoint, ToF ranges, IMU calibration, serial-link health of each ESP32 (port up / last data age), Nav2 state, errors.
  4. **Camera page**: live camera feed. **OPEN / conflict**: the hardware spec currently has NO camera (depth camera skipped for cost; AprilTags also need one). Need to choose and add a camera (e.g. Pi Camera Module or USB webcam) — this also enables AprilTags.
- **Suggested architecture**: ROS 2 nodes on the Pi + `rosbridge_suite` (WebSocket) for topics/services, plus `web_video_server` (or MJPEG) for the camera; static frontend (plain HTML/JS or small React app) served by the Pi. Alternative: a small FastAPI/Flask backend that bridges to ROS. Tour spots stored as a YAML/JSON file on the Pi.
- **Additional features under consideration** (not yet confirmed by user):
  - **Start / pause / stop tour** and an **emergency-stop button** in the UI; manual teleop (joystick) for mapping and recovery.
  - **Tour management**: multiple named tours, reorder spots, schedule tours, loop mode.
  - **Robot speed/volume settings** (max speed cap ~0.3 m/s, TTS volume/voice) editable from the UI.
  - **Set initial pose** on the map when AMCL is lost; "re-map" workflow using slam_toolbox.
  - **Authentication** (password) — the robot moves around people, so the UI must not be open to anyone on the network. Do not expose it to the internet.
  - Logs/diagnostics viewer, firmware/software update, and remote reboot/shutdown of the Pi.
  - Visitor-facing display or QR code (language choice, tour info) — optional.

## Power
- POD S3 board monitors a **3S Li-ion/LiPo pack (~11.1-12.6 V nominal)** via a resistive divider (R1=75 kΩ top, R2=19.5 kΩ bottom, ratio 4.846) into an ADC pin (GPIO6, 12-bit, ADC_11db) — comfortably covers the 0-12.6 V range within the ESP32-S3's 3.3 V ADC.
- **OPEN**: confirm whether this is the SAME pack that drives the motors (via Cytron MDD10A) or a separate logic-only battery for the POD board. If separate packs, MAIN S3 may need its own voltage-divider monitoring too.

## Build status (as of 2026-11-05)
- **Physical build has not started** — no chassis, drivetrain, or wiring assembled yet. Firmware is being prototyped on the bench ahead of the mechanical build.
- Repo layout: `ESP32S3 Code/` (three Arduino sketches below) and `raspberrypi4/pod_serial_reader/` (ROS 2 `ament_python` package, Pi side). The micro-ROS sketch has been removed from the repo.
- Original POD S3 sketch (`ESP32S3 Code/ESP32S3_SensorBoard_U8/ESP32S3_SensorBoard_U8.ino`) is plain Arduino with human-readable `Serial.print` debug output only (battery voltage, BNO055 Euler angles, 4 ToF ranges, every 50 ms). Kept as the bench-test version; not for the Pi parser.
- **POD S3 production sketch**: `ESP32S3 Code/ESP32S3_SensorBoard_U8_CSV/ESP32S3_SensorBoard_U8_CSV.ino` — same sensors/pins as the U8 sketch, but serial output (115200 baud, every 50 ms) is machine-readable CSV read by the Pi-side Python node. Mux chip is PCA9548A (addr 0x70). ToF reads: only RangeStatus 4/5 or >=8190 mm is reported as out of range (`-1`); `TOF_STRICT_STATUS 1` restores the strict `!= 0` filter. Sensors start in LONG_RANGE profile. ToFs are presence-checked every read (2 missed pings -> reported as `-2`) and re-initialised every 1 s while down, so unplug/replug recovers without a reboot (`# ToFn lost` / `# ToFn reconnected` comment lines).
  - Lines starting with `#` are comments/status (boot messages, errors, header) — parser must skip them.
  - Header (printed once after boot): `#CSV,ms,batt_v,heading_deg,roll_deg,pitch_deg,tof0_mm,tof1_mm,tof2_mm,tof3_mm,gyro_x_rads,gyro_y_rads,gyro_z_rads,lin_acc_x_ms2,lin_acc_y_ms2,lin_acc_z_ms2,cal_sys,cal_gyro,cal_accel,cal_mag`
  - Data lines: 19 comma-separated fields, no spaces (first 9 are the original columns; the Pi parser still accepts legacy 9-field lines). `ms` = millis since boot; `batt_v` volts; heading/roll/pitch = BNO055 Euler x/y/z in degrees (`nan` if BNO055 failed init); `tofN_mm` = range in mm, `-1` = out of range, `-2` = sensor failed init. `gyro_*` = rad/s, `lin_acc_*` = m/s² with gravity removed, `cal_*` = BNO055 calibration 0..3 (all `nan`/`-1` if BNO055 failed init).
  - AS5600 pod odometry is not in the CSV yet (same OPEN item below); add columns at the END when implemented so existing parsers keep working.
- Buzzer + motor S3 sketch (separate ESP32-S3 board): `ESP32S3 Code/ESP32S3_LimitBuzzer_CSV/ESP32S3_LimitBuzzer_CSV.ino` — same architecture as the CSV POD sketch (plain Arduino, 115200 baud, CSV every 50 ms, `#` lines = comments). Not ROS, not bench-tested on hardware.
  - Buzzer on **GPIO38** (`BUZZER_PIN` in the sketch; user-confirmed. Comments in the sketch still say 39 — stale). Sink-type (pull-down): buzzer + is always on power, - is on GPIO38 (confirmed by user), so **LOW = loud**, HIGH = silent; driven silent first thing in `setup()`. **OPEN**: on the bench, `BUZZER,1` (pin LOW, firmware confirmed) produced no sound — check buzzer type (active vs passive; passive needs a PWM tone), supply voltage on + and GPIO38 voltage.
  - Header: `#CSV,ms,buzzer,motor1,motor2,motor3,motor4` (6 fields; `buzzer` = 1 if sounding; `motorN` = applied duty).
  - Serial commands IN (newline-terminated): `BUZZER,1` on, `BUZZER,0` off, `BUZZER,<ms>` timed beep (2..60000 ms). Replies are `# OK ...` / `# ERROR ...` lines.
  - **Motor control added (untested on hardware)**: Cytron MDD10A DIR+PWM, pins {DIR,PWM}: Motor1=GPIO 1/2, Motor2=5/4, Motor3=7/6, Motor4=16/15 (user-specified; Motor2/3 inferred from the listed order). PWM 20 kHz 8-bit. Serial command `MOTOR,m1,m2,m3,m4` (each -255..255, sign=direction), `STOP`; firmware watchdog stops motors after 500 ms without a MOTOR command. **Drive encoders now read in the same sketch (untested on hardware)**: x4 quadrature via interrupts on both channels (pins in the Drivetrain section), `ENC_INVERT[]` per encoder. DEBUG ONLY — counts are printed as `# ENC counts a,b,c,d | cps a,b,c,d` comment lines every 200 ms (on by default, `ENC_DEBUG_DEFAULT`) and are NOT in the CSV or sent to the Pi. Serial commands `ENC,1` / `ENC,0` / `ENC,R` (debug on / off / zero counts). Still open-loop; plan is to later use encoder feedback so the robot drives straight.
  - Pi side: `limit_buzzer` node subscribes `cmd_vel` (Twist), does mecanum kinematics (motor order M1=FL, M2=FR, M3=RR, M4=RL — deduced from bench symptoms where angular.z strafed and linear.y rotated; re-verify forward/direction signs on the chassis; params `wheel_geometry_k` 0.4 m, `max_wheel_speed` 0.3 m/s -> full duty, `max_duty`, `motor_signs`, `cmd_timeout_s` 0.5), re-sends at 20 Hz, publishes `main/motor_cmd` and `main/motor_applied` (Int16MultiArray). **OPEN**: verify wheel order/direction signs on the real chassis; duty is a linear guess of speed until encoder feedback exists.
  - Likely the role of the MAIN S3's buzzer; encoders are wired to this same motor-control ESP32 (debug-only reading exists, see above). Pi-side reader: node `limit_buzzer` (name kept) in the `raspberrypi4/pod_serial_reader` package (`limit_buzzer_node.py`) — publishes `main/buzzer_state` (Bool), subscribes `main/buzzer_cmd` (Int32: 0 off, 1 on, 2..60000 ms beep → sends `BUZZER,<n>`). Default port `/dev/ttyACM1`. Run both boards concurrently with `ros2 launch pod_serial_reader both_boards.launch.py pod_port:=... limit_port:=...` (launch file in `raspberrypi4/pod_serial_reader/launch/`; use `/dev/serial/by-id/` paths). Not yet built/tested on a Pi.
- Pi side: `raspberrypi4/pod_serial_reader/` is a ROS 2 `ament_python` package (node `serial_reader`, default port `/dev/ttyACM0`) that reads the POD CSV from the serial port, shows a live terminal table, and publishes `pod/imu` (sensor_msgs/Imu, see below), `pod/battery` (BatteryState), `pod/euler_deg` (Vector3Stamped: heading, roll, pitch in deg), `pod/tof_0..3` (Range, sliding median over `tof_filter_window` samples, -1 treated as +inf so a lone dropout or spike is rejected, frames `tof_N`, FOV 0.44 rad, 0.03–2.0 m). Params: `port`, `baud`, `display`, `display_hz`, `publish`, `tof_filter_window` (5), `imu_frame_id` (imu_link), `imu_yaw_offset_deg`, `imu_orientation_var`, `imu_gyro_var`, `imu_accel_var`. Not yet built/tested on a Pi. Build steps in `raspberrypi4/README.md` (needs `python3-serial`, user in `dialout`).
- **IMU for the EKF (`pod/imu`)**: `serial_reader` builds a `sensor_msgs/Imu` from the extended CSV. Orientation is **yaw only** (roll/pitch zero with 1e6 variance; planar robot): `yaw = -(heading - imu_yaw_offset_deg)` because BNO055 heading is clockwise 0..360 and ROS yaw is CCW, wrapped to ±π. Gyro (rad/s) and gravity-free linear accel (m/s²) pass straight through. Covariances are fixed diagonals from parameters (guesses, tune on real data). Not published while the BNO055 is down (nan fields). Still TODO: a static TF `base_link -> imu_link` once mounted; `bno.begin()` uses NDOF (magnetometer fused) which may give jumpy absolute heading near motors/steel — consider `OPERATION_MODE_IMUPLUS` (relative yaw) and rely on AMCL for absolute heading; the 20 Hz print rate is below the BNO055's 100 Hz. Calibration status is shown in the terminal table but not yet used to gate anything.
- **OPEN**: AS5600 pod reading is NOT yet implemented. `odom1`/`odom2`/`odom3` are declared as plain `INPUT` pins with only a comment — still need PWM duty-cycle capture (`pulseIn` or RMT), angle-unwrapping across the 360->0 wrap, and the one-time I2C boot config to put each AS5600 into PWM mode. Magnet-placement check via the AGC/MD register is also not implemented yet. No odometry topic is published until this lands.
- **OPEN**: MAIN S3 motor/drive-encoder firmware (4 motors DIR+PWM, 4 quadrature encoders) has not been started. It should follow the same CSV-over-serial pattern (CSV out for encoders, commands in for motor drive), with a Pi-side node for it.
- **OPEN**: Nothing is built/tested end-to-end on a Pi yet (both serial nodes and the launch file).

