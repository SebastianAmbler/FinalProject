# raspberrypi4

ROS 2 (Jazzy) workspace sources for the Raspberry Pi 4.

## pod_serial_reader
Reads the CSV stream from the POD S3 (`ESP32S3_SensorBoard_U8_CSV.ino`), shows a live table in the terminal, and publishes `pod/battery`, `pod/euler_deg` (Vector3Stamped: heading, roll, pitch in deg) and `pod/tof_0..3` (Range).

```bash
sudo apt install python3-serial
mkdir -p ~/ros2_ws/src && cp -r pod_serial_reader ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select pod_serial_reader
source install/setup.bash
ros2 run pod_serial_reader serial_reader --ros-args -p port:=/dev/ttyACM0
```

Parameters: `port` (/dev/ttyACM0), `baud` (115200), `display` (true), `display_hz` (5.0), `publish` (true).
### limit_buzzer (same package)
Reads the CSV stream from the limit-switch/buzzer S3 (`ESP32S3_LimitBuzzer_CSV.ino`), shows a live table, publishes `main/limit_switch_0..3` (std_msgs/Bool, True = pressed) and `main/buzzer_state` (Bool), and subscribes to `main/buzzer_cmd` (std_msgs/Int32: `0` off, `1` on, `2..60000` beep for that many ms).

```bash
ros2 run pod_serial_reader limit_buzzer --ros-args -p port:=/dev/ttyACM1
ros2 topic pub --once /main/buzzer_cmd std_msgs/msg/Int32 "{data: 500}"
```

Parameters: `port` (/dev/ttyACM1), `baud` (115200), `display`, `display_hz`, `publish`, `pressed_level` (0 = pin reads LOW when pressed, matching `INPUT_PULLUP`).
### Both boards at once
```bash
ros2 launch pod_serial_reader both_boards.launch.py pod_port:=/dev/serial/by-id/<pod> limit_port:=/dev/serial/by-id/<limit>
```
Starts `serial_reader` and `limit_buzzer` as separate processes, each reading its own port concurrently. Only the POD terminal table is on by default (both would redraw the same screen); use `show_limit:=true show_pod:=false` to swap, or `ros2 topic echo` the `main/*` topics.

Two boards means two ports: use `/dev/serial/by-id/...` paths so they don't swap between reboots.

Add your user to `dialout` for serial access: `sudo usermod -aG dialout $USER`.
