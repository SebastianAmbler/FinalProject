"""Read the limit-switch/buzzer S3 CSV stream, publish it, and drive the buzzer.

Expected line format (see ESP32S3_LimitBuzzer_CSV.ino):
  ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer[,motor1..motor4]
(legacy 6-field lines are still accepted). Lines starting with '#' are comments and are skipped.

Topics:
  main/limit_switch_0..3  std_msgs/Bool   True = pressed (see 'pressed_level')
  main/buzzer_state       std_msgs/Bool   True = buzzer sounding
  main/buzzer_cmd         std_msgs/Int32  (subscribed) 0 = off, 1 = on,
                                          2..60000 = beep for that many ms
  cmd_vel                 geometry_msgs/Twist (subscribed) body velocity
                          (x forward, y left, z yaw CCW). Converted with
                          mecanum kinematics to "MOTOR,m1,m2,m3,m4" (-255..255).
  main/motor_cmd          std_msgs/Int16MultiArray, 4 values (published) what
                          was sent;  main/motor_applied  (published) what the
                          ESP32 reports it is outputting.

The node re-sends the last command at 'cmd_rate_hz' and zeroes it if no
cmd_vel arrives within 'cmd_timeout_s'. The ESP32 also has its own watchdog.
Motor order: 1 = front-left, 2 = front-right, 3 = rear-left, 4 = rear-right
(ASSUMED - set 'motor_invert' / adjust to match the real chassis).
"""
import time

import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16MultiArray, Int32

N_FIELDS = 6
N_FIELDS_MOTOR = 10
N_MOTORS = 4
N_SWITCHES = 4


def parse_line(line):
    """Return a dict for a valid CSV data line, else None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    parts = line.split(',')
    if len(parts) not in (N_FIELDS, N_FIELDS_MOTOR):
        return None
    try:
        v = [int(p) for p in parts]
    except ValueError:
        return None
    return {'ms': v[0], 'raw': v[1:5], 'buzzer': v[5] == 1,
            'motors': v[6:10] if len(v) == N_FIELDS_MOTOR else None}


def mecanum_wheel_speeds(vx, vy, wz, k):
    """Body twist -> wheel linear speeds (m/s) in MOTOR order
    [M1=FL, M2=FR, M3=RR, M4=RL] (bench-verified: M3/M4 are the rear
    right/left, not left/right).

    Standard X-roller mecanum layout, ROS REP-103: positive vx forward,
    vy left, wz CCW. k = lx + ly (half wheelbase + half track, metres).
    Only vy produces strafing; only wz produces rotation.
    """
    # Textbook rotation sign (+wz = CCW = left turn). An earlier inverted
    # version turned left/right swapped on the bench, so it was reverted.
    fl = vx - vy - k * wz
    fr = vx + vy + k * wz
    rl = vx + vy - k * wz
    rr = vx - vy + k * wz
    return [fl, fr, rr, rl]


class LimitBuzzerNode(Node):
    def __init__(self):
        super().__init__('limit_buzzer_reader')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('display', True)
        self.declare_parameter('display_hz', 5.0)
        self.declare_parameter('publish', True)
        # Raw pin level that means "pressed". Board uses INPUT_PULLUP, so a
        # switch to GND reads 0 when pressed.
        self.declare_parameter('pressed_level', 0)
        # --- motor / cmd_vel ---
        # Wheel linear speed that maps to full duty (255). Also acts as the
        # speed cap: wheel speeds are scaled down together if any exceeds it.
        self.declare_parameter('max_wheel_speed', 0.3)       # m/s
        # PWM duty used when a wheel is at max_wheel_speed (also the hard cap).
        # 255 would be the motor's full ~2 m/s, so keep this low until encoder
        # feedback exists; raise gradually to tune.
        self.declare_parameter('max_duty', 60)               # cap PWM (0..255)
        # lx + ly: half wheelbase + half track. ~0.2+0.2 for a 50 cm footprint;
        # measure on the real chassis.
        self.declare_parameter('wheel_geometry_k', 0.4)      # m
        # +1/-1 per motor [M1 FL, M2 FR, M3 RL, M4 RR]; -1 for motors mounted
        # mirrored so positive duty drives the robot forward.
        self.declare_parameter('motor_signs', [1, 1, 1, 1])
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('cmd_rate_hz', 20.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.display = self.get_parameter('display').value
        self.publish_topics = self.get_parameter('publish').value
        self.pressed_level = int(self.get_parameter('pressed_level').value)

        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.max_duty = int(self.get_parameter('max_duty').value)
        self.k = float(self.get_parameter('wheel_geometry_k').value)
        self.motor_signs = [int(x) for x in self.get_parameter('motor_signs').value]
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        self.twist = (0.0, 0.0, 0.0)
        self.last_twist_time = None
        self.last_motor_cmd = [0] * N_MOTORS

        self.ser = None
        self.latest = None
        self.bad_lines = 0
        self.good_lines = 0

        self.pub_sw = [self.create_publisher(Bool, f'main/limit_switch_{i}', 10)
                       for i in range(N_SWITCHES)]
        self.pub_buzzer = self.create_publisher(Bool, 'main/buzzer_state', 10)
        self.create_subscription(Int32, 'main/buzzer_cmd', self.on_buzzer_cmd, 10)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.pub_motor_cmd = self.create_publisher(Int16MultiArray, 'main/motor_cmd', 10)
        self.pub_motor_applied = self.create_publisher(Int16MultiArray, 'main/motor_applied', 10)
        self.create_timer(1.0 / self.get_parameter('cmd_rate_hz').value,
                          self.send_motors)

        self.create_timer(0.01, self.read_serial)
        if self.display:
            self.create_timer(1.0 / self.get_parameter('display_hz').value,
                              self.show)

    # ---------- serial ----------
    def open_serial(self):
        # Retry at most every 2 s: hammering open() every 10 ms made the
        # board/VM USB passthrough flap.
        now = time.monotonic()
        if now < getattr(self, '_next_open', 0.0):
            return
        self._next_open = now + 2.0
        try:
            # Keep DTR/RTS low *before* opening: asserting them resets the
            # ESP32-S3 (USB re-enumerates and the port vanishes in the VM).
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baud
            ser.timeout = 0
            ser.dtr = False
            ser.rts = False
            ser.open()
            self.ser = ser
            self.get_logger().info(f'Opened {self.port} @ {self.baud}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.port}: {e}',
                                   throttle_duration_sec=5.0)

    def read_serial(self):
        if self.ser is None:
            self.open_serial()
            return
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline().decode('ascii', errors='ignore')
                if raw.strip().startswith('#'):
                    self.get_logger().info(raw.strip())
                    continue
                data = parse_line(raw)
                if data is None:
                    if raw.strip():
                        self.bad_lines += 1
                    continue
                self.good_lines += 1
                self.latest = data
                if self.publish_topics:
                    self.publish(data)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial lost: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ---------- buzzer command ----------
    def on_buzzer_cmd(self, msg):
        v = msg.data
        if v < 0 or v > 60000:
            self.get_logger().warn(f'buzzer_cmd {v} out of range 0..60000')
            return
        if self.ser is None:
            self.get_logger().warn('buzzer_cmd ignored: serial not connected')
            return
        try:
            self.ser.write(f'BUZZER,{v}\n'.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # ---------- cmd_vel -> motors ----------
    def on_cmd_vel(self, msg):
        self.twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_twist_time = self.get_clock().now()

    def twist_to_duty(self, vx, vy, wz):
        speeds = mecanum_wheel_speeds(vx, vy, wz, self.k)
        peak = max(abs(s) for s in speeds)
        if peak > self.max_wheel_speed:       # preserve direction, cap speed
            speeds = [s * self.max_wheel_speed / peak for s in speeds]
        duty = []
        for s, sign in zip(speeds, self.motor_signs):
            d = int(round(s / self.max_wheel_speed * self.max_duty)) * sign
            duty.append(max(-255, min(255, d)))
        return duty

    def send_motors(self):
        fresh = (self.last_twist_time is not None and
                 (self.get_clock().now() - self.last_twist_time).nanoseconds * 1e-9
                 <= self.cmd_timeout)
        duty = self.twist_to_duty(*self.twist) if fresh else [0] * N_MOTORS
        self.last_motor_cmd = duty
        self.pub_motor_cmd.publish(Int16MultiArray(data=duty))
        if self.ser is None:
            return
        try:
            cmd = 'MOTOR,%d,%d,%d,%d' % tuple(duty)
            self.ser.write((cmd + chr(10)).encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # ---------- ROS output ----------
    def is_pressed(self, raw):
        return raw == self.pressed_level

    def publish(self, d):
        for i, raw in enumerate(d['raw']):
            self.pub_sw[i].publish(Bool(data=self.is_pressed(raw)))
        self.pub_buzzer.publish(Bool(data=d['buzzer']))
        if d['motors'] is not None:
            self.pub_motor_applied.publish(Int16MultiArray(data=d['motors']))

    # ---------- terminal display ----------
    def show(self):
        d = self.latest
        status = 'connected' if self.ser else f'NO PORT ({self.port})'
        lines = [f'=== Limit/Buzzer S3 serial: {status} | ok={self.good_lines} '
                 f'bad={self.bad_lines} ===']
        if d is None:
            lines.append('waiting for data...')
        else:
            lines.append(f"t        : {d['ms'] / 1000.0:10.2f} s")
            for i, raw in enumerate(d['raw']):
                state = 'PRESSED' if self.is_pressed(raw) else 'open'
                lines.append(f'limit {i}  : {state:8s} (raw {raw})')
            lines.append('buzzer   : ' + ('ON' if d['buzzer'] else 'off'))
            lines.append(f'motor cmd: {self.last_motor_cmd}   applied: {d["motors"]}')
        print('\033[2J\033[H' + '\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = LimitBuzzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            try:
                node.ser.write(b'STOP' + bytes([10]))
            except Exception:
                pass
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
