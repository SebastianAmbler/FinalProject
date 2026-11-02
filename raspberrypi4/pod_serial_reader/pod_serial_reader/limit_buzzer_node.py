"""Read the limit-switch/buzzer S3 CSV stream, publish it, and drive the buzzer.

Expected line format (see ESP32S3_LimitBuzzer_CSV.ino):
  ms,limit_switch0,limit_switch1,limit_switch2,limit_switch3,buzzer
Lines starting with '#' are comments and are skipped.

Topics:
  main/limit_switch_0..3  std_msgs/Bool   True = pressed (see 'pressed_level')
  main/buzzer_state       std_msgs/Bool   True = buzzer sounding
  main/buzzer_cmd         std_msgs/Int32  (subscribed) 0 = off, 1 = on,
                                          2..60000 = beep for that many ms
"""
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

N_FIELDS = 6
N_SWITCHES = 4


def parse_line(line):
    """Return a dict for a valid CSV data line, else None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    parts = line.split(',')
    if len(parts) != N_FIELDS:
        return None
    try:
        v = [int(p) for p in parts]
    except ValueError:
        return None
    return {'ms': v[0], 'raw': v[1:5], 'buzzer': v[5] == 1}


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

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.display = self.get_parameter('display').value
        self.publish_topics = self.get_parameter('publish').value
        self.pressed_level = int(self.get_parameter('pressed_level').value)

        self.ser = None
        self.latest = None
        self.bad_lines = 0
        self.good_lines = 0

        self.pub_sw = [self.create_publisher(Bool, f'main/limit_switch_{i}', 10)
                       for i in range(N_SWITCHES)]
        self.pub_buzzer = self.create_publisher(Bool, 'main/buzzer_state', 10)
        self.create_subscription(Int32, 'main/buzzer_cmd', self.on_buzzer_cmd, 10)

        self.create_timer(0.01, self.read_serial)
        if self.display:
            self.create_timer(1.0 / self.get_parameter('display_hz').value,
                              self.show)

    # ---------- serial ----------
    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
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

    # ---------- ROS output ----------
    def is_pressed(self, raw):
        return raw == self.pressed_level

    def publish(self, d):
        for i, raw in enumerate(d['raw']):
            self.pub_sw[i].publish(Bool(data=self.is_pressed(raw)))
        self.pub_buzzer.publish(Bool(data=d['buzzer']))

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
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
