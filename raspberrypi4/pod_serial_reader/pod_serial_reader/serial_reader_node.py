"""Read the POD S3 CSV serial stream, display it, and publish it to ROS 2.

Expected line format (see ESP32S3_SensorBoard_U8_CSV.ino):
  ms,batt_v,heading_deg,roll_deg,pitch_deg,tof0_mm,tof1_mm,tof2_mm,tof3_mm
Lines starting with '#' are comments and are skipped.
"""
import math

import rclpy
import serial
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Range

N_FIELDS = 9
TOF_OUT_OF_RANGE = -1
TOF_NOT_INIT = -2


def parse_line(line):
    """Return a dict for a valid CSV data line, else None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    parts = line.split(',')
    if len(parts) != N_FIELDS:
        return None
    try:
        v = [float(p) for p in parts]  # float('nan') parses fine
    except ValueError:
        return None
    return {
        'ms': int(v[0]),
        'batt_v': v[1],
        'euler': (v[2], v[3], v[4]),  # heading, roll, pitch (deg)
        'tof_mm': [int(x) for x in v[5:9]],
    }


class SerialReader(Node):
    def __init__(self):
        super().__init__('pod_serial_reader')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('display', True)       # live terminal table
        self.declare_parameter('display_hz', 5.0)
        self.declare_parameter('publish', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.display = self.get_parameter('display').value
        self.publish_topics = self.get_parameter('publish').value

        self.ser = None
        self.latest = None
        self.bad_lines = 0
        self.good_lines = 0

        self.pub_batt = self.create_publisher(BatteryState, 'pod/battery', 10)
        self.pub_euler = self.create_publisher(Vector3Stamped, 'pod/euler_deg', 10)
        self.pub_tof = [self.create_publisher(Range, f'pod/tof_{i}', 10)
                        for i in range(4)]

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

    # ---------- ROS output ----------
    def publish(self, d):
        stamp = self.get_clock().now().to_msg()

        b = BatteryState()
        b.header.stamp = stamp
        b.voltage = d['batt_v']
        b.current = b.charge = b.capacity = b.design_capacity = float('nan')
        b.percentage = float('nan')
        b.present = True
        self.pub_batt.publish(b)

        e = Vector3Stamped()
        e.header.stamp = stamp
        e.vector.x, e.vector.y, e.vector.z = d['euler']
        self.pub_euler.publish(e)

        for i, mm in enumerate(d['tof_mm']):
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = f'tof_{i}'
            r.radiation_type = Range.INFRARED
            r.field_of_view = 0.44  # ~25 deg, VL53L0X
            r.min_range = 0.03
            r.max_range = 2.0
            if mm == TOF_OUT_OF_RANGE:
                r.range = float('inf')   # REP 117
            elif mm == TOF_NOT_INIT:
                r.range = float('nan')
            else:
                r.range = mm / 1000.0
            self.pub_tof[i].publish(r)

    # ---------- terminal display ----------
    @staticmethod
    def fmt_tof(mm):
        if mm == TOF_OUT_OF_RANGE:
            return ' out of range'
        if mm == TOF_NOT_INIT:
            return '   not init  '
        return f'{mm:6d} mm    '

    def show(self):
        d = self.latest
        status = 'connected' if self.ser else f'NO PORT ({self.port})'
        lines = [f'=== POD S3 serial: {status} | ok={self.good_lines} '
                 f'bad={self.bad_lines} ===']
        if d is None:
            lines.append('waiting for data...')
        else:
            h, r, p = d['euler']
            lines += [
                f"t        : {d['ms'] / 1000.0:10.2f} s",
                f"battery  : {d['batt_v']:10.3f} V",
                'heading  : ' + ('       nan' if math.isnan(h) else f'{h:10.2f} deg'),
                'roll     : ' + ('       nan' if math.isnan(r) else f'{r:10.2f} deg'),
                'pitch    : ' + ('       nan' if math.isnan(p) else f'{p:10.2f} deg'),
            ]
            lines += [f'ToF{i}     : {self.fmt_tof(mm)}'
                      for i, mm in enumerate(d['tof_mm'])]
        # clear screen + home, then draw
        print('\033[2J\033[H' + '\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
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
