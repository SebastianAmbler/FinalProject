"""Read the POD S3 CSV serial stream, display it, and publish it to ROS 2.

Expected line format (see ESP32S3_SensorBoard_U8_CSV.ino):
  ms,batt_v,heading_deg,roll_deg,pitch_deg,tof0_mm,tof1_mm,tof2_mm,tof3_mm,
  gyro_x,gyro_y,gyro_z (rad/s),lin_acc_x,lin_acc_y,lin_acc_z (m/s^2, no gravity),
  cal_sys,cal_gyro,cal_accel,cal_mag (0..3, -1 = BNO055 not initialised)
Lines starting with '#' are comments and are skipped. Legacy 9-field lines
(no IMU columns) are still accepted; pod/imu is simply not published for them.

pod/imu (sensor_msgs/Imu) is built here for robot_localization:
  - orientation: yaw only (roll = pitch = 0, planar robot). BNO055 heading is
    clockwise-positive 0..360 deg; ROS yaw is CCW-positive, so
    yaw = -(heading - imu_yaw_offset_deg), wrapped to +-pi.
  - angular_velocity / linear_acceleration straight from the CSV (rad/s, m/s^2).
  - covariances come from parameters (the BNO055 does not report them).
"""
import math
import time
from collections import deque

import rclpy
import serial
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, Range

N_FIELDS_LEGACY = 9
N_FIELDS = 19
TOF_OUT_OF_RANGE = -1
TOF_NOT_INIT = -2


def parse_line(line):
    """Return a dict for a valid CSV data line, else None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    parts = line.split(',')
    if len(parts) not in (N_FIELDS_LEGACY, N_FIELDS):
        return None
    try:
        v = [float(p) for p in parts]  # float('nan') parses fine
    except ValueError:
        return None
    d = {
        'ms': int(v[0]),
        'batt_v': v[1],
        'euler': (v[2], v[3], v[4]),  # heading, roll, pitch (deg)
        'tof_mm': [int(x) for x in v[5:9]],
        'gyro': None,
        'lin_acc': None,
        'cal': None,
    }
    if len(parts) == N_FIELDS:
        d['gyro'] = (v[9], v[10], v[11])       # rad/s
        d['lin_acc'] = (v[12], v[13], v[14])   # m/s^2
        d['cal'] = tuple(int(x) for x in v[15:19])  # sys, gyro, accel, mag
    return d


class TofMedianFilter:
    """Sliding-window median for one ToF channel (values in mm).

    Out-of-range (-1) counts as +inf so it sorts last; a single stray -1 or a
    single wild spike inside the window is rejected, while a sustained change
    passes through after about window/2 samples. For an even window the lower
    middle value is used so the output is always a real measurement.
    -2 (sensor not initialised) bypasses the filter and clears the window.
    """

    def __init__(self, window):
        self.buf = deque(maxlen=max(1, int(window)))

    def update(self, mm):
        if mm == TOF_NOT_INIT:
            self.buf.clear()
            return mm
        self.buf.append(math.inf if mm == TOF_OUT_OF_RANGE else mm)
        med = sorted(self.buf)[(len(self.buf) - 1) // 2]
        return TOF_OUT_OF_RANGE if math.isinf(med) else int(med)


class SerialReader(Node):
    def __init__(self):
        super().__init__('pod_serial_reader')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('display', True)       # live terminal table
        self.declare_parameter('display_hz', 5.0)
        self.declare_parameter('publish', True)
        # Median window for ToF readings in samples (20 Hz stream); 1 = off.
        self.declare_parameter('tof_filter_window', 5)
        self.declare_parameter('imu_frame_id', 'imu_link')
        # Heading value (deg) that should count as yaw = 0 (mounting offset).
        self.declare_parameter('imu_yaw_offset_deg', 0.0)
        # Diagonal covariances (variance). Tune against real data.
        self.declare_parameter('imu_orientation_var', 0.01)   # rad^2 (yaw)
        self.declare_parameter('imu_gyro_var', 0.0004)        # (rad/s)^2
        self.declare_parameter('imu_accel_var', 0.04)         # (m/s^2)^2

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.display = self.get_parameter('display').value
        self.publish_topics = self.get_parameter('publish').value

        window = self.get_parameter('tof_filter_window').value
        self.tof_filters = [TofMedianFilter(window) for _ in range(4)]

        self.ser = None
        self.latest = None
        self.bad_lines = 0
        self.good_lines = 0

        self.pub_batt = self.create_publisher(BatteryState, 'pod/battery', 10)
        self.pub_euler = self.create_publisher(Vector3Stamped, 'pod/euler_deg', 10)
        self.pub_imu = self.create_publisher(Imu, 'pod/imu', 10)
        self.pub_tof = [self.create_publisher(Range, f'pod/tof_{i}', 10)
                        for i in range(4)]

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
                data['tof_mm_raw'] = data['tof_mm']
                data['tof_mm'] = [f.update(mm) for f, mm in
                                  zip(self.tof_filters, data['tof_mm_raw'])]
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

        if d['gyro'] is not None:
            imu = self.make_imu(d, stamp)
            if imu is not None:
                self.pub_imu.publish(imu)

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

    def make_imu(self, d, stamp):
        """Build sensor_msgs/Imu from a 19-field sample, or None if BNO055 is down."""
        heading = d['euler'][0]
        vals = list(d['gyro']) + list(d['lin_acc'])
        if math.isnan(heading) or any(math.isnan(x) for x in vals):
            return None
        gp = self.get_parameter
        off = gp('imu_yaw_offset_deg').value
        yaw = -math.radians(heading - off)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))   # wrap to +-pi

        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = gp('imu_frame_id').value
        # yaw-only quaternion
        m.orientation.z = math.sin(yaw / 2.0)
        m.orientation.w = math.cos(yaw / 2.0)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = d['gyro']
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = d['lin_acc']

        ov = gp('imu_orientation_var').value
        gv = gp('imu_gyro_var').value
        av = gp('imu_accel_var').value
        # roll/pitch are not provided -> huge variance so nothing trusts them
        m.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, ov]
        m.angular_velocity_covariance = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        m.linear_acceleration_covariance = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        return m

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
            if d['gyro'] is not None:
                gx, gy, gz = d['gyro']
                ax, ay, az = d['lin_acc']
                cs, cg, ca, cm = d['cal']
                lines += [
                    f'gyro     : {gx:8.3f} {gy:8.3f} {gz:8.3f} rad/s',
                    f'lin acc  : {ax:8.3f} {ay:8.3f} {az:8.3f} m/s^2',
                    f'IMU cal  : sys={cs} gyro={cg} accel={ca} mag={cm} (3 = good)',
                ]
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
