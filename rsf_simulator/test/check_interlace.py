import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

PITCH = 0.10471975511965977
PERIOD = 0.05
SETTLE_FRAMES = 40


class Checker(Node):
    def __init__(self, interlace):
        super().__init__('interlace_checker')
        self.interlace = interlace
        self.deskewed = []
        self.raw = {}
        self.create_subscription(
            PointCloud2, '/rsf/hokuyo_cloud2', self.deskewed_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/rsf/hokuyo3d/points', self.raw_cb, qos_profile_sensor_data)

    def stamp_of(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def deskewed_cb(self, msg):
        pts = [(float(p[0]), float(p[1]))
               for p in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=False)
               if math.isfinite(p[0]) and math.isfinite(p[1])]
        self.deskewed.append((self.stamp_of(msg), msg.width * msg.height, pts))

    def raw_cb(self, msg):
        ranges = [math.hypot(float(p[0]), float(p[1])) if math.isfinite(p[0]) else -1.0
                  for p in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=False)]
        self.raw[round(self.stamp_of(msg) / PERIOD)] = ranges


def offset_of(pts):
    residuals = sorted(
        math.fmod(math.atan2(y, x) + 1.8326 + 10 * PITCH, PITCH) for x, y in pts)
    return residuals[len(residuals) // 2]


def beams_differ(a, b):
    return sum(1 for r1, r2 in zip(a, b) if abs(r1 - r2) > 0.05)


def main():
    interlace = int(sys.argv[1])
    rclpy.init()
    node = Checker(interlace)
    needed = SETTLE_FRAMES + 3 * max(interlace, 2)
    while len(node.deskewed) < needed or len(node.raw) < needed:
        rclpy.spin_once(node, timeout_sec=2.0)
    ok = True
    for t, count, pts in node.deskewed[SETTLE_FRAMES:]:
        if count != 2664:
            print(f'FAIL points {count} != 2664 at t={t:.3f}')
            ok = False
        k = round(t / PERIOD) % interlace
        expected = k * PITCH / interlace
        measured = offset_of(pts)
        error = min(abs(measured - expected), PITCH - abs(measured - expected))
        if error > 0.002:
            print(f'FAIL offset t={t:.3f} k={k} expected={expected:.4f} measured={measured:.4f}')
            ok = False
    if interlace > 1:
        keys = sorted(k for k in node.raw if k >= SETTLE_FRAMES)
        base = next(k for k in keys if k + 1 in node.raw and k + interlace in node.raw)
        beam_count = len(node.raw[base])
        same_phase = beams_differ(node.raw[base], node.raw[base + interlace])
        cross_phase = beams_differ(node.raw[base], node.raw[base + 1])
        if same_phase > beam_count * 0.01:
            print(f'FAIL same-phase frames differ on {same_phase}/{beam_count} beams')
            ok = False
        if cross_phase <= beam_count * 0.01:
            print(f'FAIL cross-phase frames identical ({cross_phase}/{beam_count} beams differ): joint not rotating')
            ok = False
    print('PASS' if ok else 'FAIL')
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
