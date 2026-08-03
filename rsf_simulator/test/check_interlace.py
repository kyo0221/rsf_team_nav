import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

PITCH = 0.10471975511965977
PERIOD = 0.05
SETTLE_FRAMES = 40
PHASE_MIN_SAMPLES = 20
COLLECT_TIMEOUT = 180.0


class Checker(Node):
    def __init__(self, interlace):
        super().__init__('interlace_checker')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.interlace = interlace
        self.deskewed = []
        self.raw = {}
        self.phase_samples = []
        self.create_subscription(
            PointCloud2, '/rsf/hokuyo_cloud2', self.deskewed_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/rsf/hokuyo3d/points', self.raw_cb, qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, '/rsf/dynamic_pose_info', self.tf_cb, 10)

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

    def tf_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t < SETTLE_FRAMES * PERIOD:
            return
        phase = t - math.floor(t / PERIOD) * PERIOD
        if phase < 0.002 or phase > 0.048:
            return
        for tr in msg.transforms:
            if 'rsf_hokuyo3d' not in tr.child_frame_id:
                continue
            q = tr.transform.rotation
            yaw = 2.0 * math.atan2(q.z, q.w)
            self.phase_samples.append((t, yaw))


def offset_of(pts):
    residuals = sorted(
        math.fmod(math.atan2(y, x) + 1.8326 + 10 * PITCH, PITCH) for x, y in pts)
    return residuals[len(residuals) // 2]


def beams_differ(a, b):
    return sum(1 for r1, r2 in zip(a, b) if abs(r1 - r2) > 0.05)


def median(values):
    s = sorted(values)
    return s[len(s) // 2]


def normalize_centered(x, period):
    r = math.fmod(x, period)
    if r < -period / 2:
        r += period
    elif r >= period / 2:
        r -= period
    return r


def expected_k(t, interlace):
    return int(t // PERIOD) % interlace


def check_phase(node, interlace, ok):
    samples = node.phase_samples
    if len(samples) < PHASE_MIN_SAMPLES:
        print(f'FAIL phase check insufficient samples ({len(samples)} < {PHASE_MIN_SAMPLES})')
        return False
    k0_yaws = [yaw for t, yaw in samples if expected_k(t, interlace) == 0]
    if not k0_yaws:
        print('FAIL phase check no k=0 samples to establish baseline')
        return False
    baseline = median(k0_yaws)
    good = 0
    for t, yaw in samples:
        k = expected_k(t, interlace)
        expected_angle = k * PITCH / interlace
        measured_rel = normalize_centered(yaw - baseline, PITCH)
        diff = abs(measured_rel - expected_angle)
        error = min(diff, PITCH - diff)
        if error < 0.003:
            good += 1
    ratio = good / len(samples)
    print(f'phase check {good}/{len(samples)} samples within tolerance ({ratio:.3f})')
    if ratio < 0.95:
        print(f'FAIL phase check ratio {ratio:.3f} < 0.95')
        return False
    return ok


def check_raw_pairs(node, interlace, ok):
    keys = sorted(k for k in node.raw if k >= SETTLE_FRAMES)
    same_evaluated = 0
    same_passed = 0
    cross_evaluated = 0
    cross_passed = 0
    for k in keys:
        a = node.raw[k]
        if k + interlace in node.raw:
            b = node.raw[k + interlace]
            if len(a) == len(b):
                same_evaluated += 1
                if beams_differ(a, b) <= len(a) * 0.01:
                    same_passed += 1
        if k + 1 in node.raw:
            b = node.raw[k + 1]
            if len(a) == len(b):
                cross_evaluated += 1
                if beams_differ(a, b) > len(a) * 0.01:
                    cross_passed += 1
    print(f'same-phase pairs {same_passed}/{same_evaluated}, '
          f'cross-phase pairs {cross_passed}/{cross_evaluated}')
    if same_evaluated < 10:
        print(f'FAIL insufficient same-phase pairs evaluated ({same_evaluated} < 10)')
        ok = False
    elif same_passed / same_evaluated < 0.90:
        print(f'FAIL same-phase pairs pass rate {same_passed}/{same_evaluated} < 90%')
        ok = False
    if cross_evaluated < 10:
        print(f'FAIL insufficient cross-phase pairs evaluated ({cross_evaluated} < 10)')
        ok = False
    elif cross_passed / cross_evaluated < 0.90:
        print(f'FAIL cross-phase pairs pass rate {cross_passed}/{cross_evaluated} < 90%')
        ok = False
    return ok


def main():
    interlace = int(sys.argv[1])
    rclpy.init()
    node = Checker(interlace)
    needed = SETTLE_FRAMES + 3 * max(interlace, 2)
    deadline = time.monotonic() + COLLECT_TIMEOUT
    while (len(node.deskewed) < needed or len(node.raw) < needed
           or len(node.phase_samples) < PHASE_MIN_SAMPLES):
        if time.monotonic() > deadline:
            print('FAIL timeout waiting for messages')
            rclpy.shutdown()
            sys.exit(1)
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
        ok = check_raw_pairs(node, interlace, ok)
    ok = check_phase(node, interlace, ok)
    print('PASS' if ok else 'FAIL')
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
