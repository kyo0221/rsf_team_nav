#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from waypoint_plan import Leg, load_waypoints, speed_limit_for, split_legs


def yaw_to_quaternion(yaw):
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class State(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    HOLD = 'hold'
    PAUSED = 'paused'
    FINISHED = 'finished'


class WaypointNavigator(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='waypoint_navigator')
        # emcl2 は lifecycle ノードではないため waitUntilNav2Active() は呼ばない(デフォルトの amcl/get_state 待ちで無限ループする)
        self.declare_parameter('waypoints_file', '')

        self.waypoints, self.loop = load_waypoints(self.get_parameter('waypoints_file').value)
        self.legs = split_legs(self.waypoints)

        self.state = State.IDLE
        self.leg_index = 0
        self.current_leg = None
        self.paused_leg = None
        self.pause_requested = False
        self.pending_start = False
        self.pending_pause = False
        self.pending_resume = False

        self.speed_limit_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        # RViz を後から起動しても見えるように latch する。
        # /waypoints は nav2_rviz_plugins の Navigation 2 パネルが
        # MarkerArray で使うので、ノード名前空間の下に置く
        self.waypoints_pub = self.create_publisher(
            PoseArray, '~/waypoints',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/pause', self.on_pause)
        self.create_service(Trigger, '~/resume', self.on_resume)

        self.waypoints_pub.publish(self.waypoints_message())

    def waypoints_message(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            qz, qw = yaw_to_quaternion(wp['yaw'])
            pose = Pose()
            pose.position.x = wp['x']
            pose.position.y = wp['y']
            pose.orientation.z = qz
            pose.orientation.w = qw
            msg.poses.append(pose)
        return msg

    # --- サービス: 状態検査 + フラグ設定 + 即時応答のみ。navigator のメソッドはここから呼ばない(ネスト spin 防止) ---

    def on_start(self, request, response):
        if self.state not in (State.IDLE, State.FINISHED):
            response.success = False
            response.message = 'already running'
            return response
        if not self.follow_waypoints_client.server_is_ready():
            response.success = False
            response.message = 'follow_waypoints action server not available'
            return response
        self.pending_start = True
        response.success = True
        return response

    def on_pause(self, request, response):
        if self.state == State.RUNNING:
            self.pending_pause = True
            response.success = True
            return response
        if self.state in (State.HOLD, State.PAUSED):
            response.success = False
            response.message = 'already paused'
            return response
        response.success = False
        response.message = 'not running'
        return response

    def on_resume(self, request, response):
        if self.state in (State.PAUSED, State.HOLD):
            self.pending_resume = True
            response.success = True
            return response
        if self.state == State.RUNNING:
            response.success = False
            response.message = 'not paused'
            return response
        response.success = False
        response.message = 'not running'
        return response

    # --- メインループ ---

    def run(self):
        while rclpy.ok():
            if self.state == State.RUNNING:
                self.tick_running()
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.process_flags()

    def process_flags(self):
        if self.pending_start:
            self.pending_start = False
            self.do_start()
        if self.pending_pause:
            self.pending_pause = False
            self.do_pause()
        if self.pending_resume:
            self.pending_resume = False
            self.do_resume()

    def tick_running(self):
        # isTaskComplete() 自体が最大 0.1 秒 spin するのでサービスもここで処理される
        if not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is not None:
                # MPPI は制御ループが 1 秒途切れると速度制限を内部リセットするため、毎ループ publish し続ける
                self.publish_speed_limit(speed_limit_for(self.current_leg, feedback.current_waypoint))
            return

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.handle_succeeded()
        elif result == TaskResult.CANCELED:
            self.handle_canceled()
        else:
            self.handle_failed()

    def do_start(self):
        self.leg_index = 0
        self.paused_leg = None
        self.pause_requested = False
        self.set_state(State.RUNNING)
        self.send_leg(self.legs[self.leg_index])

    def do_pause(self):
        self.pause_requested = True
        self.cancelTask()

    def do_resume(self):
        if self.state == State.PAUSED:
            leg = self.paused_leg
            self.paused_leg = None
            self.set_state(State.RUNNING)
            self.send_leg(leg)
        elif self.state == State.HOLD:
            self.set_state(State.RUNNING)
            self.advance_leg()

    # --- レグ送信・結果処理 ---

    def send_leg(self, leg):
        self.current_leg = leg
        poses = [self.to_pose_stamped(wp) for wp in leg.waypoints]
        if not self.followWaypoints(poses):
            self.get_logger().error(f'waypoint leg starting at {leg.start_index} was rejected')
            self.set_state(State.IDLE)

    def to_pose_stamped(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        qz, qw = yaw_to_quaternion(wp['yaw'])
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_speed_limit(self, value):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.percentage = True
        msg.speed_limit = float(value)
        self.speed_limit_pub.publish(msg)

    def set_state(self, new_state):
        if self.state == State.RUNNING and new_state != State.RUNNING:
            # 走行中でなくなるので速度制限を解除する(speed_limit=0.0 は解除であり停止ではない)
            self.publish_speed_limit(0.0)
        self.state = new_state

    def handle_succeeded(self):
        leg = self.current_leg
        missed_local = list(self.result_future.result().result.missed_waypoints)
        if missed_local:
            missed_global = [leg.start_index + i for i in missed_local]
            self.get_logger().warn(f'missed waypoints: {missed_global}')

        last_index = len(leg.waypoints) - 1
        if leg.ends_with_checkpoint and last_index in missed_local:
            cp_index = leg.start_index + last_index
            self.get_logger().info(f'checkpoint at waypoint {cp_index} missed, retrying')
            self.send_leg(Leg(
                start_index=cp_index,
                waypoints=[leg.waypoints[last_index]],
                ends_with_checkpoint=True))
            return

        if leg.ends_with_checkpoint:
            self.get_logger().info(
                f'reached checkpoint at waypoint {leg.start_index + last_index}, waiting for resume')
            self.set_state(State.HOLD)
            return

        self.advance_leg()

    def handle_canceled(self):
        if self.pause_requested:
            self.pause_requested = False
            self.enter_paused()
        else:
            self.handle_failed()

    def handle_failed(self):
        leg = self.current_leg
        self.get_logger().warn(f'waypoint leg starting at {leg.start_index} failed')
        if not leg.ends_with_checkpoint:
            self.advance_leg()
            return

        feedback = self.getFeedback()
        index = feedback.current_waypoint if feedback is not None else 0
        if not (0 <= index < len(leg.waypoints)):
            index = 0
        self.send_leg(Leg(
            start_index=leg.start_index + index,
            waypoints=leg.waypoints[index:],
            ends_with_checkpoint=True))

    def enter_paused(self):
        leg = self.current_leg
        feedback = self.getFeedback()
        index = feedback.current_waypoint if feedback is not None else 0
        if not (0 <= index < len(leg.waypoints)):
            index = len(leg.waypoints) - 1
        self.paused_leg = Leg(
            start_index=leg.start_index + index,
            waypoints=leg.waypoints[index:],
            ends_with_checkpoint=leg.ends_with_checkpoint)
        self.set_state(State.PAUSED)

    def advance_leg(self):
        self.leg_index += 1
        if self.leg_index >= len(self.legs):
            if not self.loop:
                self.get_logger().info('finished')
                self.set_state(State.FINISHED)
                return
            self.leg_index = 0
        self.send_leg(self.legs[self.leg_index])


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGINT を受けた rclpy が context を落とすと spin が内部例外で抜ける
        if rclpy.ok():
            raise
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
