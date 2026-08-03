#!/usr/bin/env python3
import math

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

SERVER_WAIT_TIMEOUT = 5.0


def yaw_to_quaternion(yaw):
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self.declare_parameter('waypoints_file', '')

        with open(self.get_parameter('waypoints_file').value) as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.loop = bool(data.get('loop', False))

        self.index = 0
        self.running = False
        self.paused = False
        self.checkpoint_hold = False
        self.goal_handle = None
        self.goal_seq = 0

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.speed_limit_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/pause', self.on_pause)
        self.create_service(Trigger, '~/resume', self.on_resume)

    def on_start(self, request, response):
        if self.running:
            response.success = False
            response.message = 'already running'
            return response
        self.running = True
        self.paused = False
        self.index = 0
        if not self.send_current_goal():
            response.success = False
            response.message = 'navigate_to_pose action server not available'
            return response
        response.success = True
        return response

    def on_pause(self, request, response):
        if not self.running or self.paused:
            response.success = False
            response.message = 'not running' if not self.running else 'already paused'
            return response
        self.paused = True
        self.goal_seq += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        response.success = True
        return response

    def on_resume(self, request, response):
        if not self.running or not self.paused:
            response.success = False
            response.message = 'not running' if not self.running else 'not paused'
            return response
        self.paused = False
        if self.checkpoint_hold:
            self.checkpoint_hold = False
            sent = self.advance_and_send()
        else:
            sent = self.send_current_goal()
        if not sent:
            response.success = False
            response.message = 'navigate_to_pose action server not available'
            return response
        response.success = True
        return response

    def send_current_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT):
            self.get_logger().error('navigate_to_pose action server not available')
            self.running = False
            return False

        wp = self.waypoints[self.index]
        qz, qw = yaw_to_quaternion(float(wp.get('yaw', 0.0)))

        self.publish_speed_limit(wp)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.goal_seq += 1
        seq = self.goal_seq
        self.get_logger().info(f'Sending waypoint {self.index}: x={wp["x"]}, y={wp["y"]}')
        self.action_client.send_goal_async(goal_msg).add_done_callback(
            lambda future: self.on_goal_response(future, seq))
        return True

    def publish_speed_limit(self, wp):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.percentage = True
        msg.speed_limit = float(wp.get('speed_limit', 0.0))
        self.speed_limit_pub.publish(msg)

    def on_goal_response(self, future, seq):
        if seq != self.goal_seq:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.running = False
            self.get_logger().error(f'Waypoint {self.index} rejected, stopping')
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self.on_result(result_future, seq))

    def on_result(self, future, seq):
        if seq != self.goal_seq:
            return

        checkpoint = self.waypoints[self.index].get('checkpoint', False)
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Waypoint {self.index} did not succeed (status {status})')
            if checkpoint:
                self.get_logger().info(f'Waypoint {self.index} is a checkpoint, retrying')
                self.send_current_goal()
            else:
                self.advance_and_send()
            return

        if checkpoint:
            self.paused = True
            self.checkpoint_hold = True
            self.get_logger().info(f'Reached checkpoint at waypoint {self.index}, waiting for resume')
            return

        self.advance_and_send()

    def advance_and_send(self):
        self.index += 1
        if self.index >= len(self.waypoints):
            if not self.loop:
                self.running = False
                self.get_logger().info('Waypoint navigation finished')
                return True
            self.index = 0
        return self.send_current_goal()


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGINT を受けた rclpy が context を落とすと spin が内部例外で抜ける
        if rclpy.ok():
            raise
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
