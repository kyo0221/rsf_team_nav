#!/usr/bin/env python3
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointRecorder(Node):

    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('output_file', '')

        self.latest_pose = None
        self.waypoints = []

        self.create_subscription(
            PoseWithCovarianceStamped, 'mcl_pose', self.on_pose, 10)
        self.create_service(Trigger, '~/record', self.on_record)
        self.create_service(Trigger, '~/save', self.on_save)

    def on_pose(self, msg):
        self.latest_pose = msg.pose.pose

    def on_record(self, request, response):
        if self.latest_pose is None:
            response.success = False
            response.message = 'no pose received yet'
            return response

        yaw = quaternion_to_yaw(self.latest_pose.orientation)
        self.waypoints.append({
            'x': round(self.latest_pose.position.x, 3),
            'y': round(self.latest_pose.position.y, 3),
            'yaw': round(yaw, 3),
        })
        response.success = True
        response.message = f'recorded waypoint {len(self.waypoints) - 1}'
        self.get_logger().info(response.message)
        return response

    def on_save(self, request, response):
        output_file = self.get_parameter('output_file').value
        if not self.waypoints:
            response.success = False
            response.message = 'no waypoints recorded'
            return response

        with open(output_file, 'w') as f:
            yaml.safe_dump({'loop': False, 'waypoints': self.waypoints}, f, sort_keys=False)

        response.success = True
        response.message = f'saved {len(self.waypoints)} waypoints to {output_file}'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
