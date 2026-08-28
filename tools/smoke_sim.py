#!/usr/bin/env python3
"""Standalone smoke input for the asm_mighty module (no Isaac, no controller).

Publishes, at canonical AirStack topic names for ROBOT_NAME:
  - nav_msgs/Odometry  (hover at (0,0,2), map frame, zero body twist)
  - sensor_msgs/PointCloud2 in the `ouster` frame: two synthetic pillars at
    (5, +/-1.8) plus a back wall sample, expressed relative to the sensor
  - static TFs map->base_link->ouster at the hover pose

Then the module can be exercised with:
  ros2 action send_goal /$ROBOT_NAME/tasks/navigate task_msgs/action/NavigateTask ...
Pass criteria: mighty publishes mighty/trajectory and the bridge republishes
trajectory_controller/trajectory_segment_to_add continuously.
"""

import math
import os
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

ROBOT = os.environ.get('ROBOT_NAME', 'robot_1')
HOVER = (0.0, 0.0, 2.0)


def make_cloud(stamp, frame):
    pts = []
    # two pillars (r=0.5) at (5, +/-1.8), z 0..6, plus a sparse far wall at x=12
    for cx, cy in ((5.0, 1.8), (5.0, -1.8)):
        for zi in range(0, 61, 2):
            z = zi * 0.1
            for ai in range(0, 360, 20):
                a = math.radians(ai)
                pts.append((cx + 0.5 * math.cos(a) - HOVER[0],
                            cy + 0.5 * math.sin(a) - HOVER[1],
                            z - HOVER[2]))
    for yi in range(-40, 41, 2):
        for zi in range(0, 61, 4):
            pts.append((12.0 - HOVER[0], yi * 0.1 - HOVER[1], zi * 0.1 - HOVER[2]))

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.data = b''.join(struct.pack('<fff', *p) for p in pts)
    msg.is_dense = True
    return msg


class SmokeSim(Node):
    def __init__(self):
        super().__init__('mighty_smoke_sim')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(
            Odometry, f'/{ROBOT}/odometry_conversion/odometry', qos)
        self.cloud_pub = self.create_publisher(
            PointCloud2, f'/{ROBOT}/sensors/ouster/point_cloud', qos)

        self.static_tf = StaticTransformBroadcaster(self)
        tfs = []
        for parent, child, trans in (
                ('map', 'base_link', HOVER),
                ('base_link', 'ouster', (0.0, 0.0, 0.0))):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.static_tf.sendTransform(tfs)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'smoke sim publishing for {ROBOT}')

    def tick(self):
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = HOVER[0]
        odom.pose.pose.position.y = HOVER[1]
        odom.pose.pose.position.z = HOVER[2]
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(make_cloud(now, 'ouster'))


def main():
    rclpy.init()
    rclpy.spin(SmokeSim())


if __name__ == '__main__':
    main()
