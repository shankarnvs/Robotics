import rclpy
from rclpy.node import Node

from continuum_msgs.msg import RobotState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class ControlNode(Node):

    def __init__(self):
        super().__init__('continuum_control')

        self.sub = self.create_subscription(
            RobotState,
            '/continuum/state',
            self.callback,
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def callback(self, msg):
        parent = "world"

        for link in msg.links:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = f"link_{link.id}"

            t.transform.translation.z = link.length

            self.tf_broadcaster.sendTransform(t)

            parent = t.child_frame_id


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
