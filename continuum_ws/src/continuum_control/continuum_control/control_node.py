#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

import tf_transformations
import tf2_ros

from continuum_msgs.msg import RobotState


class ContinuumControlNode(Node):

    def __init__(self):
        super().__init__('continuum_control_node')

        self.sub = self.create_subscription(
            RobotState,
            '/continuum/state',
            self.state_callback,
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/continuum/markers', 10)

        self.links = []
        self.joint_angles = {}

        self.timer = self.create_timer(0.05, self.update)

    # --------------------------------------------------
    def state_callback(self, msg: RobotState):

        self.links = msg.links
        servos = msg.servos

        # Initialize joint contributions
        joint_contrib = {
            link.id: [0.0, 0.0]
            for link in self.links
        }

        # Distribute tendon influence → joints
        for servo in servos:

            chain = [l for l in self.links if l.id in servo.affects_links]

            if not chain:
                continue

            n = len(chain)

            for link in chain:
                joint_contrib[link.id][0] += servo.bend_x / n
                joint_contrib[link.id][1] += servo.bend_y / n

        self.joint_angles = joint_contrib

    # --------------------------------------------------
    def update(self):

        if not self.links:
            return

        self.publish_tf()
        self.publish_marker()

    # --------------------------------------------------
    def publish_tf(self):

        transforms = []

        parent = "world"

        for i, link in enumerate(self.links):

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = f"link_{i}"

            # -------------------------
            # 1. Joint rotation (except base)
            # -------------------------
            if i == 0:
                # base link: no rotation
                R = tf_transformations.identity_matrix()
            else:
                bend_x, bend_y = self.joint_angles.get(i-1, [0.0, 0.0])

                Rx = tf_transformations.rotation_matrix(bend_x, (1, 0, 0))
                Ry = tf_transformations.rotation_matrix(bend_y, (0, 1, 0))

                R = tf_transformations.concatenate_matrices(Ry, Rx)

            # -------------------------
            # 2. Translation along rigid link
            # -------------------------
            trans = tf_transformations.translation_matrix((0, 0, link.length))

            # LOCAL transform: parent → child
            T_local = tf_transformations.concatenate_matrices(R, trans)

            # extract
            translation = tf_transformations.translation_from_matrix(T_local)
            rotation = tf_transformations.quaternion_from_matrix(T_local)

            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])

            t.transform.rotation.x = float(rotation[0])
            t.transform.rotation.y = float(rotation[1])
            t.transform.rotation.z = float(rotation[2])
            t.transform.rotation.w = float(rotation[3])

            transforms.append(t)

            parent = f"link_{i}"

        self.tf_broadcaster.sendTransform(transforms)

    # --------------------------------------------------
    def publish_marker(self):

        marker = Marker()

        marker.header = Header()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "continuum"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.02

        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.3
        marker.color.a = 1.0

        T = tf_transformations.identity_matrix()

        for i, link in enumerate(self.links):

            pos = tf_transformations.translation_from_matrix(T)

            p = Point()
            p.x, p.y, p.z = pos
            marker.points.append(p)

            # move along rigid link
            T = tf_transformations.concatenate_matrices(
                T,
                tf_transformations.translation_matrix((0, 0, link.length))
            )

            # apply joint rotation
            bend_x, bend_y = self.joint_angles.get(link.id, [0.0, 0.0])

            Rx = tf_transformations.rotation_matrix(bend_x, (1, 0, 0))
            Ry = tf_transformations.rotation_matrix(bend_y, (0, 1, 0))
            R = tf_transformations.concatenate_matrices(Ry, Rx)

            T = tf_transformations.concatenate_matrices(T, R)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ContinuumControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()