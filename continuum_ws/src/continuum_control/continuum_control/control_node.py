import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from continuum_msgs.msg import RobotState

import tf_transformations


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # -------- Parameters --------
        self.declare_parameter('num_links', 5)
        self.num_links = self.get_parameter('num_links').value

        self.L = 1.0

        # -------- State --------
        self.last_msg = None

        # -------- Publishers --------
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -------- Subscriber --------
        self.sub = self.create_subscription(
            RobotState,
            '/continuum/state',
            self.state_callback,
            10
        )

        # -------- Timer --------
        self.create_timer(0.05, self.update)

    # =========================================================
    def state_callback(self, msg):
        self.last_msg = msg

        # update number of links dynamically
        if hasattr(msg, 'links') and len(msg.links) > 0:
            self.num_links = len(msg.links)

    # =========================================================
    def compute_joints(self):

        joints = {}

        # initialize all joints
        for i in range(1, self.num_links):
            joints[f"joint_{i}_x"] = 0.0
            joints[f"joint_{i}_y"] = 0.0

        if self.last_msg is None:
            return joints

        # apply servo effects
        for s in self.last_msg.servos:

            if len(s.affects_links) == 0:
                continue

            n = len(s.affects_links)

            for link_id in s.affects_links:

                if link_id == 0 or link_id >= self.num_links:
                    continue

                joints[f"joint_{link_id}_x"] += s.bend_x / n
                joints[f"joint_{link_id}_y"] += s.bend_y / n

        return joints

    # =========================================================
    def compute_chain_transforms(self, joints):

        transforms = []
        T = tf_transformations.identity_matrix()

        for i in range(self.num_links):

            transforms.append(T.copy())

            if i < self.num_links - 1:
                bx = joints.get(f"joint_{i+1}_x", 0.0)
                by = joints.get(f"joint_{i+1}_y", 0.0)

                Rx = tf_transformations.rotation_matrix(bx, (1, 0, 0))
                Ry = tf_transformations.rotation_matrix(by, (0, 1, 0))
                trans = tf_transformations.translation_matrix((0, 0, self.L))

                T = tf_transformations.concatenate_matrices(T, Rx, Ry, trans)

        return transforms

    # =========================================================
    def publish_joint_states(self, joints):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joints.keys())
        msg.position = list(joints.values())

        self.joint_pub.publish(msg)

    # =========================================================
    def publish_markers(self, transforms):

        markers = MarkerArray()
        marker_id = 0

        for i, T in enumerate(transforms):

            pos = tf_transformations.translation_from_matrix(T)
            quat = tf_transformations.quaternion_from_matrix(T)

            # ---- Cylinder ----
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "links"
            m.id = marker_id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2] + self.L / 2.0

            m.pose.orientation.x = quat[0]
            m.pose.orientation.y = quat[1]
            m.pose.orientation.z = quat[2]
            m.pose.orientation.w = quat[3]

            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = self.L

            m.color.r = float(0.2 + 0.8 * i / self.num_links)
            m.color.g = 0.4
            m.color.b = float(1.0 - 0.8 * i / self.num_links)
            m.color.a = 1.0

            markers.markers.append(m)
            marker_id += 1

            # ---- Joint spheres ----
            for z in [0.0, self.L]:
                s = Marker()
                s.header.frame_id = "world"
                s.header.stamp = self.get_clock().now().to_msg()
                s.ns = "joints"
                s.id = marker_id
                s.type = Marker.SPHERE
                s.action = Marker.ADD

                offset = tf_transformations.translation_matrix((0, 0, z))
                T_s = tf_transformations.concatenate_matrices(T, offset)
                pos_s = tf_transformations.translation_from_matrix(T_s)

                s.pose.position.x = pos_s[0]
                s.pose.position.y = pos_s[1]
                s.pose.position.z = pos_s[2]

                s.scale.x = 0.25
                s.scale.y = 0.25
                s.scale.z = 0.25

                s.color.r = 1.0
                s.color.g = 0.2
                s.color.b = 0.2
                s.color.a = 1.0

                markers.markers.append(s)
                marker_id += 1

        self.marker_pub.publish(markers)

    # =========================================================
    def publish_axes(self, transforms):

        markers = MarkerArray()
        marker_id = 1000

        for T in transforms:

            pos = tf_transformations.translation_from_matrix(T)

            axes = [
                ((1, 0, 0), (1.0, 0.0, 0.0)),
                ((0, 1, 0), (0.0, 1.0, 0.0)),
                ((0, 0, 1), (0.0, 0.0, 1.0)),
            ]

            for axis_vec, color in axes:

                m = Marker()
                m.header.frame_id = "world"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "axes"
                m.id = marker_id
                m.type = Marker.ARROW
                m.action = Marker.ADD

                m.points.append(self.make_point(pos))

                T_end = tf_transformations.concatenate_matrices(
                    T,
                    tf_transformations.translation_matrix(tuple(0.3 * x for x in axis_vec))
                )

                end_pos = tf_transformations.translation_from_matrix(T_end)
                m.points.append(self.make_point(end_pos))

                m.scale.x = 0.02
                m.scale.y = 0.04

                m.color.r = float(color[0])
                m.color.g = float(color[1])
                m.color.b = float(color[2])
                m.color.a = 1.0

                markers.markers.append(m)
                marker_id += 1

        self.marker_pub.publish(markers)

    # =========================================================
    def publish_tip_tf(self, transforms):

        T = transforms[-1]

        pos = tf_transformations.translation_from_matrix(T)
        quat = tf_transformations.quaternion_from_matrix(T)

        t = TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = "tip"

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    # =========================================================
    def make_point(self, xyz):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        return p

    # =========================================================
    def update(self):

        joints = self.compute_joints()
        transforms = self.compute_chain_transforms(joints)

        self.publish_joint_states(joints)
        self.publish_markers(transforms)
        self.publish_axes(transforms)
        self.publish_tip_tf(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()