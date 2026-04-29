import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

import tf_transformations
import tf2_ros

# 👉 Replace with your actual package
from continuum_msgs.msg import RobotState


class ContinuumControlNode(Node):

    def __init__(self):
        super().__init__('continuum_control_node')

        # Subscriber
        self.sub = self.create_subscription(
            RobotState,
            '/continuum/state',
            self.state_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/continuum/markers', 10)

        self.get_logger().info("Continuum control node started")

    # --------------------------------------------------
    # 🔗 Utility: get chain from parent → target
    # --------------------------------------------------
    def get_link_chain(self, parent_id, target_id, links):
        chain = []
        collecting = False

        for link in links:
            if link.id == parent_id:
                collecting = True

            if collecting:
                chain.append(link)

            if link.id == target_id:
                break

        return chain

    # --------------------------------------------------
    # ⚖️ Smooth weight distribution
    # --------------------------------------------------
    def compute_weights(self, n):
        weights = [math.sin((i + 1) / n * math.pi / 2.0) for i in range(n)]
        s = sum(weights)
        return [w / s for w in weights]

    # --------------------------------------------------
    # 🔁 Main callback
    # --------------------------------------------------
    def state_callback(self, msg: RobotState):

        links = msg.links
        servos = msg.servos

        # ------------------------------------------
        # 🧱 Step 1: reset contributions
        # ------------------------------------------
        link_contrib = {link.id: 0.0 for link in links}

        # ------------------------------------------
        # 🔁 Step 2: accumulate servo influence
        # ------------------------------------------
        for servo in servos:

            chain = self.get_link_chain(
                servo.parent_link_id,
                servo.target_link_id,
                links
            )

            if len(chain) == 0:
                continue

            weights = self.compute_weights(len(chain))

            for link, w in zip(chain, weights):
                link_contrib[link.id] += servo.angle * w

        # ------------------------------------------
        # ✅ Step 3: assign final angles
        # ------------------------------------------
        for link in links:
            link.angle = link_contrib[link.id]

        # ------------------------------------------
        # 🔄 Step 4: forward kinematics + TF
        # ------------------------------------------
        T = tf_transformations.identity_matrix()

        for i, link in enumerate(links):

            # Rotation about Z (change if needed)
            R = tf_transformations.rotation_matrix(link.angle, (0, 0, 1))
            T = tf_transformations.concatenate_matrices(T, R)

            # Translation along X (link length)
            trans = tf_transformations.translation_matrix((link.length, 0, 0))
            T = tf_transformations.concatenate_matrices(T, trans)

            # Publish TF
            self.publish_tf(link, T, i)

        # ------------------------------------------
        # 🎯 Step 5: markers
        # ------------------------------------------
        self.publish_markers(links)

    # --------------------------------------------------
    # 📡 TF publishing
    # --------------------------------------------------
    def publish_tf(self, link, T, index):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = f"link_{link.id}"

        translation = tf_transformations.translation_from_matrix(T)
        rotation = tf_transformations.quaternion_from_matrix(T)

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(t)

    # --------------------------------------------------
    # 🎨 Marker visualization
    # --------------------------------------------------
    def publish_markers(self, links):

        marker = Marker()

        marker.header = Header()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "continuum"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.02

        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0

        # Recompute positions
        T = tf_transformations.identity_matrix()

        for link in links:

            pos = tf_transformations.translation_from_matrix(T)

            from geometry_msgs.msg import Point
            p = Point()
            p.x, p.y, p.z = pos
            marker.points.append(p)

            # Apply transform
            R = tf_transformations.rotation_matrix(link.angle, (0, 0, 1))
            T = tf_transformations.concatenate_matrices(T, R)

            trans = tf_transformations.translation_matrix((link.length, 0, 0))
            T = tf_transformations.concatenate_matrices(T, trans)

        self.marker_pub.publish(marker)


# --------------------------------------------------
# 🚀 Main
# --------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ContinuumControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()