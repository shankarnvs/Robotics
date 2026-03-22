import rclpy
from rclpy.node import Node
from continuum_msgs.msg import RobotState
from continuum_kinematics.kinematics import ContinuumKinematics
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ContinuumControl(Node):

    def __init__(self):
        super().__init__('continuum_control')

        self.sub = self.create_subscription(
            RobotState,
            '/continuum/state',
            self.callback,
            10
        )

        self.br = tf2_ros.TransformBroadcaster(self)
        self.kin = ContinuumKinematics()

    def callback(self, msg):
        transforms = self.kin.compute_transforms(msg)

        for i, (pos, rot) in enumerate(transforms):
            t = TransformStamped()
            t.header.frame_id = "world"
            t.child_frame_id = f"link_{i}"

            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])

            # simplified rotation
            t.transform.rotation.w = 1.0

            self.br.sendTransform(t)


def main():
    rclpy.init()
    node = ContinuumControl()
    rclpy.spin(node)
    rclpy.shutdown()
