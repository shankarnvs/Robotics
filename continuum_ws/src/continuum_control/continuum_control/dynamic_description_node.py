import rclpy
from rclpy.node import Node
from continuum_msgs.msg import RobotState
import time

import subprocess

class DynamicDescriptionNode(Node):

    def __init__(self):
        super().__init__('dynamic_description_node')

        self.sub = self.create_subscription(
            RobotState,
            '/continuum/state',
            self.callback,
            10
        )

        #self.first_update = True
        self.current_links = None

    def callback(self, msg):

        num_links = len(msg.links)

        # -----------------------------
        # First update trigger
        # -----------------------------
        #if self.first_update:
        #    self.first_update = False
        #    self.get_logger().info("First model generation")
        #else:
            # Avoid unnecessary reloads
        if num_links == self.current_links:
            return

        self.current_links = num_links
        self.get_logger().info(f"Updating model: {num_links} links")

        # -----------------------------
        # Xacro path (FIXED VERSION)
        # -----------------------------
        from ament_index_python.packages import get_package_share_directory
        import os

        pkg_path = get_package_share_directory('continuum_description')
        xacro_path = os.path.join(pkg_path, 'urdf', 'continuum.xacro')

        # -----------------------------
        # Run xacro
        # -----------------------------
        import subprocess

        cmd = [
            'xacro',
            xacro_path,
            f'num_links:={num_links}'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            self.get_logger().error(result.stderr)
            return

        urdf = result.stdout

        # -----------------------------
        # Update robot_state_publisher
        # -----------------------------
        self.set_robot_description(urdf)
        time.sleep(0.1)   # allow robot_state_publisher to reload

    def set_robot_description(self, urdf):

        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter

        client = self.create_client(SetParameters, '/robot_state_publisher/set_parameters')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot_state_publisher...')

        param = Parameter(
            'robot_description',
            Parameter.Type.STRING,
            urdf
        ).to_parameter_msg()

        req = SetParameters.Request()
        req.parameters.append(param)

        future = client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicDescriptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
