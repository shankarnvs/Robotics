from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---- Launch argument ----
    num_links = LaunchConfiguration('num_links')

    num_links_arg = DeclareLaunchArgument(
        'num_links',
        default_value='5',
        description='Number of links'
    )

    # ---- Packages ----
    pkg_desc = get_package_share_directory('continuum_description')
    pkg_bringup = get_package_share_directory('continuum_bringup')

    # ---- Xacro ----
    xacro_file = os.path.join(pkg_desc, 'urdf', 'continuum.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' num_links:=', num_links
        ]),
        value_type=str
    )

    # ---- RViz config ----
    rviz_config = os.path.join(pkg_bringup, 'launch', 'CustomLaunch.rviz')

    return LaunchDescription([

        num_links_arg,

        # ---- Control node ----
        Node(
            package='continuum_control',
            executable='control_node',
            output='screen'
        ),

        # ---- Robot State Publisher ----
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # ---- Dynamic description ----
        Node(
            package='continuum_control',
            executable='dynamic_description_node',
            output='screen'
        ),

        # ---- RViz (WITH CONFIG) ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            on_exit=[Shutdown()]
        ),
    ])