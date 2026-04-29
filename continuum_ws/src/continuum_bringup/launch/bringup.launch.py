from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
        #Launching control node
        Node(
            package='continuum_control',
            executable='control_node'
        ), 
        #Launching RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            on_exit=[Shutdown()]
        ),
    ])
