from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import sys

def generate_launch_description():
    
    for arg in sys.argv:
        if arg.startswith("ns:="):
            ns = arg.split(":=")[1]
            sys_id = int(arg.split("_")[1])

    return LaunchDescription([
        Node(
            package='obstacle_detection',
            node_name=ns,
            node_executable='ground_truth_cylinders',
            output='screen',
            emulate_tty=True
        )
    ])
