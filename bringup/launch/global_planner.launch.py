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
            package='global_planner',
            node_namespace=ns,
            node_executable='rrt_cylinders',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='global_planner',
            node_namespace=ns,
            node_executable='waypoint_publisher',
            output='screen',
            emulate_tty=True
        ),
    ])
