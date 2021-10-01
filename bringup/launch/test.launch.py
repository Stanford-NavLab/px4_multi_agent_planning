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
            package='multi_rtd',
            node_namespace=ns,
            node_executable='trajectory_sampler',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='offboard_ibqr',
            node_namespace=ns,
            node_executable='trajectory_logger',
            #output='screen',
            emulate_tty=True
        ),
        Node(
            package='offboard_ibqr',
            node_namespace=ns,
            node_executable='offboard_tester',
            #output='screen',
            emulate_tty=True
        )
    ])