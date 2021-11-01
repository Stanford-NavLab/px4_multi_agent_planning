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
        # Node(
        #     package='global_planner',
        #     node_name=ns,
        #     node_executable='direct_hlp',
        #     output='screen',
        #     emulate_tty=True
        # ),
        Node(
            package='multi_rtd',
            node_name=ns,
            node_executable='multi_planner',
            output='screen',
            emulate_tty=True
        )
    ])