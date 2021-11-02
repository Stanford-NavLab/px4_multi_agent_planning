# px4_multi_agent_planning

## Setup
 - Follow PX4 instructions for ROS 2 setup: https://docs.px4.io/v1.12/en/ros/ros2_comm.html
 - `git clone https://github.com/Stanford-NavLab/px4_multi_agent_planning.git` (clone to `~/px4_ros_com_ros2/src`)
 - From `~px4_ros_com_ros2`: `colcon build`
 - `source install/setup.bash`

### Notes
 - Add `source ~/px4_ros_com_ros2/install/setup.bash` to `.bashrc`
   - No need to additionally source ROS 2
 - May need to make clean before building (`source ~/px4_ros_com_ros2/src/px4_ros_com/scripts/clean_all.bash`)

## Usage
 - Run `./startup` in order to start Gazebo and spawn and takeoff vehicles in offboard mode (may need to `chmod +x` for permissions)
 - Edit `bringup/config/global_params.yaml` to change number of vehicles, spawn locations, goal positions, etc.
 - Run `ros2 launch bringup multi.launch.py ns:=iris_X` from `~/px4_ros_com_ros2` where X is agent index starting from 0 to start multi-agent planner
 - Once planners for all agents are up, run `ros2 topic pub -r 10 /simulation_start std_msgs/msg/Bool "data: True"` to send the start signal and start the planners
