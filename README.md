# px4_multi_agent_planning

## Setup
 - `git clone https://github.com/Stanford-NavLab/px4_multi_agent_planning.git` (clone to `~/px4_ros_com_ros2/src`)
 - `cd px4_multi_agent_planning`
 - `git submodule init`
 - `git submodule update`
 - `cd ~/px4_ros_com_ros2`
 - `colcon build`

### Notes
 - Add `source ~/px4_ros_com_ros2/install/setup.bash` to `.bashrc`
   - No need to additionally source ROS 2
 - May need to make clean before building (`source ~/px4_ros_com_ros2/src/px4_ros_com/scripts/clean_all.bash`)
