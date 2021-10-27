#!/bin/bash
# Initialize drones in Gazebo with PX4 and micrortps agents and have them take off
# Reads parameters from /bringup/config/global_params.yaml

# Usage:
#./startup.sh -n 10 -m iris

source yaml.sh

# extract parameters from yaml config file
# assumes px4_ros_com_ros2 installed in home directory
# TODO: change to relative path
config_path=$HOME/px4_ros_com_ros2/src/px4_multi_agent_planning/bringup/config
create_variables ${config_path}/global_params.yaml

# use shorter names
world=${global_parameter_server_ros__parameters_world_file}
vehicle=${global_parameter_server_ros__parameters_vehicle_model}
num_vehicles=${global_parameter_server_ros__parameters_num_vehicles}
spawn_x=("${global_parameter_server_ros__parameters_spawn_x[@]}")
spawn_y=("${global_parameter_server_ros__parameters_spawn_y[@]}")

# generate vehicle spawn string
# TODO: put in check that num_vehicles == len(spawn_x/y)
spawn_str=''
for ((i = 0 ; i < ${num_vehicles} ; i++)); do
	spawn_str+=${vehicle}:1:${spawn_x[i]}:${spawn_y[i]}
	if ((i != ${num_vehicles}-1)); then
		spawn_str+=,
	fi
done

# assumes PX4-Autopilot is installed in home directory
gnome-terminal --tab -- $HOME/PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh -w ${world} -t px4_sitl_rtps -s ${spawn_str}
# use a heuristic for how long to wait for gazebo to finish starting up
spawn_time=$((12+2*${num_vehicles}))
sleep ${spawn_time}

# Start the ROS global parameter server
gnome-terminal --tab -- ros2 launch bringup param_server.launch.py

# Start a microRTPS_agent for each drone and launch the offboard node
# takeoff is still inconsistent, unclear what the best order or combination of sleeps is
n=0
while [ ${n} -lt ${num_vehicles} ]; do
	gnome-terminal --tab -- micrortps_agent -t UDP -n iris_${n} -r $((2020+2*${n})) -s $((2019+2*${n}))
	sleep 0.1
	gnome-terminal --tab -- ros2 launch bringup offboard.launch.py ns:=iris_${n}
	sleep 0.1
	n=$(($n + 1))
done
