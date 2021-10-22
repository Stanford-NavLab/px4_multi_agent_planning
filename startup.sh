#!/bin/bash
# Initialize drones in Gazebo with PX4 and micrortps agents and have them take off

# Usage:
#./startup.sh -n 10 -m iris

num_vehicles=${NUM_VEHICLES:=1}
world="static_world"

gnome-terminal --tab -- /home/navlab-exxact-18/PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh -n ${num_vehicles} -w ${world} -m iris -t px4_sitl_rtps 
spawn_time=$((12+2*${num_vehicles}))
sleep ${spawn_time}

n=0
while [ ${n} -lt ${num_vehicles} ]; do
	gnome-terminal --tab -- micrortps_agent -t UDP -n iris_${n} -r $((2020+2*${n})) -s $((2019+2*${n}))
	gnome-terminal --tab -- ros2 launch bringup takeoff.launch.py ns:=iris_${n}
	n=$(($n + 1))
done

# n=0
# while [ ${n} -lt ${num_vehicles} ]; do
# 	gnome-terminal --tab -- ros2 launch bringup takeoff.launch.py ns:=iris_${n}
# 	n=$(($n + 1))
# done

# if [ "$1" == "-h" ] || [ "$1" == "--help" ]
# then
# 	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>]"
# 	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
# 	exit 1
# fi

# while getopts n:m:w:s:t:l: option
# do
# 	case "${option}"
# 	in
# 		n) NUM_VEHICLES=${OPTARG};;
# 		m) VEHICLE_MODEL=${OPTARG};;
# 		w) WORLD=${OPTARG};;
# 		s) SCRIPT=${OPTARG};;
# 		t) TARGET=${OPTARG};;
# 		l) LABEL=_${OPTARG};;
# 	esac
# done

# num_vehicles=${NUM_VEHICLES:=3}
# world=${WORLD:=empty}
# target=${TARGET:=px4_sitl_default}
# vehicle_model=${VEHICLE_MODEL:="iris"}
# export PX4_SIM_MODEL=${vehicle_model}${LABEL}

# echo ${SCRIPT}
# SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# src_path="$SCRIPT_DIR/.."

# build_path=${src_path}/build/${target}
# mavlink_udp_port=14560
# mavlink_tcp_port=4560

# echo "killing running instances"
# pkill -x px4 || true

# sleep 1

# source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

# echo "Starting gazebo"
# gzserver ${src_path}/Tools/sitl_gazebo/worlds/${world}.world --verbose &
# sleep 5

# n=0
# if [ -z ${SCRIPT} ]; then
# 	if [ $num_vehicles -gt 255 ]
# 	then
# 		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
# 		exit 1
# 	fi

# 	while [ $n -lt $num_vehicles ]; do
# 		spawn_model ${vehicle_model} $n
# 		n=$(($n + 1))
# 	done
# else
# 	IFS=,
# 	for target in ${SCRIPT}; do
# 		target="$(echo "$target" | tr -d ' ')" #Remove spaces
# 		target_vehicle=$(echo $target | cut -f1 -d:)
# 		target_number=$(echo $target | cut -f2 -d:)
# 		target_x=$(echo $target | cut -f3 -d:)
# 		target_y=$(echo $target | cut -f4 -d:)

# 		if [ $n -gt 255 ]
# 		then
# 			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
# 			exit 1
# 		fi

# 		m=0
# 		while [ $m -lt ${target_number} ]; do
# 			spawn_model ${target_vehicle} $n $target_x $target_y
# 			m=$(($m + 1))
# 			n=$(($n + 1))
# 		done
# 	done

# fi
# trap "cleanup" SIGINT SIGTERM EXIT

# echo "Starting gazebo client"
# gzclient
