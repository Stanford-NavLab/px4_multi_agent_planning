/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define LOG_TRAJECTORY_ODOMETRY				0		// Set to 1 to log the trajectory odometry. Set to 0 to turn


//Home position-related global constants
#define HOME_POSITION_X                     0       // x-coordinate for home
#define HOME_POSITION_Y                     0       // y-coordinate for home
#define HOME_POSITION_Z                     -2      // z-coordinate for home
#define HOME_POSITION_YAW					3.14	// Yaw for home (in rad)
#define TAKEOFF_SPEED						-0.5	// In meters/second

#define OFFSET_Z							0.5


class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
#if LOG_TRAJECTORY_ODOMETRY
		trajectory_odometry_publisher_	 = this->create_publisher<VehicleOdometry>("TrajectoryOdometry", 10);
		trajectory_log_action_publisher_ = this->create_publisher<Char>("TrajectoryLogAction", 10);
#endif
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
#endif
		// // Subscribe to odometry
		odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out", 10, std::bind(&OffboardControl::odom_callback, this, _1));

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		homeReachedFlag = false;
		homeLocation.x = HOME_POSITION_X;
		homeLocation.y = HOME_POSITION_Y;
		homeLocation.z = HOME_POSITION_Z + OFFSET_Z;
		homeLocation.yaw = HOME_POSITION_YAW;
		homeLocation.vz = TAKEOFF_SPEED;


		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

           		 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);

		// Get system ID from namespace
		name_space = this->get_namespace();
		system_ID = name_space.back() - '0';
		system_ID++; 
		RCLCPP_INFO(this->get_logger(), "System ID: %i", system_ID);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	std::atomic<uint64_t> timestamp_;   			//!< common synced timestamped
	mutable std::uint8_t homeReachedFlag;	        // Boolean to determine if the set home pose has been reached


	mutable TrajectorySetpoint positionTargetMsg{};		// Goal Position
	mutable TrajectorySetpoint homeLocation{};		// Home Position

	mutable VehicleOdometry ibqrOdometry;			// Store the vehicle's latest odometry information




	std::string name_space;                         // namespace of the node

	uint64_t system_ID;				                // ID for this target system

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	//Class private methods:

	// Copy the new trajectory point to positionTargetMsg to be published to mavros
	void get_newPositionTarget(void) const;
	// Determine if the home location has been reached
	bool isHomeReached(void) const;
	// Determine if a particular goal pose has been reached
	bool isGoalReached(void) const;
	// Set the PositionTarget to the home coordinates
	void goHome(void) const;



	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {
	RCLCPP_INFO(this->get_logger(), "Testing 123")

	if (homeReachedFlag){		
		positionTargetMsg.timestamp = timestamp_.load();
		trajectory_setpoint_publisher_->publish(positionTargetMsg);

		if (isGoalReached())
		{
			goHome();
		}	 

	}
	else
	{
		if (isHomeReached())
		{
			RCLCPP_INFO(this->get_logger(), "***Home Location Reached: Switching to Position Target Mode***");
			homeReachedFlag = true;
			get_newPositionTarget();
		}
		else
		{
      		homeLocation.timestamp = timestamp_.load();
			trajectory_setpoint_publisher_->publish(homeLocation);
		}
	}
}
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = system_ID;
	msg.target_component = 1;
	msg.source_system = system_ID;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}



void OffboardControl::get_newPositionTarget(void) const
{
  if (!traj_planned->points.empty())
  {
    RCLCPP_INFO(this->get_logger(), "New Target Received:");


	positionTargetMsg.x = 0.0;
    positionTargetMsg.y = 0.0;
    positionTargetMsg.z = -5.0
	positionTargetMsg.yaw = -3.14

    RCLCPP_INFO(this->get_logger(), "rx = %f ", positionTargetMsg.x);
    RCLCPP_INFO(this->get_logger(), "ry = %f ", positionTargetMsg.y);
    RCLCPP_INFO(this->get_logger(), "rz = %f ", positionTargetMsg.z);
  }
}

bool OffboardControl::isHomeReached(void) const
{
	double epsilon = ALLOWED_ERROR_4_HOME_REACHED;
	bool check = false;


	//RCLCPP_INFO(this->get_logger(), "Home: x = %f, y = %f, z = %f", homeLocation.x, homeLocation.y, homeLocation.z);
	RCLCPP_INFO(this->get_logger(), "IBQR: x = %f, y = %f, z = %f", ibqrOdometry.x, ibqrOdometry.y, ibqrOdometry.z);

	// Check x boundary
	if ( (ibqrOdometry.x < homeLocation.x + epsilon) && (ibqrOdometry.x > homeLocation.x - epsilon) )
	{
		// Check y boundary
		if ( (ibqrOdometry.y < homeLocation.y + epsilon) && (ibqrOdometry.y > homeLocation.y - epsilon) )
		{
		// Check z boundary
			if ( (ibqrOdometry.z < (homeLocation.z - OFFSET_Z + epsilon)) && (ibqrOdometry.z > (homeLocation.z - OFFSET_Z - epsilon)) )
			{
				check = true;
			}
		}
	}

	return check;
}


bool OffboardControl::isGoalReached(void) const
{
	bool check = false;

    double epsilon = ALLOWED_ERROR_4_POS_GOAL_REACHED;

	// RCLCPP_INFO(this->get_logger(), "dx: %f, dy: %f, dz: %f", ibqrOdometry.x-positionTargetMsg.x,
	// 														  ibqrOdometry.y-positionTargetMsg.y,
	// 														  ibqrOdometry.z-positionTargetMsg.z);
    // Check x boundary
    if ( (ibqrOdometry.x < positionTargetMsg.x + epsilon) && (ibqrOdometry.x > positionTargetMsg.x - epsilon) )
    { 
        // Check y boundary
        if ( (ibqrOdometry.y < positionTargetMsg.y + epsilon) && (ibqrOdometry.y > positionTargetMsg.y - epsilon) )
        {
            // Check z boundary
            if ( (ibqrOdometry.z < (positionTargetMsg.z + epsilon)) && (ibqrOdometry.z > (positionTargetMsg.z - epsilon)) )
            {
                check = true;
            }
        }
    }

    return check;
}
/**
 * @brief Set the PositionTarget to the home coordinates
*/
void OffboardControl::goHome(void) const
{
	// Set goal to home coordinates
	positionTargetMsg.x = HOME_POSITION_X;
    positionTargetMsg.y = HOME_POSITION_Y;
    positionTargetMsg.z = HOME_POSITION_Z + OFFSET_Z;
	positionTargetMsg.yaw = HOME_POSITION_YAW;
	
	// make the reach flag false
	homeReachedFlag = false;

    RCLCPP_INFO(this->get_logger(), "Going Home");
}

/*--------------- Custom Offboard Functions ---------------*/
/**
 * @brief Calback for subscription to vehicle odometry
*/
void OffboardControl::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom) const
{
	ibqrOdometry.timestamp = odom->timestamp;

	ibqrOdometry.local_frame = odom->local_frame;
	ibqrOdometry.x = odom->x;
	ibqrOdometry.y = odom->y;
	ibqrOdometry.z = odom->z;

	ibqrOdometry.velocity_frame = odom->velocity_frame;
	ibqrOdometry.vx = odom->vx;
	ibqrOdometry.vy = odom->vy;
	ibqrOdometry.vz = odom->vz;

	ibqrOdometry.q[X] = odom->q[X];
	ibqrOdometry.q[Y] = odom->q[Y];
	ibqrOdometry.q[Z] = odom->q[Z];
	ibqrOdometry.q[W] = odom->q[W];

	ibqrOdometry.rollspeed = odom->rollspeed;
	ibqrOdometry.pitchspeed = odom->pitchspeed;
	ibqrOdometry.yawspeed = odom->yawspeed;
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}