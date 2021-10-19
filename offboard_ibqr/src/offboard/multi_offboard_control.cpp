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
 * Original file information:
 *
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info.
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 *

 * Modified by
 * @author Marco Nunez <maez@alumni.stanford.edu>
 */

/******************************** Include Files *******************************/
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cstdint>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

/*********************************** Macros ***********************************/
/* Index values for the x, y, and z quantities in the traj_planned.points array */
#define X                                   0
#define Y                                   1
#define Z                                   2
#define W									3

/* Use the following parameters to configure simulation */
/* Note PX4 uses NED coordinate frame */
#define HOME_POSITION_X                     0       // x-coordinate for home
#define HOME_POSITION_Y                     0       // y-coordinate for home
#define HOME_POSITION_Z                     -2      // z-coordinate for home
#define HOME_POSITION_YAW					3.14	// Yaw for home (in rad)

#define TAKEOFF_SPEED						-0.5	// In meters/second

#define MAX_NUM_TRAJECTORY_PTS              30

#define ALLOWED_ERROR_4_HOME_REACHED        0.2		// In meters
#define ALLOWED_ERROR_4_POS_GOAL_REACHED    0.1		// In meters

#define OFFSET_Z							0.5		// TODO: There seems to be an offset on the z-dir odometry when commanding for home...

#define OFFBOARD_SETPOINT_COUNTER			100		// This is a counter to wait for the vehicle to be ready before sending the arm command

// 
#define TIME_BASED							1		// Set to 1 to use the time-based method. Set to 0 to use location-based method
#define TRAJ_TIME_INTERVAL					100		// in milliseconds

#define LOG_TRAJECTORY_ODOMETRY				1		// Set to 1 to log the trajectory odometry. Set to 0 to turn off

/************************** Local Function Prototypes *************************/


/****************************** Module Variables ******************************/


/**************************** OffboardControl Class ***************************/
class OffboardControl : public rclcpp::Node
{
public:
	//---- Constructor ----//
	OffboardControl() : Node("offboard_control", "iris1")
	{

#if LOG_TRAJECTORY_ODOMETRY
		trajectory_odometry_publisher_	 = this->create_publisher<VehicleOdometry>("TrajectoryOdometry", 10);
		trajectory_log_action_publisher_ = this->create_publisher<Char>("TrajectoryLogAction", 10);
#endif
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		trajectory_setpoint_publisher_   = this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
		vehicle_command_publisher_       = this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);


		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		// Subscribe to trajectories
		traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("planner/traj", 10, std::bind(&OffboardControl::traj_callback, this, _1));
		// Subscribe to odometry
		odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("VehicleOdometry_PubSubTopic", 10, std::bind(&OffboardControl::odom_callback, this, _1));
		// Subscribe to Vehicle control mode
		control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>("VehicleControlMode_PubSubTopic", 10, std::bind(&OffboardControl::control_mode_callback, this, _1));

		traj_planned  = new trajectory_msgs::msg::JointTrajectory;
		traj_index = -1;
		index_increment = 1;
		max_index = MAX_NUM_TRAJECTORY_PTS;

		homeReachedFlag = false;
		homeLocation.x = HOME_POSITION_X;
		homeLocation.y = HOME_POSITION_Y;
		homeLocation.z = HOME_POSITION_Z + OFFSET_Z;
		homeLocation.yaw = HOME_POSITION_YAW;
		homeLocation.vz = TAKEOFF_SPEED;

		offboard_setpoint_counter_ = 0;

		timer_callback_counter = 0;

		// ** MAIN LOOP ** //
		auto timer_callback = [this]() -> void {
			timer_callback_counter++;

			if ( (offboard_setpoint_counter_ == OFFBOARD_SETPOINT_COUNTER) || 
				 (offboard_setpoint_counter_ > OFFBOARD_SETPOINT_COUNTER && !ibqrMode.flag_armed)
			   )
			{
				// Change to Offboard mode after 100 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

           		 // stop the counter after reaching 101
			if (offboard_setpoint_counter_ < (OFFBOARD_SETPOINT_COUNTER + 1))
			{
				offboard_setpoint_counter_++;
			}
		};
		period = 5ms;
		timer_ = this->create_wall_timer(period, timer_callback);
	}

	//---- Class Public Methods ----//
	void arm() const;
	void disarm() const;

private:
	//---- Class Variables ----//
	rclcpp::TimerBase::SharedPtr timer_;

	// Publishers
#if LOG_TRAJECTORY_ODOMETRY
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr trajectory_odometry_publisher_;
	rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr trajectory_log_action_publisher_;
#endif
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// Subscribers
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;

	mutable TrajectorySetpoint positionTargetMsg{};	// Store the next position target from the planned trajectory

	mutable trajectory_msgs::msg::JointTrajectory *traj_planned; // Pointer to trajectory points
	mutable std::uint8_t traj_index;                // Current array position of the planned trajectory
	std::uint8_t index_increment;	                // Number of array positions to increment at the end of each execution
	std::uint8_t max_index;							// Maximum number of points expected in a planned trajectory

	mutable std::uint8_t homeReachedFlag;	        // Boolean to determine if the set home pose has been reached
	mutable TrajectorySetpoint homeLocation{};		// Home Position

	mutable VehicleOdometry ibqrOdometry;			// Store the vehicle's latest odometry information

	mutable VehicleControlMode ibqrMode;			// Store the current control mode for the vehicle

	std::atomic<uint64_t> timestamp_;				//!< common synced timestamped

	std::chrono::milliseconds period;

	uint64_t offboard_setpoint_counter_;			//!< counter for the number of setpoints sent

	uint64_t timer_callback_counter;				// count how many times the timer has ran

	//---- Class Private Methods ----//
	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;

	void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr traj) const;
	void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom) const;
	void control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr mode) const;

	// Copy the new trajectory point to positionTargetMsg to be published to mavros
	void get_newPositionTarget(void) const;
	// Determine if the home location has been reached
	bool isHomeReached(void) const;
	// Determine if a particular goal pose has been reached
	bool isGoalReached(void) const;
	// Set the PositionTarget to the home coordinates
	void goHome(void) const;
	// Clear the currently stored trajectory
	void clearTrajPlan(void) const;
};

/******************* OffboardControl Class Method Definitions *****************/
/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 */
void OffboardControl::publish_offboard_control_mode() const
{
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoints
 */
void OffboardControl::publish_trajectory_setpoint() const
{
	if (homeReachedFlag)
	{
    	positionTargetMsg.timestamp = timestamp_.load();
		trajectory_setpoint_publisher_->publish(positionTargetMsg);

#if TIME_BASED
		// We use (timer_callback_counter % (TRAJ_TIME_INTERVAL/period.count()) == 0) because we only want to get the next target
		// every TRAJ_TIME_INTERVAL milliseconds. Because the time is running at a period of period.count() we only want to enter
		// this if statement once the timer has expired a number of times that corresponds to TRAJ_TIME_INTERVAL
		// For example if TRAJ_TIME_INTERVAL = 100ms and period.count() = 5ms, the timer has to expire 20 times to cover the 100ms 
		// 
		if ( (timer_callback_counter % (TRAJ_TIME_INTERVAL/period.count()) == 0) && ((traj_index < max_index) && (traj_index >= 0)) ) // traj_index will only be less than 0 if no trajectory has ever been received
		{
			traj_index++;
			get_newPositionTarget();
		}
#if LOG_TRAJECTORY_ODOMETRY
		// We want to log twice as many trajectory points as given by the planner, so we just divide the frequency by two
		if ( (timer_callback_counter % (TRAJ_TIME_INTERVAL/(2*period.count())) == 0) )
		{
			trajectory_odometry_publisher_->publish(ibqrOdometry);
		}
#endif
		// For the last trajectory point we check that the goal is reached to ensure that the command is actually performed
		// If we did not check for the goal, the positionTargetMsg would be cleared period.count() milliseconds after the command
		// was sent and the controller would not have enough time to actually perform the operation
		if ( (traj_index == max_index) && isGoalReached() )
		{
#if LOG_TRAJECTORY_ODOMETRY
			// Prepare for the next trajectory log by adding a newline to the log file
			Char ch;
			ch.data = 'n';
			trajectory_log_action_publisher_->publish(ch);
#endif
			clearTrajPlan();
			goHome();
		}
#else
		if (isGoalReached())
		{
			if ((traj_index < max_index) && (traj_index >= 0)) // traj_index will only be less than 0 if no trajectory has ever been received
			{
				RCLCPP_INFO(this->get_logger(), "Goal Reached");
				traj_index++;
				get_newPositionTarget();
			}
		}
#endif
	}
	else
	{
		if (isHomeReached())
		{
			RCLCPP_INFO(this->get_logger(), "***Home Location Reached: Switching to Position Target Mode***");
			homeReachedFlag = true;

			// Get first position target
			if (traj_planned->points.empty())
			{
				RCLCPP_INFO(this->get_logger(), "No trajectory received. Setting target to home location");
				goHome();
			}
			else
			{
				get_newPositionTarget();
			}
		}
		else
		{
      		homeLocation.timestamp = timestamp_.load();

			/* For debugging */
			// static uint32_t count = 0;
			// count++;
			// TrajectorySetpoint msg{};
			// msg.timestamp = timestamp_.load();
			// msg.x = HOME_POSITION_X;
			// msg.y = HOME_POSITION_Y;
			// msg.z = -2;
			// msg.yaw = HOME_POSITION_YAW;
			// RCLCPP_INFO(this->get_logger(), "-pub %u: x=%f, y=%f, z=%f", count, msg.x, msg.y, msg.z);
			/*****************/

			trajectory_setpoint_publisher_->publish(homeLocation);
		}
	}
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) const
{
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
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
/**
 * @brief Calback for subscription to the planned trajectories
*/
void OffboardControl::traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr traj) const
{
	/* Incoming message description:
     * 'x': traj->points[0]
     * 'y': traj->points[1]
     * 'z': traj->points[2]
     *
     * x-positions: traj->points[0].positions[0..30]
     * y-positions: traj->points[1].positions[0..30]
     * z-positions: traj->points[2].positions[0..30]
     *
     * x-velocity: traj->points[0].velocities[0..30]
     * y-velocity: traj->points[1].velocities[0..30]
     * z-velocity: traj->points[2].velocities[0..30]
     *
     * x-acceleration: traj->points[0].accelerations[0..30]
     * y-acceleration: traj->points[1].accelerations[0..30]
     * z-acceleration: traj->points[2].accelerations[0..30]
     *
     * x-effort: traj->points[0].effort[0..30]
     * y-effort: traj->points[1].effort[0..30]
     * z-effort: traj->points[2].effort[0..30]
     *
     * x-time from start: traj->points[0].time_from_start
     * y-time from start: traj->points[1].time_from_start
     * z-time from start: traj->points[2].time_from_start
     */

	// Clear any pre-existing trajectory
	clearTrajPlan();
    // Store new trajectory - TODO : I am pretty sure there is no memory leak, but keep an eye out (clearTrajPlan should release the memory)
    traj_planned->header = traj->header;
    traj_planned->joint_names = traj->joint_names;
    traj_planned->points = traj->points;
    // Reset array index
    traj_index = 0;
    get_newPositionTarget();
}
/**
 * @brief Get the current control mode for the vehicle
*/
void OffboardControl::control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr mode) const
{
	// For now only check for armed and offboard flags
	ibqrMode.flag_armed = mode->flag_armed;
	ibqrMode.flag_control_offboard_enabled = mode->flag_control_offboard_enabled;
}
/**
 * @brief Copy the next trajectory point into positionTargetMsg to be published to px4
*/
void OffboardControl::get_newPositionTarget(void) const
{
  if (!traj_planned->points.empty())
  {
    RCLCPP_INFO(this->get_logger(), "New Target Received:");

	// We apply a transformation to the planned trajectory points because we plan in ENU and px4 expects NED
    positionTargetMsg.x = traj_planned->points[Y].positions[traj_index] + homeLocation.x;
    positionTargetMsg.y = traj_planned->points[X].positions[traj_index] + homeLocation.y;
    positionTargetMsg.z = -traj_planned->points[Z].positions[traj_index] + homeLocation.z - OFFSET_Z;

	positionTargetMsg.vx = traj_planned->points[Y].velocities[traj_index];
	positionTargetMsg.vy = traj_planned->points[X].velocities[traj_index];
	positionTargetMsg.vz = -traj_planned->points[Z].velocities[traj_index];

	positionTargetMsg.acceleration[X] = traj_planned->points[Y].accelerations[traj_index];
	positionTargetMsg.acceleration[Y] = traj_planned->points[X].accelerations[traj_index];
	positionTargetMsg.acceleration[Z] = -traj_planned->points[Z].accelerations[traj_index];

    RCLCPP_INFO(this->get_logger(), "rx = %f -- vx = %f -- ax = %f", positionTargetMsg.x, positionTargetMsg.vx, positionTargetMsg.acceleration[X]);
    RCLCPP_INFO(this->get_logger(), "ry = %f -- vy = %f -- ay = %f", positionTargetMsg.y, positionTargetMsg.vy, positionTargetMsg.acceleration[Y]);
    RCLCPP_INFO(this->get_logger(), "rz = %f -- vz = %f -- az = %f", positionTargetMsg.z, positionTargetMsg.vz, positionTargetMsg.acceleration[Z]);
  }
}
/**
 * @brief Determine if the home location has been reached
*/
bool OffboardControl::isHomeReached(void) const
{
	double epsilon = ALLOWED_ERROR_4_HOME_REACHED;
	bool check = false;

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
/**
 * @brief Determine if a particular goal pose has been reached
*/
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
/**
 * @brief Clear the currently stored trajectory and release memory back to the system
*/
void OffboardControl::clearTrajPlan(void) const
{
	delete traj_planned;
    traj_planned = new trajectory_msgs::msg::JointTrajectory;
    traj_index = -1;
    RCLCPP_INFO(this->get_logger(), "Trajectory Reset");
}

/********************************* Main Code *********************************/
int main(int argc, char* argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}


/********************************* Footnotes *********************************/


/******************************** End of file ********************************/
