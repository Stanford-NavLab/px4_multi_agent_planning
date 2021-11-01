import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from px4_msgs.msg import VehicleOdometry
from trajectory_msgs.msg import JointTrajectory
from multi_rtd_interfaces.msg import RobotTrajectory
from utility.parameter_utils import get_param_value
import numpy as np
from std_msgs.msg import Header

class PX4Interface(Node):
    """PX4 Interface

    "Middle-man" node which transforms incoming PX4 odometry messages to global ENU frame.
    
    """

    def __init__(self):
        super().__init__('px4_interface')

        # get name and agent number
        self.name = self.get_name()
        print("name: ", self.name)
        self.agent_num = int(self.name[-1])

        # get spawn locations from global parameters
        # client for querying global parameters
        self.client = self.create_client(GetParameters,
                                         '/global_parameter_server/get_parameters')
        request = GetParameters.Request()
        request.names = ['spawn_x','spawn_y']

        # dict for storying global params
        self.global_params = dict.fromkeys(request.names, None)

        # call GetParameters service
        self.client.wait_for_service()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_global_param)

        # interface is ready after params (spawn location) have been set
        self.ready = False

        # placeholder (gets set by parameter callback)
        self.SPAWN_LOC = None
        
        # trajectory subscriber
        self.traj_sub = self.create_subscription(
                RobotTrajectory,
                self.name + '/planner/traj',
                self.traj_callback,
                10)

        # trajectory publisher
        self.traj_pub = self.create_publisher(
                JointTrajectory,
                self.name + '/fmu/traj',
                10)
        
        # odometry subscriber
        self.odom_sub = self.create_subscription(
                VehicleOdometry,
                self.name + '/fmu/vehicle_odometry/out',
                self.odom_callback,
                10)
        
        # odometry publisher
        self.odom_pub = self.create_publisher(
                VehicleOdometry,
                self.name + '/planner/vehicle_odometry/out',
                10)


    def callback_global_param(self, future):
        """Parameter service callback.

        Parses the response from the parameter server and stores the parameters as class variables.

        """
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("service call failed %r" % (e,))
        else:
            i = 0
            for k in self.global_params.keys():
                param = result.values[i]
                self.global_params[k] = get_param_value(param)
                i = i + 1
            self.SPAWN_LOC = (self.global_params['spawn_x'][self.agent_num], 
                              self.global_params['spawn_y'][self.agent_num])
            print("spawn location ", self.SPAWN_LOC)
            print("PX4 Interface ready")
            self.ready = True
            

    def traj_callback(self, msg):
        """Trajectory callback.

        Handles Planner -> PX4 (offboard) trajectory translation. 

        """
        if self.ready:
            traj = msg.trajectory
            new_msg = JointTrajectory()
            new_msg.header = traj.header
            new_msg.joint_names = traj.joint_names
            new_msg.points = traj.points
            # transform position from ENU global to NED local
            new_msg.points[0].positions = list(np.asarray(traj.points[1].positions) - self.SPAWN_LOC[1])
            new_msg.points[1].positions = list(np.asarray(traj.points[0].positions) - self.SPAWN_LOC[0])
            new_msg.points[2].positions = list(-np.asarray(traj.points[2].positions))
            # transform velocity and acceleration from ENU to NED
            new_msg.points[0].velocities = traj.points[1].velocities
            new_msg.points[1].velocities = traj.points[0].velocities
            new_msg.points[2].velocities = list(-np.asarray(traj.points[2].velocities))
            new_msg.points[0].accelerations = traj.points[1].accelerations
            new_msg.points[1].accelerations = traj.points[0].accelerations
            new_msg.points[2].accelerations = list(-np.asarray(traj.points[2].accelerations))
            # copy over time_from_starts
            new_msg.points[0].time_from_start = traj.points[0].time_from_start
            new_msg.points[1].time_from_start = traj.points[1].time_from_start
            new_msg.points[2].time_from_start = traj.points[2].time_from_start
            # re-publish transformed message
            self.traj_pub.publish(new_msg)

    
    def odom_callback(self, msg):
        """Odometry callback.

        Handles PX4 -> Planner odometry translation. 

        """
        if self.ready:
            # TODO: handle transformation of orientation states
            #       and covariances
            new_msg = VehicleOdometry()
            # transform position from NED local to ENU global
            new_msg.x = msg.y + self.SPAWN_LOC[0]
            new_msg.y = msg.x + self.SPAWN_LOC[1]
            new_msg.z = -msg.z
            # transform velocity from NED to ENU
            new_msg.vx = msg.vy
            new_msg.vy = msg.vx
            new_msg.vz = -msg.vz
            # re-publish transformed message
            self.odom_pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    px4_interface = PX4Interface()

    rclpy.spin(px4_interface)

    px4_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
