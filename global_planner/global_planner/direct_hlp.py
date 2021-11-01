import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleOdometry

import numpy as np

from utility.parameter_utils import get_param_value

class DirectHLP(Node):
    """Direct High-level-planner

    High-level planner which simply takes the high-level goals and feeds them directly to
    the trajectory planner. Mostly used for debugging purposes.
    
    """

    def __init__(self):
        super().__init__('direct_hlp')

        # get name and agent number
        self.name = self.get_name()
        self.agent_num = int(self.name[-1])

        # get goal position from global params
        # client for querying global parameters
        self.client = self.create_client(GetParameters,
                                         '/global_parameter_server/get_parameters')
        request = GetParameters.Request()
        request.names = ['goal_x','goal_y','goal_z']

        # dict for storying global params
        self.global_params = dict.fromkeys(request.names, None)

        # call GetParameters service
        self.client.wait_for_service()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_global_param)

        # placeholder (gets set by parameter callback)
        self.GOAL_POS = np.zeros((3,))

        # goal pub timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_goal)
        
        # goal publisher
        self.goal_pub = self.create_publisher(
                PointStamped,
                self.name + '/planner/goal',
                10)

        # goal set flag
        self.goal_set = False


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
            self.GOAL_POS[0] = self.global_params['goal_x'][self.agent_num]
            self.GOAL_POS[1] = self.global_params['goal_y'][self.agent_num]
            self.GOAL_POS[2] = self.global_params['goal_z'][self.agent_num]
            print("Goal position:", tuple(self.GOAL_POS))
            self.goal_set = True
            

    def publish_goal(self):
        if self.goal_set:
            #print("publishing goal")
            pose = PointStamped()
            pose.point.x = float(self.GOAL_POS[0])
            pose.point.y = float(self.GOAL_POS[1])
            pose.point.z = float(self.GOAL_POS[2])
            self.goal_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    direct_hlp = DirectHLP()

    rclpy.spin(direct_hlp)

    direct_hlp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
