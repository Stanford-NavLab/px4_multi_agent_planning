import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseStamped,PointStamped
from px4_msgs.msg import VehicleOdometry

import numpy as np

class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.ns = self.get_namespace()
        self.ns = '/iris_0/'

        self.declare_parameter('proximity_threshold',0.5)
        self.pose = np.array([0,0,0],dtype=float)
        self.global_plan = None
        self.waypoint_idx = 0

        # Publishers and subscribers
        self.odom_sub = self.create_subscription(
                VehicleOdometry,
                self.ns + 'fmu/vehicle_odometry/out',
                self.set_pose,
                10)

        self.plan_sub = self.create_subscription(
                Path,
                self.ns + 'global_plan',
                self.get_global_plan,
                10)
        
        self.waypoint_pub = self.create_publisher(
                PointStamped,
                self.ns + 'planner/goal',
                10)
        

    def get_global_plan(self,msg):
        self.global_plan = [[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]
        self.destroy_subscription(self.plan_sub)

    def set_pose(self,msg):
        self.pose[0] = msg.x
        self.pose[1] = msg.y
        self.pose[2] = msg.z

        if self.global_plan:
            self.publish_waypoint()

    def publish_waypoint(self):
        waypoint = self.global_plan[self.waypoint_idx]
        flipped_pose = np.array([self.pose[1],self.pose[0],-self.pose[2]])
        d = np.linalg.norm(waypoint[:2]-flipped_pose[:2])
        print(d)
        if d < self.get_parameter('proximity_threshold').value:
            self.waypoint_idx += 1
            waypoint = self.global_plan[self.waypoint_idx]
            print('Update!')
        pose = PointStamped()
        pose.point.x = waypoint[0]
        pose.point.y = waypoint[1]
        pose.point.z = waypoint[2]
        self.waypoint_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)

    waypoint_pub = WaypointPublisher()

    rclpy.spin(waypoint_pub)

    waypoint_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
