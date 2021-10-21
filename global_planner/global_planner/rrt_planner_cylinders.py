import rclpy
from rclpy.node import Node

class RRTCylinder(Node):

    def __init__(self):
        super().__init__('rrt_planner_cylinders')

        self.ns = self.get_namespace()

    def get_map(self):
        pass
        


