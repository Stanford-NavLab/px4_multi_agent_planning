import rclpy
from rclpy.node import Node
from multi_rtd_interfaces.msg import Cylinder, CylinderArray

from world_parser import WorldParser
import numpy as np

class CylinderPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_cylinder_publisher')

        self.name = self.get_namespace()
        self.publisher = self.create_publisher(CylinderArray,self.name + 'map/cylinders',10)

        #self.declare_parameter('worldfile','/home/navlab-exxact-18/PX4-Autopilot/Tools/sitl_gazebo/worlds/static_forest.world')
        self.declare_parameter('worldfile','/home/navlab-exxact-18/PX4-Autopilot/Tools/sitl_gazebo/worlds/single_cylinder.world')
        self.declare_parameter('rate',10.0)

        self.cylinders = self.get_cylinders()
        self.cylinder_array = self.get_cylinder_array()

        period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(period,self.publish_map)

    def publish_map(self):
        self.publisher.publish(self.cylinder_array)

    def get_cylinders(self):
        worldfile = self.get_parameter('worldfile').value
        parser = WorldParser(worldfile)
        return parser.getCylinders()

    def get_cylinder_array(self):
        arr = [Cylinder() for _ in range(len(self.cylinders))]
        for i in range(len(self.cylinders)):
            c = self.cylinders[i]
            r = c['radius']; l = c['length']; center = c['pose'][:3]
            arr[i].center = center
            arr[i].length = l
            arr[i].radius = r

        cyl_arr = CylinderArray()
        cyl_arr.cylinders = arr
        return cyl_arr

def main(args=None):
    rclpy.init(args=args)

    map_publisher = CylinderPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
