import rclpy
from rclpy.node import Node
from multi_rtd_interfaces.msg import Cylinder, CylinderArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseStamped,PointStamped

import numpy as np
import matplotlib.pyplot as plt
import time

class RRTCylinder(Node):

    def __init__(self):
        super().__init__('rrt_planner_cylinders')

        self.ns = self.get_namespace()
        self.ns = '/iris_0/'

        # Setup default parameters
        self.declare_parameter('worldfile','/home/navlab-exxact-18/PX4-Autopilot/Tools/sitl_gazebo/worlds/static_forest.world')
        self.declare_parameter('planning_bounds',[0,50,0,50,0,10])
        self.declare_parameter('start_pos',[0,0,2])
        self.declare_parameter('goal_pos',[35,25,2])
        self.declare_parameter('plan_topic',self.ns + 'global_plan')
        self.declare_parameter('map_topic','/map/cylinders')
        self.declare_parameter('robot_radius',0.125)
        self.declare_parameter('max_step',0.75)
        self.declare_parameter('neighbor_radius',2.0)
        self.declare_parameter('max_planning_time',10.0)
        self.declare_parameter('goal_bias',0.1)
        self.declare_parameter('publish_rate',2)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_step = self.get_parameter('max_step').value
        self.goal_bias = self.get_parameter('goal_bias').value
        self.neighbor_radius = self.get_parameter('neighbor_radius').value
        self.cylinders = None

        # Setup publisher and subscribers
        self.map_sub = self.create_subscription(
                CylinderArray,
                self.get_parameter('map_topic').value,
                self.plan,
                10)

        self.plan_pub = self.create_publisher(
                Path,
                self.ns + 'global_plan',
                10)

    def plan(self,msg):

        # We only need to get the global map once
        self.cylinders = msg.cylinders
        self.destroy_subscription(self.map_sub)

        # Once we have the map we should plan
        path = None
        while not path:
            path = self.plan_path()
        print('Planned successfully')
        self.plan_msg = self.get_plan_msg(path)
        pub_period = 1.0 / self.get_parameter('publish_rate').value
        timer = self.create_timer(pub_period,self.publish_msg)

    def publish_msg(self):
        self.plan_pub.publish(self.plan_msg)

    def plan_path(self):

        self.start = np.array(self.get_parameter('start_pos').value,dtype=float)
        self.goal = np.array(self.get_parameter('goal_pos').value,dtype=float)

        bounds = np.array(self.get_parameter('planning_bounds').value)
        min_bound = bounds.take([0,2,4])
        max_bound = bounds.take([1,3,5])

        if not self.valid_point(self.start):
            raise Exception('Starting point intersects with an obstacle!')
        if not self.valid_point(self.goal):
            raise Exception('Goal point intersects with an obstacle!')
        
        # G looks like [(p1,parent1,cost1),...,(pn,parentn,costn)]
        self.G = [(self.start,-1,0)]
        xgoal = None
        t0 = time.time()
        while (time.time() - t0) < self.get_parameter('max_planning_time').value:
            prand = self.random_point(min_bound,max_bound)
            xnear_idx = self.nearest_idx(prand)
            xnear = self.G[xnear_idx]
            pnew = self.extend(xnear,prand)

            if not self.valid_point(pnew) or not self.valid_edge(xnear[0],pnew):
                continue

            neighbors = self.get_neighbors(pnew)
            (parent_idx,cost) = self.choose_parent(neighbors,xnear_idx,pnew)
            xnew = (pnew,parent_idx,cost)
            if np.all(pnew==self.goal):
                print('Found goal!')
                self.goal_bias = 0
                xgoal = xnew
            self.G.append(xnew)
            self.rewire(neighbors,xnew)
        if xgoal:
            #self.visualize_path(path)
            return self.get_path(xgoal)
        return None

    def get_plan_msg(self,path):
        poses = []
        for p in path:
            print(p)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            poses.append(pose)

        path = Path()
        path.poses = poses
        return path

    def get_path(self,xgoal):
        path = [xgoal[0]]
        curr = xgoal
        while True:
            if np.all(curr[0] == self.start):
                break
            curr = self.G[curr[1]]
            path.insert(0,curr[0])
        return path
            
    def random_point(self,min_bound,max_bound):
        '''
        Sample a random point given the problem bounds.
        Sample the goal position with probability goal_bias
        '''
        if np.random.random() < self.goal_bias:
            return self.goal
        return np.random.uniform(low=min_bound,high=max_bound)


    def rewire(self,neighbors,xnew):
        '''
        Rewire nodes in neighborhood of xnew to go through xnew
        if the cost is lower
        '''
        cost = xnew[2]
        xnew_idx = len(self.G)-1
        for (x,i) in neighbors:
            if self.valid_edge(x[0],xnew[0]):
                c = cost + self.dist(x[0],xnew[0])
                if c < x[2]:
                    self.G[i] = (x[0],xnew_idx,c)

    def choose_parent(self,neighbors,xnear_idx,pnew):
        '''
        '''
        xmin_idx = xnear_idx
        xnear = self.G[xnear_idx]
        min_cost = xnear[2] + self.dist(xnear[0],pnew)

        for (x,i) in neighbors:
            if self.valid_edge(x[0],pnew):
                cost = x[2] + self.dist(x[0],pnew)
                if cost < min_cost:
                    min_cost = cost
                    xmin_idx = i
        return (xmin_idx,min_cost)


    def get_neighbors(self,p):
        '''
        Return all neighbors within neighbor_radius from p
        neighbors = [(node,idx),...]
        '''
        neighbors = []
        for i in range(len(self.G)):
            x = self.G[i]
            if self.dist(p,x[0]) <= self.neighbor_radius:
                neighbors.append((x,i))

        return neighbors

    def extend(self,xnear,prand):
        '''
        Create a new node along the line xnear->xrand
        with length no more that max_step_size
        '''
        pnear = xnear[0]
        v = prand - pnear
        d = self.dist(pnear,prand)
        if d <= self.max_step:
            return prand
        return pnear + v/d*self.max_step

    def valid_point(self,p):
        '''
        Check if the point p is in free space
        '''
        for cyl in self.cylinders:
            c = cyl.center
            r = cyl.radius + self.robot_radius
            l = cyl.length + self.robot_radius

            d = self.dist(p[:2],c[:2])
            if d < r + self.robot_radius:
                return False

        return True

    def nearest_idx(self,prand):
        '''
        Find the node that is closest to xrand
        '''
        min_dist = float('inf')
        min_idx = None
        for i in range(len(self.G)):
            p = self.G[i][0]
            d = self.dist(p,prand)
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def valid_edge(self,s1,s2):
        '''
        Check if the edge connecting s1 and s2 is valid
        based on our obstacle set in self.cylinders
        '''
        s1 = np.array(s1); s2 = np.array(s2)
        edge_len = self.dist(s1,s2)
        for cyl in self.cylinders:
            c = cyl.center
            r = cyl.radius + self.robot_radius
            l = cyl.length + self.robot_radius

            # If we're too far away from the cylinder to have an intersection
            if self.dist(s1,c) > (edge_len + r + self.robot_radius):
                continue

            a = (s2[0]-s1[0])**2 + (s2[1]-s1[1])**2
            b = 2*(s1[0]-c[0])*(s2[0]-s1[0]) + 2*(s1[2]-c[1])*(s2[1]-s1[1])
            c = (s1[0]-c[0])**2 + (s1[1]-c[1])**2 - r**2

            # No solution
            if 4*a*c > b*b:
                continue

            # Valid solution
            disc = np.sqrt(b*b-4*a*c)
            t1 = (-b+disc)/(2*a); t2 = (-b-disc)/(2*a)

            # If we have a collision lets also check the height
            if (t1 >= 0 and t1 <= 1):
                return False
                '''
                p = s1 + (s2-s1)*t1
                if p[2] <= l:
                    return False
                '''
            if (t2 >= 0 and t2 <= 1):
                return False
                '''
                p = s1 + (s2-s1)*t2
                if p[2] <= l:
                    return False
                '''

        return True

    @staticmethod
    def dist(s1,s2):
        '''
        Euclidean distance between s1 and s2
        '''
        return np.sqrt(sum([(s1[i] - s2[i])**2 for i in range(len(s1))]))

    def visualize_path(self,path):
        bounds = np.array(self.get_parameter('planning_bounds').value)
        fig,ax = plt.subplots()

        ax.add_patch(plt.Circle(self.start[:2],0.5,color='r'))
        ax.add_patch(plt.Circle(self.goal[:2],0.5,color='g'))

        for cyl in self.cylinders:
            r = cyl.radius
            c = cyl.center[:2]
            ax.add_patch(plt.Circle(c,r,fill=False,color='b'))

        for i in range(len(path)-1):
            x = [path[i][0],path[i+1][0]]
            y = [path[i][1],path[i+1][1]]
            ax.add_line(plt.Line2D(x,y,color='g'))

        ax.set_xlim(bounds.take([0,1])); ax.set_ylim(bounds.take([2,3]))
        plt.show(block=True)


    def get_map(self,msg):

        # We only need to get the global map once
        self.cylinders = msg.cylinders
        self.destroy_subscription(self.map_sub)

        # Once we have the map we should plan
        self.build_tree()

def main(args=None):
    rclpy.init(args=args)
    global_planner = RRTCylinder()

    rclpy.spin(global_planner)

    global_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
