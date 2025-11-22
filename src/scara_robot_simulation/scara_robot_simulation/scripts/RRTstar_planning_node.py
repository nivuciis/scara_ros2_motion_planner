import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

L1 = 0.3
L2 = 0.2
L3 = 0.45
L12_WH = 0.05
BASE_HEIGHT = 0.5
J4_LIMIT_MIN = -L3 - L12_WH
J4_LIMIT_MAX = BASE_HEIGHT + L12_WH
ARM_PLANE_HEIGHT = BASE_HEIGHT - L3

def calculate_dk(q):
    theta1, theta2, theta3, d4 = q
    theta12 = theta1 + theta2
    x = L1 * np.cos(theta1) + L2 * np.cos(theta12)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta12)
    z = ARM_PLANE_HEIGHT + d4
    phi = theta12 + theta3
    return np.array([x, y, z, phi])


class RRTStarPlanner(Node):
    
    # Class to represent a node in the RRT* tree
    class RRTNode:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.cost = 0.0  # Initial cost
            self.parent = None # Parent node

    def __init__(self):
        super().__init__('rrt_star_planner')

        # Defines the GOAL position in the workspace
        self.goal_pose_xy = np.array([0.15, 0.2])  # x, y coordinates
        self.goal_z = 0.1 # Fixed height for the end-effector (above the table)


        #Obstacle map
        # taken from the .world file
        self.obstacles = [
            {'pos': np.array([-0.1, 0.25]),  'radius': 0.08}, #Obstacle 1  
            {'pos': np.array([0.3, 0.1]), 'radius': 0.2}, # Obstacle 2
    
        ]
        
        # RRT* parameters
        self.N_ITERATIONS = 2000      # number of iterations to build the tree
        self.STEP_SIZE = 0.08         # size between nodes
        self.NEAR_RADIUS = 0.15       # length to search for nearby nodes (RRT* specific)
        self.GOAL_THRESHOLD = 0.05    # distance to consider goal reached
        self.OBSTACLE_MARGIN = 0.02   # minimum distance from obstacles
        self.GOAL_BIAS_PROB = 0.1     # probability of sampling the goal

        # Workspace boundaries for sampling
        self.X_MIN, self.X_MAX = -0.4, 0.4 #The node should sample only between these limits in x-axis
        self.Y_MIN, self.Y_MAX = -0.4, 0.4 #The node should sample only between these limits in y-axis
        
        self.get_logger().info('RRT* planner Node has started.')


        self.current_pose_xy = None
        self.path_waypoints = None
        self.path_index = 0

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.target_pose_pub = self.create_publisher(
            Pose, '/scara/target_pose', 10)

        self.plan_timer = None
        self.exec_timer = None
        
        self.get_logger().info('waiting initial position ( /joint_states)...')
        
    def joint_state_callback(self, msg):
        """Update current joint positions and compute current end-effector pose."""
        if self.current_pose_xy is not None:
            try:
                q = [msg.position[msg.name.index(j)] for j in ['joint1', 'joint2', 'joint3', 'joint4']]
                full_pose = calculate_dk(q)
                self.current_pose_xy = full_pose[:2]
            except (ValueError, IndexError): pass
            return

        try:
            q = [msg.position[msg.name.index(j)] for j in ['joint1', 'joint2', 'joint3', 'joint4']]
            full_pose = calculate_dk(q)
            self.current_pose_xy = full_pose[:2]
            
            if self.plan_timer is None:
                self.get_logger().info(f'Initial position: {self.current_pose_xy}. initiating RRT*...')
                self.plan_timer = self.create_timer(0.1, self.run_planning_phase)
                
        except (ValueError, IndexError): pass

    def run_planning_phase(self):
        """Runs the RRT* planning algorithm."""
        self.plan_timer.cancel()

        # setup RRT* tree starting from current position
        start_node = self.RRTNode(self.current_pose_xy[0], self.current_pose_xy[1])
        self.tree = [start_node]
        
        for i in range(self.N_ITERATIONS):
            sampled_point = self.get_sample()
            nearest_node = self.get_nearest_node(self.tree, sampled_point)
            new_point_xy = self.steer(nearest_node, sampled_point)

            if not self.is_collision_free(new_point_xy[0], new_point_xy[1]):
                continue

            near_nodes = self.get_near_nodes(self.tree, new_point_xy)
            parent_node, parent_cost = self.choose_parent(nearest_node, near_nodes, new_point_xy)

            new_node = self.RRTNode(new_point_xy[0], new_point_xy[1])
            new_node.cost = parent_cost
            new_node.parent = parent_node
            self.tree.append(new_node)
            
            self.rewire_tree(new_node, near_nodes)

        self.get_logger().info(f'RRT* Plan complete {len(self.tree)} nodes generated. Generating final path...')
        self.generate_final_path()

    def get_sample(self):
        """Takes a random sample in the workspace, with goal biasing."""
        if np.random.rand() < self.GOAL_BIAS_PROB:
            return self.goal_pose_xy
        
        x_rand = np.random.uniform(self.X_MIN, self.X_MAX)
        y_rand = np.random.uniform(self.Y_MIN, self.Y_MAX)
        return np.array([x_rand, y_rand])

    def get_nearest_node(self, node_list, point_xy):
        """finds the nearest node in node_list to the given point_xy. using Euclidean distance."""
        distances = [np.linalg.norm([n.x - point_xy[0], n.y - point_xy[1]]) for n in node_list]
        return node_list[np.argmin(distances)]

    def steer(self, from_node, to_point_xy):
        """creates a new point in the direction from from_node to to_point_xy, at a distance of STEP_SIZE."""
        direction = to_point_xy - np.array([from_node.x, from_node.y])
        dist = np.linalg.norm(direction)
        direction = direction / (dist + 1e-6) # avoid division by zero
        
        if dist < self.STEP_SIZE:
            return to_point_xy
        else:
            return np.array([from_node.x, from_node.y]) + direction * self.STEP_SIZE

    def is_collision_free(self, x, y):
        """Cheks if the point (x, y) is collision-free considering obstacles and margins."""
        for obs in self.obstacles:
            dist = np.linalg.norm(obs['pos'] - np.array([x, y]))
            if dist < (obs['radius'] + self.OBSTACLE_MARGIN):
                return False
        return True

    def get_near_nodes(self, node_list, point_xy):
        """finds all nodes in node_list within NEAR_RADIUS of point_xy."""
        return [node for node in node_list if np.linalg.norm([node.x - point_xy[0], node.y - point_xy[1]]) < self.NEAR_RADIUS]

    def choose_parent(self, nearest_node, near_nodes, new_point_xy):
        """chooses the best parent for the new node based on minimum cost."""
        min_cost_node = nearest_node
        min_cost = nearest_node.cost + np.linalg.norm([nearest_node.x - new_point_xy[0], nearest_node.y - new_point_xy[1]])

        for near_node in near_nodes:
            cost = near_node.cost + np.linalg.norm([near_node.x - new_point_xy[0], near_node.y - new_point_xy[1]])
            if cost < min_cost and self.is_collision_free(new_point_xy[0], new_point_xy[1]):
                min_cost = cost
                min_cost_node = near_node
        return min_cost_node, min_cost

    def rewire_tree(self, new_node, near_nodes):
        """checks if any nearby nodes can be reached more cheaply via the new_node, and rewires them if so."""
        for near_node in near_nodes:
            cost_via_new = new_node.cost + np.linalg.norm([new_node.x - near_node.x, new_node.y - near_node.y])
            if cost_via_new < near_node.cost:
                near_node.parent = new_node
                near_node.cost = cost_via_new

    def generate_final_path(self):
        """Generates the final path from start to goal by backtracking from the goal node."""
        goal_node = self.get_nearest_node(self.tree, self.goal_pose_xy)
        
        if np.linalg.norm([goal_node.x - self.goal_pose_xy[0], goal_node.y - self.goal_pose_xy[1]]) > self.STEP_SIZE:
            self.get_logger().error('Plan failed to reach the goal within threshold !!!')
            return

        self.get_logger().info('PPath find to goal! Generating waypoints for execution...')
        self.path_waypoints = []
        curr = goal_node
        while curr is not None:
            self.path_waypoints.append(np.array([curr.x, curr.y]))
            curr = curr.parent
        self.path_waypoints.reverse()
        
        self.exec_timer = self.create_timer(0.1, self.run_execution_phase)


    def run_execution_phase(self):
        """Executes the planned path by publishing target poses sequentially."""
        if self.path_waypoints is None or self.current_pose_xy is None:
            return

        if self.path_index >= len(self.path_waypoints):
            self.get_logger().info(f'Path execution complete! distance to goal: {np.linalg.norm(self.goal_pose_xy - self.current_pose_xy):.3f}m ')
            self.exec_timer.cancel()
            return

        target_waypoint = self.path_waypoints[self.path_index]
        self.publish_target_pose(target_waypoint[0], target_waypoint[1], self.goal_z)
        dist_to_waypoint = np.linalg.norm(target_waypoint - self.current_pose_xy)
        
        if dist_to_waypoint < self.GOAL_THRESHOLD:
            self.get_logger().info(f'Waypoint {self.path_index} reached (distance: {dist_to_waypoint:.3f}m). Moving to next waypoint.')
            self.path_index += 1

    def publish_target_pose(self, x, y, z):
        """Creates and publishes a Pose message for the target end-effector position."""
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.w = 1.0
        self.target_pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()