import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import csv
import os

L1 = 0.3
L2 = 0.2
L3 = 0.45
L12_WH = 0.05
BASE_HEIGHT = 0.5
J4_LIMIT_MIN = -L3 - L12_WH
J4_LIMIT_MAX = BASE_HEIGHT + L12_WH
ARM_PLANE_HEIGHT = BASE_HEIGHT - L3

# Prismatic Joint Limits (Z-axis limits for the end effector)
# The end effector z is determined by: z = ARM_PLANE_HEIGHT + d4
# Assuming d4 ranges from approx 0 to L3 based scara limits
Z_MIN = ARM_PLANE_HEIGHT + J4_LIMIT_MIN # Lowest point
Z_MAX = ARM_PLANE_HEIGHT + J4_LIMIT_MAX # Highest point

# Clamp workspace Z to realistic values 
WORKSPACE_Z_MIN = 0.05 
WORKSPACE_Z_MAX = 0.5

def calculate_dk(q):
    theta1, theta2, theta3, d4 = q
    theta12 = theta1 + theta2
    x = L1 * np.cos(theta1) + L2 * np.cos(theta12)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta12)
    z = ARM_PLANE_HEIGHT - d4
    phi = theta12 + theta3
    return np.array([x, y, z, phi])


class RRTStarPlanner(Node):
    
    # Class to represent a node in the RRT* tree
    class RRTNode:
        def __init__(self, position):
            self.pos = np.array(position) #x, y, z
            self.cost = 0.0  # Initial cost
            self.parent = None # Parent node

    def __init__(self):
        super().__init__('rrt_star_planner')

        # Defines the GOAL position in the workspace
        self.goal_pose = np.array([0.15, 0.18, 0.1])  # x, y and z coordinates

        #Obstacle map
        # taken from the .world file
        self.obstacles = [
            # Cylinder: Check radius in XY if within Z range
            {
                'type': 'cylinder',
                'pos': np.array([-0.1, 0.23]), # X, Y center
                'z_min': -0.1, # Center Z (0.05) - Length/2 (0.15)
                'z_max': 0.2,  # Center Z (0.05) + Length/2 (0.15)
                'radius': 0.05
            },
            # Box: Check AABB
            {
                'type': 'box',
                'min': np.array([0.2, 0.0, -0.1]), # Center - Size/2
                'max': np.array([0.4, 0.2, 0.2])   # Center + Size/2
            }
        ]
        
        # RRT* parameters
        self.N_ITERATIONS = 3000      # number of iterations to build the tree
        self.STEP_SIZE = 0.05         # size between nodes
        self.NEAR_RADIUS = 0.04       # length to search for nearby nodes (RRT* specific)
        self.GOAL_THRESHOLD = 0.03    # distance to consider goal reached
        self.COLLISION_CHECK_RESOLUTION = 0.05
        self.OBSTACLE_MARGIN = 0.05   # minimum distance from obstacles
        self.GOAL_BIAS_PROB = 0.12        # probability of sampling the goal

        # Workspace boundaries for sampling
        self.X_MIN, self.X_MAX = -0.5, 0.5 #The node should sample only between these limits in x-axis
        self.Y_MIN, self.Y_MAX = -0.5, 0.5 #The node should sample only between these limits in y-axis
        self.Z_MIN, self.Z_MAX = WORKSPACE_Z_MIN, WORKSPACE_Z_MAX #The node should sample only between these limits in z-axis
        
        self.get_logger().info('RRT* planner Node has started.')


        self.current_pose = None
        self.path_waypoints = None
        self.path_index = 0

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.target_pose_pub = self.create_publisher(
            Pose, '/scara/target_pose', 10)

        self.plan_timer = None
        self.exec_timer = None
        
        self.get_logger().info('waiting initial pose ( /joint_states)...')
        
    def joint_state_callback(self, msg):
        """Update current joint positions and compute current end-effector pose."""
        if self.current_pose is not None:
            try:
                q = [msg.position[msg.name.index(j)] for j in ['joint1', 'joint2', 'joint3', 'joint4']]
                full_pose = calculate_dk(q)
                self.current_pose = full_pose[:3]
            except (ValueError, IndexError): pass
            return

        try:
            q = [msg.position[msg.name.index(j)] for j in ['joint1', 'joint2', 'joint3', 'joint4']]
            full_pose = calculate_dk(q)
            self.current_pose = full_pose[:3]
            
            if self.plan_timer is None:
                self.get_logger().info(f'Initial position: {self.current_pose}. initiating RRT*...')
                self.plan_timer = self.create_timer(0.1, self.run_planning_phase)
                
        except (ValueError, IndexError): pass

    def save_data_to_csv(self):
        """Saves the RRT* tree, path and obstacles to CSV files for visualization."""
        self.get_logger().info("Saving data to CSV files...")
        
        # 1. Save Tree Structure (Node -> Parent)
        with open('src/assets/rrt_tree.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'parent_x', 'parent_y', 'parent_z'])
            for node in self.tree:
                if node.parent is not None:
                    writer.writerow([
                        node.pos[0], node.pos[1], node.pos[2],
                        node.parent.pos[0], node.parent.pos[1], node.parent.pos[2]
                    ])

        # 2. Save Final Path
        with open('src/assets/rrt_path.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            if self.path_waypoints:
                for pt in self.path_waypoints:
                    writer.writerow([pt[0], pt[1], pt[2]])

        # 3. Save Obstacles
        with open('src/assets/obstacles.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['type', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6'])
            for obs in self.obstacles:
                if obs['type'] == 'cylinder':
                    # cylinder: x, y, z_min, z_max, radius
                    writer.writerow(['cylinder', obs['pos'][0], obs['pos'][1], obs['z_min'], obs['z_max'], obs['radius'], 0])
                elif obs['type'] == 'box':
                    # box: min_x, min_y, min_z, max_x, max_y, max_z
                    writer.writerow(['box', obs['min'][0], obs['min'][1], obs['min'][2], obs['max'][0], obs['max'][1], obs['max'][2]])

        self.get_logger().info(f"Data saved: {os.getcwd()}/rrt_tree.csv")
        
    def run_planning_phase(self):
        """Runs the RRT* planning algorithm."""
        self.plan_timer.cancel()

        # setup RRT* tree starting from current position
        start_node = self.RRTNode(self.current_pose)
        self.tree = [start_node]

        self.get_logger().info(f"Planning from {self.current_pose} to {self.goal_pose}")
        
        for i in range(self.N_ITERATIONS):
            sampled_point = self.get_sample()
            nearest_node = self.get_nearest_node(self.tree, sampled_point)
            new_point = self.steer(nearest_node, sampled_point)

            if not self.is_collision_free(new_point):
                continue

            if not self.check_segment_collision(nearest_node.pos, new_point):
                continue

            near_nodes = self.get_near_nodes(self.tree, new_point)
            parent_node, parent_cost = self.choose_parent(nearest_node, near_nodes, new_point)

            new_node = self.RRTNode(new_point)
            new_node.cost = parent_cost
            new_node.parent = parent_node
            self.tree.append(new_node)
            
            self.rewire_tree(new_node, near_nodes)

        self.get_logger().info(f'RRT* Plan complete {len(self.tree)} nodes generated. Generating final path...')
        self.generate_final_path()

    def get_sample(self):
        """Takes a random sample in the workspace, with goal biasing."""
        if np.random.rand() < self.GOAL_BIAS_PROB:
            return self.goal_pose
        
        #Random sample in the workspace boundaries
        return np.array([
            np.random.uniform(self.X_MIN, self.X_MAX),
            np.random.uniform(self.Y_MIN, self.Y_MAX),
            np.random.uniform(self.Z_MIN, self.Z_MAX)
        ])

    def get_nearest_node(self, node_list, point):
        """finds the nearest node in node_list to the given point_xy. using Euclidean distance."""
        distances = [np.linalg.norm(n.pos - point) for n in node_list]
        return node_list[np.argmin(distances)]

    def steer(self, from_node, to_point):
        """Interpolates on 3d space from from_node to to_point by STEP_SIZE."""
        direction = to_point - from_node.pos
        dist = np.linalg.norm(direction)
        direction = direction / (dist + 1e-6) # avoid division by zero
        
        if dist < self.STEP_SIZE:
            return to_point
        else:
            return from_node.pos + direction * self.STEP_SIZE

    def is_collision_free(self, point):
        """Checks if the point is collision-free considering obstacles and margins."""
        x, y, z = point
        for obs in self.obstacles:
            if obs['type'] == 'cylinder':
                # Check Z bounds first
                if obs['z_min'] <= z <= obs['z_max']:
                    # Check XY distance
                    dist_xy = np.linalg.norm(obs['pos'] - np.array([x, y]))
                    if dist_xy < (obs['radius'] + self.OBSTACLE_MARGIN):
                        return False
            
            elif obs['type'] == 'box':
                # Check AABB with margin
                if (obs['min'][0] - self.OBSTACLE_MARGIN <= x <= obs['max'][0] + self.OBSTACLE_MARGIN) and \
                   (obs['min'][1] - self.OBSTACLE_MARGIN <= y <= obs['max'][1] + self.OBSTACLE_MARGIN) and \
                   (obs['min'][2] - self.OBSTACLE_MARGIN <= z <= obs['max'][2] + self.OBSTACLE_MARGIN):
                    return False
        return True

    def check_segment_collision(self, start_pos, end_pos):
        """
        Discretizes the line between start and end and checks collision 
        at every interval. Prevents tunneling through obstacles.
        """
        dist = np.linalg.norm(end_pos - start_pos)
        if dist < 0.001:
            return True

        # Calculate number of steps based on resolution
        steps = int(dist / self.COLLISION_CHECK_RESOLUTION) + 1
        
        for i in range(1, steps + 1):
            t = i / steps
            intermediate_point = start_pos + t * (end_pos - start_pos)
            if not self.is_collision_free(intermediate_point):
                return False # Collision detected along path
        return True

    def get_near_nodes(self, node_list, point):
        """finds all nodes in node_list within NEAR_RADIUS of point."""
        x, y, z = point
        return [node for node in node_list if np.linalg.norm(node.pos - point) < self.NEAR_RADIUS]

    def choose_parent(self, nearest_node, near_nodes, new_point):
        """chooses the best parent for the new node based on minimum cost."""
        min_cost_node = nearest_node
        min_cost = nearest_node.cost + np.linalg.norm(nearest_node.pos - new_point)

        for near_node in near_nodes:
            if not self.check_segment_collision(near_node.pos, new_point):
                continue

            cost = near_node.cost + np.linalg.norm(near_node.pos - new_point)
            if cost < min_cost and self.is_collision_free(new_point):
                min_cost = cost
                min_cost_node = near_node
        return min_cost_node, min_cost

    def rewire_tree(self, new_node, near_nodes):
        """checks if any nearby nodes can be reached more cheaply via the new_node, and rewires them if so."""
        for near_node in near_nodes:
            if not self.check_segment_collision(new_node.pos, near_node.pos):
                continue
            cost_via_new = new_node.cost + np.linalg.norm(new_node.pos - near_node.pos)
            if cost_via_new < near_node.cost:
                near_node.parent = new_node
                near_node.cost = cost_via_new

    def generate_final_path(self):
        """Generates the final path from start to goal by backtracking from the goal node."""
        goal_node = self.get_nearest_node(self.tree, self.goal_pose)
        
        if np.linalg.norm(goal_node.pos - self.goal_pose) > self.GOAL_THRESHOLD:
            self.get_logger().warn('Goal not reached accurately. Executing closest path.')

        self.path_waypoints = []
        curr = goal_node
        while curr is not None:
            self.path_waypoints.append(curr.pos)
            curr = curr.parent
        self.path_waypoints.reverse()
        self.path_waypoints.append(self.goal_pose)
        
        self.save_data_to_csv()
        self.get_logger().info(f'Path generated with {len(self.path_waypoints)} waypoints.')
        self.exec_timer = self.create_timer(0.2, self.run_execution_phase)


    def run_execution_phase(self):
        """Executes the planned path by publishing target poses sequentially."""
        if self.path_waypoints is None or self.current_pose is None:
            return

        if self.path_index >= len(self.path_waypoints):
            self.get_logger().info(f'Execution finished, distance to goal: {np.linalg.norm(self.goal_pose - self.current_pose):.3f}m ')
            self.exec_timer.cancel()
            return

        target_waypoint = self.path_waypoints[self.path_index]
        self.publish_target_pose(target_waypoint)
        dist_to_waypoint = np.linalg.norm(target_waypoint - self.current_pose)
        
        if dist_to_waypoint < self.GOAL_THRESHOLD:
            self.get_logger().info(f'Waypoint {self.path_index} reached (distance: {dist_to_waypoint:.3f}m). Moving to next waypoint...')
            self.path_index += 1

    def publish_target_pose(self, point):
        """Creates and publishes a Pose message for the target end-effector position."""
        msg = Pose()
        x, y, z = point
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