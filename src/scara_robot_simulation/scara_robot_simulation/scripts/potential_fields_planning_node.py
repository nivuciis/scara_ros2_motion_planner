import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

L1 = 0.3 
L2 = 0.2
L3 = 0.45
L12_WH = 0.05
BASE_HEIGHT = 0.5
J4_LIMIT_MIN = -L3 - L12_WH
J4_LIMIT_MAX = BASE_HEIGHT + L12_WH
ARM_PLANE_HEIGHT = BASE_HEIGHT - L3 # z = 0.05 quando d4 = 0

def calculate_dk(q):
    """
    Calculate the Direct Kinematics for the SCARA robot.
    Input:
        q: list or array of joint positions [theta1, theta2, theta3, d4]
    Output:
        pose: array [x, y, z, phi] where phi is the end-effector orientation
    Based on the kinematics_node.py implementation.
    """
    theta1, theta2, theta3, d4 = q
    theta12 = theta1 + theta2
    
    x = L1 * np.cos(theta1) + L2 * np.cos(theta12)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta12)  
    
    z = ARM_PLANE_HEIGHT + d4 
    
    phi = theta12 + theta3
    return np.array([x, y, z, phi])


class PotentialFieldsPlanner(Node):
    def __init__(self):
        super().__init__('potential_fields_planner')

        
        # Defines the GOAL position in the workspace
        self.goal_pose = np.array([0.15, 0.225])  # x, y coordinates
        self.goal_z = 0.1 # Fixed height for the end-effector (above the table)

        #Obstacle map
        # taken from the .world file
        self.obstacles = [
            {'pos': np.array([-0.1, 0.25]),  'radius': 0.08}, #Obstacle 1  
            {'pos': np.array([0.3, 0.1]), 'radius': 0.2}, # Obstacle 2
    
        ]
        

        #Pot Field parameters
        self.k_att = 1.0  # Atractive gain
        self.k_rep = 0.2 # Repulsive gain
        self.step_size = 0.02 # robot step size 
        self.goal_threshold = 0.02 #  to consider goal reached

        self.get_logger().info('Potential Fields Planner Node has started.')
        self.get_logger().info(f'==> Goal at: {self.goal_pose}')
        self.get_logger().info(f'==> Map obstacles: {len(self.obstacles)} ')

        
        self.current_joint_positions = None
        self.current_pose_xy = None

        # Subscriber (to get current joint states)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish on the target pose topic (the kinematics node will read it and move the robot)
        self.target_pose_pub = self.create_publisher(
            Pose,
            '/scara/target_pose', 
            10
        )

       # Timer to run the planner loop periodically
        self.timer = self.create_timer(0.05, self.planner_loop)

    def joint_state_callback(self, msg):
        """Update current joint positions and compute current end-effector pose."""
        try:
            q = [
                msg.position[msg.name.index('joint1')],
                msg.position[msg.name.index('joint2')],
                msg.position[msg.name.index('joint3')],
                msg.position[msg.name.index('joint4')]
            ]
            self.current_joint_positions = q
            
            #calculate current end-effector pose
            full_pose = calculate_dk(q)
            self.current_pose_xy = full_pose[:2] 
            

    def planner_loop(self):
        """main loop of the potential fields planner."""
        
        if self.current_pose_xy is None:
            self.get_logger().warn('Waiting for curent_pose...', throttle_duration_sec=5.0)
            return

    #Chek if goal is reached
        dist_to_goal = np.linalg.norm(self.goal_pose - self.current_pose_xy)
        if dist_to_goal < self.goal_threshold:
            self.get_logger().info(f'Goal reached, distance to goal: {dist_to_goal:.3f}m) ***')
            self.timer.cancel() 
            # Send final goal pose to ensure robot stops there
            self.publish_target_pose(self.goal_pose[0], self.goal_pose[1], self.goal_z)
            return

        # Calculate attractive force
        # Vector that points TOWARDS the goal
        F_att = self.k_att * (self.goal_pose - self.current_pose_xy)

        # Calculate repulsive forces from obstacles
        F_rep = np.array([0.0, 0.0])
        for obs in self.obstacles:
            vec_to_obs = obs['pos'] - self.current_pose_xy
            dist_to_obs = np.linalg.norm(vec_to_obs)
            
            # If within influence radius
            if dist_to_obs < obs['radius']:
                # Vector that points AWAY from the obstacle
                vec_away_from_obs = self.current_pose_xy - obs['pos']
                
                # Magnitude of the repulsive force
                # (1/d - 1/r_danger)
                F_rep_mag = self.k_rep * (1.0 / dist_to_obs - 1.0 / obs['radius'])
                
                # Add to total repulsive force
                F_rep += (F_rep_mag / dist_to_obs) * vec_away_from_obs

        # Sum of forces
        F_total = F_att + F_rep
        
        # Computes the direction of movement
        direction = F_total / np.linalg.norm(F_total)
        
        # Compute the next step
        next_step_xy = self.current_pose_xy + direction * self.step_size

        # Publishes the next target pose
        #  sends small steps
        self.publish_target_pose(next_step_xy[0], next_step_xy[1], self.goal_z)

        self.get_logger().info(f'Dist: {dist_to_goal:.2f}m | F_attr: [{F_att[0]:.2f}] | F_rep: [{F_rep[0]:.2f}]', throttle_duration_sec=0.5)

    def publish_target_pose(self, x, y, z):
        """Publish the target pose for the end-effector."""
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        
        # Fixed orientation (no rotation)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        self.target_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldsPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Potential Fields Planner Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()