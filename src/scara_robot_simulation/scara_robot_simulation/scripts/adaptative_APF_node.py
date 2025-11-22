#!/usr/bin/env python3
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
ARM_PLANE_HEIGHT = BASE_HEIGHT - L3 # z = 0.05 when d4 = 0

def calculate_dk(q):
    """
    Calculate the Direct Kinematics for the SCARA robot.
    
    """
    theta1, theta2, theta3, d4 = q
    theta12 = theta1 + theta2
    
    x = L1 * np.cos(theta1) + L2 * np.cos(theta12)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta12)  
    
    z = ARM_PLANE_HEIGHT + d4 
    
    phi = theta12 + theta3
    return np.array([x, y, z, phi])

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Pose().orientation
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class AdaptivePotentialFieldsPlanner(Node):
    def __init__(self):
        super().__init__('adaptative_APF_node')

        # --- Parâmetros do Planejador ---
        self.declare_parameter('goal.x', 0.15)
        self.declare_parameter('goal.y', 0.2)
        self.declare_parameter('goal.z', 0.1)
        self.declare_parameter('goal.phi_rad', 0.8) 

        self.declare_parameter('gains.k_att', 1.0)      # Atrative gain 
        self.declare_parameter('gains.eta_rep', 0.2)    # Repulsive gain
        self.declare_parameter('gains.n_adaptive', 1.0) # Adaptive exponent

        self.declare_parameter('robot.step_size', 0.02)
        self.declare_parameter('robot.goal_threshold', 0.02)
        
        self.goal_pose = np.array([
            self.get_parameter('goal.x').value,
            self.get_parameter('goal.y').value
        ])
        self.goal_z = self.get_parameter('goal.z').value
        self.goal_phi = self.get_parameter('goal.phi_rad').value

        self.k_att = self.get_parameter('gains.k_att').value
        self.eta_rep = self.get_parameter('gains.eta_rep').value
        self.n_adaptive = self.get_parameter('gains.n_adaptive').value

        self.step_size = self.get_parameter('robot.step_size').value
        self.goal_threshold = self.get_parameter('robot.goal_threshold').value

        
        self.obstacles = [
            {'pos': np.array([-0.1, 0.25]),  'radius': 0.08}, 
            {'pos': np.array([0.3, 0.1]), 'radius': 0.2},     
        ]
        
        self.get_logger().info(f'Adaptive Potential Fields Planner Node has started. goal at:{self.goal_pose}')
        self.get_logger().info(f'==> Goal at: {self.goal_pose}, Z={self.goal_z}, Phi={self.goal_phi} rad')
        self.get_logger().info(f'==> Gains: k_att={self.k_att}, eta_rep={self.eta_rep}, n_adaptive={self.n_adaptive}')

        
        self.current_joint_positions = None
        self.current_pose_xy = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.target_pose_pub = self.create_publisher(
            Pose,
            '/scara/target_pose', 
            10
        )

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
            
            full_pose = calculate_dk(q)
            self.current_pose_xy = full_pose[:2] 
        except ValueError as e:
            #Treat error if joint names are not found 
            pass

    def planner_loop(self):
        """main loop of the potential fields planner."""
        
        if self.current_pose_xy is None:
            self.get_logger().warn('Waiting actual pose...', throttle_duration_sec=5.0)
            return

        # Check if goal is reached
        dist_to_goal = np.linalg.norm(self.goal_pose - self.current_pose_xy)
        if dist_to_goal < self.goal_threshold:
            self.get_logger().info(f'Goal reached (dist: {dist_to_goal:.3f}m) ')
            self.timer.cancel() 
            self.publish_target_pose(self.goal_pose[0], self.goal_pose[1], self.goal_z)
            return

        # Calculate atractive and repulsive forces
        # Vector from current position to goal
        # Points TOWARDS the goal
        F_att = self.k_att * (self.goal_pose - self.current_pose_xy)

        # Repulsive forces from obstacles
        F_rep = np.array([0.0, 0.0])
        
        # Adaptive term based on distance to goal
        rho_g = dist_to_goal
        
        for obs in self.obstacles:
            dist_to_obs = np.linalg.norm(self.current_pose_xy - obs['pos'])
            rho_0 = obs['radius'] # Obstacle influence radius

            if dist_to_obs < rho_0:
                
                rho = dist_to_obs + 1e-6 # Avoid division by zero
                
                #Unit vector from obstacle to robot 
                gamma_OR = (self.current_pose_xy - obs['pos']) / rho
                
                # Unit vector from robot to goal
                gamma_RG = (self.goal_pose - self.current_pose_xy) / (rho_g + 1e-6)

                # commom term for both forces
                term_rho_common = (1.0 / rho) - (1.0 / rho_0)

                F_rep1_mag = self.eta_rep * term_rho_common * (rho_g**self.n_adaptive) / \
                             (rho**2 * (1.0 + rho_g**self.n_adaptive))
                
                F_rep2_mag = (self.n_adaptive / 2.0) * self.eta_rep * (term_rho_common**2) * \
                             (rho_g**(self.n_adaptive - 1.0)) / \
                             ((1.0 + rho_g**self.n_adaptive)**2)
                
                #Total repulsive force from this obstacle
                F_rep_obs = (F_rep1_mag * gamma_OR) + (F_rep2_mag * gamma_RG)
                
                F_rep += F_rep_obs

        # Sum of forces
        F_total = F_att + F_rep
        
        # Normalize total force to get direction
        norm_F_total = np.linalg.norm(F_total)
        if norm_F_total == 0:
            self.get_logger().warn('Total force is zero, no movement direction can be computed.( local minima?)', throttle_duration_sec=5.0)
            return
            
        direction = F_total / norm_F_total
        
        next_step_xy = self.current_pose_xy + direction * self.step_size

        self.publish_target_pose(next_step_xy[0], next_step_xy[1], self.goal_z)

        self.get_logger().info(f'Dist: {dist_to_goal:.2f}m | F_attr: [{F_att[0]:.2f}, {F_att[1]:.2f}] | F_rep: [{F_rep[0]:.2f}, {F_rep[1]:.2f}]', throttle_duration_sec=0.5)

    def publish_target_pose(self, x, y, z):
        """Publish the target pose for the end-effector."""
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        # Fixed orientation based on goal_phi
        q_orientation = quaternion_from_euler(0.0, 0.0, self.goal_phi)
        msg.orientation = q_orientation
        
        self.target_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePotentialFieldsPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Adaptive Potential Fields Planner Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()