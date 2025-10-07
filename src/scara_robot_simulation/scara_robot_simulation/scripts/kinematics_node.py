import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import math

# Message/Action imports
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Robot physical constants from the URDF
L1 = 0.5
L2 = 0.4
L3 = 0.4
BASE_HEIGHT = 1.0
J3_Z_OFFSET = -0.5
J3_LIMIT_MIN = 0.0
J3_LIMIT_MAX = 1.0
ARM_PLANE_HEIGHT = BASE_HEIGHT + J3_Z_OFFSET

class KinematicsNode(Node):
    """A single node for both DK and IK for the Planar 3R+P robot."""

    def __init__(self):
        super().__init__('kinematics_node')

        # --- NEW: State tracking variable ---
        self.current_joint_positions = None

        self.dk_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.ik_subscription = self.create_subscription(Pose, '/scara/target_pose', self.ik_target_pose_callback, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scara_arm_controller/follow_joint_trajectory')

        self.get_logger().info('Smart Kinematics Node for Planar 3R+P Robot has started.')
        self.print_workspace_limits()

    def print_workspace_limits(self):
        self.get_logger().info('-----------------------------------------')
        self.get_logger().info('Calculating Robot Workspace Limits...')
        max_reach = L1 + L2 + L3
        self.get_logger().info(f'Max Horizontal Reach (R_max): {max_reach:.3f} meters')
        z_min = ARM_PLANE_HEIGHT + J3_LIMIT_MIN
        z_max = ARM_PLANE_HEIGHT + J3_LIMIT_MAX
        self.get_logger().info(f'Vertical Range (Z-axis): {z_min:.3f} m to {z_max:.3f} m')
        self.get_logger().info('-----------------------------------------')

    def euler_from_quaternion(self, quaternion):
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    # --- UPDATED: This callback now handles both DK logging and state tracking ---
    def joint_state_callback(self, msg):
        try:
            q = [
                msg.position[msg.name.index('joint1')],
                msg.position[msg.name.index('joint2')],
                msg.position[msg.name.index('joint3')],
                msg.position[msg.name.index('joint4')]
            ]
            # Store the current position for the IK solver
            self.current_joint_positions = q

            # Log the DK calculation (throttled)
            pose = self.calculate_dk(q)
            self.get_logger().info(
                f'DK Pose: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}, phi={pose[3]:.3f} rad',
                throttle_duration_sec=1.0
            )
        except (ValueError, IndexError):
            self.get_logger().warn('Joints not fully available in /joint_states message yet.')

    def calculate_dk(self, q):
        theta1, theta2, d3, theta4 = q
        theta12 = theta1 + theta2
        phi = theta12 + theta4
        x = L1 * np.cos(theta1) + L2 * np.cos(theta12) + L3 * np.cos(phi)
        y = L1 * np.sin(theta1) + L2 * np.sin(theta12) + L3 * np.sin(phi)
        z = ARM_PLANE_HEIGHT + d3
        return [x, y, z, phi]

    # --- UPDATED: This function now returns BOTH IK solutions ---
    def calculate_ik(self, pose):
        x, y, z, phi = pose
        d3 = z - ARM_PLANE_HEIGHT

        if not (J3_LIMIT_MIN <= d3 <= J3_LIMIT_MAX):
            self.get_logger().error(f'Target Z position ({z:.3f}m) is UNREACHABLE.')
            return None, None

        xw = x - L3 * np.cos(phi)
        yw = y - L3 * np.sin(phi)
        r_sq = xw**2 + yw**2

        if r_sq > (L1 + L2)**2 or r_sq < (L1 - L2)**2:
            self.get_logger().error(f'Target XY wrist position is UNREACHABLE.')
            return None, None

        cos_theta2 = (r_sq - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

        # Calculate both "elbow up" and "elbow down" solutions
        sin_theta2_up = np.sqrt(1 - cos_theta2**2)
        sin_theta2_down = -sin_theta2_up

        theta2_up = np.arctan2(sin_theta2_up, cos_theta2)
        theta2_down = np.arctan2(sin_theta2_down, cos_theta2)

        # --- Elbow Up Solution ---
        k1_up = L1 + L2 * cos_theta2
        k2_up = L2 * sin_theta2_up
        theta1_up = np.arctan2(yw, xw) - np.arctan2(k2_up, k1_up)
        theta4_up = phi - theta1_up - theta2_up
        sol_up = [theta1_up, theta2_up, d3, theta4_up]

        # --- Elbow Down Solution ---
        k1_down = L1 + L2 * cos_theta2
        k2_down = L2 * sin_theta2_down
        theta1_down = np.arctan2(yw, xw) - np.arctan2(k2_down, k1_down)
        theta4_down = phi - theta1_down - theta2_down
        sol_down = [theta1_down, theta2_down, d3, theta4_down]

        return sol_up, sol_down

    # --- UPDATED: This callback now chooses the best solution ---
    def ik_target_pose_callback(self, msg):
        phi = self.euler_from_quaternion(msg.orientation)
        self.get_logger().info(f'IK received target pose: [x={msg.position.x}, y={msg.position.y}, z={msg.position.z}, phi={phi:.2f}]')

        target_pose = [msg.position.x, msg.position.y, msg.position.z, phi]

        sol_up, sol_down = self.calculate_ik(target_pose)

        if sol_up is None: # If one is None, both are
            return # Error message was already printed in calculate_ik

        # --- Logic to choose the closest solution ---
        if self.current_joint_positions is None:
            self.get_logger().warn('Current joint positions not yet known. Defaulting to elbow-up solution for first move.')
            best_solution = sol_up
        else:
            q_current = np.array(self.current_joint_positions)
            q_up = np.array(sol_up)
            q_down = np.array(sol_down)

            # Calculate distance in joint space (sum of absolute differences for revolute joints)
            # We ignore the prismatic joint as it's the same for both solutions
            dist_up = np.sum(np.abs(q_current[[0, 1, 3]] - q_up[[0, 1, 3]]))
            dist_down = np.sum(np.abs(q_current[[0, 1, 3]] - q_down[[0, 1, 3]]))

            if dist_up <= dist_down:
                best_solution = sol_up
                self.get_logger().info(f'Choosing elbow-up solution (closest path, distance: {dist_up:.3f})')
            else:
                best_solution = sol_down
                self.get_logger().info(f'Choosing elbow-down solution (closest path, distance: {dist_down:.3f})')

        self.send_trajectory_goal(best_solution)

    def send_trajectory_goal(self, joint_positions):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = [float(q) for q in joint_positions]
        point.time_from_start = Duration(sec=3)
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal with joint positions: {np.round(joint_positions, 3)}')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
