import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Robot physical constants from the URDF
L1 = 0.3
L2 = 0.2
L3 = 0.45
L12_WH = 0.05
BASE_HEIGHT = 0.5
J4_LIMIT_MIN = -L3 - L12_WH
J4_LIMIT_MAX = BASE_HEIGHT + L12_WH
ARM_PLANE_HEIGHT = BASE_HEIGHT - L3


class KinematicsNode(Node):
    """A single node for both DK and IK for the Planar 3R+P robot."""

    def __init__(self):
        super().__init__('kinematics_node')

        self.current_joint_positions = None

        self.dk_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.ik_subscription = self.create_subscription(Pose, '/scara/target_pose', self.ik_target_pose_callback, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scara_arm_controller/follow_joint_trajectory')
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        
        self.get_logger().info('Smart Kinematics Node for Planar 3R+P Robot has started.')
        self.print_workspace_limits()

    def print_workspace_limits(self):
        self.get_logger().info('-----------------------------------------')
        self.get_logger().info('Calculating Robot Workspace Limits...')
        max_reach = L1 + L2 + L3
        self.get_logger().info(f'Max Horizontal Reach (R_max): {max_reach:.3f} meters')
        z_min = ARM_PLANE_HEIGHT - J4_LIMIT_MAX
        z_max = ARM_PLANE_HEIGHT - J4_LIMIT_MIN
        self.get_logger().info(f'Vertical Range (Z-axis): {z_min:.3f} m to {z_max:.3f} m')
        self.get_logger().info('-----------------------------------------')

    def euler_from_quaternion(self, quaternion):
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        t3 = 2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

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
                f'DK Pose: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}, phi={pose[3]:.3f} rad (with joints: {[round(e, 2) for e in q]})',
                throttle_duration_sec=1.0
            )
        except (ValueError, IndexError):
            self.get_logger().warn('Joints not fully available in /joint_states message yet.')

    @staticmethod
    def calculate_dk(q):
        theta1, theta2, theta3, d4 = q
        theta12 = theta1 + theta2
        phi = theta12 + theta3
        x = L1 * np.cos(theta1) + L2 * np.cos(theta12)
        y = L1 * np.sin(theta1) + L2 * np.sin(theta12)
        z = ARM_PLANE_HEIGHT + d4
        return [x, y, z, phi]

    def calculate_ik(self, pose):
        x, y, z, phi = pose
        dz = ARM_PLANE_HEIGHT - z

        if not (J4_LIMIT_MIN < dz < J4_LIMIT_MAX):
            self.get_logger().error(f'Target Z position ({z:.3f}m) is UNREACHABLE (joint4 would be {dz:.2f}).')
            return None, None

        c2 = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

        if c2 > 1 or c2 < -1:
            self.get_logger().error(f'Target XY wrist position is UNREACHABLE.')
            return None, None

        s2 = np.sqrt(1 - c2 ** 2)
        atan2_yx = np.arctan2(y, x)
        L2c2_L1 = L2 * c2 + L1

        theta1_p = atan2_yx - np.arctan2(L2 * s2, L2c2_L1)
        theta2_p = np.arctan2(s2, c2)
        theta3_p = phi - theta1_p - theta2_p
        sol_up = (theta1_p, theta2_p, theta3_p, dz)

        theta1_m = atan2_yx - np.arctan2(-L2 * s2, L2c2_L1)
        theta2_m = np.arctan2(-s2, c2)
        theta3_m = phi - theta1_m - theta2_m
        sol_down = (theta1_m, theta2_m, theta3_m, dz)
        return sol_up, sol_down

    def ik_target_pose_callback(self, msg):
        phi = self.euler_from_quaternion(msg.orientation)
        self.get_logger().info(
            f'IK received target pose: [x={msg.position.x}, y={msg.position.y}, z={msg.position.z}, phi={phi:.2f}]')

        target_pose = [msg.position.x, msg.position.y, msg.position.z, phi]

        sol_up, sol_down = self.calculate_ik(target_pose)

        if sol_up is None:  # If one is None, both are
            return  # Error message was already printed in calculate_ik

        if self.current_joint_positions is None:
            self.get_logger().warn(
                'Current joint positions not yet known. Defaulting to elbow-up solution for first move.')
            best_solution = sol_up
        else:
            # We ignore the prismatic joint as it's the same for both solutions
            q_current = np.array(self.current_joint_positions[:-1])
            q_up = np.array(sol_up[:-1])
            q_down = np.array(sol_down[:-1])

            # Calculate distance in joint space (sum of absolute differences for revolute joints) [Manhattan]
            dist_up = np.sum(np.abs(q_current - q_up))
            dist_down = np.sum(np.abs(q_current - q_down))

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

        point.time_from_start = Duration(sec=0, nanosec=100000000)

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        #self.get_logger().info('Waiting for action server...')
        #self._action_client.wait_for_server()
        #self.get_logger().info(f'Sending goal with joint positions: {np.round(joint_positions, 3)}')
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
