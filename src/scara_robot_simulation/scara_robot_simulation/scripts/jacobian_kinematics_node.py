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

# Robot dimensions and limits 
L1 = 0.3
L2 = 0.2
L3 = 0.45
L12_WH = 0.05
BASE_HEIGHT = 0.5
J4_LIMIT_MIN = -L3 - L12_WH
J4_LIMIT_MAX = BASE_HEIGHT + L12_WH
ARM_PLANE_HEIGHT = BASE_HEIGHT - L3


class KinematicsJacobianNode(Node):
    """
    Inverse kinematics node using jacobian (Newton-Raphson).
    Topology: RRRP
    """

    def __init__(self):
        super().__init__('kinematics_jacobian_node')

        # Initial state
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joints_initialized = False

        self.dk_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.ik_subscription = self.create_subscription(
            Pose, '/scara/target_pose', self.ik_target_pose_callback, 10)
        
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/scara_arm_controller/follow_joint_trajectory')
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server coneected! Jacobian IK Node is ready.')

    def euler_from_quaternion(self, quaternion):
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        t3 = 2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def joint_state_callback(self, msg):
        try:
            # Map joint names to indices
            idx1 = msg.name.index('joint1')
            idx2 = msg.name.index('joint2')
            idx3 = msg.name.index('joint3')
            idx4 = msg.name.index('joint4')
            
            q = [msg.position[idx1], msg.position[idx2], msg.position[idx3], msg.position[idx4]]
            
            self.current_joint_positions = q
            self.joints_initialized = True

            # Forward Kinematics log
            pose = self.calculate_fk(q)
            self.get_logger().info(
                f'FK: x={pose[0]:.2f}, y={pose[1]:.2f}, z={pose[2]:.2f}, phi={pose[3]:.2f}',
                throttle_duration_sec=2.0
            )
        except (ValueError, IndexError):
            pass

    def calculate_fk(self, q):
        """Compute the forward kinematics for the SCARA RRRP robot."""
        theta1, theta2, theta3, d4 = q
        
        theta12 = theta1 + theta2
        phi = theta12 + theta3 
        
        x = L1 * np.cos(theta1) + L2 * np.cos(theta12)
        y = L1 * np.sin(theta1) + L2 * np.sin(theta12)
        z = ARM_PLANE_HEIGHT - d4 # Z axis is downwards
        
        return np.array([x, y, z, phi])

    def compute_jacobian(self, q):
        """
        Compute the Jacobian matrix for the SCARA RRRP robot.
        States: [x, y, z, phi]
        Joints:  [theta1, theta2, theta3, d4]
        """
        theta1, theta2, theta3, d4 = q
        
        s1 = np.sin(theta1)
        c1 = np.cos(theta1)
        s12 = np.sin(theta1 + theta2)
        c12 = np.cos(theta1 + theta2)

        # Parcial derivatives   
        # dx/dtheta1, dx/dtheta2, dx/dtheta3, dx/dd4
        j11 = -L1 * s1 - L2 * s12
        j12 = -L2 * s12
        j13 = 0.0
        j14 = 0.0

        # dy/dtheta1, dy/dtheta2, dy/dtheta3, dy/dd4
        j21 = L1 * c1 + L2 * c12
        j22 = L2 * c12
        j23 = 0.0
        j24 = 0.0

        # dz/dtheta1 ... dz/dd4 (only d4 affects Z)
        j31 = 0.0
        j32 = 0.0
        j33 = 0.0
        j34 = -1.0

        # dphi/dtheta1 ... (phi = th1 + th2 + th3)
        j41 = 1.0
        j42 = 1.0
        j43 = 1.0
        j44 = 0.0

        J = np.array([
            [j11, j12, j13, j14],
            [j21, j22, j23, j24],
            [j31, j32, j33, j34],
            [j41, j42, j43, j44]
        ])
        
        return J

    def solve_ik_jacobian(self, target_pose):
        """
        Solve ik using Levenberg-Marquardt algorithm or Damped Least Squares (DLS) method.
        target_pose: [x, y, z, phi]
        Returns joint angles [theta1, theta2, theta3, d4] or None if no solution found.
        """
        if not self.joints_initialized:
            self.get_logger().warn("Joints not initialized, using zeros as initial guess.")
            q_current = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            q_current = np.array(self.current_joint_positions)

        target_vector = np.array(target_pose) # [x, y, z, phi]
        
        # Parameters
        max_iterations = 500
        tolerance = 0.005 
        alpha = 0.5 
        
        # Damping factor
        # If higher, it tends to be more stable but converges slower near singularities
        # if lower, its faster but unstable near singularities (aproximates to the LMP(Least Mean Squares) problem)
        damping_lambda = 1.2

        for i in range(max_iterations):
            current_pose_fk = self.calculate_fk(q_current)
            error = target_vector - current_pose_fk
            
            if np.linalg.norm(error) < tolerance:
                self.get_logger().info(f'Converges in {i} iteractions.')
                return q_current

            J = self.compute_jacobian(q_current)

            # DLS Inverse Kinematics Step
            # delta_q = J.T * inv(J * J.T + lambda^2 * Identity) * error
            
            JJT = np.dot(J, J.T)
            
            damping_matrix = (damping_lambda**2) * np.eye(4)
            
            try:
                inv_term = np.linalg.inv(JJT + damping_matrix)
            except np.linalg.LinAlgError:
                 self.get_logger().error("MATH ERROR: Singular matrix encountered in IK computation.")
                 return None

            delta_q = np.dot(J.T, np.dot(inv_term, error))
            

            q_current = q_current + (alpha * delta_q)

            # Normalize angles to [-pi, pi]
            q_current[0] = (q_current[0] + np.pi) % (2 * np.pi) - np.pi
            q_current[1] = (q_current[1] + np.pi) % (2 * np.pi) - np.pi
            q_current[2] = (q_current[2] + np.pi) % (2 * np.pi) - np.pi

        self.get_logger().warn('Does not converge within the maximum number of iterations.')
        return q_current

    def ik_target_pose_callback(self, msg):
        phi = self.euler_from_quaternion(msg.orientation)
        target_pose = [msg.position.x, msg.position.y, msg.position.z, phi]
        
        self.get_logger().info(f'Computing IK for pose: {np.round(target_pose, 3)}')

        # Check reachability
        r_target = np.sqrt(target_pose[0]**2 + target_pose[1]**2)
        if r_target > (L1 + L2 + 0.05): # Margem pequena
             self.get_logger().error("Target out of reach in XY plane.")
             return

        solution = self.solve_ik_jacobian(target_pose)

        if solution is not None:
            self.send_trajectory_goal(solution)

    def send_trajectory_goal(self, joint_positions):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = [float(q) for q in joint_positions]
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.5 sec to reach
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsJacobianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()