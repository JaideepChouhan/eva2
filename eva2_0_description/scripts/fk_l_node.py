import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
import numpy as np

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_l_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.publisher = self.create_publisher(
            Pose,
            '/eva2_0/left_arm/end_effector_pose',
            10
        )

        self.get_logger().info('FK node started')

    def compute_fk(self, q):

        def trans(x, y, z):
            M = np.eye(4)
            M[0,3], M[1,3], M[2,3] = x, y, z
            return M

        def Rz(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[c, -s, 0, 0],
                             [s,  c, 0, 0],
                             [0,  0, 1, 0],
                             [0,  0, 0, 1]], dtype=float)

        def Ry(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[ c, 0, s, 0],
                             [ 0, 1, 0, 0],
                             [-s, 0, c, 0],
                             [ 0, 0, 0, 1]], dtype=float)

        return (trans(0, 0, 0.3)
              @ trans(0.18, 0, 0.18)    
              			   @ Rz(q[0])
              @ trans(0, 0, -0.06) @ Ry(q[1])
              @ trans(0, 0, -0.18) @ Ry(q[2])
              @ trans(0, 0, -0.16) @ Rz(q[3])
              @ trans(0, 0, -0.04) @ Ry(q[4]))

    def joint_callback(self, msg):
        LEFT_ARM_JOINTS = [
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint'
        ]

        name_to_angle = dict(zip(msg.name, msg.position))

        if not all(j in name_to_angle for j in LEFT_ARM_JOINTS):
            return

        angles = [name_to_angle[j] for j in LEFT_ARM_JOINTS]

        T = self.compute_fk(angles)

        pose = Pose()
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]

        quat = Rotation.from_matrix(T[:3, :3]).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        self.publisher.publish(pose)
        self.get_logger().info(
            f'End effector → x={T[0,3]:.3f} y={T[1,3]:.3f} z={T[2,3]:.3f}'
        )


def main():
    rclpy.init()
    node = FKNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
