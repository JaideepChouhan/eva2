import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_h_node')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Pose,
            '/eva2_0/head/kinect_node',
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

        def Rx(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[1, 0,  0, 0],
                             [0, c, -s, 0],
                             [0, s,  c, 0],
                             [0, 0,  0, 1]], dtype=float)

        # order -> pan, tilt, roll, kinect offset or where ever kinect is 		 
        return (trans(0, 0, 0.08) @ Rz(q[0]) 
                @ trans(0, 0, 0.07) @ Ry(q[1]) 
                @ trans(0, 0, 0) @ Rx(q[2])
                @ trans(0, 0, 0.06)) 	
            
    def joint_callback(self, msg):
        HEAD_JOINTS = [
            'head_pan_joint',
            'head_tilt_joint',
            'head_roll_joint',
        ]

        name_to_angle = dict(zip(msg.name, msg.position))

        if not all(j in name_to_angle for j in HEAD_JOINTS):
            return

        angles = [name_to_angle[j] for j in HEAD_JOINTS]

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
