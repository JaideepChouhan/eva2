import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
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
		
	def dh_mat(self, theta, d, a, alpha):  # hehehe dih parameters hehehe
		"""
		input values are standatd DH parameters
		theta = rotation about z axis (joint angle)
		d = translation along z
		a = translation along x (link lneght)
		alpha = rotation about x(twist)
		
		
		order has change cause it is much cleaner to code this way
		"""
		ct, st = np.cos(theta), np.sin(theta)
		ca, sa = np.cos(alpha), np.sin(alpha)
		
		""" here a is alpha cause writing alpha 
		again again would be a pain in ass """
		
		return np.array([
		[ct     , -st    ,   0,         a],
		[st * ca, ct * ca, -sa, -(d * sa)],
		[st * sa, ct * sa,  ca,    d * ca],
		[0      , 0      ,  0 ,      1   ]
		])
		
	def compute_fk(self, q):
		# q = [q1, q2, q3, q4, q5]
		
		pi = np.pi
		
		"""
		as dh matrix is just matrix multiplication of i nuber
		of matrices where i is dof or joint
		"""
		
		T1 = self.dh_mat(q[0], 0.06, 0, -pi/2)
		T2 = self.dh_mat(q[1], 0   , 0, 0    )
		T3 = self.dh_mat(q[2], 0.18, 0, pi/2 )
		T4 = self.dh_mat(q[3], 0.16, 0, -pi/2)
		T5 = self.dh_mat(q[4], 0.04, 0, 0    )
		
		T =T1 @ T2 @ T3 @ T4 @ T5
		return T
		
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
