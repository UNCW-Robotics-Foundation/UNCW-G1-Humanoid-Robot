import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import numpy as np
import logging_mp
from g1_ik.robot_arm_ik_v3 import G1_29_ArmIK
import time
import pinocchio as pin
logger_mp = logging_mp.getLogger(__name__)


def main(args=None):
	arm_ik = G1_29_ArmIK()
	initial_x = 0.207
	initial_y = 0.129
	initial_z = 0.067
	matrix_default = np.array([
	    [0.982, 0.108, 0.155, 0.177],
	    [-0.105, 0.994, -0.024, -0.168],
	    [-0.157, 0.007, 0.988, 0.066],
	    [0.0, 0.0, 0.0, 1.0]])

	matrix = np.array([
      [0.984, 0.091, 0.156, initial_x],
      [-0.090, 0.996, -0.014, initial_y],
      [-0.156, 0.000, 0.988, initial_z],
      [0.0, 0.0, 0.0, 1.0]])

	urdf_path = "/home/temo/ik_ws/UNCW-G1-Humanoid-Robot/src/g1_ik/assets/g1_v2/g1_29dof_mode_15_brainco_hand.urdf"
	model = pin.buildModelFromUrdf(urdf_path)
	data = model.createData()
	model.gravity.linear = np.array([0.0, 0.0, -9.81])

	q = np.zeros(model.nq)
	v = np.zeros(model.nv)
	a = np.zeros(model.nv)

	for joint_id, joint in enumerate(model.joints):
		print(model.names[joint_id], ":", joint_id)

	tauf_ff = pin.rnea(model, data, q, v, a)
	print(len(tauf_ff))
    
	while True:
		time_start = time.time()
		sol_q, sol_tauff = arm_ik.solve_ik(matrix, matrix_default, np.array([ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), np.array([ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
		time_end = time.time()
		print("IK Solve Time: " + str(time_end - time_start))


if __name__ == '__main__':
	main()
