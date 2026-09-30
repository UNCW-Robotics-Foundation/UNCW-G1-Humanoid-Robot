import rclpy
from rclpy.node import Node
from g1_msgs.msg import ArmStates, MotorState, DebugData
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException, TransformBroadcaster
import numpy as np
from g1_ik.robot_arm_ik_v3 import G1_29_ArmIK
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import JointTrajectoryPoint
import logging_mp
from scipy.spatial.transform import Rotation as R
logger_mp = logging_mp.getLogger(__name__)
import pinocchio as pin

def main(args=None):
	arm_ik = G1_29_ArmIK()
	model = arm_ik.reduced_robot.model
	data = arm_ik.reduced_robot.data
	q0 = np.zeros(model.nq)

	pin.computeJointJacobians(model, data, q0)
	pin.updateFramePlacements(model, data)

	J_L = pin.getFrameJacobian(model, data, arm_ik.L_hand_id, pin.LOCAL_WORLD_ALIGNED)
	J_R = pin.getFrameJacobian(model, data, arm_ik.R_hand_id, pin.LOCAL_WORLD_ALIGNED)

	np.set_printoptions(precision=4, suppress=True)
	print("Left hand Jacobian:\n", J_L)
	print("Right hand Jacobian:\n", J_R)


if __name__ == '__main__':
    main()
