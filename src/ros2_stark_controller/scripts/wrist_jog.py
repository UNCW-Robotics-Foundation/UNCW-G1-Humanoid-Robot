#!/usr/bin/env python3
"""
wrist_jog.py — CAUTIOUS, MANUAL, ONE-STEP-AT-A-TIME tool for discovering the
G1's WristRoll sign convention and incrementally working toward a ~90 degree
palm-up rotation for Experiment A (see current.md, "GRIP FORCE / WEIGHT
EXPERIMENT PLAN" and commands.md, "Grip-force / fingertip-force experiments").

WHY THIS EXISTS / WHY IT'S SEPARATE FROM experiment.py:
This talks to the G1's whole-arm torque-controlled arm_sdk interface
(unitree_hg/msg/LowCmd on topic "arm_sdk"), NOT the hand-only stark_node
path everything else in this project uses. That subsystem had never been
run or verified anywhere in this project before tonight. Investigated
brainco_ws/src/control_py/arm_ik_control/cal_arm_ik.py and confirmed real,
already-tuned kp/kd/gravity-compensation values exist there (not
placeholders — specific non-round tau_ff numbers only come from tuning on
real hardware), and robot_dof=29 is confirmed (not guessed) from
brainco_ws/src/control_py/config/smach_config.yaml. The gains and joint
list below are copied directly from that file. What is NOT known, and can
only be determined by watching the real robot, is which SIGN of WristRoll
rotation actually turns the palm up vs down. This script exists to find
that out as safely as possible:

  - Moves ONLY the wrist you specify (--hand), by a SMALL increment
    (--delta-deg, hard-capped at 20 degrees per run) from wherever it
    currently is — never a hardcoded absolute target.
  - Reads and holds every OTHER arm + waist joint at its currently-measured
    position for the whole run, so nothing else moves.
  - Uses the exact kp/kd/tau_ff gains already tuned in cal_arm_ik.py.
  - Follows Unitree's real arm_sdk enable/disable convention (the
    "kNotUsedJoint" weight slot), ramped, matching how
    brainco_ws/.../robot_control.py does it — no instantaneous jumps.
  - ALWAYS ramps the wrist back to its exact starting position and
    releases arm_sdk control before exiting — on normal completion AND on
    Ctrl+C.
  - Requires typed confirmation before enabling any motor control.

HOW TO USE IT: run once with a small --delta-deg (e.g. 10), physically
watch which way the wrist turns, note the direction. If you got the sign
wrong, run again with the opposite sign. Once you know the right sign, run
it again a few times in that direction (or raise --delta-deg up to the
20-degree cap) to walk up to ~90 degrees total, watching each time. This
is deliberately NOT a "just do 90 degrees in one shot" script.

DO NOT run this unattended. Stand where you can see the arm and can reach
the robot's stop/e-stop before starting.

Sourcing (needs unitree_hg message definitions, in addition to the usual
ROS2 + Stark environment):
  source /opt/ros/foxy/setup.bash
  source ~/unitree_ros2/setup.sh
  source ~/unitree-g1-brainco-hand/brainco_ws/install/setup.bash

Usage:
  python3 wrist_jog.py --hand right --delta-deg 10
  python3 wrist_jog.py --hand right --delta-deg -10   # if the first try turned the wrong way
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState

# G1JointIndex values, copied from
# brainco_ws/src/control_py/control_py/arm_ik_control/arm_joints.py
WAIST_YAW, WAIST_A, WAIST_B = 12, 13, 14
LEFT_SHOULDER_PITCH, LEFT_SHOULDER_ROLL, LEFT_SHOULDER_YAW, LEFT_ELBOW = 15, 16, 17, 18
LEFT_WRIST_ROLL, LEFT_WRIST_PITCH, LEFT_WRIST_YAW = 19, 20, 21
RIGHT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, RIGHT_SHOULDER_YAW, RIGHT_ELBOW = 22, 23, 24, 25
RIGHT_WRIST_ROLL, RIGHT_WRIST_PITCH, RIGHT_WRIST_YAW = 26, 27, 28
NOT_USED_JOINT = 29  # arm_sdk enable/disable "weight" slot

# Arm joint order + tau_ff, copied from cal_arm_ik.py's Arm(robot_dof=29).
# kp=100.0 and kd=1.5 uniformly for all of these (also copied, not guessed).
ARM_JOINTS = [
    LEFT_SHOULDER_PITCH, LEFT_SHOULDER_ROLL, LEFT_SHOULDER_YAW, LEFT_ELBOW,
    LEFT_WRIST_ROLL, LEFT_WRIST_PITCH, LEFT_WRIST_YAW,
    RIGHT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, RIGHT_SHOULDER_YAW, RIGHT_ELBOW,
    RIGHT_WRIST_ROLL, RIGHT_WRIST_PITCH, RIGHT_WRIST_YAW,
]
ARM_TAU_FF = [-1.5, 0.5, 0.7, -1.2, 0., -1.5, -0.5,
              -0.57, -1.5, -0.17, -0.5, -0.57, -1.5, -0.17]
ARM_KP = 100.0
ARM_KD = 1.5
WAIST_JOINTS = [WAIST_YAW, WAIST_A, WAIST_B]
WAIST_KP = 60.0
WAIST_KD = 1.5

WRIST_ROLL_FOR_HAND = {'left': LEFT_WRIST_ROLL, 'right': RIGHT_WRIST_ROLL}

MAX_DELTA_DEG = 20.0  # hard cap per run — walk up to 90 degrees over several runs
ENABLE_S = 0.5        # hold at weight=1, q=current (zero delta, no jump) before ramping
RAMP_S = 2.0           # how long the wrist move itself takes
RELEASE_S = 4.0        # ramp-down duration, matches robot_control.py's arm_hand_end
RATE_HZ = 50.0


class WristJog(Node):
    def __init__(self):
        super().__init__('wrist_jog')
        self.pub = self.create_publisher(LowCmd, 'arm_sdk', 10)
        self.low_state = None
        self.create_subscription(LowState, 'lowstate', self._on_state, 10)

    def _on_state(self, msg):
        self.low_state = msg

    def wait_for_state(self, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.low_state is not None:
                return True
        return False

    def _publish(self, weight, hold_q, target_joint=None, target_q=None):
        msg = LowCmd()
        for j in WAIST_JOINTS:
            msg.motor_cmd[j].q = hold_q[j]
            msg.motor_cmd[j].dq = 0.
            msg.motor_cmd[j].tau = 0.
            msg.motor_cmd[j].kp = WAIST_KP
            msg.motor_cmd[j].kd = WAIST_KD
        for i, j in enumerate(ARM_JOINTS):
            q = target_q if (target_joint is not None and j == target_joint) else hold_q[j]
            msg.motor_cmd[j].q = q
            msg.motor_cmd[j].dq = 0.
            msg.motor_cmd[j].tau = ARM_TAU_FF[i]
            msg.motor_cmd[j].kp = ARM_KP
            msg.motor_cmd[j].kd = ARM_KD
        msg.motor_cmd[NOT_USED_JOINT].q = weight
        self.pub.publish(msg)

    def run(self, hand, delta_rad):
        wrist_j = WRIST_ROLL_FOR_HAND[hand]
        period = 1.0 / RATE_HZ

        if not self.wait_for_state():
            raise RuntimeError(
                "No /lowstate received — is the robot's low-level state publishing?")

        hold_q = [self.low_state.motor_state[j].q for j in range(30)]
        start_q = hold_q[wrist_j]
        target_q = start_q + delta_rad
        self.get_logger().info(
            f"[{hand}] current WristRoll q={start_q:.4f} rad "
            f"({start_q * 57.2958:.1f} deg) -> target {target_q:.4f} rad "
            f"({target_q * 57.2958:.1f} deg), delta={delta_rad * 57.2958:+.1f} deg")

        try:
            # 1) enable at zero delta (weight 0->1, position = current, no jump)
            t0 = time.monotonic()
            while rclpy.ok() and time.monotonic() - t0 < ENABLE_S:
                rclpy.spin_once(self, timeout_sec=0.0)
                self._publish(1.0, hold_q)
                time.sleep(period)

            # 2) ramp the wrist from start_q to target_q, holding everything else
            t0 = time.monotonic()
            while rclpy.ok() and time.monotonic() - t0 < RAMP_S:
                rclpy.spin_once(self, timeout_sec=0.0)
                ratio = min(1.0, (time.monotonic() - t0) / RAMP_S)
                q = start_q + ratio * (target_q - start_q)
                self._publish(1.0, hold_q, target_joint=wrist_j, target_q=q)
                time.sleep(period)

            self.get_logger().info(
                "At target. Holding — physically observe the wrist now. "
                "Press Enter to release and revert to the starting position.")
            input()

        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted — reverting and releasing.")
        finally:
            # 3) ALWAYS ramp the wrist back to start_q while releasing weight 1->0,
            # whether we got here normally or via Ctrl+C.
            t0 = time.monotonic()
            while rclpy.ok() and time.monotonic() - t0 < RELEASE_S:
                rclpy.spin_once(self, timeout_sec=0.0)
                ratio = min(1.0, (time.monotonic() - t0) / RELEASE_S)
                q = target_q + ratio * (start_q - target_q)
                weight = 1.0 - ratio
                self._publish(weight, hold_q, target_joint=wrist_j, target_q=q)
                time.sleep(period)
            self.get_logger().info("Released.")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--hand', choices=['left', 'right'], required=True)
    parser.add_argument('--delta-deg', type=float, required=True,
                         help=f'signed degrees to rotate from the CURRENT position, '
                              f'max +/-{MAX_DELTA_DEG}')
    args = parser.parse_args()

    if abs(args.delta_deg) > MAX_DELTA_DEG:
        sys.exit(f"--delta-deg magnitude capped at {MAX_DELTA_DEG} per run for safety "
                 f"— run it multiple times to walk up to 90 degrees, watching each time.")

    print(f"\nThis will enable direct torque control of the G1's {args.hand.upper()} arm "
          f"and rotate ONLY its wrist by {args.delta_deg:+.1f} degrees from wherever it "
          f"currently is. Every other joint will be held at its current position.\n"
          f"Physically stand where you can see the arm and can reach the robot's stop "
          f"before continuing. This is NOT simulated — it moves the real robot.\n")
    if input("Type 'yes' to proceed: ").strip().lower() != 'yes':
        sys.exit("Aborted.")

    rclpy.init()
    node = WristJog()
    try:
        node.run(args.hand, args.delta_deg * 3.14159265 / 180.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
