#!/usr/bin/env python3
"""
revo2_udp_bridge.py — runs ON THE ROBOT. Listens on a UDP socket for
JSON joint packets sent by mediapipe_revo2_teleop.py (which runs on a
Mac, since that's where the webcam is and ROS2 isn't installed there),
and republishes them as JointState on /joint_commands_right, exactly
the way approach_until_contact.py and slider_control.py do.

Packet format (one UDP datagram per frame, ~30/sec):
    {"thumb_curl": 0.0-1.0, "thumb_rot": 0.0-1.0, "index": 0.0-1.0,
     "middle": 0.0-1.0, "ring": 0.0-1.0, "pinky": 0.0-1.0}

thumb_curl -> thumb, thumb_rot -> thumb_aux, others map 1:1.

NOTE ON UNITS (same as approach_until_contact.py): stark_node.cpp
multiplies /joint_commands_* positions by 100 before handing them to
the SDK, which expects 0-1000. So this bridge converts each 0-1 value
to raw 0-1000 (raw = value * 1000), then divides by 100 before
publishing (raw_to_joint_cmd), so the node's *100 nets out correctly.

No ack/retry: UDP is fire-and-forget. A dropped packet just means the
next one (whichever hand pose it captures next) supersedes it. If
packets stop arriving entirely (Mac disconnects, script killed, etc.)
this bridge simply stops publishing new commands -- the hand holds
its last commanded position, it does not go slack or move on its own.

Usage (on the robot, with ROS2 + this workspace already sourced):
    python3 revo2_udp_bridge.py --port 5599 --hand right
"""

import argparse
import json
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ['thumb', 'thumb_aux', 'index', 'middle', 'ring', 'pinky']
RAW_MAX = 1000  # SDK unified position range: 0=open, 1000=closed

# mediapipe_revo2_teleop.py channel name -> JOINT_NAMES entry
CHANNEL_TO_JOINT = {
    "thumb_curl": "thumb",
    "thumb_rot": "thumb_aux",
    "index": "index",
    "middle": "middle",
    "ring": "ring",
    "pinky": "pinky",
}


def raw_to_joint_cmd(raw: float) -> float:
    """0-1000 raw SDK position -> value to publish on /joint_commands_*,
    compensating for stark_node.cpp's *100 conversion."""
    return raw / 100.0


class Revo2UDPBridge(Node):
    def __init__(self, hand: str):
        super().__init__('revo2_udp_bridge')
        topic = f'/joint_commands_{"right" if hand == "right" else "left"}'
        self.pub = self.create_publisher(JointState, topic, 10)
        self.get_logger().info(f"Publishing to {topic}")

    def publish_from_channels(self, channels: dict):
        raw = {CHANNEL_TO_JOINT[k]: max(0.0, min(1.0, v)) * RAW_MAX
               for k, v in channels.items() if k in CHANNEL_TO_JOINT}
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [raw_to_joint_cmd(raw.get(name, 0)) for name in JOINT_NAMES]
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=5599)
    parser.add_argument('--hand', choices=['left', 'right'], default='right')
    args = parser.parse_args()

    rclpy.init()
    node = Revo2UDPBridge(args.hand)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    node.get_logger().info(f"Listening for UDP joint packets on port {args.port}")

    try:
        while rclpy.ok():
            data, _addr = sock.recvfrom(4096)
            try:
                channels = json.loads(data.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                node.get_logger().warn(f"Dropped malformed packet: {e}")
                continue
            node.publish_from_channels(channels)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
