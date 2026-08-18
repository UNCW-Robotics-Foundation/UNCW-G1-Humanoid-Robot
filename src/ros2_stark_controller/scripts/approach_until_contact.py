#!/usr/bin/env python3
"""
approach_until_contact.py — close a finger (or fingers) slowly until the
touch sensor reports contact, then stop.

Contact = any of a finger's normal_force{1,2,3} or tangential_force{1,2,3}
readings exceeds --force-threshold (default 20, raw sensor units) for
--debounce (default 3) consecutive ticks — the debounce exists because a
single noisy sample above threshold would otherwise permanently freeze a
finger right at its starting position. Proximity values are logged on
contact for visibility but are not a stop condition by default — pass
--proximity-threshold to also stop on self/mutual proximity if you have a
calibrated value for your object.

NOTE ON UNITS: stark_node.cpp converts /joint_commands_* positions by
multiplying by 100 (`pos[i] = msg->position[i] * 100`) before handing them
to the SDK, which expects the unified 0-1000 range. That factor is left
over from an older SDK generation that used a 0-100 range and was never
updated for the v2.0.2 upgrade. Rather than touch the already-fragile
compiled stark_node (see the SDK memory-leak mitigation in
stark_node.service), this script works around it by publishing commands
pre-divided by 100 (see raw_to_joint_cmd), so the node's *100 nets out to
the real 0-1000 SDK range. If stark_node.cpp is ever fixed to send raw
positions straight through, change raw_to_joint_cmd to `raw / 1000.0`.

Also note: ros2_stark_interfaces/MotorStatus.msg declares positions as
uint8[6] (max 255), but the SDK's real range is 0-1000 — /motor_status
truncates for any position above ~25% closed. This script does not read
/motor_status for that reason; it tracks its own commanded position,
starting from a known fully-open state at the start of each run.

Usage:
  python3 approach_until_contact.py --mode single --hand right --finger index
  python3 approach_until_contact.py --mode hand   --hand right
  python3 approach_until_contact.py --mode both

Pass --log-csv <path> to record a per-tick time series (elapsed time,
commanded position, forces, proximity, contact flag) for every targeted
finger, for offline plotting with plot_approach_log.py.

THUMB_AUX / --thumb-mode: thumb_aux is the thumb's rotation joint (opposition),
separate from the main thumb flexion joint used for approach-until-contact.
It's held at a fixed position for the whole run (it has no touch sensor, so
it isn't something that "approaches" anything). --thumb-mode flat/out picks
a preset raw position; THUMB_AUX_RAW below is confirmed correct on hardware
(2026-07-02) — 'out' rotates the thumb out for opposition as expected. Use
--thumb-aux-raw to dial in a different exact 0-1000 value if ever needed.
"""

import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_stark_interfaces.msg import TouchStatus

JOINT_NAMES = ['thumb', 'thumb_aux', 'index', 'middle', 'ring', 'pinky']
FINGERS = ['thumb', 'index', 'middle', 'ring', 'pinky']  # touch-sensed fingers (no thumb_aux)
TOUCH_INDEX = {name: i for i, name in enumerate(FINGERS)}

RAW_MAX = 1000  # SDK unified position range: 0=open, 1000=closed

# thumb_aux presets — raw 0-1000. Confirmed correct on hardware 2026-07-02.
THUMB_AUX_RAW = {
    'flat': 0,
    'out': 1000,
}


def parse_start_pos(value: str):
    """argparse type for --start-pos. Accepts either a single raw 0-1000
    int (applied to every targeted finger) or a comma-separated list of
    finger=value pairs, e.g. 'thumb=250,index=400,middle=380,ring=350,
    pinky=200', for an asymmetric per-finger starting pose (e.g. wrapped
    around a tool where each finger sits at a different position).
    Fingers not mentioned in the list default to 0."""
    value = value.strip()
    try:
        return int(value)
    except ValueError:
        pass
    result = {}
    for pair in value.split(','):
        pair = pair.strip()
        if not pair:
            continue
        if '=' not in pair:
            raise argparse.ArgumentTypeError(
                f"invalid --start-pos entry {pair!r} — expected finger=value (e.g. thumb=250)")
        finger, raw = pair.split('=', 1)
        finger = finger.strip()
        if finger not in FINGERS:
            raise argparse.ArgumentTypeError(
                f"unknown finger {finger!r} in --start-pos — must be one of {FINGERS}")
        try:
            result[finger] = int(raw.strip())
        except ValueError:
            raise argparse.ArgumentTypeError(f"invalid --start-pos value {raw!r} for {finger!r}")
    if not result:
        raise argparse.ArgumentTypeError(f"could not parse --start-pos value {value!r}")
    return result


def raw_to_joint_cmd(raw: float) -> float:
    """0-1000 raw SDK position -> value to publish on /joint_commands_*,
    compensating for stark_node.cpp's *100 conversion (see module docstring)."""
    return raw / 100.0


def touch_in_contact(item, force_threshold: float) -> bool:
    forces = (item.normal_force1, item.normal_force2, item.normal_force3,
              item.tangential_force1, item.tangential_force2, item.tangential_force3)
    return any(f > force_threshold for f in forces)


class ApproachUntilContact(Node):
    def __init__(self):
        super().__init__('approach_until_contact')
        self.pub_left = self.create_publisher(JointState, '/joint_commands_left', 10)
        self.pub_right = self.create_publisher(JointState, '/joint_commands_right', 10)
        self.touch_left = None
        self.touch_right = None
        self.thumb_aux_raw = THUMB_AUX_RAW['flat']
        self.create_subscription(TouchStatus, '/touch_status', self._on_touch_left, 10)
        self.create_subscription(TouchStatus, '/touch_status_r', self._on_touch_right, 10)

    def _on_touch_left(self, msg):
        self.touch_left = msg

    def _on_touch_right(self, msg):
        self.touch_right = msg

    def touch_for(self, hand):
        return self.touch_left if hand == 'left' else self.touch_right

    def publisher_for(self, hand):
        return self.pub_left if hand == 'left' else self.pub_right

    def publish_positions(self, hand, raw_positions):
        """raw_positions: {joint_name: raw 0-1000}. Joints not given are
        held open (0), except thumb_aux which holds at self.thumb_aux_raw."""
        defaults = {'thumb_aux': self.thumb_aux_raw}
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [raw_to_joint_cmd(raw_positions.get(name, defaults.get(name, 0)))
                         for name in JOINT_NAMES]
        self.publisher_for(hand).publish(msg)

    def wait_for_touch(self, hands, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.touch_for(h) is not None for h in hands):
                return True
        return False

    def approach(self, targets, step=5, rate_hz=10.0, force_threshold=20.0,
                 proximity_threshold=None, settle_s=1.0, step_overrides=None,
                 debounce=3, log_csv=None, start_pos=None):
        """targets: list of (hand, finger) pairs to close simultaneously.
        Each finger stops independently as soon as it detects contact.
        Fingers on a targeted hand that aren't in `targets` are held open.
        step_overrides: optional {finger_name: step} to close specific
        fingers slower/faster than the shared `step` (e.g. a smaller value
        for 'thumb').
        debounce: number of consecutive over-threshold ticks required before
        declaring contact — filters single-sample sensor noise spikes that
        would otherwise permanently freeze a finger on one bad reading.
        log_csv: optional path — if given, records one row per target per
        tick (elapsed time, commanded position, forces, proximity, contact
        flag) for offline plotting with plot_approach_log.py.
        start_pos: optional starting position to close from instead of
        forcing the hand fully open first -- either a single raw 0-1000
        int (applied to every targeted finger) or a {finger_name:
        raw_value} dict for an asymmetric per-finger starting pose (e.g.
        wrapped around a tool where each finger sits at a different
        position); fingers not present in the dict default to 0. Use
        this when the hand is already holding something in a specific
        pose (e.g. positioned via slider_control.py) and forcing it open
        first would drop/release it. There's no auto-detect for this:
        /motor_status can't be trusted as a "what position is it at"
        check (its uint8 positions field truncates above ~25% closed,
        see module docstring), so the caller must supply the actual
        current position(s) themselves rather than have it inferred."""
        step_overrides = step_overrides or {}
        hands = sorted({h for h, _ in targets})

        if start_pos is None:
            self.get_logger().info(f"Opening hand(s) {hands} before starting…")
            for hand in hands:
                self.publish_positions(hand, {})
            if not self.wait_for_touch(hands):
                self.get_logger().warn("No touch data received yet — continuing anyway")
            time.sleep(settle_s)
        else:
            self.get_logger().info(
                f"Skipping auto-open — starting hand(s) {hands} from raw_pos={start_pos} "
                f"(assumed already there, not read back from hardware)")

        if start_pos is None:
            raw_pos = {h: {f: 0 for f in FINGERS} for h in hands}
        elif isinstance(start_pos, dict):
            raw_pos = {h: {f: start_pos.get(f, 0) for f in FINGERS} for h in hands}
        else:
            raw_pos = {h: {f: start_pos for f in FINGERS} for h in hands}
        contacted = {t: False for t in targets}
        hit_streak = {t: 0 for t in targets}
        period = 1.0 / rate_hz

        self.get_logger().info(
            f"Approaching {targets} step={step}/tick rate={rate_hz}Hz "
            f"force_threshold={force_threshold} debounce={debounce}"
            + (f" proximity_threshold={proximity_threshold}" if proximity_threshold else ""))

        csv_file = open(log_csv, 'w', newline='') if log_csv else None
        csv_writer = csv.writer(csv_file) if csv_file else None
        if csv_writer:
            csv_writer.writerow([
                't_s', 'hand', 'finger', 'raw_pos',
                'normal_force1', 'normal_force2', 'normal_force3',
                'tangential_force1', 'tangential_force2', 'tangential_force3',
                'self_proximity1', 'self_proximity2', 'mutual_proximity',
                'contact',
            ])
            self.get_logger().info(f"Logging time series to {log_csv}")

        start_t = time.monotonic()

        try:
            while rclpy.ok() and not all(contacted.values()):
                rclpy.spin_once(self, timeout_sec=0.0)

                for (hand, finger) in targets:
                    if contacted[(hand, finger)]:
                        continue

                    touch_msg = self.touch_for(hand)
                    item = None
                    hit = False
                    if touch_msg is not None:
                        item = touch_msg.data[TOUCH_INDEX[finger]]
                        hit = touch_in_contact(item, force_threshold)
                        if not hit and proximity_threshold is not None:
                            hit = (item.self_proximity1 > proximity_threshold or
                                   item.self_proximity2 > proximity_threshold or
                                   item.mutual_proximity > proximity_threshold)

                    confirmed = False
                    if hit:
                        hit_streak[(hand, finger)] += 1
                        confirmed = hit_streak[(hand, finger)] >= debounce
                    else:
                        hit_streak[(hand, finger)] = 0

                    if csv_writer and item is not None:
                        csv_writer.writerow([
                            f"{time.monotonic() - start_t:.3f}", hand, finger, raw_pos[hand][finger],
                            item.normal_force1, item.normal_force2, item.normal_force3,
                            item.tangential_force1, item.tangential_force2, item.tangential_force3,
                            item.self_proximity1, item.self_proximity2, item.mutual_proximity,
                            int(confirmed),
                        ])

                    if hit:
                        if confirmed:
                            contacted[(hand, finger)] = True
                            self.get_logger().info(
                                f"[{hand}/{finger}] CONTACT at raw_pos={raw_pos[hand][finger]} "
                                f"(after {hit_streak[(hand, finger)]} consecutive over-threshold ticks) "
                                f"forces=({item.normal_force1},{item.normal_force2},{item.normal_force3},"
                                f"{item.tangential_force1},{item.tangential_force2},{item.tangential_force3}) "
                                f"proximity=({item.self_proximity1},{item.self_proximity2},{item.mutual_proximity})")
                        continue

                    if raw_pos[hand][finger] >= RAW_MAX:
                        contacted[(hand, finger)] = True
                        self.get_logger().warn(
                            f"[{hand}/{finger}] Reached fully closed (raw={RAW_MAX}) without contact — stopping")
                        continue

                    finger_step = step_overrides.get(finger, step)
                    raw_pos[hand][finger] = min(RAW_MAX, raw_pos[hand][finger] + finger_step)

                for hand in hands:
                    self.publish_positions(hand, raw_pos[hand])

                time.sleep(period)
        finally:
            if csv_file:
                csv_file.close()

        self.get_logger().info("Done.")
        return raw_pos


def approach_single_finger(node, hand, finger, **kwargs):
    return node.approach([(hand, finger)], **kwargs)


def approach_full_hand(node, hand, fingers=FINGERS, **kwargs):
    return node.approach([(hand, f) for f in fingers], **kwargs)


def approach_both_hands(node, fingers=FINGERS, **kwargs):
    return node.approach([(h, f) for h in ('left', 'right') for f in fingers], **kwargs)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=['single', 'hand', 'both'], default='single')
    parser.add_argument('--hand', choices=['left', 'right'], default='right',
                         help='ignored for --mode both')
    parser.add_argument('--finger', choices=FINGERS, default='index',
                         help='only used with --mode single')
    parser.add_argument('--step', type=int, default=5, help='raw position units (0-1000) per tick')
    parser.add_argument('--thumb-step', type=int, default=None,
                         help='override --step for the thumb only (e.g. a smaller value to '
                              'close it slower than the other fingers)')
    parser.add_argument('--rate-hz', type=float, default=10.0)
    parser.add_argument('--force-threshold', type=float, default=20.0)
    parser.add_argument('--proximity-threshold', type=float, default=None,
                         help='optional additional stop condition on self/mutual proximity')
    parser.add_argument('--debounce', type=int, default=3,
                         help='consecutive over-threshold ticks required before declaring '
                              'contact (filters single-sample noise spikes)')
    parser.add_argument('--thumb-mode', choices=list(THUMB_AUX_RAW.keys()), default='flat',
                         help="thumb_aux rotation preset — 'flat' is the current default, "
                              "'out' rotates for opposition (unverified guess, see module docstring)")
    parser.add_argument('--thumb-aux-raw', type=int, default=None,
                         help='override --thumb-mode with an exact raw 0-1000 thumb_aux position')
    parser.add_argument('--log-csv', type=str, default=None,
                         help='path to write a per-tick time series (position, forces, proximity, '
                              'contact flag) for offline plotting with plot_approach_log.py')
    parser.add_argument('--start-pos', type=parse_start_pos, default=None,
                         help='starting position to close from, instead of forcing the hand '
                              'fully open first — either a single raw 0-1000 value (applied to '
                              'every targeted finger) or a per-finger list like '
                              '"thumb=250,index=400,middle=380,ring=350,pinky=200" for an '
                              'asymmetric tool grasp (fingers not listed default to 0). Use '
                              'when the hand is already holding something in a specific pose '
                              '(e.g. positioned via slider_control.py) and auto-opening would '
                              'drop it. Not auto-detected — /motor_status can\'t be trusted for '
                              'this (see module docstring), so pass the actual current '
                              'position(s) yourself.')
    args = parser.parse_args()

    rclpy.init()
    node = ApproachUntilContact()
    node.thumb_aux_raw = (args.thumb_aux_raw if args.thumb_aux_raw is not None
                           else THUMB_AUX_RAW[args.thumb_mode])
    try:
        step_overrides = {'thumb': args.thumb_step} if args.thumb_step is not None else None
        kwargs = dict(step=args.step, rate_hz=args.rate_hz,
                      force_threshold=args.force_threshold,
                      proximity_threshold=args.proximity_threshold,
                      step_overrides=step_overrides,
                      debounce=args.debounce,
                      log_csv=args.log_csv,
                      start_pos=args.start_pos)
        if args.mode == 'single':
            approach_single_finger(node, args.hand, args.finger, **kwargs)
        elif args.mode == 'hand':
            approach_full_hand(node, args.hand, **kwargs)
        else:
            approach_both_hands(node, **kwargs)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
