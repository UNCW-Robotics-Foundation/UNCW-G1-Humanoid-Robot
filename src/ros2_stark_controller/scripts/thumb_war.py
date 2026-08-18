#!/usr/bin/env python3
"""
thumb_war.py — grip a held hand gently, then play thumb war.

Two phases:
  1. Grip: close index/middle/ring/pinky toward --grip-pos (default 900,
     i.e. 90%% closed), same directory/import pattern as experiment.py —
     must sit next to approach_until_contact.py on the robot. Any finger
     stops early (freezes at whatever position it's at) the instant it
     senses --grip-force-threshold, exactly like approach_until_contact.py's
     contact stop — grip-pos is a ceiling, not a forced target, so it never
     squeezes past what it actually feels on a real hand.
  2. Thumb war: thumb_aux (the thumb's rotation/opposition joint) sweeps a
     sine wave between --aux-min (default 200) and --aux-max (default
     1000) over --aux-period-s per full cycle. At BOTH extremes of that
     swing, the thumb's flexion joint ('thumb') closes slightly by
     --thumb-close-amount — a synchronized pulse, not a separate timer:
     the pulse magnitude is abs(sin(theta))**--thumb-close-sharpness, which
     is exactly 0 at the swing's midpoint and peaks exactly at both
     extremes (sharpness > 1 narrows the pulse so it's only near the
     extremes, not a linear ramp the whole swing). Safety: if the thumb
     senses --thumb-force-threshold, it stops advancing further closed
     (holds at its current position) until the pulse naturally recedes —
     it never keeps pressing into a hand that's pushing back.

Ctrl+C at any point releases (opens the whole hand) before exiting.

Usage:
  python3 thumb_war.py --hand right
  python3 thumb_war.py --hand right --grip-pos 900 --aux-min 200 \\
      --aux-max 1000 --aux-period-s 1.5 --thumb-close-amount 150 \\
      --duration-s 30
"""

import argparse
import math
import time

import rclpy

from approach_until_contact import (
    ApproachUntilContact, FINGERS, THUMB_AUX_RAW, TOUCH_INDEX, touch_in_contact,
)

GRIP_FINGERS = ['index', 'middle', 'ring', 'pinky']  # thumb excluded — it's the active player


def grip(node, hand, grip_pos, step, rate_hz, force_threshold, debounce):
    """Close index-pinky toward grip_pos, freezing any finger early the
    instant it senses force above force_threshold — grip_pos is a ceiling,
    not a forced target."""
    raw_pos = {f: 0 for f in FINGERS}
    hit_streak = {f: 0 for f in GRIP_FINGERS}
    done = {f: False for f in GRIP_FINGERS}
    period = 1.0 / rate_hz

    node.get_logger().info(f"Gripping toward raw_pos={grip_pos} on {GRIP_FINGERS}...")
    while rclpy.ok() and not all(done.values()):
        rclpy.spin_once(node, timeout_sec=0.0)
        touch_msg = node.touch_for(hand)

        for finger in GRIP_FINGERS:
            if done[finger]:
                continue
            hit = False
            if touch_msg is not None:
                item = touch_msg.data[TOUCH_INDEX[finger]]
                hit = touch_in_contact(item, force_threshold)
            if hit:
                hit_streak[finger] += 1
                if hit_streak[finger] >= debounce:
                    done[finger] = True
                    node.get_logger().info(f"[{finger}] gentle stop at raw_pos={raw_pos[finger]} (contact)")
                    continue
            else:
                hit_streak[finger] = 0

            if raw_pos[finger] >= grip_pos:
                done[finger] = True
                continue
            raw_pos[finger] = min(grip_pos, raw_pos[finger] + step)

        node.publish_positions(hand, raw_pos)
        time.sleep(period)

    return raw_pos


def thumb_war(node, hand, held_pos, aux_min, aux_max, aux_period_s,
              close_amount, close_sharpness, rate_hz, force_threshold,
              debounce, duration_s):
    """Hold index-pinky at held_pos. Sweep thumb_aux through a sine wave
    between aux_min and aux_max. At both extremes of that sweep, pulse the
    thumb's flexion joint closed by close_amount (magnitude shaped by
    abs(sin(theta))**close_sharpness, so the pulse is centered exactly on
    each extreme and ~0 at the swing's midpoint). Safety: if the thumb
    senses force_threshold, it never advances further closed than wherever
    it already is that tick — it can still open back up as the pulse
    recedes, it just won't push in harder."""
    raw_pos = dict(held_pos)
    thumb_base = raw_pos.get('thumb', 0)
    raw_pos['thumb'] = thumb_base
    aux_center = (aux_max + aux_min) / 2.0
    aux_amplitude = (aux_max - aux_min) / 2.0
    period = 1.0 / rate_hz
    hit_streak = 0

    node.get_logger().info(
        f"Thumb war: thumb_aux sweeping {aux_min}-{aux_max} every {aux_period_s}s, "
        f"thumb pulses +{close_amount} at both extremes, "
        f"holding {GRIP_FINGERS} at {held_pos}, force safety={force_threshold}")

    start_t = time.monotonic()
    while rclpy.ok() and (duration_s is None or time.monotonic() - start_t < duration_s):
        rclpy.spin_once(node, timeout_sec=0.0)

        theta = 2 * math.pi * (time.monotonic() - start_t) / aux_period_s
        aux_pos = aux_center + aux_amplitude * math.sin(theta)
        pulse = abs(math.sin(theta)) ** close_sharpness
        target_thumb = min(1000, thumb_base + close_amount * pulse)

        touch_msg = node.touch_for(hand)
        hit = False
        if touch_msg is not None:
            item = touch_msg.data[TOUCH_INDEX['thumb']]
            hit = touch_in_contact(item, force_threshold)
        if hit:
            hit_streak += 1
        else:
            hit_streak = 0
        force_tripped = hit_streak >= debounce

        if not (force_tripped and target_thumb > raw_pos['thumb']):
            raw_pos['thumb'] = target_thumb

        raw_pos['thumb_aux'] = aux_pos
        node.publish_positions(hand, raw_pos)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--hand', choices=['left', 'right'], default='right')
    parser.add_argument('--grip-pos', type=int, default=900,
                         help='ceiling raw 0-1000 for index/middle/ring/pinky (default 900 = 90%%)')
    parser.add_argument('--grip-step', type=int, default=5)
    parser.add_argument('--grip-force-threshold', type=float, default=20.0,
                         help='stop closing a finger early once it senses this much force')
    parser.add_argument('--aux-min', type=int, default=200,
                         help='thumb_aux sine wave lower bound (raw 0-1000)')
    parser.add_argument('--aux-max', type=int, default=1000,
                         help='thumb_aux sine wave upper bound (raw 0-1000)')
    parser.add_argument('--aux-period-s', type=float, default=1.5,
                         help='time for one full thumb_aux sine cycle')
    parser.add_argument('--thumb-close-amount', type=int, default=150,
                         help='how far (raw units) the thumb flexion joint closes at both '
                              'aux-sweep extremes, added on top of its baseline (open)')
    parser.add_argument('--thumb-close-sharpness', type=float, default=3.0,
                         help='exponent on the closing pulse — higher values narrow the '
                              'pulse so it only happens near the extremes, not throughout '
                              'the swing (1.0 = plain sine, no extra narrowing)')
    parser.add_argument('--thumb-force-threshold', type=float, default=20.0,
                         help='stop the thumb from closing further once it senses this much force')
    parser.add_argument('--rate-hz', type=float, default=15.0)
    parser.add_argument('--debounce', type=int, default=3)
    parser.add_argument('--duration-s', type=float, default=None,
                         help='total time to play (default: run until Ctrl+C)')
    parser.add_argument('--thumb-mode', choices=list(THUMB_AUX_RAW.keys()), default='out',
                         help="thumb_aux rotation preset used only during the grip phase "
                              "(before the sine sweep takes over) — 'out' (default) rotates "
                              "for opposition, matching a hand wrapped around another hand")
    parser.add_argument('--thumb-aux-raw', type=int, default=None,
                         help='override --thumb-mode with an exact raw 0-1000 thumb_aux '
                              'position for the grip phase only')
    args = parser.parse_args()

    rclpy.init()
    node = ApproachUntilContact()
    node.thumb_aux_raw = (args.thumb_aux_raw if args.thumb_aux_raw is not None
                           else THUMB_AUX_RAW[args.thumb_mode])

    try:
        node.wait_for_touch([args.hand], timeout_s=5.0)
        held = grip(node, args.hand, args.grip_pos, args.grip_step, args.rate_hz,
                    args.grip_force_threshold, args.debounce)
        thumb_war(node, args.hand, held, args.aux_min, args.aux_max, args.aux_period_s,
                  args.thumb_close_amount, args.thumb_close_sharpness, args.rate_hz,
                  args.thumb_force_threshold, args.debounce, args.duration_s)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Releasing — opening hand")
        node.publish_positions(args.hand, {})
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
