#!/usr/bin/env python3
"""
slider_control.py — Tkinter GUI with one slider per finger joint, publishing
live position commands to /joint_commands_left and/or /joint_commands_right
as you drag.

Same units/joint order as approach_until_contact.py: sliders are raw SDK
0 (open) - 1000 (closed) positions, published pre-divided by 100
(raw_to_joint_cmd) to compensate for stark_node.cpp's `position * 100`
conversion (see approach_until_contact.py's module docstring for the full
explanation of that bug). If stark_node.cpp's *100 is ever fixed to pass
raw positions straight through, change raw_to_joint_cmd here to raw / 1000.0.

thumb_aux is included as its own slider (it's the thumb's rotation/opposition
joint, no touch sensor, separate from the thumb flexion joint) rather than
held at a fixed preset like approach_until_contact.py's --thumb-mode does.

A "Record Pose" button snapshots every currently-shown slider's raw position
to one CSV row (one column per joint per hand), so a whole session of poses
(e.g. different grips on a medical tool) can be built up as you find them —
move the sliders to a grip, type an optional label, hit Record Pose, repeat.
Defaults to ~/logs/poses_<timestamp>.csv if --log-csv isn't given; pass an
explicit path to append to an existing pose file across multiple runs (the
pose_index counter picks up from however many rows are already in the file).
Recording under a label that already exists (exact match, whitespace
trimmed) overwrites that pose in place instead of adding a duplicate —
handy for refining a named grip over several tries. Blank labels never
collide with each other, so unlabeled poses always add a new row.

Every pose already in that file (from this run or a previous one) is listed
in a "Recorded Poses" box. Select one and click "Play Selected Pose" (or
just double-click it) to move the sliders to that saved position and
publish it, reproducing the grip on the real hand — one file holds both the
poses you've saved and the means to play any of them back.

Needs a display — run over SSH X11 forwarding same as rqt_plot (see
commands.md "Live plotting via rqt_plot"):
  ssh -X unitree@192.168.123.164

Usage:
  python3 slider_control.py                                   # both hands, auto-named CSV
  python3 slider_control.py --hand right                       # one hand only
  python3 slider_control.py --log-csv ~/logs/tool_grips.csv    # explicit / resumable pose file
"""

import argparse
import csv
import os
import time
import tkinter as tk
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ['thumb', 'thumb_aux', 'index', 'middle', 'ring', 'pinky']
RAW_MAX = 1000


def raw_to_joint_cmd(raw: float) -> float:
    """0-1000 raw SDK position -> value to publish on /joint_commands_*,
    compensating for stark_node.cpp's *100 conversion (see module docstring)."""
    return raw / 100.0


class SliderControl(Node):
    def __init__(self):
        super().__init__('slider_control')
        self.pub_left = self.create_publisher(JointState, '/joint_commands_left', 10)
        self.pub_right = self.create_publisher(JointState, '/joint_commands_right', 10)

    def publisher_for(self, hand):
        return self.pub_left if hand == 'left' else self.pub_right

    def publish_positions(self, hand, raw_positions):
        """raw_positions: 6 raw 0-1000 values, in JOINT_NAMES order."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [raw_to_joint_cmd(r) for r in raw_positions]
        self.publisher_for(hand).publish(msg)


class HandPanel(tk.LabelFrame):
    """One hand's worth of sliders (6 joints). Publishes on every move."""

    def __init__(self, master, node, hand, **kwargs):
        super().__init__(master, text=f"{hand.upper()} HAND", padx=8, pady=8, **kwargs)
        self.node = node
        self.hand = hand
        self.vars = []

        for i, name in enumerate(JOINT_NAMES):
            row = tk.Frame(self)
            row.pack(fill='x', pady=2)
            tk.Label(row, text=name, width=10, anchor='w').pack(side='left')
            var = tk.IntVar(value=0)
            self.vars.append(var)
            tk.Scale(row, from_=0, to=RAW_MAX, orient='horizontal',
                     variable=var, length=260, showvalue=True,
                     command=lambda _val, idx=i: self.publish()
                     ).pack(side='left', fill='x', expand=True)

        btns = tk.Frame(self)
        btns.pack(fill='x', pady=(8, 0))
        tk.Button(btns, text="Open all", command=self.open_all).pack(side='left', padx=4)
        tk.Button(btns, text="Close all", command=self.close_all).pack(side='left', padx=4)

    def publish(self):
        self.node.publish_positions(self.hand, [v.get() for v in self.vars])

    def open_all(self):
        for v in self.vars:
            v.set(0)
        self.publish()

    def close_all(self):
        for v in self.vars:
            v.set(RAW_MAX)
        self.publish()

    def get_positions(self):
        return [v.get() for v in self.vars]


class PoseRecorder:
    """Reads/appends one CSV row per pose: timestamp, pose_index, label,
    then <hand>_<joint> raw 0-1000 columns for every hand in `hands`.
    Keeps every row (loaded from disk at startup, plus any recorded this
    run) in self.rows so the same file can be browsed and played back."""

    def __init__(self, path, hands):
        self.path = os.path.expanduser(path)
        self.hands = hands
        self.fieldnames = ['timestamp', 'pose_index', 'label'] + [
            f'{hand}_{name}' for hand in hands for name in JOINT_NAMES]
        self.rows = []

        existing = os.path.exists(self.path) and os.path.getsize(self.path) > 0
        if existing:
            with open(self.path, newline='') as f:
                self.rows = list(csv.DictReader(f))
            self.pose_index = len(self.rows)
        else:
            dirname = os.path.dirname(self.path)
            if dirname:
                os.makedirs(dirname, exist_ok=True)
            with open(self.path, 'w', newline='') as f:
                csv.writer(f).writerow(self.fieldnames)
            self.pose_index = 0

    def record(self, label, positions_by_hand):
        """Records a pose under `label`. If a non-blank label matches an
        existing pose's label exactly (after stripping whitespace), that
        pose's row is overwritten in place (same pose_index, new position/
        timestamp) instead of adding a new one — lets you re-record over a
        named pose (e.g. "tripod") as you refine it. Blank labels never
        match each other, so unlabeled poses always add a new row. Rewrites
        the whole CSV file each time so it never holds a stale duplicate of
        an overwritten pose.

        Returns (pose_index, overwritten: bool)."""
        label = label.strip()
        row = {'timestamp': datetime.now().isoformat(timespec='seconds'), 'label': label}
        for hand in self.hands:
            for name, val in zip(JOINT_NAMES, positions_by_hand[hand]):
                row[f'{hand}_{name}'] = str(val)

        existing_i = None
        if label:
            for i, existing_row in enumerate(self.rows):
                if existing_row.get('label', '').strip() == label:
                    existing_i = i
                    break

        if existing_i is not None:
            row['pose_index'] = self.rows[existing_i]['pose_index']
            self.rows[existing_i] = row
        else:
            self.pose_index += 1
            row['pose_index'] = str(self.pose_index)
            self.rows.append(row)

        with open(self.path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        return row['pose_index'], existing_i is not None

    def positions_for_row(self, row, hand):
        """Raw 0-1000 positions for `hand` from a loaded/recorded row, in
        JOINT_NAMES order, or None if this row has no columns for that hand
        (e.g. it was recorded with a different --hand selection)."""
        try:
            return [int(float(row[f'{hand}_{name}'])) for name in JOINT_NAMES]
        except KeyError:
            return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--hand', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--log-csv', type=str, default=None,
                         help='pose file path (default: ~/logs/poses_<timestamp>.csv). '
                              'Pass an existing file to keep appending poses to it across runs.')
    args = parser.parse_args()

    log_csv = args.log_csv or f"~/logs/poses_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    rclpy.init()
    node = SliderControl()

    root = tk.Tk()
    root.title("Stark Hand — Slider Control")

    hands = ['left', 'right'] if args.hand == 'both' else [args.hand]
    panels = {}
    hands_frame = tk.Frame(root)
    hands_frame.pack(side='top')
    for hand in hands:
        panel = HandPanel(hands_frame, node, hand)
        panel.pack(side='left', padx=10, pady=10, fill='y')
        panels[hand] = panel

    recorder = PoseRecorder(log_csv, hands)

    record_frame = tk.Frame(root, padx=10)
    record_frame.pack(side='top', fill='x', pady=(0, 10))
    tk.Label(record_frame, text="Pose label:").pack(side='left')
    label_var = tk.StringVar()
    tk.Entry(record_frame, textvariable=label_var, width=24).pack(side='left', padx=(4, 10))
    status_var = tk.StringVar(
        value=f"{recorder.pose_index} pose(s) already in {recorder.path}"
              if recorder.pose_index else f"Recording to {recorder.path}")
    tk.Label(record_frame, textvariable=status_var, anchor='w').pack(side='left', padx=(0, 10))

    poses_frame = tk.LabelFrame(root, text="Recorded Poses", padx=10, pady=10)
    poses_frame.pack(side='top', fill='both', expand=True, padx=10, pady=(0, 10))

    listbox = tk.Listbox(poses_frame, height=8, width=50)
    listbox.pack(side='left', fill='both', expand=True)
    scrollbar = tk.Scrollbar(poses_frame, command=listbox.yview)
    scrollbar.pack(side='left', fill='y')
    listbox.config(yscrollcommand=scrollbar.set)

    def refresh_listbox():
        listbox.delete(0, tk.END)
        for row in recorder.rows:
            listbox.insert(tk.END, f"{row['pose_index']}: {row.get('label') or '(no label)'}")

    refresh_listbox()

    def do_record():
        positions = {hand: panels[hand].get_positions() for hand in hands}
        idx, overwritten = recorder.record(label_var.get(), positions)
        tag = f' "{label_var.get().strip()}"' if label_var.get().strip() else ""
        verb = "Updated" if overwritten else "Recorded"
        status_var.set(f"{verb} pose {idx}{tag} -> {recorder.path}")
        refresh_listbox()
        if not overwritten:
            listbox.see(tk.END)

    tk.Button(record_frame, text="Record Pose", command=do_record).pack(side='left')

    def do_play():
        selection = listbox.curselection()
        if not selection:
            status_var.set("Select a pose in the list first")
            return
        row = recorder.rows[selection[0]]
        applied, skipped = [], []
        for hand in hands:
            positions = recorder.positions_for_row(row, hand)
            if positions is None:
                skipped.append(hand)
                continue
            for var, val in zip(panels[hand].vars, positions):
                var.set(val)
            panels[hand].publish()
            applied.append(hand)
        msg = f"Playing pose {row['pose_index']}"
        if applied:
            msg += f" on {', '.join(applied)}"
        if skipped:
            msg += f" (no saved data for {', '.join(skipped)})"
        status_var.set(msg)

    play_frame = tk.Frame(poses_frame)
    play_frame.pack(side='left', fill='y', padx=(10, 0))
    tk.Button(play_frame, text="Play Selected Pose", command=do_play).pack(side='top')
    listbox.bind('<Double-Button-1>', lambda _event: do_play())

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
