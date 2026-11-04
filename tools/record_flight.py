#!/usr/bin/env python3
"""
Screen-record only the flight of a scenario: every command runs in a hidden tmux session (no terminal on the screen),
the windows (Gazebo, plots) are placed, the recording starts when --start appears in the flight pane and stops a few
seconds after --stop.  Uses the screen recorder and the window placer of the project (analysis/video).  Run it under
an idle inhibitor (the screen must not lock):

    gnome-session-inhibit --inhibit idle python3 record_flight.py --out raw.webm --flight "<cmd>" --start RE --stop RE
        [--setup "<cmd>" --ready RE] [--windows "<cmd>"] [--layout "title=WxH+X+Y" ...] [--track]
"""

import argparse
import os
import re
import signal
import subprocess
import time
from pathlib import Path

V4 = Path.home() / "fully_actuated_hexacopter_f680_v4" / "analysis"
S = "fa_hidden_flight"
ENV = ("source /opt/ros/jazzy/setup.bash && source {ws}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp "
       "&& export QT_QPA_PLATFORM=xcb")
TRACK = ('track_mode: FOLLOW_LOOK_AT, follow_target: {name: "uav1"}, track_target: {name: "uav1"}, '
         'follow_offset: {x: -2.6, y: -2.6, z: 1.3}, follow_pgain: 1.0, track_pgain: 1.0')


def tmux(*a, **kw):
    return subprocess.run(["tmux", *a], check=False, **kw)


def pane(i):
    return f"{S}:0.{i}"


def text(i):
    return tmux("capture-pane", "-p", "-J", "-S", "-5000", "-t", pane(i), capture_output=True, text=True).stdout


def wait(i, pattern, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if re.search(pattern, text(i)):
            return True
        time.sleep(1.0)
    raise RuntimeError(f"timeout waiting for {pattern!r}; last lines:\n" + "\n".join(text(i).splitlines()[-15:]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--out", required=True)
    ap.add_argument("--ws", default="~/hexacopter_ws_f680_v4")
    ap.add_argument("--stop-sim", default=str(V4 / "stop_simulation.sh"))
    ap.add_argument("--setup"); ap.add_argument("--ready")
    ap.add_argument("--windows", help="command that opens extra windows (plots), after the setup")
    ap.add_argument("--flight", required=True)
    ap.add_argument("--start", required=True); ap.add_argument("--stop", required=True)
    ap.add_argument("--layout", nargs="*", default=[])
    ap.add_argument("--track", action="store_true", help="make the Gazebo camera follow uav1")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--max", type=float, default=1800.0)
    args = ap.parse_args()
    env = ENV.format(ws=args.ws)
    subprocess.run([args.stop_sim], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    tmux("kill-session", "-t", S)
    tmux("new-session", "-d", "-s", S, "-x", "200", "-y", "50")
    for _ in range(2):
        tmux("split-window", "-t", S)
    for i in range(3):
        tmux("send-keys", "-t", pane(i), env + " && clear", "Enter")
    time.sleep(2)

    def place():
        if args.layout:
            subprocess.run(["python3", str(V4 / "video" / "place_windows.py"), *args.layout],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def track():
        if args.track:                                       # from a pane of the simulation (same environment)
            tmux("send-keys", "-t", pane(2), f"gz topic -t /gui/track -m gz.msgs.CameraTrack -p '{TRACK}'", "Enter")

    rec = None
    t0 = None
    try:
        if args.setup:
            tmux("send-keys", "-t", pane(0), args.setup, "Enter")
            wait(0, args.ready, 400)
            time.sleep(3)
        if args.windows:
            tmux("send-keys", "-t", pane(2), args.windows, "Enter")
            time.sleep(10)
        tmux("send-keys", "-t", pane(1), args.flight, "Enter")
        end = time.time() + 600
        while not re.search(args.start, text(1)):           # do not resize Gazebo while it starts (it renders black)
            time.sleep(2)
            if time.time() > end:
                raise RuntimeError("flight did not start:\n" + "\n".join(text(1).splitlines()[-15:]))
        place(); time.sleep(4); place(); track()
        time.sleep(2)
        rec_log = Path(args.out).with_suffix(".recorder.txt")
        rec = subprocess.Popen(["python3", str(V4 / "video" / "screen_record.py"), args.out, "--fps", str(args.fps)],
                               stdout=open(rec_log, "w"), stderr=subprocess.STDOUT)
        t0 = time.time()
        print("recording", flush=True)
        n = 0
        while not re.search(args.stop, text(1)):
            time.sleep(2); n += 1
            if n % 8 == 0:
                place(); track()
            if time.time() - t0 > args.max:
                raise RuntimeError("flight too long")
        time.sleep(4)
        print(f"flight recorded: {time.time() - t0:.0f} s", flush=True)
    finally:
        if rec:
            rec.send_signal(signal.SIGINT)
            try:
                rec.wait(timeout=30)
            except subprocess.TimeoutExpired:
                rec.kill()
        Path(args.out).with_suffix(".flight.txt").write_text(text(1))
        for i in range(3):
            tmux("send-keys", "-t", pane(i), "Enter")             # fly_pixhawk_sim.sh waits for Enter
        time.sleep(3)
        for i in range(3):
            tmux("send-keys", "-t", pane(i), "C-c")
        time.sleep(2)
        subprocess.run([args.stop_sim], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        tmux("kill-session", "-t", S)
        for pat in ("live_plot.py",):
            subprocess.run(["pkill", "-f", f"python3 {pat}"], stderr=subprocess.DEVNULL)
    print("TAKE OK" if Path(args.out).exists() else "TAKE FAILED", flush=True)


if __name__ == "__main__":
    main()
