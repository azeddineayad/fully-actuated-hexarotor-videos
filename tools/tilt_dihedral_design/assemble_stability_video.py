#!/usr/bin/env python3
"""
Video of the design paper's Scenario 5 (stable and unstable rotor angles), in the format of the other scenario videos
(assemble_scenario_videos.py): a title card, the circle flight of each design recorded by record_stability_takes.sh
(cropped, sped up, captions at the logged events, "CONTROL LOST" where the body passes 60 deg) and the figure of the
paper made from the same flights (stability_report.py).  WebM, VP9, 1280 x 720, 25 fps.

    python3 assemble_stability_video.py [--speed 3]   -> analysis/results/video_stability/final/design_stability.webm
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import assemble_scenario_videos as asv  # noqa: E402

P = asv.P
asv.D = P / "analysis" / "results" / "video_stability"
asv.RUNS = P / "analysis" / "results" / "stability_runs"
asv.OUT = asv.D / "final"
FIG = asv.RUNS / "figures" / "stability.pdf"
DESIGNS = ["reference", "tilt20", "tilt15", "tilt10", "tilt5"]
asv.NAMES.update({"reference": "Tilt 28 deg (reference design), no dihedral",
                  "tilt20": "Tilt 20 deg, no dihedral", "tilt15": "Tilt 15 deg, no dihedral",
                  "tilt10": "Tilt 10 deg, no dihedral", "tilt5": "Tilt 5 deg, no dihedral"})
TITLE = "Scenario 5: stable and unstable rotor angles"
LINES = ["Purpose: show which rotor angles give a stable flight: the lateral",
         "acceleration index of a design must exceed what the flight demands",
         "(1 m/s2 on this circle).",
         "Setup: the same flight for five designs, tilt 28, 20, 15, 10 and 5 deg,",
         "no dihedral: take-off to 3 m, 4 m to the start point, two circles of",
         "radius 4 m at 2 m/s with the body level, landing. Frame script and",
         "parameters generated from the model for each design; Pixhawk flight",
         "code (ArduPilot) in software in the loop, simulation only."]
LOST = 60.0                                          # body tilt taken as the loss of control [deg]


def lost_wall(truth):
    x, y = truth["qx"], truth["qy"]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    i = np.flatnonzero(tilt > LOST)
    return float(truth["wall"][i[0]]) if len(i) else None


def clip(design, speed, out):
    """asv.flight_clip with a CONTROL LOST caption, the clip ending 12 s after the loss of control."""
    sc = "circle"
    raw = asv.D / f"{sc}_{design}_raw.webm"
    meta = json.load(open(asv.latest_run(sc, design)))
    a = np.genfromtxt(asv.latest_run(sc, design).with_suffix(".truth.csv"), delimiter=",", names=True)
    truth = {n: a[n] for n in a.dtype.names}
    dur = asv.duration(raw)
    t_end = raw.stat().st_mtime
    vt = lambda wall: dur - (t_end - wall)            # noqa: E731  wall clock -> time in the raw video
    caps = asv.events(sc, meta, truth)
    lw = lost_wall(truth)
    if lw is not None:                                # captions after the loss are replaced by the red one
        caps = [(w0, min(w1, lw), t, c) for w0, w1, t, c in caps if w0 < lw]
        caps.append((lw, 1e12, "CONTROL LOST: rotors saturated, the body overturns", "red"))
    t0 = max(0.0, vt(caps[1][0]) - 8 * speed)
    t1 = dur
    land = [ln for ln in (asv.D / f"{sc}_{design}_raw.flight.txt").read_text().splitlines() if "disarmed, landed" in ln]
    if land:
        hh = land[-1].split("]")[0].strip("[")
        wall = datetime.strptime(f"{datetime.fromtimestamp(t_end):%Y-%m-%d} {hh}", "%Y-%m-%d %H:%M:%S").timestamp()
        t1 = min(dur, vt(wall) + 2)
    if lw is not None:
        t1 = min(t1, vt(lw) + 12)
    vf = ["crop=1300:700:66:32", "drawbox=x=401:y=0:w=500:h=100:color=white:t=fill", "scale=1280:-2",
          "pad=1280:720:0:(720-ih)/2:color=black", f"setpts=(PTS-STARTPTS)/{speed}", "fps=25",
          f"drawtext=fontfile={asv.FONT}:text='{asv.esc(asv.NAMES[design])}':fontsize=24:fontcolor=white:box=1:"
          "boxcolor=black@0.7:boxborderw=8:x=20:y=20",
          f"drawtext=fontfile={asv.FONT}:text='{asv.esc(f'simulation (ArduPilot SITL), x{speed:g} speed')}':fontsize=18:"
          "fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=6:x=w-tw-20:y=h-40"]
    for w0, w1, text, col in caps:
        s0, s1 = (max(vt(w0), t0) - t0) / speed, (min(vt(w1), t1) - t0) / speed
        if s1 <= 0 or s0 >= (t1 - t0) / speed:
            continue
        vf.append(f"drawtext=fontfile={asv.FONT}:text='{asv.esc(text)}':fontsize=28:fontcolor={col}:box=1:"
                  f"boxcolor=black@0.7:boxborderw=8:x=20:y=70:enable='between(t,{s0:.2f},{s1:.2f})'")
    asv.run([asv.FF, "-v", "error", "-y", "-ss", f"{t0:.2f}", "-to", f"{t1:.2f}", "-i", str(raw), "-vf", ",".join(vf),
             *asv.ENC, str(out)])
    print(f"  {design}: raw {dur:.0f} s, clip {t0:.0f}-{t1:.0f} s -> {(t1 - t0) / speed:.0f} s"
          + ("" if lw is None else f", control lost at {vt(lw):.0f} s"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=3.0)
    a = ap.parse_args()
    parts = asv.OUT / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    asv.ms.OUT = parts
    asv.ms.slide("stability_title", TITLE, LINES)
    asv.ms.slide("stability_result", TITLE + ": result", image=FIG, img_box=(0.05, 0.03, 0.9, 0.78),
                 sub="figure of the paper, made from the five flights of this video")
    files = [parts / "stability_0.webm"]
    asv.card(parts / "stability_title.png", 12, files[0])
    for d in DESIGNS:
        f = parts / f"stability_{d}.webm"
        clip(d, a.speed, f)
        files.append(f)
    files.append(parts / "stability_9.webm")
    asv.card(parts / "stability_result.png", 14, files[-1])
    asv.concat(files, asv.OUT / "design_stability.webm")


if __name__ == "__main__":
    main()
