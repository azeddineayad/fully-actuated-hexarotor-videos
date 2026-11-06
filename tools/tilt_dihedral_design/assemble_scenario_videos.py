#!/usr/bin/env python3
"""
One video per design scenario (record_takes.sh): a title card (purpose, setup), the flight of the reference and of the
optimum design (cropped, sped up, captions at the logged events) and the result figure of the paper; and the video of
the optimization (NSGA-II frames of pareto_evolution.py --frames).  WebM, VP9, 1280 x 720, 25 fps.

    python3 assemble_scenario_videos.py [scenario ...]     (default: all)  -> analysis/results/video_design/final/
"""

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_design_slides as ms  # noqa: E402

P = Path.home() / "fully_actuated_hexacopter_f680_v5"
D = P / "analysis" / "results" / "video_design"
RUNS = P / "analysis" / "results" / "video_design_runs"
OUT = D / "final"
FF, FP = str(Path.home() / "tools/bin/ffmpeg"), str(Path.home() / "tools/bin/ffprobe")
FONT = subprocess.run(["fc-match", "-f", "%{file}", "DejaVu Sans:bold"], capture_output=True, text=True).stdout
ENC = ["-c:v", "libvpx-vp9", "-b:v", "0", "-crf", "34", "-deadline", "good", "-cpu-used", "4", "-row-mt", "1",
       "-pix_fmt", "yuv420p", "-an"]
NAMES = {"reference": "Reference design: tilt 28 deg, no dihedral",
         "optimum": "Optimum design: tilt 28 deg, dihedral 9.5 deg outwards"}
FIGS = P / "docs" / "paper_design" / "figures"

SCEN = {
    "circle": dict(
        title="Scenario 1: nominal flight on a circle",
        lines=["Purpose: the new rotor orientation must fly as well as the current one,",
               "and its hover power must be the one predicted (+2 %).",
               "Setup: take-off to 3 m, two circles of radius 4 m at 2 m/s (1 m/s2 towards",
               "the centre), body level, position + velocity + acceleration setpoints; landing.",
               "Pixhawk flight code (ArduPilot) in software in the loop, MRS Gazebo world."],
        figure="circle"),
    "attitude_hold": dict(
        title="Scenario 2: attitude held in hover",
        lines=["Purpose: requirement R1, hold the body at 12 deg with at least 1 N on every rotor,",
               "in the weakest direction of the design.",
               "Setup: hover at 3 m, body commanded to 12 deg nose up (azimuth 5 deg) for 20 s,",
               "position held, thrust kept vertical by the rotors; back to level; landing."],
        figure="hold_gust"),
    "gust": dict(
        title="Scenario 3: gusts at level attitude",
        lines=["Purpose: reject lateral disturbances with the body level.",
               "Setup: hover at 3 m; three horizontal gusts of 4 N (1.6 m/s2) for 2 s each,",
               "6 s apart, towards +y, +x and -x-y, applied by the physics; landing."],
        figure="hold_gust"),
    "attitude_envelope": dict(
        title="Scenario 3: attitude envelope in flight",
        lines=["Purpose: check in flight the attitude limit predicted for each design: with rotor forces",
               "between 0 and F_max, 14.8 deg for the reference and 15.4 deg for the optimum.",
               "Setup: hover at 3 m; body tilted nose up (weakest direction) in steps of 10, 12, 14, 15",
               "and 16 deg, 10 s each, thrust kept vertical, position held; flight-code attitude and lean",
               "limits raised for this test only (simulation)."],
        figure="envelopes"),
    "rotor_failure": dict(
        title="Scenario 4: loss of a rotor in hover",
        lines=["Purpose: requirement R2, hover after the loss of any rotor with >= 1 N on the others.",
               "Setup: hover at 5 m; rotor 1 is cut in the physics; the flight code is told 0.15 s later",
               "and loads a five-rotor mixer (research copy of ArduPilot, simulation only);",
               "position held 20 s, then landing on five rotors."],
        figure="rotor_failure"),
}


def run(cmd):
    subprocess.run(cmd, check=True)


def esc(t):
    return t.replace(":", r"\:").replace("'", r"\'").replace(",", r"\,")


def duration(f):
    return float(subprocess.run([FP, "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(f)],
                                capture_output=True, text=True).stdout)


def card(png, sec, out):
    run([FF, "-v", "error", "-y", "-loop", "1", "-t", str(sec), "-i", str(png),
         "-vf", "fps=25,scale=1280:720,format=yuv420p", *ENC, str(out)])


def latest_run(sc, design):
    return sorted(RUNS.glob(f"{sc}_{design}_*.json"))[-1]


def events(sc, meta, truth):
    """Captions (wall_start, wall_end, text, colour) of a flight."""
    ev = meta["events"]
    if sc == "circle":
        c = ev["circle"]
        return [(0, ev["circle_start_wall"], "hover, then to the start of the circle", "yellow"),
                (ev["circle_start_wall"], ev["circle_end_wall"],
                 f"circle R {c['radius']:g} m at {c['speed']:g} m/s, body level", "white"),
                (ev["circle_end_wall"], 1e12, "back to the centre, landing", "yellow")]
    if sc == "attitude_hold":
        return [(0, ev["tilt_wall"], "hover, body level", "yellow"),
                (ev["tilt_wall"], ev["level_wall"], "body held at 12 deg (weakest direction)", "white"),
                (ev["level_wall"], 1e12, "back to level, landing", "yellow")]
    if sc == "gust":
        on = np.hypot(truth["gx"], truth["gy"]) > 0
        out, k = [], 0
        edges = np.flatnonzero(np.diff(on.astype(int)))
        for s0, s1 in zip(edges[::2], edges[1::2]):
            g = ev["gusts"][k]; k += 1
            out.append((truth["wall"][s0 + 1], truth["wall"][s1], f"gust {np.hypot(g[1], g[2]):.0f} N "
                        f"towards ({g[1]:+.1f}, {g[2]:+.1f})", "red"))
        return [(0, out[0][0], "hover, body level", "yellow")] + out + [(ev["event_end_wall"], 1e12, "landing", "yellow")]
    if sc == "attitude_envelope":
        st = ev["steps"]
        return ([(0, st[0][0], "hover, body level", "yellow")] +
                [(w0, (st[k + 1][0] if k + 1 < len(st) else ev["level_wall"]), f"body tilted {v:g} deg", "white")
                 for k, (w0, v) in enumerate(st)] + [(ev["level_wall"], 1e12, "back to level, landing", "yellow")])
    return [(0, ev["failure_wall"], "hover at 5 m", "yellow"),
            (ev["failure_wall"], ev["event_end_wall"], "ROTOR 1 CUT - five-rotor mixer", "red"),
            (ev["event_end_wall"], 1e12, "landing on five rotors", "yellow")]


def flight_clip(sc, design, speed, out):
    raw = D / f"{sc}_{design}_raw.webm"
    meta = json.load(open(latest_run(sc, design)))
    a = np.genfromtxt(latest_run(sc, design).with_suffix(".truth.csv"), delimiter=",", names=True)
    truth = {n: a[n] for n in a.dtype.names}
    dur = duration(raw)
    t_end = raw.stat().st_mtime                                    # written when the recording stops
    vt = lambda wall: dur - (t_end - wall)                         # noqa: E731  wall clock -> time in the raw video
    caps = events(sc, meta, truth)
    first_event = caps[1][0]
    t0 = max(0.0, vt(first_event) - 8 * speed)
    land = [ln for ln in (D / f"{sc}_{design}_raw.flight.txt").read_text().splitlines() if "disarmed, landed" in ln]
    t1 = dur
    if land:
        hh = land[-1].split("]")[0].strip("[")
        wall = datetime.strptime(f"{datetime.fromtimestamp(t_end):%Y-%m-%d} {hh}", "%Y-%m-%d %H:%M:%S").timestamp()
        t1 = min(dur, vt(wall) + 2)
    vf = ["crop=1300:700:66:32", "drawbox=x=401:y=0:w=500:h=100:color=white:t=fill", "scale=1280:-2",
          "pad=1280:720:0:(720-ih)/2:color=black", f"setpts=(PTS-STARTPTS)/{speed}", "fps=25",
          f"drawtext=fontfile={FONT}:text='{esc(NAMES[design])}':fontsize=24:fontcolor=white:box=1:boxcolor=black@0.7:"
          "boxborderw=8:x=20:y=20",
          f"drawtext=fontfile={FONT}:text='{esc(f'simulation (ArduPilot SITL), x{speed:g} speed')}':fontsize=18:"
          "fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=6:x=w-tw-20:y=h-40"]
    for w0, w1, text, col in caps:
        s0, s1 = (max(vt(w0), t0) - t0) / speed, (min(vt(w1), t1) - t0) / speed
        if s1 <= 0 or s0 >= (t1 - t0) / speed:
            continue
        vf.append(f"drawtext=fontfile={FONT}:text='{esc(text)}':fontsize=28:fontcolor={col}:box=1:"
                  f"boxcolor=black@0.7:boxborderw=8:x=20:y=70:enable='between(t,{s0:.2f},{s1:.2f})'")
    run([FF, "-v", "error", "-y", "-ss", f"{t0:.2f}", "-to", f"{t1:.2f}", "-i", str(raw), "-vf", ",".join(vf),
         *ENC, str(out)])
    print(f"  {sc} {design}: raw {dur:.0f} s, clip {t0:.0f}-{t1:.0f} s -> {(t1 - t0) / speed:.0f} s")


def concat(files, out):
    lst = out.with_suffix(".txt")
    lst.write_text("".join(f"file '{f}'\n" for f in files))
    run([FF, "-v", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", str(out)])
    print(f"wrote {out} ({duration(out):.0f} s)")


def scenario_video(sc, speed):
    s = SCEN[sc]
    parts = OUT / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    ms.OUT = parts
    ms.slide(f"{sc}_title", s["title"], s["lines"])
    ms.slide(f"{sc}_result", s["title"] + ": result", image=FIGS / f"{s['figure']}.pdf", img_box=(0.05, 0.03, 0.9, 0.78),
             sub="figure of the paper, made from the two flights of this video")
    files = []
    card(parts / f"{sc}_title.png", 10, parts / f"{sc}_0.webm"); files.append(parts / f"{sc}_0.webm")
    for d in ("reference", "optimum"):
        f = parts / f"{sc}_{d}.webm"; flight_clip(sc, d, speed, f); files.append(f)
    card(parts / f"{sc}_result.png", 12, parts / f"{sc}_9.webm"); files.append(parts / f"{sc}_9.webm")
    concat(files, OUT / f"design_{sc}.webm")


def optimization_video():
    frames = P / "design_optimization" / "results" / "pareto_frames"
    parts = OUT / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    ms.OUT = parts
    ms.slide("opt_title", "Scenario 1: convergence of the optimization",
             ["Purpose: show that the genetic algorithm (NSGA-II) finds the Pareto front of the",
              "design problem, and that it converges to the exact front of the exhaustive map.",
              "Setup: two design variables (tilt, dihedral), three objectives (hover power,",
              "lateral acceleration, rotor-failure margin), two constraints (yaw, propeller",
              "clearance); population 80, 100 generations; one frame per generation."])
    card(parts / "opt_title.png", 10, parts / "opt_0.webm")
    n = len(list(frames.glob("f*.png")))
    run([FF, "-v", "error", "-y", "-framerate", "4", "-i", str(frames / "f%03d.png"),
         "-vf", f"tpad=stop_mode=clone:stop_duration=6,fps=25,scale=1280:720,format=yuv420p", *ENC,
         str(parts / "opt_1.webm")])
    ms.slide("opt_result", "Scenario 1: result", image=FIGS / "pareto_evolution.pdf", img_box=(0.04, 0.03, 0.92, 0.78),
             sub="generations 0, 5, 20 and 100 in the design plane (top) and the power - failure plane (bottom)")
    card(parts / "opt_result.png", 12, parts / "opt_9.webm")
    concat([parts / "opt_0.webm", parts / "opt_1.webm", parts / "opt_9.webm"], OUT / "design_optimization.webm")
    print(f"  {n} generation frames")


def ellipsoid_video():
    frames = P / "design_optimization" / "results" / "ellipsoid_frames"
    parts = OUT / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    ms.OUT = parts
    ms.slide("ell_title", "Scenario 2: force and torque ellipsoids",
             ["Purpose: show what the exact attainable sets change for the design, compared with the",
              "force and torque ellipsoids used in design studies.",
              "Setup: at hover, forces with zero moment and moments with zero force, for the optimum;",
              "exact polytope (rotor forces between 1 N and F_max), axis ellipsoid (exact on the body axes),",
              "manipulability ellipsoid (largest ball of rotor changes); then the same lateral requirement",
              "sized with each set and checked with the exact index."])
    card(parts / "ell_title.png", 12, parts / "ell_0.webm")
    run([FF, "-v", "error", "-y", "-stream_loop", "1", "-framerate", "12", "-i", str(frames / "f%03d.png"),
         "-vf", "fps=25,scale=1280:720,format=yuv420p", *ENC, str(parts / "ell_1.webm")])
    ms.slide("ell_result", "Scenario 2: result", image=FIGS / "ellipsoids.pdf", img_box=(0.03, 0.05, 0.94, 0.70),
             sub="(a)(b) the axis ellipsoid leaves the exact set in the weak directions, the manipulability ellipsoid "
                 "is far inside; (c) designs sized with each set: % = true lateral acceleration against the requirement")
    card(parts / "ell_result.png", 14, parts / "ell_9.webm")
    concat([parts / "ell_0.webm", parts / "ell_1.webm", parts / "ell_9.webm"], OUT / "design_ellipsoids.webm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenarios", nargs="*", default=["optimization", "ellipsoids", "attitude_envelope", "rotor_failure"])
    ap.add_argument("--speed", type=float, default=3.0)
    a = ap.parse_args()
    for sc in a.scenarios:
        {"optimization": optimization_video, "ellipsoids": ellipsoid_video}.get(sc, lambda: scenario_video(sc, a.speed))()


if __name__ == "__main__":
    main()
