#!/usr/bin/env python3
"""
Slides (1280 x 720 PNG) of the v5 results video: the design question, the model, the design chart, the Pareto
fronts, the comparison of the designs and the conclusions.  The flight clips are added by assemble_design_video.sh.

    python3 make_design_slides.py            -> analysis/results/video_design/slides/NN_name.png
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

P = Path.home() / "fully_actuated_hexacopter_f680_v5"
OUT = P / "analysis" / "results" / "video_design" / "slides"
FIG = P / "docs" / "paper_design" / "figures"
W, H, DPI = 12.8, 7.2, 100
BG, FG, ACC = "#10151c", "#f2f2f2", "#7fd18b"
plt.rcParams.update({"font.family": "DejaVu Sans", "mathtext.fontset": "dejavusans"})


def slide(name, title, lines=(), image=None, img_box=(0.06, 0.08, 0.88, 0.70), sub=None, table=None, size=21):
    fig = plt.figure(figsize=(W, H), dpi=DPI, facecolor=BG)
    fig.text(0.05, 0.9, title, color=ACC, fontsize=30, weight="bold", va="center")
    if sub:
        import textwrap
        fig.text(0.05, 0.84, "\n".join(textwrap.wrap(sub, 105)), color=FG, fontsize=15, va="top")
    y = 0.74
    if table:                                         # rows of cells at fixed columns
        cols = table["x"]
        for r, row in enumerate(table["rows"]):
            for x, cell in zip(cols, row):
                fig.text(x, y, cell, color=ACC if r == 0 else FG, fontsize=size, va="top",
                         weight="bold" if r == 0 else "normal")
            y -= 0.08
        y -= 0.04
    for ln in lines:
        fig.text(0.07, y, ln, color=FG, fontsize=size, va="top")
        y -= 0.095 * (1 + ln.count("\n"))
    if image:
        import subprocess
        png = OUT / f"_{Path(image).stem}.png"
        subprocess.run(["pdftoppm", "-r", "220", "-png", "-singlefile", str(image), str(png.with_suffix(""))], check=True)
        ax = fig.add_axes(img_box)
        ax.imshow(mpimg.imread(png))
        ax.axis("off")
    fig.savefig(OUT / f"{name}.png", facecolor=BG)
    plt.close(fig)
    print("wrote", name)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    pts = json.load(open(P / "design_optimization" / "results" / "design_points.json"))
    cur, opt = pts["current"], pts["optimum"]
    sc = json.load(open(P / "analysis" / "results" / "design_scenarios" / "summary.json"))

    slide("01_title", "Optimal tilt and dihedral angles",
          ["of the fixed-tilt fully actuated F680 hexarotor",
           "",
           "Exact wrench indices, rotor-failure tolerance and",
           "validation with the ArduPilot flight code (software in the loop)",
           "",
           "Azzeddine AYAD - Ecole Militaire Polytechnique, Laboratoire Mecanique des Structures",
           "All flights in this video are simulations."])
    slide("02_question", "The question",
          ["The six rotors of the F680 are tilted 28 deg about their arms, alternately.",
           "This value came from a master thesis that optimized a 1 kg vehicle with 0.25 m arms,",
           "with force ellipsoids and a geometric efficiency.",
           "The F680 is 2.48 kg with 0.34 m arms and 13 inch propellers.",
           "",
           "What orientation should its rotors have?",
           "We add a dihedral angle to each rotor and choose both angles from exact indices."])
    slide("03_model", "Two angles per rotor", image=FIG / "geometry.pdf", img_box=(0.04, 0.05, 0.92, 0.72),
          sub="tilt about the arm (alternate) + dihedral along the arm (common);  det A = -54 C C Q^2 D2 in closed form")
    slide("04_indices", "Exact indices (no ellipsoids)",
          ["A is square and invertible: F = A^-1 W, and every bound is a ratio test.",
           "  - guaranteed lateral acceleration at level attitude",
           "  - attitude the body can hold while hovering",
           "  - roll, pitch and yaw accelerations",
           "  - hover power (momentum theory, P ~ F^1.5)",
           "  - hover after the loss of any rotor: force kept by the 5 others",
           "  - clearance between neighbouring propellers"])
    slide("05_chart", "Design chart: 12 221 designs computed exactly", image=FIG / "design_chart.pdf",
          img_box=(0.05, 0.03, 0.90, 0.76),
          sub="minimum hover power with: attitude >= 12 deg, rotor-failure margin >= 1 N, yaw >= 20 rad/s2, clearance >= 20 mm")
    slide("06_pareto", "Pareto fronts: the dihedral must lean outwards", image=FIG / "pareto.pdf",
          img_box=(0.05, 0.05, 0.90, 0.72),
          sub="an inward dihedral is never Pareto-optimal; NSGA-II (black) agrees with the exhaustive map")
    slide("07_result", "Result: 28 deg tilt + 9.5 deg outward dihedral",
          ["Checked with NSGA-II, with 12 free angles (no meaningful gain)",
           "and with +-20 % parameter errors: the mass and the centre-of-gravity",
           "height must be measured before the motor mounts are made."], size=19,
          table={"x": [0.07, 0.42, 0.66], "rows": [
              ["", "current (28, 0)", "optimum (28, -9.5)"],
              ["hover power", f"{cur['power_ratio']:.3f}", f"{opt['power_ratio']:.3f}  (+2 %)"],
              ["lateral acceleration", f"{cur['lat_acc_f1']:.2f} m/s2", f"{opt['lat_acc_f1']:.2f} m/s2"],
              ["attitude held in hover", f"{cur['hover_tilt_f1_deg']:.1f} deg", f"{opt['hover_tilt_f1_deg']:.1f} deg"],
              ["after a rotor loss", f"{cur['fail_fmin_N']:.2f} N (< 1 N)", f"{opt['fail_fmin_N']:.2f} N"]]})
    rf, ah = sc["rotor_failure"], sc["attitude_hold"]
    slide("08_flights", "Flights with the ArduPilot code (SITL)",
          ["Same take-off point, same flight code; only the rotor geometry changes.",
           f"Hover power measured: reference {sc['hover']['reference']['power_ratio']:.2f}, optimum {sc['hover']['optimum']['power_ratio']:.2f}",
           "12 deg held in the weakest direction, smallest rotor force:",
           f"      reference {ah['reference']['min_force_N']:.2f} N (below 1 N), optimum {ah['optimum']['min_force_N']:.2f} N",
           "Next: rotor 1 is cut in hover at 5 m. The flight code is told 0.15 s later and",
           "switches to a 5-rotor mixer (research copy of ArduPilot, simulation only)."])
    slide("09_failure_fig", "Rotor failure: both designs survive", image=FIG / "rotor_failure.pdf",
          img_box=(0.12, 0.02, 0.76, 0.78),
          sub=f"steady hover tilted ~23 deg; weakest remaining rotor {rf['reference']['settled_min_live_force_N']:.2f} N "
              f"(reference) vs {rf['optimum']['settled_min_live_force_N']:.2f} N (optimum)")
    slide("10_end", "Conclusions",
          ["The tilt buys agility; an outward dihedral buys rotor-failure tolerance cheaply.",
           "For the F680: keep 28 deg, add 9.5 deg outward dihedral, +2 % hover power.",
           "The transient after a failure is set by the controller, not by the static margin.",
           "Gust rejection is limited by the position loop, not by the rotors.",
           "",
           "Limits: simulation only, parameters estimated, no rotor-rotor aerodynamics.",
           "Next: measure mass, CoG and rotors; recompute; machine the mounts; real hover test."])


if __name__ == "__main__":
    main()
