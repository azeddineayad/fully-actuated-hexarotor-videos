#!/usr/bin/env python3
"""
Design paper, Scenario 5: stable and unstable rotor angles.  The circle scenario (design_scenarios.py circle: take-off
to 3 m, 4 m to the start point, two circles of radius 4 m at 2 m/s, body level) flown with designs of decreasing tilt
(dihedral 0) by record_stability_takes.sh; this script makes the figure of the paper and the numbers of the text.

    python3 stability_report.py [--dir results/stability_runs] [--fig DIR]   -> DIR/stability.pdf/.svg + summary.json
"""

import argparse
import glob
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
MAP = HERE.parent / "design_optimization" / "results" / "design_map.npz"
DESIGNS = {"reference": (28, "tab:gray"), "tilt20": (20, "tab:blue"), "tilt15": (15, "tab:orange"),
           "tilt10": (10, "tab:red"), "tilt5": (5, "tab:purple")}
F_MIN, F_MAX, LOST = 1.0, 17.9, 60.0                 # [N], [N], body tilt taken as the loss of control [deg]
plt.rcParams.update({"font.family": "serif", "font.size": 9, "mathtext.fontset": "cm", "savefig.bbox": "tight",
                     "svg.fonttype": "none"})


def load(run_dir, design):
    files = sorted(glob.glob(str(run_dir / f"circle_{design}_*.truth.csv")))
    if not files:
        return None
    f = files[-1]
    d = np.genfromtxt(f, delimiter=",", names=True)
    m = json.load(open(f.replace(".truth.csv", ".json")))
    ev = m["events"]
    q = np.stack([d["qw"], d["qx"], d["qy"], d["qz"]], 1)
    w, x, y, z = q.T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))      # angle of the body z axis to the vertical
    F = np.stack([d[f"f{i}"] for i in range(1, 7)], 1)
    t0 = float(np.interp(ev["hover_start_wall"], d["wall"], d["t"]))           # end of the take-off
    t1 = float(np.interp(ev["event_end_wall"], d["wall"], d["t"]))
    sel = (d["t"] >= t0 - 2) & (d["t"] <= t1)
    t = d["t"][sel] - t0
    lost = np.flatnonzero(tilt[sel] > LOST)
    k = lost[0] if len(lost) else len(t)              # the curves stop where the control is lost
    c = ev["circle"]
    return dict(t=t[:k], x=d["x"][sel][:k], y=d["y"][sel][:k], tilt=tilt[sel][:k], roll=roll[sel][:k],
                pitch=pitch[sel][:k], yaw=yaw[sel][:k] - yaw[sel][0], F=F[sel][:k], lost_t=float(t[k]) if len(lost) else None,
                centre=c["centre_enu"][:2], radius=c["radius"], file=Path(f).name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=str(HERE / "results" / "stability_runs"))
    ap.add_argument("--fig", default=None, help="figure directory (default: <dir>/figures)")
    a = ap.parse_args()
    run_dir = Path(a.dir).resolve()
    fig_dir = Path(a.fig).resolve() if a.fig else run_dir / "figures"
    (fig_dir / "svg").mkdir(parents=True, exist_ok=True)
    runs = {k: load(run_dir, k) for k in DESIGNS}
    runs = {k: v for k, v in runs.items() if v is not None}
    D = dict(np.load(MAP))
    j0 = int(np.argmin(abs(D["zetab_deg"])))

    fig, axs = plt.subplots(3, 2, figsize=(7.2, 7.6), layout="constrained")
    a = axs[0, 0]                                     # (a) index of the designs against the demand of the flight
    a.plot(D["xib_deg"], D["lat_acc"][:, j0], "k-", lw=1.0, label=r"index $a_{\mathrm{lat}}$ ($\bar\zeta=0$)")
    a.axhline(1.0, color="k", ls="--", lw=0.8, label="demand of the circle")
    for k, r in runs.items():
        xi, col = DESIGNS[k]
        i = int(np.argmin(abs(D["xib_deg"] - xi)))
        a.plot(xi, D["lat_acc"][i, j0], "o" if r["lost_t"] is None else "X", color=col, mec="k", ms=7)
    a.set_xlim(0, 32); a.set_ylim(0, 3.2)
    a.set_xlabel(r"tilt $\bar\xi$ [deg]"); a.set_ylabel(r"lateral acceleration $a_{\mathrm{lat}}$ [m/s$^2$]")
    a.set_title("(a) designs flown (o: stable, x: lost)", fontsize=9); a.legend(fontsize=6.5, loc="upper left")
    a.grid(alpha=0.3)
    a = axs[0, 1]                                     # (b) flown paths
    ref = next(iter(runs.values()))
    psi = np.linspace(0, 2 * math.pi, 200)
    a.plot(ref["centre"][0] + ref["radius"] * np.cos(psi), ref["centre"][1] + ref["radius"] * np.sin(psi), "k--", lw=0.8,
           label="circle")
    for k, r in runs.items():
        xi, col = DESIGNS[k]
        a.plot(r["x"], r["y"], color=col, lw=0.9, label=rf"$\bar\xi={xi}^\circ$")
        if r["lost_t"] is not None:
            a.plot(r["x"][-1], r["y"][-1], "X", color=col, mec="k", ms=8)
    a.set_aspect("equal"); a.set_xlim(-6.5, 6.5); a.set_ylim(-6.5, 7.5)
    a.set_xlabel("$x$ [m]"); a.set_ylabel("$y$ [m]"); a.set_title("(b) flown paths (x: control lost)", fontsize=9)
    a.legend(fontsize=6, ncol=3, loc="upper center"); a.grid(alpha=0.3)
    panels = ((axs[1, 0], "tilt", "body tilt [deg]", "(c) body tilt (roll and pitch)"),
              (axs[1, 1], "yaw", "yaw change [deg]", "(d) yaw"),
              (axs[2, 0], "fmin", "smallest rotor force [N]", "(e) smallest rotor force"),
              (axs[2, 1], "fmax", "largest rotor force [N]", "(f) largest rotor force"))
    for a, key, ylab, title in panels:
        for k, r in runs.items():
            xi, col = DESIGNS[k]
            v = {"tilt": r["tilt"], "yaw": r["yaw"], "fmin": r["F"].min(axis=1), "fmax": r["F"].max(axis=1)}[key]
            a.plot(r["t"], v, color=col, lw=0.8, label=rf"$\bar\xi={xi}^\circ$")
            if r["lost_t"] is not None:
                a.plot(r["t"][-1], v[-1], "X", color=col, mec="k", ms=7)
        a.set_ylabel(ylab); a.set_xlabel("time after the take-off [s]"); a.set_title(title, fontsize=9); a.grid(alpha=0.3)
    axs[1, 0].set_yscale("log"); axs[1, 0].set_ylim(0.05, 100)
    axs[2, 0].axhline(F_MIN, color="k", ls="--", lw=0.8, label=r"$F_{\min}=1$ N")
    axs[2, 1].axhline(F_MAX, color="k", ls="--", lw=0.8, label=r"$F_{\max}$")
    for a in (axs[2, 0], axs[2, 1]):
        a.legend(fontsize=6, ncol=3, loc="upper center")
        lo, hi = a.get_ylim(); a.set_ylim(lo, hi + 0.35 * (hi - lo))
    fig.savefig(fig_dir / "stability.pdf"); fig.savefig(fig_dir / "svg" / "stability.svg")
    print("wrote", fig_dir / "stability.pdf")

    summary = {}
    for k, r in runs.items():
        F = r["F"]
        summary[k] = {"tilt_deg": DESIGNS[k][0], "file": r["file"], "lost_s": r["lost_t"],
                      "max_tilt_deg": float(r["tilt"].max()), "yaw_span_deg": float(np.ptp(r["yaw"])),
                      "min_force_N": float(F.min()), "max_force_N": float(F.max()),
                      "time_a_rotor_at_zero_pct": float(100 * (F.min(axis=1) < 0.05).mean())}
        print(k, {a_: (round(b, 3) if isinstance(b, float) else b) for a_, b in summary[k].items()})
    (fig_dir / "summary.json").write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
