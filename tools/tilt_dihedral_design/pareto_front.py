#!/usr/bin/env python3
"""
Scenario 1 of the paper: NSGA-II on the two objectives that select the design, from the random start to convergence.

    minimise the hover power ratio P/P0, maximise the rotor-failure margin F_fail,
    subject to (R1) attitude held in hover >= 12 deg, (R3) yaw acceleration >= 20 rad/s^2, (R4) clearance >= 20 mm

The population of every generation is saved (results/pareto2_history.npz); the exact front of the same problem comes from
the exhaustive map (results/design_map.npz).  Figure and video frames are black on white only.

    python3 pareto_front.py run        NSGA-II (population 60, 60 generations)
    python3 pareto_front.py figure     figures/pareto_front.pdf/.svg + results/pareto2.json
    python3 pareto_front.py frames     results/pareto2_frames/f###.png (1280 x 720)
"""

import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import indices as ix
import nsga2
import tilt_model as tm

HERE = Path(__file__).resolve().parent
RES, FIG = HERE / "results", HERE / "figures"
Q = tm.load_params()
FMIN, TILT_MIN, YAW_MIN, CLEAR_MIN = 1.0, 12.0, 20.0, 20.0
POP, GENS = 60, 60


def one(x):
    xi, ze = tm.pattern(math.radians(x[0]), math.radians(x[1]))
    A = tm.allocation(Q, xi, ze)
    r, d = tm.geometry(Q, xi, ze)
    clear = 1000 * ix.clearance(r, d, Q["prop_d"])
    if np.linalg.cond(A) > 1e8:
        return [10, 0], [TILT_MIN, YAW_MIN, CLEAR_MIN - clear]
    F = [ix.power_ratio(A, Q), -ix.failure_fmin(A, Q)]
    G = [TILT_MIN - math.degrees(ix.hover_tilt(A, Q, FMIN)[0]), YAW_MIN - ix.moment_acc(A, Q, FMIN)[2],
         CLEAR_MIN - clear]
    return F, G


def evaluate(X):
    with Pool() as p:
        out = p.map(one, list(X))
    return np.array([o[0] for o in out]), np.array([o[1] for o in out])


def run():
    hist = {"gen": [], "X": [], "F": [], "V": []}

    def keep(gen, X, F, V):
        for k, v in zip(("gen", "X", "F", "V"), (gen, X, F, V)):
            hist[k].append(v)

    nsga2.nsga2(evaluate, [0, -30], [50, 30], pop=POP, gens=GENS, seed=1, log=10, history=keep)
    np.savez(RES / "pareto2_history.npz", **{k: np.array(v) for k, v in hist.items()})


def exact_front():
    D = np.load(RES / "design_map.npz")
    ok = (np.isfinite(D["power_ratio"]) & (D["hover_tilt_deg"] >= TILT_MIN) & (D["acc_z"] >= YAW_MIN)
          & (D["clearance_mm"] >= CLEAR_MIN))
    P, f = D["power_ratio"][ok], D["fail_fmin"][ok]
    o = np.argsort(P)
    front, best = [], -np.inf
    for i in o:                                       # non-dominated: more margin than every cheaper design
        if f[i] > best:
            front.append(i); best = f[i]
    return np.c_[P[front], f[front]]


def style(ax):
    ax.set_xlim(1.15, 2.25); ax.set_ylim(0.0, 2.2)
    ax.set_xlabel(r"hover power $P/P_0$"); ax.set_ylabel(r"rotor-failure margin $F_{\mathrm{fail}}$ [N]")


def igd(F, V, E):
    P = np.c_[F[V == 0, 0], -F[V == 0, 1]]
    if not len(P):
        return np.nan
    lo, span = E.min(0), E.max(0) - E.min(0)
    A, B = (E - lo) / span, (P - lo) / span
    return float(np.sqrt(((A[:, None] - B[None]) ** 2).sum(-1)).min(1).mean())


def figure():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 9, "mathtext.fontset": "cm", "savefig.bbox": "tight",
                         "svg.fonttype": "none"})
    H = np.load(RES / "pareto2_history.npz")
    E = exact_front()
    gens = H["gen"]
    I = [igd(H["F"][k], H["V"][k], E) for k in range(len(gens))]
    feas = (H["V"] == 0).sum(1)
    last = H["F"][-1][H["V"][-1] == 0]
    k_opt = np.argmin(np.where(-last[:, 1] >= 1.0, last[:, 0], np.inf))
    xopt = H["X"][-1][H["V"][-1] == 0][k_opt]
    json.dump({"pop": POP, "gens": int(gens[-1]), "igd": I, "feasible": feas.tolist(), "exact_front_size": len(E),
               "best_meeting_1N": {"xib": float(xopt[0]), "zetab": float(xopt[1]), "P": float(last[k_opt, 0]),
                                   "F_fail": float(-last[k_opt, 1])}},
              open(RES / "pareto2.json", "w"), indent=1)
    print(f"IGD gen 0 {I[0]:.4f}, 5 {I[5]:.4f}, 10 {I[10]:.4f}, 20 {I[20]:.4f}, last {I[-1]:.4f}; feasible gen 0 "
          f"{feas[0]}, last {feas[-1]}; cheapest design with F_fail >= 1 N: xib {xopt[0]:.1f}, zetab {xopt[1]:.1f}, "
          f"P {last[k_opt, 0]:.4f}, F_fail {-last[k_opt, 1]:.3f}")
    snaps = [0, 5, 20, int(gens[-1])]
    fig, axs = plt.subplots(1, 4, figsize=(7.4, 2.2), sharey=True)
    for ax, g in zip(axs, snaps):
        k = int(np.flatnonzero(gens == g)[0])
        F, V = H["F"][k], H["V"][k]
        ax.plot(F[V == 0, 0], -F[V == 0, 1], "k.", ms=3)
        style(ax)
        ax.set_title(f"generation {g}", fontsize=8)
        if g != snaps[0]:
            ax.set_ylabel("")
        ax.tick_params(labelsize=7); ax.xaxis.label.set_size(8); ax.yaxis.label.set_size(8)
    fig.subplots_adjust(wspace=0.12)
    (FIG / "svg").mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / "svg" / "pareto_front.svg"); fig.savefig(FIG / "pareto_front.pdf")
    print("wrote figures/pareto_front.pdf")


def frames():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 16, "mathtext.fontset": "cm"})
    H = np.load(RES / "pareto2_history.npz")
    out = RES / "pareto2_frames"
    out.mkdir(exist_ok=True)
    for old in out.glob("f*.png"):
        old.unlink()
    for k, g in enumerate(H["gen"]):
        fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
        F, V = H["F"][k], H["V"][k]
        ax.plot(F[V == 0, 0], -F[V == 0, 1], "k.", ms=9)
        style(ax)
        ax.text(0.03, 0.95, f"generation {int(g)}", transform=ax.transAxes, fontsize=18, va="top")
        fig.subplots_adjust(left=0.08, right=0.97, bottom=0.11, top=0.97)
        fig.savefig(out / f"f{k:03d}.png")
        plt.close(fig)
    print(f"wrote {len(H['gen'])} frames in {out}")


if __name__ == "__main__":
    {"run": run, "figure": figure, "frames": frames}[sys.argv[1]]()
