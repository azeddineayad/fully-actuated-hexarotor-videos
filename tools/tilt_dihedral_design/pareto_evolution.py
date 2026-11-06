#!/usr/bin/env python3
"""
Evolution of the NSGA-II population (results/nsga2_history.npz, run_nsga2.py) against the exact map
(results/design_map.npz): one frame per generation for the video, the figure of the paper (snapshots + convergence),
and the inverted generational distance (IGD) of each generation to the exact Pareto front.

    python3 pareto_evolution.py [--frames]      figure (figures/pareto_evolution.pdf/.svg) [+ PNG frames]
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
RES, FIG = HERE / "results", HERE / "figures"
FRAMES = RES / "pareto_frames"
YAW_MIN, CLEAR_MIN = 20.0, 20.0
plt.rcParams.update({"font.family": "serif", "font.size": 9, "mathtext.fontset": "cm", "savefig.bbox": "tight",
                     "svg.fonttype": "none"})


def nondominated(O):
    """Indices of the non-dominated rows of O (all objectives minimised)."""
    order = np.argsort(O[:, 0])
    keep = []
    for i in order:
        if not any(np.all(O[j] <= O[i]) and np.any(O[j] < O[i]) for j in keep):
            keep = [j for j in keep if not (np.all(O[i] <= O[j]) and np.any(O[i] < O[j]))] + [i]
    return np.array(keep)


def load():
    D = dict(np.load(RES / "design_map.npz"))
    H = dict(np.load(RES / "nsga2_history.npz"))
    X, Z = np.meshgrid(D["xib_deg"], D["zetab_deg"], indexing="ij")
    ok = np.isfinite(D["power_ratio"]) & (D["acc_z"] >= YAW_MIN) & (D["clearance_mm"] >= CLEAR_MIN)
    G = np.c_[D["power_ratio"][ok], -D["lat_acc"][ok], -D["fail_fmin"][ok]]
    front = nondominated(G)
    return D, H, X, Z, ok, G, front


def igd(Fpop, V, Gf, lo, span):
    """Mean distance of the exact front points to the feasible first front of the population (normalised objectives)."""
    P = Fpop[V == 0]
    if not len(P):
        return np.nan
    P = P[nondominated(P)]
    A, B = (Gf - lo) / span, (P - lo) / span
    return float(np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)).min(1).mean())


def draw(ax_d, ax_a, ax_f, D, X, Z, ok, G, front, x, F, V, gen):
    ax_d.contourf(X, Z, np.where(np.isfinite(D["power_ratio"]), D["power_ratio"], np.nan),
                  levels=np.arange(1.0, 2.21, 0.1), cmap="Greys", alpha=0.5)
    ax_d.contourf(X, Z, (~ok).astype(float), levels=[0.5, 1.5], colors="none", hatches=["///"])
    feas = V == 0
    ax_d.scatter(x[feas, 0], x[feas, 1], s=9, c="tab:red", edgecolors="none", label="feasible")
    ax_d.scatter(x[~feas, 0], x[~feas, 1], s=12, c="tab:blue", marker="x", lw=0.8, label="infeasible")
    ax_d.set_xlim(0, 50); ax_d.set_ylim(-30, 30)
    ax_d.set_xlabel(r"tilt $\bar\xi$ [deg]"); ax_d.set_ylabel(r"dihedral $\bar\zeta$ [deg]")
    ax_d.set_title(f"(a) design plane, generation {gen}", fontsize=9)
    for ax, k, lab in ((ax_a, 1, r"$a_{\mathrm{lat}}$ [m/s$^2$]"), (ax_f, 2, r"$F_{\mathrm{fail}}$ [N]")):
        if ax is None:
            continue
        ax.scatter(G[:, 0], -G[:, k], s=1, c="0.85", rasterized=True, label="all feasible designs (exact map)")
        ax.scatter(G[front, 0], -G[front, k], s=2, c="k", rasterized=True, label="exact Pareto front")
        ax.scatter(F[feas, 0], -F[feas, k], s=9, c="tab:red", edgecolors="none", label="NSGA-II population")
        ax.set_xlim(1.0, 2.2)
        ax.set_xlabel(r"hover power $P/P_0$"); ax.set_ylabel(lab)
    if ax_a is not None:
        ax_a.set_ylim(0, 6)
        ax_a.set_title("(b) power - lateral acceleration", fontsize=9)
    if ax_f is not None:
        ax_f.set_ylim(0, 2.2)
        ax_f.set_title("(c) power - rotor-failure margin", fontsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", action="store_true")
    args = ap.parse_args()
    D, H, X, Z, ok, G, front = load()
    Gf = G[front]
    lo, span = G.min(0), G.max(0) - G.min(0)
    gens = H["gen"]
    I = np.array([igd(H["F"][k], H["V"][k], Gf, lo, span) for k in range(len(gens))])
    nfeas = (H["V"] == 0).sum(1)
    json.dump({"gen": gens.tolist(), "igd": I.tolist(), "feasible": nfeas.tolist(), "exact_front_size": int(len(front))},
              open(RES / "pareto_evolution.json", "w"))
    print(f"exact front {len(front)} designs; IGD gen 0 {I[0]:.4f}, gen 10 {I[10]:.4f}, gen 50 {I[50]:.4f}, "
          f"gen {gens[-1]} {I[-1]:.4f}; feasible gen 0 {nfeas[0]}, last {nfeas[-1]}")

    # figure of the paper: four generations in the power - failure plane and in the design plane, and the IGD
    snaps = [0, 5, 20, int(gens[-1])]
    fig, axs = plt.subplots(2, 4, figsize=(7.4, 4.6), sharex="row", sharey="row")
    fig.subplots_adjust(hspace=0.62, wspace=0.12)
    for j, g in enumerate(snaps):
        k = int(np.flatnonzero(gens == g)[0])
        draw(axs[0, j], None, axs[1, j], D, X, Z, ok, G, front, H["X"][k], H["F"][k], H["V"][k], g)
        axs[0, j].set_title(f"generation {g}", fontsize=8)
        axs[1, j].set_title(f"IGD {I[k]:.3f}", fontsize=8)
        if j:
            axs[0, j].set_ylabel(""); axs[1, j].set_ylabel("")
    for a in axs.ravel():
        a.tick_params(labelsize=7)
        a.xaxis.label.set_size(8); a.yaxis.label.set_size(8)
    (FIG / "svg").mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / "svg" / "pareto_evolution.svg"); fig.savefig(FIG / "pareto_evolution.pdf")
    plt.close("all")
    fig, ax = plt.subplots(figsize=(3.6, 2.2))
    ax.semilogy(gens, I, "k-", lw=1)
    ax.set_xlabel("generation"); ax.set_ylabel("IGD to the exact front")
    ax2 = ax.twinx(); ax2.plot(gens, nfeas, color="tab:red", lw=0.8); ax2.set_ylabel("feasible designs", color="tab:red")
    fig.savefig(FIG / "svg" / "pareto_igd.svg"); fig.savefig(FIG / "pareto_igd.pdf")
    plt.close(fig)
    print("wrote figures/pareto_evolution.pdf, figures/pareto_igd.pdf")

    if args.frames:                                  # one 1280 x 720 frame per generation for the video
        FRAMES.mkdir(parents=True, exist_ok=True)
        for k, g in enumerate(gens):
            fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
            gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.25, left=0.06, right=0.97, top=0.88, bottom=0.08)
            ax_d, ax_i = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0])
            ax_a, ax_f = fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
            draw(ax_d, ax_a, ax_f, D, X, Z, ok, G, front, H["X"][k], H["F"][k], H["V"][k], int(g))
            ax_d.legend(fontsize=8, loc="lower right")
            ax_a.legend(fontsize=8, loc="lower right", markerscale=3)
            ax_i.semilogy(gens[:k + 1], I[:k + 1], "k-", lw=1.2)
            ax_i.set_xlim(0, gens[-1]); ax_i.set_ylim(np.nanmin(I) * 0.7, np.nanmax(I) * 1.5)
            ax_i.set_xlabel("generation"); ax_i.set_ylabel("IGD to the exact front")
            ax_i.set_title(f"(d) convergence: IGD {I[k]:.4f}, feasible {nfeas[k]}/80", fontsize=9)
            fig.suptitle("NSGA-II on the F680 design problem: minimise hover power, maximise lateral acceleration "
                         "and rotor-failure margin\n(yaw acceleration >= 20 rad/s$^2$, propeller clearance >= 20 mm; "
                         "population 80)", fontsize=11)
            fig.savefig(FRAMES / f"f{k:03d}.png")
            plt.close(fig)
        print(f"wrote {len(gens)} frames in {FRAMES}")


if __name__ == "__main__":
    main()
