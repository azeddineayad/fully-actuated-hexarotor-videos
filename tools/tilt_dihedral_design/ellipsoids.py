#!/usr/bin/env python3
"""
Force and torque ellipsoids against the exact attainable sets at hover (Scenario 1 of the paper).

At level hover F_h = A^-1 W_h; a change of wrench dW needs the rotor changes df = A^-1 dW, with F_min <= F_h + df <= F_max.
For the force (zero moment change) and the moment (zero force change) three sets are compared:

    exact     the polytope {dW : F_min <= F_h + A^-1 dW <= F_max}: its radius in a direction u is the ratio test t*(u)
    axis      the ellipsoid of the master thesis and of most design studies: semi-axes = the exact extents along the
              three body axes (mean of the + and - sides), i.e. it is exact on the axes and interpolates in between
    manip     the manipulability ellipsoid {dW = A df_restricted : |df| <= delta}, delta = min_i distance of F_h,i to
              its bounds (largest ball of rotor changes inside the box): {dW : |A^-1 dW| <= delta}; always inside the
              exact set (conservative)

    python3 ellipsoids.py            -> results/ellipsoids.json, results/ellipsoid_map.npz, figures/ellipsoids.pdf/.svg
"""

import json
import math
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import indices as ix
import tilt_model as tm

HERE = Path(__file__).resolve().parent
RES, FIG = HERE / "results", HERE / "figures"
FMIN = 1.0


def fib_sphere(n):
    i = np.arange(n) + 0.5
    phi, th = np.arccos(1 - 2 * i / n), math.pi * (1 + 5 ** 0.5) * i
    return np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]


def sets(A, q, rows, U):
    """Radii of the exact, axis and manipulability sets in the unit directions U (n x 3) of the wrench rows
    (0:3 force, 3:6 moment), the other three wrench components unchanged."""
    Ai = np.linalg.inv(A)
    f0 = ix.hover_forces(A, q)
    B = Ai[:, rows]                                     # 6 x 3: rotor change per unit wrench change
    exact = ix.ratio_step(f0, B @ U.T, FMIN, q["Fmax"])
    ax_p = ix.ratio_step(f0, B, FMIN, q["Fmax"])        # + axes
    ax_m = ix.ratio_step(f0, -B, FMIN, q["Fmax"])       # - axes
    semi = (ax_p + ax_m) / 2
    axis = 1.0 / np.sqrt(((U / semi) ** 2).sum(1))
    delta = min(f0.min() - FMIN, q["Fmax"] - f0.max())
    manip = delta / np.linalg.norm(B @ U.T, axis=0)
    return exact, axis, manip, semi, delta


def horizontal(n=360):
    psi = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return psi, np.c_[np.cos(psi), np.sin(psi), 0 * psi]


def design_values(xd, zd, q):
    xi, ze = tm.pattern(math.radians(xd), math.radians(zd))
    A = tm.allocation(q, xi, ze)
    psi, U = horizontal()
    e, a, m, semi, delta = sets(A, q, slice(0, 3), U)
    Us = fib_sphere(2000)
    ef, af, mf, _, _ = sets(A, q, slice(0, 3), Us)
    em, am, mm, semim, _ = sets(A, q, slice(3, 6), Us)
    return {"lat_exact": e.min() / q["m"], "lat_axis": a.min() / q["m"], "lat_manip": m.min() / q["m"],
            "force_exact_min": ef.min(), "force_axis_min": af.min(), "force_manip_min": mf.min(),
            "axis_over_max": float((af / ef).max()), "axis_under_min": float((af / ef).min()),
            "moment_exact_min": em.min(), "moment_axis_min": am.min(), "moment_manip_min": mm.min(),
            "moment_axis_over_max": float((am / em).max()), "semi_force": semi.tolist(), "semi_moment": semim.tolist(),
            "delta": delta}


def _map_row(args):
    xd, zs, q = args
    out = []
    for zd in zs:
        if xd < 3:
            out.append([np.nan] * 3)
            continue
        v = design_values(xd, zd, q)
        out.append([v["lat_exact"], v["lat_axis"], v["lat_manip"]])
    return out


def main():
    q = tm.load_params()
    res = {name: design_values(x, z, q) for name, (x, z) in
           {"current": (28, 0), "optimum": (28, -9.5)}.items()}
    for k, v in res.items():
        print(k, {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items() if not isinstance(b, list)})
    # map of the guaranteed lateral acceleration estimated by each set (1 deg grid)
    xs, zs = np.arange(0, 50.01, 1.0), np.arange(-30, 30.01, 1.0)
    with Pool() as p:
        M = np.array(p.map(_map_row, [(x, zs, q) for x in xs]))
    np.savez(RES / "ellipsoid_map.npz", xib_deg=xs, zetab_deg=zs, lat_exact=M[..., 0], lat_axis=M[..., 1],
             lat_manip=M[..., 2])
    ok = np.isfinite(M[..., 0])
    r_axis, r_manip = M[..., 1][ok] / M[..., 0][ok], M[..., 2][ok] / M[..., 0][ok]
    res["map"] = {"axis_over_exact": [float(r_axis.min()), float(r_axis.max())],
                  "manip_over_exact": [float(r_manip.min()), float(r_manip.max())]}
    print("lateral acceleration, ellipsoid / exact over the map: axis", res["map"]["axis_over_exact"],
          "manip", res["map"]["manip_over_exact"])
    json.dump(res, open(RES / "ellipsoids.json", "w"), indent=1)


if __name__ == "__main__" and len(__import__("sys").argv) == 1:
    main()


def polytope(A, q, rows):
    """Vertices and hull of the exact 3D set (force or moment) as a convex hull of a halfspace intersection."""
    from scipy.spatial import ConvexHull, HalfspaceIntersection
    Ai = np.linalg.inv(A)
    f0 = ix.hover_forces(A, q)
    B = Ai[:, rows]
    H = np.vstack([np.c_[B, -(q["Fmax"] - f0)], np.c_[-B, -(f0 - FMIN)]])     # B x <= Fmax - f0, -B x <= f0 - Fmin
    V = HalfspaceIntersection(H, np.zeros(3)).intersections
    return V, ConvexHull(V)


def figure():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 9, "mathtext.fontset": "cm", "savefig.bbox": "tight",
                         "svg.fonttype": "none"})
    q = tm.load_params()
    xi, ze = tm.pattern(math.radians(28), math.radians(-9.5))
    A = tm.allocation(q, xi, ze)
    fig, axs = plt.subplots(1, 3, figsize=(7.4, 2.75), gridspec_kw={"width_ratios": [1, 1, 1.15]})
    psi, U = horizontal(720)
    for ax, rows, cols, lab in ((axs[0], slice(0, 3), (0, 1), ("$\\Delta F_x$ [N]", "$\\Delta F_y$ [N]")),
                                (axs[1], slice(3, 6), (0, 2), ("$\\Delta M_x$ [N m]", "$\\Delta M_z$ [N m]"))):
        V = np.zeros((len(psi), 3))
        V[:, cols[0]], V[:, cols[1]] = np.cos(psi), np.sin(psi)
        e, a, m, _, _ = sets(A, q, rows, V)
        for r, st, nm in ((e, "k-", "exact set"), (a, "r--", "axis ellipsoid"), (m, "b:", "manipulability ellipsoid")):
            ax.plot(r * np.cos(psi), r * np.sin(psi), st, lw=1.1, label=nm)
        k = np.argmin(e)
        ax.plot([0, e[k] * np.cos(psi[k])], [0, e[k] * np.sin(psi[k])], "k-", lw=0.6)
        ax.set_aspect("equal"); ax.set_xlabel(lab[0]); ax.set_ylabel(lab[1])
    axs[0].set_title("(a) horizontal force", fontsize=9)
    axs[1].set_title("(b) moment $(M_x, M_z)$", fontsize=9)
    h, l = axs[0].get_legend_handles_labels()
    d = json.load(open(RES / "ellipsoid_design.json"))
    reqs = sorted(d, key=float)
    w = 0.25
    for j, (k, c, nm) in enumerate((("lat_exact", "k", "exact"), ("lat_axis", "tab:red", "axis ell."),
                                    ("lat_manip", "tab:blue", "manip. ell."))):
        x = np.arange(len(reqs)) + (j - 1) * w
        axs[2].bar(x, [d[r][k]["P"] for r in reqs], w, color=c, alpha=0.8, label=nm)
        for xx, r in zip(x, reqs):
            miss = d[r][k]["true_lat"] / float(r) - 1
            axs[2].text(xx, d[r][k]["P"] + 0.02, f"{100 * miss:+.0f}%", ha="center", fontsize=6, rotation=90)
    axs[2].set_xticks(range(len(reqs)), [f"{float(r):g}" for r in reqs])
    axs[2].set_xlabel("required $a_{\\mathrm{lat}}$ [m/s$^2$]"); axs[2].set_ylabel("hover power $P/P_0$ of the design")
    axs[2].set_ylim(1, 2.35)
    axs[2].legend(fontsize=6.5, loc="upper left")
    axs[2].set_title("(c) designs sized with each set", fontsize=9)
    fig.legend(h, l, loc="lower center", ncol=3, fontsize=7, frameon=False, bbox_to_anchor=(0.36, -0.02))
    fig.subplots_adjust(wspace=0.45, bottom=0.25)
    (FIG / "svg").mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / "svg" / "ellipsoids.svg"); fig.savefig(FIG / "ellipsoids.pdf")
    print("wrote figures/ellipsoids.pdf")


if __name__ == "__main__" and "--figure" in __import__("sys").argv:
    figure()


def frames(n=120):
    """Rotating 3D view (1280 x 720 PNG frames): exact force and moment sets of the optimum against the ellipsoids."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    q = tm.load_params()
    xi, ze = tm.pattern(math.radians(28), math.radians(-9.5))
    A = tm.allocation(q, xi, ze)
    out = RES / "ellipsoid_frames"
    out.mkdir(exist_ok=True)
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20))
    S = np.stack([np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v)], -1)
    data = []
    for rows in (slice(0, 3), slice(3, 6)):
        V, hull = polytope(A, q, rows)
        e, a, m, semi, delta = sets(A, q, rows, S.reshape(-1, 3))
        data.append((V, hull, (S * a.reshape(S.shape[:2])[..., None]), (S * m.reshape(S.shape[:2])[..., None])))
    d = json.load(open(RES / "ellipsoids.json"))["optimum"]
    for k in range(n):
        fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
        for j, (V, hull, Ea, Em) in enumerate(data):
            ax = fig.add_subplot(1, 2, j + 1, projection="3d")
            ax.add_collection3d(Poly3DCollection([V[s] for s in hull.simplices], facecolor=(0.2, 0.2, 0.2, 0.18),
                                                 edgecolor=(0, 0, 0, 0.25), lw=0.3))
            ax.plot_wireframe(*np.moveaxis(Ea, -1, 0), color="tab:red", lw=0.4, rstride=2, cstride=2)
            ax.plot_wireframe(*np.moveaxis(Em, -1, 0), color="tab:blue", lw=0.4, rstride=2, cstride=2)
            lim = np.abs(np.vstack([V, Ea.reshape(-1, 3)])).max(0) * 1.05     # each axis to its own range
            ax.set_xlim(-lim[0], lim[0]); ax.set_ylim(-lim[1], lim[1]); ax.set_zlim(-lim[2], lim[2])
            ax.set_box_aspect((1, 1, 1))
            ax.view_init(elev=20 + 10 * math.sin(2 * math.pi * k / n), azim=360 * k / n)
            if j == 0:
                ax.set_xlabel("$\\Delta F_x$ [N]"); ax.set_ylabel("$\\Delta F_y$ [N]"); ax.set_zlabel("$\\Delta F_z$ [N]")
                ax.set_title(f"force with zero moment (axes not to scale)\nweakest direction: exact "
                             f"{d['force_exact_min']:.2f} N\naxis ellipsoid {d['force_axis_min']:.2f} N, "
                             f"manipulability {d['force_manip_min']:.2f} N", fontsize=11)
            else:
                ax.set_xlabel("$\\Delta M_x$ [N m]"); ax.set_ylabel("$\\Delta M_y$ [N m]"); ax.set_zlabel("$\\Delta M_z$ [N m]")
                ax.set_title(f"moment with zero force\nweakest direction: exact {d['moment_exact_min']:.2f} N m\n"
                             f"axis ellipsoid {d['moment_axis_min']:.2f} N m, manipulability "
                             f"{d['moment_manip_min']:.2f} N m", fontsize=11)
        fig.suptitle("Optimum design (28 deg, -9.5 deg) at hover: exact attainable set (grey polytope), axis ellipsoid "
                     "(red), manipulability ellipsoid (blue)", fontsize=13)
        fig.savefig(out / f"f{k:03d}.png")
        plt.close(fig)
    print(f"wrote {n} frames in {out}")


if __name__ == "__main__" and "--frames" in __import__("sys").argv:
    frames()
