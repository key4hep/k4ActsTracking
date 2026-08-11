#!/usr/bin/env python3
#
# Copyright (c) 2014-2024 Key4hep-Project.
#
# This file is part of Key4hep.
# See https://key4hep.github.io/key4hep-doc/ for further info.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""Compare a geantino scan against the material re-measured from the map.

Step 5 of doc/material_mapping.md. Takes the Geant4 scan (the truth) and the
output of ``material_validation.py`` (the mapped geometry, re-measured by
propagation) and reports how faithfully the map reproduces the material.

Both files are in the same ``material_tracks`` format and, because the validation
propagates the scan's own directions in order, entry *i* of one is the same
geantino as entry *i* of the other. The comparison is therefore track by track,
with no binning or sampling differences in the way.

  python3 compare_material_tracks.py geant4_material_tracks.root \\
      propagated_material_tracks.root -o validation/

Produces, in the output directory:

  x0_vs_eta.png     <X0> against eta, both samples plus their ratio
  x0_vs_phi.png     the same against phi
  x0_ratio.png      distribution of the per-track ratio
  x0_map_eta_phi.png  2D map of the ratio, to localise where material is missing

and prints a summary. A uniform ratio near 1 is the goal. A deficit confined to
some eta range points at material that fell outside every designated receiver;
see the "Designation" section of the workflow doc.
"""

import argparse
import math
import sys
from pathlib import Path


def read_tracks(path, tree_name="material_tracks", max_tracks=-1):
    """Read (eta, phi, X0, L0) per track. Returns four numpy arrays."""
    import numpy as np
    import ROOT

    f = ROOT.TFile.Open(str(path))
    if f is None or f.IsZombie():
        raise SystemExit(f"Could not open {path}")
    tree = f.Get(tree_name)
    if not tree:
        raise SystemExit(f"No tree '{tree_name}' in {path}")

    n = tree.GetEntries()
    if max_tracks >= 0:
        n = min(n, max_tracks)

    eta = np.empty(n)
    phi = np.empty(n)
    x0 = np.empty(n)
    l0 = np.empty(n)
    for i in range(n):
        tree.GetEntry(i)
        eta[i] = tree.v_eta
        phi[i] = tree.v_phi
        x0[i] = tree.t_X0
        l0[i] = tree.t_L0
    f.Close()
    return eta, phi, x0, l0


def profile(x, y, bins, lo, hi):
    """Mean of y in bins of x, with the standard error on the mean."""
    import numpy as np

    edges = np.linspace(lo, hi, bins + 1)
    idx = np.digitize(x, edges) - 1
    centres, means, errs = [], [], []
    for b in range(bins):
        sel = y[idx == b]
        if sel.size == 0:
            continue
        centres.append(0.5 * (edges[b] + edges[b + 1]))
        means.append(sel.mean())
        errs.append(sel.std() / math.sqrt(sel.size) if sel.size > 1 else 0.0)
    return np.array(centres), np.array(means), np.array(errs)


def plot_profile(ax_top, ax_bot, var, scan_v, prop_v, x0_scan, x0_prop, bins, lo, hi, xlabel):
    import numpy as np

    cs, ms, es = profile(scan_v, x0_scan, bins, lo, hi)
    cp, mp, ep = profile(prop_v, x0_prop, bins, lo, hi)

    ax_top.errorbar(cs, ms, yerr=es, fmt="o-", ms=3, label="Geant4 scan (truth)")
    ax_top.errorbar(cp, mp, yerr=ep, fmt="s-", ms=3, label="mapped geometry")
    ax_top.set_ylabel(r"$\langle X/X_0 \rangle$")
    ax_top.legend()
    ax_top.grid(alpha=0.3)

    # ratio on the common bins only
    n = min(len(cs), len(cp))
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(ms[:n] > 0, mp[:n] / ms[:n], np.nan)
    ax_bot.plot(cs[:n], ratio, "o-", ms=3, color="k")
    ax_bot.axhline(1.0, color="r", ls="--", lw=1)
    ax_bot.set_ylim(0.0, 2.0)
    ax_bot.set_ylabel("mapped / truth")
    ax_bot.set_xlabel(xlabel)
    ax_bot.grid(alpha=0.3)
    return ratio


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("scan", help="Geant4 geantino scan ROOT file")
    p.add_argument("propagated", help="output of material_validation.py")
    p.add_argument("-o", "--output-dir", default="material_validation")
    p.add_argument("--tree-name", default="material_tracks")
    p.add_argument("--bins", type=int, default=50)
    p.add_argument("--eta-range", nargs=2, type=float, default=None,
                   help="defaults to the range present in the scan")
    p.add_argument("--max-tracks", type=int, default=-1)
    args = p.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    eta_s, phi_s, x0_s, l0_s = read_tracks(args.scan, args.tree_name, args.max_tracks)
    eta_p, phi_p, x0_p, l0_p = read_tracks(args.propagated, args.tree_name, args.max_tracks)

    n = min(len(eta_s), len(eta_p))
    if len(eta_s) != len(eta_p):
        print(
            f"note: {len(eta_s)} scan tracks vs {len(eta_p)} propagated; comparing the first {n}",
            file=sys.stderr,
        )
    eta_s, phi_s, x0_s, l0_s = eta_s[:n], phi_s[:n], x0_s[:n], l0_s[:n]
    eta_p, phi_p, x0_p, l0_p = eta_p[:n], phi_p[:n], x0_p[:n], l0_p[:n]

    # The validation propagates the scan's own directions in order, so a
    # mismatch here means the two files are not track-aligned and everything
    # below would be comparing unrelated geantinos.
    misaligned = np.abs(eta_s - eta_p) > 1e-6
    if misaligned.any():
        print(
            f"WARNING: {misaligned.sum()} of {n} tracks have different eta in the two "
            "files -- they are not aligned, so the per-track ratio is meaningless. "
            "Were they produced from the same scan, in the same order?",
            file=sys.stderr,
        )

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    lo, hi = args.eta_range if args.eta_range else (eta_s.min(), eta_s.max())

    for var_s, var_p, lo_, hi_, xlabel, fname in [
        (eta_s, eta_p, lo, hi, r"$\eta$", "x0_vs_eta.png"),
        (phi_s, phi_p, -math.pi, math.pi, r"$\phi$ [rad]", "x0_vs_phi.png"),
    ]:
        fig, (a, b) = plt.subplots(
            2, 1, sharex=True, height_ratios=[3, 1], figsize=(7, 6)
        )
        plot_profile(a, b, None, var_s, var_p, x0_s, x0_p, args.bins, lo_, hi_, xlabel)
        a.set_title("Material: Geant4 scan vs mapped geometry")
        fig.tight_layout()
        fig.savefig(out / fname, dpi=140)
        plt.close(fig)

    # per-track ratio
    with np.errstate(divide="ignore", invalid="ignore"):
        per_track = np.where(x0_s > 0, x0_p / x0_s, np.nan)
    finite = per_track[np.isfinite(per_track)]

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.hist(finite, bins=100, range=(0, 2), histtype="step", color="k")
    ax.axvline(1.0, color="r", ls="--", lw=1)
    ax.set_xlabel("mapped / truth, per track")
    ax.set_ylabel("tracks")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "x0_ratio.png", dpi=140)
    plt.close(fig)

    # 2D map of where material is missing
    fig, ax = plt.subplots(figsize=(7.5, 5))
    nb = max(10, args.bins // 2)
    sum_p, xe, ye = np.histogram2d(eta_s, phi_s, bins=[nb, nb],
                                   range=[[lo, hi], [-math.pi, math.pi]], weights=x0_p)
    sum_s, _, _ = np.histogram2d(eta_s, phi_s, bins=[nb, nb],
                                 range=[[lo, hi], [-math.pi, math.pi]], weights=x0_s)
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio2d = np.where(sum_s > 0, sum_p / sum_s, np.nan)
    im = ax.pcolormesh(xe, ye, ratio2d.T, vmin=0.0, vmax=2.0, cmap="RdBu_r")
    fig.colorbar(im, ax=ax, label="mapped / truth")
    ax.set_xlabel(r"$\eta$")
    ax.set_ylabel(r"$\phi$ [rad]")
    ax.set_title("Where the map under- or over-counts material")
    fig.tight_layout()
    fig.savefig(out / "x0_map_eta_phi.png", dpi=140)
    plt.close(fig)

    tot_s, tot_p = x0_s.sum(), x0_p.sum()
    print(f"tracks compared        : {n}")
    print(f"<X/X0> Geant4 scan     : {x0_s.mean():.4f}")
    print(f"<X/X0> mapped geometry : {x0_p.mean():.4f}")
    print(f"integral ratio         : {tot_p / tot_s:.4f}" if tot_s > 0 else "integral ratio: n/a")
    if finite.size:
        print(f"per-track ratio median : {np.median(finite):.4f}")
        print(f"tracks with <50% of truth: {100.0 * (finite < 0.5).mean():.1f}%")
    print(f"\nplots written to {out}/")


if __name__ == "__main__":
    main()
