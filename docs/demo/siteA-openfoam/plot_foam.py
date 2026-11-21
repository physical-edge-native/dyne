import re, sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
case = sys.argv[1]; out = sys.argv[2]
def vals(path):
    s = open(path).read()
    m = re.search(r"internalField\s+nonuniform\s+List<\w+>\s*\n?(\d+)\s*\(", s)
    n = int(m.group(1)); body = s[m.end():]
    return np.array([float(v) for v in re.findall(r"[-+0-9.eE]+", body.split(")\n;")[0])[:n]])
cx, cy = vals(f"{case}/0/Cx"), vals(f"{case}/0/Cy")
h = 0.1
fig, axs = plt.subplots(1, 2, figsize=(11, 3.9), constrained_layout=True)
for ax, t, lab in zip(axs, ["0", "3600"], ["start (initial guess from 6 point readings)", "after 60 min of diffusion"]):
    T = vals(f"{case}/{t}/T")
    norm = matplotlib.colors.Normalize(18, 24); cmap = plt.get_cmap("coolwarm")
    ax.add_patch(Rectangle((cx.min()-h/2-0.15, cy.min()-h/2-0.15), cx.max()-cx.min()+h+0.3, cy.max()-cy.min()+h+0.3, color="#b9b9b4", zorder=0))
    for x, y, v in zip(cx, cy, T):
        ax.add_patch(Rectangle((x-h/2, y-h/2), h, h, color=cmap(norm(v)), lw=0))
    ax.set_xlim(cx.min()-0.3, cx.max()+0.3); ax.set_ylim(cy.min()-0.3, cy.max()+0.3); ax.set_aspect("equal")
    ax.set_title(lab, fontsize=11); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap); fig.colorbar(sm, ax=axs, shrink=0.85, label="T [°C] (dummy readings)")
fig.suptitle("OpenFOAM v2606 scalarTransportFoam on the room shape computed by six devices (grey: walls and objects, no cells)", fontsize=11)
fig.savefig(out, dpi=110)
print("cells", len(cx), "T0", vals(f"{case}/0/T").min(), vals(f"{case}/0/T").max(), "T3600", vals(f"{case}/3600/T").min(), vals(f"{case}/3600/T").max())
