"""Architecture figure, journal style: panels = paper sections (IV, V, VI), one reading direction
(left to right), closed loop through the shared workcell, short box names, symbols on the arrows,
flat icons. IEEE double-column width 7.16 in."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle

plt.rcParams.update({"font.family": "Times New Roman", "mathtext.fontset": "stix"})

INK = "#2B2B2B"
ICON = "#4A4A4A"
PANEL = {"cell": "#EEF3F8", "est": "#FDF6E3", "dec": "#EAF5EE", "safe": "#FBEDEA"}
EDGEC = {"cell": "#5B7FA6", "est": "#B8952B", "dec": "#3E8E5E", "safe": "#B5523F"}
FS_T, FS_S, FS_P = 7.6, 7.0, 7.4    # box text, symbols and labels, panel titles
NL = "\n"
LB = dict(fc="white", ec="none", pad=0.6)

fig = plt.figure(figsize=(7.16, 2.7))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 716); ax.set_ylim(2, 272); ax.axis("off")


def panel(x0, y0, x1, y1, title, kind):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=7",
                                fc=PANEL[kind], ec=EDGEC[kind], lw=0.9))
    ax.text((x0 + x1) / 2, y1 - 9, title, ha="center", va="center", fontsize=FS_P, fontweight="bold",
            color=EDGEC[kind])


def box(x0, y0, x1, y1, title, sub=None, kind="dec", dashed=False):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=4",
                                fc="white", ec=EDGEC[kind], lw=0.8, ls=(0, (3, 2)) if dashed else "-"))
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if sub:
        ax.text(cx, cy + 6, title, ha="center", va="center", fontsize=FS_T, color=INK)
        ax.text(cx, cy - 7, sub, ha="center", va="center", fontsize=FS_S, color=INK)
    else:
        ax.text(cx, cy, title, ha="center", va="center", fontsize=FS_T, color=INK, linespacing=1.1)


def arrow(p, q, ls="-", both=False):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="<|-|>" if both else "-|>", mutation_scale=7, lw=0.8,
                                 color=INK, ls=ls, shrinkA=0, shrinkB=0))


def path(points, ls="-"):
    xs, ys = zip(*points)
    ax.plot(xs[:-1], ys[:-1], color=INK, lw=0.8, ls=ls, solid_capstyle="butt")
    arrow(points[-2], points[-1], ls=ls)


def lab(x, y, t):
    ax.text(x, y, t, ha="center", va="center", fontsize=FS_S, color=INK, bbox=LB)


Y0, Y1 = 36, 265                       # common panel bottom / top

# ======================= Shared workcell (flat icons)
panel(6, Y0, 134, Y1, "Shared workcell", "cell")
ax.add_patch(FancyBboxPatch((47, 216), 44, 13, boxstyle="round,pad=0,rounding_size=5", fc=ICON, ec=ICON))
for x in (58, 80):
    ax.add_patch(Circle((x, 222.5), 3.6, fc="white", ec="white"))
ax.plot([69, 69], [216, 207], color=ICON, lw=1.6)
ax.text(69, 198, "RGB-D camera", ha="center", va="center", fontsize=FS_S, color=INK)
ax.add_patch(Circle((38, 140), 10, fc=ICON, ec=ICON))
ax.add_patch(FancyBboxPatch((22, 80), 32, 48, boxstyle="round,pad=0,rounding_size=12", fc=ICON, ec=ICON))
ax.text(38, 66, "Worker", ha="center", va="center", fontsize=FS_S, color=INK)
ax.add_patch(Rectangle((88, 76), 28, 9, fc=ICON, ec=ICON))
for (xa, ya), (xb, yb) in (((102, 85), (95, 124)), ((95, 124), (118, 140))):
    ax.plot([xa, xb], [ya, yb], color=ICON, lw=4.2, solid_capstyle="round")
for (x, y) in ((102, 85), (95, 124)):
    ax.add_patch(Circle((x, y), 3.4, fc="white", ec=ICON, lw=1.2))
ax.plot([118, 124, 124], [140, 144, 134], color=ICON, lw=1.4)
ax.plot([118, 121, 128], [140, 130, 130], color=ICON, lw=1.4)
ax.text(102, 66, "Robot", ha="center", va="center", fontsize=FS_S, color=INK)
for xe, ye in ((44, 154), (106, 146)):
    ax.plot([69, xe], [189, ye], color="#7A8FA6", lw=0.6, ls=(0, (2, 2)))

# ======================= Human-state estimation (Sec. IV), including the safety geometry
panel(152, Y0, 300, Y1, "Human-state estimation (IV)", "est")
box(162, 214, 290, 240, "Mental fatigue", kind="est")
box(162, 179, 290, 205, "Physical fatigue", kind="est")
box(162, 144, 290, 170, "Attention", kind="est")
box(162, 52, 290, 112, "Safety geometry", r"$d_h,\ c,\ Z,\ R$", kind="est")

# ======================= POMDP decision layer (Sec. V)
panel(320, Y0, 474, Y1, "POMDP decision layer (V)", "dec")
box(330, 144, 392, 240, "Belief", r"$b_t(F_p,F_m,A)$", kind="dec")
box(406, 144, 466, 240, "QMDP" + NL + "policy", kind="dec")
box(348, 52, 448, 92, "Task layer", "mode, task step", kind="dec", dashed=True)

# ======================= Adaptation and safety layer (Sec. VI)
panel(496, Y0, 640, Y1, "Adaptation and safety (VI)", "safe")
box(508, 200, 630, 240, "Fatigue adaptation", r"and dialogue, $a_A$", kind="safe")
box(508, 134, 630, 174, "Safety supervisor", r"$a_S$", kind="safe")
box(508, 68, 630, 108, "Solo fatigue–distance law", r"$a_F,\ s$", kind="safe")

# ======================= outputs on the right
box(652, 200, 712, 240, "Worker" + NL + "interface", kind="cell")
box(652, 132, 712, 172, "Decision" + NL + "attribution", kind="cell", dashed=True)

# ======================= arrows
# camera -> estimators, and hand position -> safety geometry (one bus)
path([(91, 222.5), (142, 222.5), (142, 227), (162, 227)])
path([(142, 222.5), (142, 192), (162, 192)])
path([(142, 192), (142, 157), (162, 157)])
path([(142, 157), (142, 96), (162, 96)])
lab(152, 104, r"$\mathbf{p}_h$")
# robot joints -> safety geometry
arrow((134, 64), (162, 64))
lab(148, 72, r"$\mathbf{q}$")
# estimators -> belief
for y, t in ((227, r"$\hat F_m$"), (192, r"$\hat F_p$"), (157, r"$\hat A,\,\kappa_t$")):
    arrow((290, y), (330, y))
    lab(310, y + 8, t)
# belief -> policy -> adaptation
arrow((392, 192), (406, 192))
arrow((466, 220), (508, 220))
lab(486, 229, r"$a_N,\,\ell_t$")
# context into the policy
path([(290, 104), (420, 104), (420, 144)])
lab(352, 104, r"$Z,\ R$")
arrow((440, 92), (440, 144))
lab(440, 118, r"$M,\ I$")
# safety geometry -> supervisor and solo law (routed below the decision layer)
path([(226, 52), (226, 24), (486, 24), (486, 154), (508, 154)])
path([(486, 88), (508, 88)])
lab(356, 24, r"$d_h,\ c,\ Z,\ R$")
# chain inside the adaptation and safety layer
arrow((569, 200), (569, 174))
arrow((569, 134), (569, 108))
# dialogue and attribution
arrow((630, 220), (652, 220), both=True)
path([(640, 152), (652, 152)], ls="--")
# command back to the robot (closes the loop)
path([(569, 68), (569, 10), (102, 10), (102, 36)])
lab(400, 10, r"robot command $(a_F,\ s)$")

for out in sys.argv[1:]:
    fig.savefig(out, dpi=600)
print("saved")
