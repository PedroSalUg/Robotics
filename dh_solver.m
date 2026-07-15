"""
Denavit-Hartenberg Method - Complete Solver & Animator
All 10 exercises from the PDF
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D
import os

OUT = "/mnt/user-data/outputs/"
os.makedirs(OUT, exist_ok=True)

# ─────────────────────────────────────────────────────────────
# CORE DH FUNCTIONS
# ─────────────────────────────────────────────────────────────
def dh_matrix(theta, d, a, alpha):
    """Homogeneous transformation matrix for one DH link."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def chain(matrices):
    T = np.eye(4)
    for M in matrices:
        T = T @ M
    return T

def fmt_matrix(T, decimals=4):
    """Return list of formatted row strings."""
    rows = []
    for r in T:
        rows.append([f"{v:.{decimals}f}" for v in r])
    return rows

def end_effector(T):
    return T[:3, 3]

# ─────────────────────────────────────────────────────────────
# ANIMATION HELPERS
# ─────────────────────────────────────────────────────────────
def save_frames_gif(frames, path, fps=12):
    """Save list of matplotlib figures as animated GIF."""
    from PIL import Image
    import io
    imgs = []
    for fig in frames:
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=90, bbox_inches='tight',
                    facecolor='#0d1117')
        buf.seek(0)
        imgs.append(Image.open(buf).copy())
        plt.close(fig)
    duration = int(1000 / fps)
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 loop=0, duration=duration)
    print(f"  GIF saved → {path}")

STYLE = dict(facecolor='#0d1117', edgecolor='none')
LINK_COLOR = '#00bfff'
JOINT_COLOR = '#ff6b35'
EE_COLOR = '#39ff14'
AXIS_COLOR = '#888'
TEXT_COLOR = '#e0e0e0'
GRID_COLOR = '#1e2a3a'

def styled_fig_2d(xlim, ylim, title=""):
    fig, ax = plt.subplots(figsize=(7, 6), facecolor='#0d1117')
    ax.set_facecolor('#0d1117')
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect('equal')
    ax.grid(True, color=GRID_COLOR, linewidth=0.6)
    ax.tick_params(colors=TEXT_COLOR)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID_COLOR)
    ax.set_title(title, color=TEXT_COLOR, fontsize=11, pad=8)
    ax.axhline(0, color=AXIS_COLOR, lw=0.8)
    ax.axvline(0, color=AXIS_COLOR, lw=0.8)
    return fig, ax

def draw_link_2d(ax, p0, p1):
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=LINK_COLOR, lw=3, solid_capstyle='round')

def draw_joint_2d(ax, p, r=0.08):
    c = plt.Circle(p[:2], r, color=JOINT_COLOR, zorder=5)
    ax.add_patch(c)

def draw_ee_2d(ax, p):
    ax.plot(p[0], p[1], '*', color=EE_COLOR, markersize=14, zorder=6)

# ─────────────────────────────────────────────────────────────
# EXERCISE 1 – 2-DOF Planar (a1=2, a2=1)
# ─────────────────────────────────────────────────────────────
def exercise1():
    print("Exercise 1: 2-DOF Planar")
    frames = []
    n_frames = 60
    a1, a2 = 2.0, 1.0

    for i in range(n_frames):
        t = i / n_frames
        th1 = 2 * np.pi * t
        th2 = np.pi * np.sin(2 * np.pi * t)

        T1 = dh_matrix(th1, 0, a1, 0)
        T2 = dh_matrix(th2, 0, a2, 0)
        T  = T1 @ T2

        p0 = np.array([0, 0])
        p1 = T1[:2, 3]
        p2 = T[:2, 3]

        fig, ax = styled_fig_2d((-4, 4), (-4, 4),
            f"Ej.1 – 2-DOF Planar | θ1={np.degrees(th1):.0f}° θ2={np.degrees(th2):.0f}°")
        draw_link_2d(ax, p0, p1)
        draw_link_2d(ax, p1, p2)
        draw_joint_2d(ax, np.append(p0, 0))
        draw_joint_2d(ax, np.append(p1, 0))
        draw_ee_2d(ax, p2)
        ax.set_xlabel('X [m]', color=TEXT_COLOR)
        ax.set_ylabel('Y [m]', color=TEXT_COLOR)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej1_2dof_planar.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 2 – SCARA (d1=const, a1=1, θ2 var, d3 var)
# ─────────────────────────────────────────────────────────────
def exercise2():
    print("Exercise 2: SCARA")
    frames = []
    n_frames = 72
    a1, a2 = 1.0, 0.8
    d1_const = 0.5

    for i in range(n_frames):
        t = i / n_frames
        th1 = 2 * np.pi * t
        th2 = -np.pi/2 * np.sin(2 * np.pi * t)
        d3  = 0.2 + 0.3 * (1 - np.cos(2 * np.pi * t)) / 2

        T1 = dh_matrix(th1,  d1_const, a1,  0)
        T2 = dh_matrix(th2,  0,        a2,  np.pi)
        T3 = dh_matrix(0,    -d3,      0,   0)
        T  = T1 @ T2 @ T3

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.array([0, 0, 0])
        p1 = T1[:3, 3]
        p2 = (T1@T2)[:3, 3]
        p3 = T[:3, 3]

        pts = [p0, p1, p2, p3]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]

        ax.plot(xs, ys, zs, color=LINK_COLOR, lw=3)
        ax.scatter(xs[:-1], ys[:-1], zs[:-1], color=JOINT_COLOR, s=80, zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=EE_COLOR, s=120, marker='*', zorder=6)

        lim = 1.8
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-0.1, 1.2)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.2 – SCARA | θ1={np.degrees(th1):.0f}° θ2={np.degrees(th2):.0f}° d3={d3:.2f}m",
                     color=TEXT_COLOR, fontsize=10)
        ax.view_init(elev=25, azim=i*3)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej2_scara.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 3 – RRR Planar (a1=a2=a3=1)
# ─────────────────────────────────────────────────────────────
def exercise3():
    print("Exercise 3: RRR Planar")
    frames = []
    n_frames = 60

    for i in range(n_frames):
        t = i / n_frames
        th1 = np.pi/2 * np.sin(2*np.pi*t)
        th2 = np.pi/3 * np.sin(4*np.pi*t + 0.5)
        th3 = np.pi/4 * np.cos(2*np.pi*t)

        T1 = dh_matrix(th1, 0, 1, 0)
        T2 = dh_matrix(th2, 0, 1, 0)
        T3 = dh_matrix(th3, 0, 1, 0)

        p0 = np.array([0, 0])
        p1 = T1[:2, 3]
        p2 = (T1@T2)[:2, 3]
        p3 = (T1@T2@T3)[:2, 3]

        fig, ax = styled_fig_2d((-3.5, 3.5), (-3.5, 3.5),
            f"Ej.3 – RRR Planar | θ1={np.degrees(th1):.0f}° θ2={np.degrees(th2):.0f}° θ3={np.degrees(th3):.0f}°")
        draw_link_2d(ax, p0, p1)
        draw_link_2d(ax, p1, p2)
        draw_link_2d(ax, p2, p3)
        for p in [p0, p1, p2]:
            draw_joint_2d(ax, np.append(p, 0))
        draw_ee_2d(ax, p3)
        ax.set_xlabel('X [m]', color=TEXT_COLOR)
        ax.set_ylabel('Y [m]', color=TEXT_COLOR)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej3_rrr_planar.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 4 – RR Perpendicular Axes
# ─────────────────────────────────────────────────────────────
def exercise4():
    print("Exercise 4: RR Perpendicular Axes")
    frames = []
    n_frames = 72
    a1, a2 = 1.0, 1.0

    for i in range(n_frames):
        t = i / n_frames
        th1 = 2 * np.pi * t
        th2 = np.pi/2 * np.sin(2*np.pi*t + np.pi/4)

        # Link 1: rotates around Z0, alpha=90° (makes axis 2 perpendicular)
        T1 = dh_matrix(th1, 0, a1, np.pi/2)
        # Link 2: rotates around X1
        T2 = dh_matrix(th2, 0, a2, 0)
        T  = T1 @ T2

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.zeros(3)
        p1 = T1[:3, 3]
        p2 = T[:3, 3]

        for pa, pb in [(p0, p1), (p1, p2)]:
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                    color=LINK_COLOR, lw=3)
        ax.scatter(*zip(p0, p1), color=JOINT_COLOR, s=80)
        ax.scatter([p2[0]], [p2[1]], [p2[2]], color=EE_COLOR, s=120, marker='*')

        lim = 2.2
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.4 – RR Perp. | θ1={np.degrees(th1):.0f}° θ2={np.degrees(th2):.0f}°",
                     color=TEXT_COLOR, fontsize=10)
        ax.view_init(elev=20, azim=i*3)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej4_rr_perpendicular.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 5 – 2-Link Right-Angle Assignment
# ─────────────────────────────────────────────────────────────
def exercise5():
    print("Exercise 5: 2-Link Right Angle DH")
    frames = []
    n_frames = 60
    a1, a2 = 1.2, 0.8
    d1 = 0.5

    for i in range(n_frames):
        t = i / n_frames
        th1 = np.pi * np.sin(2*np.pi*t)
        th2 = np.pi/2 * np.cos(2*np.pi*t)

        T1 = dh_matrix(th1, d1, a1, np.pi/2)
        T2 = dh_matrix(th2, 0,  a2, 0)
        T  = T1 @ T2

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.zeros(3)
        p1 = T1[:3, 3]
        p2 = T[:3, 3]

        for pa, pb in [(p0, p1), (p1, p2)]:
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                    color=LINK_COLOR, lw=3)
        ax.scatter(*zip(p0, p1), color=JOINT_COLOR, s=80)
        ax.scatter([p2[0]], [p2[1]], [p2[2]], color=EE_COLOR, s=120, marker='*')

        lim = 2.2
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-0.2, 2)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.5 – 2-Link Ángulo Recto | θ1={np.degrees(th1):.0f}° θ2={np.degrees(th2):.0f}°",
                     color=TEXT_COLOR, fontsize=10)
        ax.view_init(elev=25, azim=i*4)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej5_right_angle.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 6 – PUMA 560 First 3 Joints (RRR)
# ─────────────────────────────────────────────────────────────
def exercise6():
    print("Exercise 6: PUMA 560 (3 joints)")
    frames = []
    n_frames = 72

    # PUMA 560 DH parameters (first 3 links, generic)
    # d = [0, 0, d3], a = [0, a2, a3], alpha = [pi/2, 0, -pi/2]
    a2, a3, d3 = 0.4318, 0.0, 0.1500

    for i in range(n_frames):
        t = i / n_frames
        th1 = 2*np.pi*t
        th2 = np.pi/3*np.sin(2*np.pi*t) - np.pi/4
        th3 = np.pi/4*np.cos(4*np.pi*t)

        T1 = dh_matrix(th1, 0,   0,    np.pi/2)
        T2 = dh_matrix(th2, 0,   a2,   0)
        T3 = dh_matrix(th3, d3,  a3,  -np.pi/2)
        T  = T1 @ T2 @ T3

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.zeros(3)
        p1 = T1[:3, 3]
        p2 = (T1@T2)[:3, 3]
        p3 = T[:3, 3]

        pts = [p0, p1, p2, p3]
        for pa, pb in zip(pts[:-1], pts[1:]):
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                    color=LINK_COLOR, lw=3)
        for p in pts[:-1]:
            ax.scatter([p[0]], [p[1]], [p[2]], color=JOINT_COLOR, s=80)
        ax.scatter([p3[0]], [p3[1]], [p3[2]], color=EE_COLOR, s=120, marker='*')

        lim = 0.7
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-0.2, 0.6)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.6 – PUMA 560 (3 DOF) | θ1={np.degrees(th1):.0f}°",
                     color=TEXT_COLOR, fontsize=10)
        ax.view_init(elev=20, azim=i*3)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej6_puma560.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 7 – PR Robot (Prismatic + Rotational)
# ─────────────────────────────────────────────────────────────
def exercise7():
    print("Exercise 7: PR Robot (Prismatic + Rotational)")
    frames = []
    n_frames = 60
    a2 = 0.8

    for i in range(n_frames):
        t = i / n_frames
        d1 = 0.3 + 0.4*(1 - np.cos(2*np.pi*t))/2   # prismatic
        th2 = np.pi * np.sin(2*np.pi*t)              # rotational

        # Link1 prismatic along Z: theta=0, d=d1, a=0, alpha=0
        T1 = dh_matrix(0,   d1, 0,  0)
        # Link2 rotational in XY plane
        T2 = dh_matrix(th2, 0,  a2, 0)
        T  = T1 @ T2

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.zeros(3)
        p1 = T1[:3, 3]
        p2 = T[:3, 3]

        for pa, pb in [(p0, p1), (p1, p2)]:
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                    color=LINK_COLOR, lw=3)
        # Prismatic = rectangle visual
        ax.scatter([p0[0]], [p0[1]], [p0[2]], color='#aaaaff', s=100, marker='s')
        ax.scatter([p1[0]], [p1[1]], [p1[2]], color=JOINT_COLOR, s=80)
        ax.scatter([p2[0]], [p2[1]], [p2[2]], color=EE_COLOR, s=120, marker='*')

        ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2); ax.set_zlim(-0.1, 1.0)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.7 – PR Robot | d1={d1:.2f}m θ2={np.degrees(th2):.0f}°",
                     color=TEXT_COLOR, fontsize=10)
        ax.view_init(elev=25, azim=i*4)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej7_pr_robot.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 8 – RRP Robot (RR + Prismatic)
# ─────────────────────────────────────────────────────────────
def exercise8():
    print("Exercise 8: RRP Robot")
    frames = []
    n_frames = 60
    a1 = 0.6

    for i in range(n_frames):
        t = i / n_frames
        th1 = np.pi * np.sin(2*np.pi*t)
        th2 = np.pi/2 * np.cos(2*np.pi*t)
        d3  = 0.2 + 0.4*(1 - np.cos(4*np.pi*t))/2

        T1 = dh_matrix(th1, 0,  a1, np.pi/2)
        T2 = dh_matrix(th2, 0,  0,  0)
        T3 = dh_matrix(0,   d3, 0,  0)
        T  = T1 @ T2 @ T3

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.zeros(3)
        p1 = T1[:3, 3]
        p2 = (T1@T2)[:3, 3]
        p3 = T[:3, 3]

        pts = [p0, p1, p2, p3]
        for pa, pb in zip(pts[:-1], pts[1:]):
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                    color=LINK_COLOR, lw=3)
        for p in [p0, p1, p2]:
            ax.scatter([p[0]], [p[1]], [p[2]], color=JOINT_COLOR, s=80)
        ax.scatter([p3[0]], [p3[1]], [p3[2]], color=EE_COLOR, s=120, marker='*')

        ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.5, 1.5); ax.set_zlim(-0.5, 1.2)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.8 – RRP | θ1={np.degrees(th1):.0f}° θ2={np.degrees(th2):.0f}° d3={d3:.2f}m",
                     color=TEXT_COLOR, fontsize=10)
        ax.view_init(elev=20, azim=i*4)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej8_rrp_robot.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 9 – 3R evaluated at θ1=0, θ2=90°, θ3=0
# ─────────────────────────────────────────────────────────────
def exercise9():
    print("Exercise 9: 3R Evaluated")
    frames = []
    n_frames = 48

    # Trajectory: interpolate to target pose
    th1_t = np.radians(0)
    th2_t = np.radians(90)
    th3_t = np.radians(0)

    for i in range(n_frames):
        t = min(i / (n_frames * 0.7), 1.0)
        # ease-in-out
        t_e = t*t*(3 - 2*t)

        th1 = 0 + th1_t * t_e
        th2 = 0 + th2_t * t_e
        th3 = 0 + th3_t * t_e

        T1 = dh_matrix(th1, 0, 1, 0)
        T2 = dh_matrix(th2, 0, 1, 0)
        T3 = dh_matrix(th3, 0, 1, 0)

        p0 = np.array([0, 0])
        p1 = T1[:2, 3]
        p2 = (T1@T2)[:2, 3]
        p3 = (T1@T2@T3)[:2, 3]

        fig, ax = styled_fig_2d((-3.5, 3.5), (-1.5, 3.5),
            f"Ej.9 – 3R Evaluado | θ2→90°  t={t_e:.2f}")
        draw_link_2d(ax, p0, p1)
        draw_link_2d(ax, p1, p2)
        draw_link_2d(ax, p2, p3)
        for p in [p0, p1, p2]:
            draw_joint_2d(ax, np.append(p, 0))
        draw_ee_2d(ax, p3)

        # annotate end-effector
        ax.annotate(f"EE: ({p3[0]:.2f}, {p3[1]:.2f})",
                    xy=p3, xytext=(p3[0]+0.3, p3[1]+0.3),
                    color=EE_COLOR, fontsize=8,
                    arrowprops=dict(arrowstyle='->', color=EE_COLOR))

        ax.set_xlabel('X [m]', color=TEXT_COLOR)
        ax.set_ylabel('Y [m]', color=TEXT_COLOR)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej9_3r_evaluated.gif")

# ─────────────────────────────────────────────────────────────
# EXERCISE 10 – Anthropomorphic 3-DOF (RRR 3D)
# ─────────────────────────────────────────────────────────────
def exercise10():
    print("Exercise 10: Anthropomorphic 3-DOF")
    frames = []
    n_frames = 72

    a2, a3, d1 = 0.5, 0.4, 0.3

    for i in range(n_frames):
        t = i / n_frames
        th1 = 2*np.pi*t
        th2 = np.pi/3*np.sin(2*np.pi*t) - np.pi/6
        th3 = np.pi/4*np.cos(4*np.pi*t) + np.pi/8

        # Shoulder (rot Z), Elbow (rot Y), Wrist (rot Y)
        T1 = dh_matrix(th1, d1, 0,   np.pi/2)
        T2 = dh_matrix(th2, 0,  a2,  0)
        T3 = dh_matrix(th3, 0,  a3,  0)
        T  = T1 @ T2 @ T3

        fig = plt.figure(figsize=(8, 6), facecolor='#0d1117')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#0d1117')

        p0 = np.zeros(3)
        p1 = T1[:3, 3]
        p2 = (T1@T2)[:3, 3]
        p3 = T[:3, 3]

        pts = [p0, p1, p2, p3]
        for pa, pb in zip(pts[:-1], pts[1:]):
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                    color=LINK_COLOR, lw=3)
        for p in pts[:-1]:
            ax.scatter([p[0]], [p[1]], [p[2]], color=JOINT_COLOR, s=80)
        ax.scatter([p3[0]], [p3[1]], [p3[2]], color=EE_COLOR, s=140, marker='*')

        # Draw base
        theta_base = np.linspace(0, 2*np.pi, 30)
        ax.plot(0.1*np.cos(theta_base), 0.1*np.sin(theta_base),
                np.zeros(30), color='#555', lw=1)

        lim = 0.8
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-0.1, 0.9)
        ax.set_xlabel('X', color=TEXT_COLOR); ax.set_ylabel('Y', color=TEXT_COLOR)
        ax.set_zlabel('Z', color=TEXT_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        ax.set_title(f"Ej.10 – Antropomórfico 3-DOF | EE=({p3[0]:.2f},{p3[1]:.2f},{p3[2]:.2f})",
                     color=TEXT_COLOR, fontsize=9)
        ax.view_init(elev=25, azim=i*3)
        frames.append(fig)

    save_frames_gif(frames, OUT + "ej10_anthropomorphic.gif")


# ─────────────────────────────────────────────────────────────
# RUN ALL SIMULATIONS
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    plt.rcParams['animation.writer'] = 'pillow'
    exercise1()
    exercise2()
    exercise3()
    exercise4()
    exercise5()
    exercise6()
    exercise7()
    exercise8()
    exercise9()
    exercise10()
    print("\n✅ All simulations complete!")