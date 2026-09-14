"""
MuJoCo rendering of the laparoscopy palpation benchmark.

Runs the same task as `lap_benchmark.py` (tool-tip palpation through a compliant
trocar onto respiring Hunt--Crossley tissue, with a lateral RCM-stress load) on
the FR3 MuJoCo model (`LapEnv` -> `FR3MuJoCoEnv`) and renders it off-screen with
`mujoco.Renderer`.

The robot, floor and lighting are the real MuJoCo scene. The laparoscopy-
specific elements that live analytically in Python (the rigid instrument shaft,
the trocar / remote-centre-of-motion port, the moving tissue surface and the
tool--tissue contact force) are drawn as extra visualization geoms on top of the
MuJoCo scene so that the video shows exactly what the controller is regulating.

Run:
    python3 lap_video.py                       # default: mpc_kalman, lap_video.mp4
    python3 lap_video.py --mode mpc --fps 50
"""

from __future__ import annotations
import sys, argparse
from pathlib import Path
import numpy as np
import mujoco
import imageio.v2 as imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).parent))
from lap_env import LapEnv                                    # noqa: E402
from lap_controller import LapController, LapCtrlParams       # noqa: E402
from lap_benchmark import (references, lateral_axis, s_ref,   # noqa: E402
                           T_END, DT_DECIM, PUSH_WIN, PUSH_MAG)

# --------------------------------------------------------------------------
# rendering parameters
# --------------------------------------------------------------------------
W, H          = 1280, 720
PANEL_W       = 600          # width of the live-curve panel (px)
RENDER_EVERY  = 20            # render every N inner (1 kHz) steps -> 50 fps real
FPS           = 50
F_REF         = 4.0           # force (N) that maps to a full-length arrow
F_MAX         = 3.0           # hard tool--tissue force budget (N), paper eq.(10c)

MODE_LABEL = {
    "impedance":  "Classical Impedance",
    "mpc":        "Impedance MPC",
    "mpc_kalman": "Impedance MPC + oscillator Kalman",
}

# Task timeline (see lap_benchmark.py): the palpation manoeuvre, stage by stage.
# (start, end, short name, what the sim is doing, RGB colour)
STAGES = [
    (0.0,  2.0,  "Approach",       "advancing the tool through the trocar to the tissue",
     (90, 170, 255)),
    (2.0,  5.0,  "Palpation hold", "holding commanded depth on respiring tissue",
     (90, 210, 110)),
    (5.0,  7.0,  "RCM-stress load", "5 N lateral load stresses the remote centre of motion",
     (255, 140, 0)),
    (7.0,  9.0,  "Recovery",       "load removed; tracking and RCM recover",
     (90, 210, 200)),
    (9.0,  12.0, "Retraction",     "withdrawing the tool from the tissue",
     (180, 140, 255)),
]


def stage_info(t):
    """Return (index, name, description, colour) for the current task stage."""
    for i, (a, b, name, desc, col) in enumerate(STAGES):
        if a <= t < b:
            return i, name, desc, col
    last = len(STAGES) - 1
    return last, STAGES[last][2], STAGES[last][3], STAGES[last][4]


# --------------------------------------------------------------------------
# helpers for adding visualization geoms on top of the MuJoCo scene
# --------------------------------------------------------------------------
def _add_geom(scene, gtype, size, pos, mat, rgba):
    if scene.ngeom >= scene.maxgeom:
        return None
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, gtype,
                        np.asarray(size, dtype=np.float64),
                        np.asarray(pos, dtype=np.float64),
                        np.asarray(mat, dtype=np.float64).reshape(9),
                        np.asarray(rgba, dtype=np.float32))
    scene.ngeom += 1
    return g


def _add_connector(scene, gtype, width, frm, to, rgba):
    """Capsule/arrow connecting two world points (e.g. shaft, force arrow)."""
    if scene.ngeom >= scene.maxgeom:
        return None
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.asarray(rgba, dtype=np.float32))
    mujoco.mjv_connector(g, gtype, width,
                         np.asarray(frm, dtype=np.float64),
                         np.asarray(to, dtype=np.float64))
    scene.ngeom += 1
    return g


def _frame_from_shaft(s):
    """Orthonormal rotation matrix (columns) whose local z-axis is the shaft s."""
    s = s / np.linalg.norm(s)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(s @ ref) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    u = ref - (ref @ s) * s
    u /= np.linalg.norm(u)
    v = np.cross(s, u)
    return np.column_stack([u, v, s])


def _overlay_from_snap(scene, env, s, trail=None):
    draw_overlay(scene, env, s["ee_pos"], s["p_tip"], s["s_hat"],
                 s["F_tis"], s["e_rcm"], s["t"], s["pushing"], trail)


def draw_overlay(scene, env, flange, p_tip, s_hat, F_tis, e_rcm, t, pushing,
                 trail=None):
    """Append the analytical laparoscopy elements as MuJoCo viz geoms."""
    Rg = _frame_from_shaft(s_hat).reshape(9)

    # 1) instrument shaft: flange -> tip (metallic grey capsule)
    _add_connector(scene, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.006,
                   flange, p_tip, [0.75, 0.78, 0.82, 1.0])

    # 2) trocar / RCM port: dark ring (flat cylinder) perpendicular to shaft
    _add_geom(scene, mujoco.mjtGeom.mjGEOM_CYLINDER,
              [0.018, 0.018, 0.006], env.p_rcm0, Rg, [0.15, 0.15, 0.18, 1.0])
    #    RCM target marker (small green sphere at the nominal pivot)
    _add_geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE,
              [0.006, 0, 0], env.p_rcm0, np.eye(3).reshape(9),
              [0.1, 0.9, 0.2, 1.0])
    #    actual lateral RCM deviation: yellow connector pivot -> shaft axis pt
    if np.linalg.norm(e_rcm) > 1e-4:
        _add_connector(scene, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.0025,
                       env.p_rcm0, env.p_rcm0 - e_rcm, [1.0, 0.85, 0.0, 1.0])

    # 3) tissue surface: translucent red plate at the (respiring) surface point;
    #    kept lightly transparent so the cyan tip stays visible when embedded.
    surf = env.surface_point(t)
    _add_geom(scene, mujoco.mjtGeom.mjGEOM_BOX,
              [0.06, 0.06, 0.004], surf, Rg, [0.80, 0.25, 0.25, 0.22])

    # 4a) tip trail: fading dots of recent tip positions, so motion is visible
    if trail is not None and len(trail) > 1:
        pts = trail[-40:]
        npts = len(pts)
        for i, p in enumerate(pts[:-1]):
            frac = i / max(npts - 1, 1)                 # 0 (old) -> 1 (recent)
            _add_geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE,
                      [0.0022 + 0.0016 * frac, 0, 0], p, np.eye(3).reshape(9),
                      [0.2, 0.95, 1.0, 0.25 + 0.5 * frac])

    # 4b) tool tip: solid high-contrast cyan sphere, always visible against the
    #     red tissue plate; contact force is shown by the arrow in (5).
    fmag = float(np.linalg.norm(F_tis))
    _add_geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.010, 0, 0], p_tip,
              np.eye(3).reshape(9), [0.1, 0.95, 1.0, 1.0])

    # 5) contact-force arrow (reaction force, red), started just outside the
    #    tip sphere so it never hides the cyan marker.
    if fmag > 1e-3:
        fdir = F_tis / max(fmag, 1e-9)
        base = p_tip + fdir * 0.012
        tipend = base + fdir * (0.05 * min(fmag / F_REF, 1.5))
        _add_connector(scene, mujoco.mjtGeom.mjGEOM_ARROW, 0.005,
                       base, tipend, [0.95, 0.1, 0.1, 1.0])

    # 6) lateral RCM-stress load arrow (orange) during the push window
    if pushing:
        u_lat = lateral_axis(env.s0)
        _add_connector(scene, mujoco.mjtGeom.mjGEOM_ARROW, 0.006,
                       p_tip, p_tip + 0.06 * u_lat, [1.0, 0.55, 0.0, 1.0])


# --------------------------------------------------------------------------
# HUD text drawn onto the rendered RGB frame
# --------------------------------------------------------------------------
def _font(size):
    for path in ("/System/Library/Fonts/Supplemental/Arial.ttf",
                 "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _draw_stage_strip(d, t):
    """Segmented task timeline with the active stage highlighted + a caption."""
    idx, name, desc, col = stage_info(t)
    x0, x1, y0, hh = 18, W - 18, 100, 26
    span = x1 - x0
    small = _font(15)
    for i, (a, b, nm, _desc, c) in enumerate(STAGES):
        lx = x0 + int(span * a / T_END)
        rx = x0 + int(span * b / T_END)
        active = (i == idx)
        fill = c + (235,) if active else (70, 78, 92, 200)
        d.rectangle([lx + 1, y0, rx - 1, y0 + hh], fill=fill)
        tcol = (15, 18, 24, 255) if active else (200, 206, 216, 255)
        bb = d.textbbox((0, 0), nm, font=small)
        tw = bb[2] - bb[0]
        if rx - lx > tw + 6:
            d.text((lx + (rx - lx - tw) / 2, y0 + 4), nm, font=small, fill=tcol)
    # progress cursor
    cx = x0 + int(span * min(t, T_END) / T_END)
    d.line([cx, y0 - 3, cx, y0 + hh + 3], fill=(255, 255, 255, 255), width=2)
    # caption: what the sim is doing now
    cap = _font(20)
    d.text((x0, y0 + hh + 7), f"▶  {name}: {desc}", font=cap,
           fill=col + (255,))


# legend: (swatch colour, shape, label) for the overlay markers
LEGEND = [
    ((26, 230, 51),  "circle", "RCM pivot (trocar, fixed)"),
    ((26, 242, 255), "circle", "Tool tip"),
    ((204, 64, 64),  "square", "Tissue surface (respiring)"),
    ((242, 26, 26),  "arrow",  "Contact force"),
    ((255, 140, 0),  "arrow",  "RCM-stress load"),
]


def _draw_legend(d):
    f = _font(16); hf = _font(16)
    x, y = 14, 188
    rows = len(LEGEND)
    d.rectangle([0, y - 26, 270, y + 26 * rows + 6], fill=(15, 18, 24, 165))
    d.text((x, y - 24), "Legend", font=hf, fill=(200, 206, 216, 255))
    for (col, shape, label) in LEGEND:
        cy = y + 11
        if shape == "circle":
            d.ellipse([x, cy - 7, x + 14, cy + 7], fill=col + (255,))
        elif shape == "square":
            d.rectangle([x, cy - 6, x + 14, cy + 6], fill=col + (200,))
        else:  # arrow
            d.line([x, cy, x + 14, cy], fill=col + (255,), width=3)
            d.polygon([(x + 14, cy - 4), (x + 20, cy), (x + 14, cy + 4)],
                      fill=col + (255,))
        d.text((x + 28, y), label, font=f, fill=(230, 235, 244, 255))
        y += 26


def add_hud(rgb, mode, t, tip_err_mm, fmag, rcm_mm, depth_mm, pushing):
    img = Image.fromarray(rgb)
    d = ImageDraw.Draw(img, "RGBA")
    title_f = _font(26)
    f = _font(22)
    d.rectangle([0, 0, W, 92], fill=(15, 18, 24, 175))
    d.text((18, 10), "Laparoscopic palpation  —  FR3 MuJoCo simulation",
           font=title_f, fill=(255, 255, 255, 255))
    d.text((18, 50), MODE_LABEL.get(mode, mode), font=f, fill=(120, 220, 255, 255))
    d.text((W - 200, 14), f"t = {t:5.2f} s", font=title_f, fill=(255, 255, 255, 255))

    _draw_stage_strip(d, t)
    _draw_legend(d)

    lines = [
        (f"tip error      {tip_err_mm:6.2f} mm", (255, 255, 255)),
        (f"insertion      {depth_mm:6.2f} mm", (255, 255, 255)),
        (f"tissue force   {fmag:6.2f} N",  (255, 140, 140)),
        (f"RCM deviation  {rcm_mm:6.2f} mm", (255, 220, 110)),
    ]
    # dynamic readout: top-right, just below the stage strip ("Retraction")
    # and just left of the curve panel.
    box_w = 272
    bx = W - box_w
    y0 = 134
    d.rectangle([bx, y0, W, y0 + 24 * len(lines) + 14], fill=(15, 18, 24, 160))
    y = y0 + 10
    for txt, col in lines:
        d.text((bx + 14, y), txt, font=f, fill=col + (255,))
        y += 24
    return np.asarray(img)


# --------------------------------------------------------------------------
# live time-series panel (the benchmark curves, drawn up to the current time)
# --------------------------------------------------------------------------
class LivePlot:
    """Dark-themed 4-row time-series panel rendered to an RGB array per frame."""

    def __init__(self, log, px_w=PANEL_W, px_h=H):
        self.log = log
        dpi = 100
        self.fig, axes = plt.subplots(
            4, 1, figsize=(px_w / dpi, px_h / dpi), dpi=dpi, sharex=True)
        self.fig.patch.set_facecolor("#0f1218")
        self.fig.subplots_adjust(left=0.16, right=0.97, top=0.965,
                                 bottom=0.07, hspace=0.32)
        t = log["t"]
        specs = [
            ("tip_err", "tip error (mm)", "#7ee0ff", None),
            ("F_tissue", "tissue force (N)", "#ff8c8c", F_MAX),
            ("depth", "insertion (mm)", "#9cff9c", None),
            ("rcm", "RCM dev. (mm)", "#ffd86e", None),
        ]
        self.lines, self.markers = [], []
        self.axes = axes
        for ax, (key, ylabel, hline) in zip(axes, [(s[0], s[1], s[3]) for s in specs]):
            ax.set_facecolor("#161a22")
            for sp in ax.spines.values():
                sp.set_color("#444b57")
            ax.tick_params(colors="#aab2c0", labelsize=8)
            ax.set_ylabel(ylabel, color="#dfe5ee", fontsize=9)
            ax.grid(alpha=0.18, color="#5a6273")
            ax.axvspan(*PUSH_WIN, color="#ff8c1a", alpha=0.10)
            ax.set_xlim(0, T_END)
            y = log[key]
            pad = 0.08 * (y.max() - y.min() + 1e-6)
            lo = min(0.0, y.min()) - pad
            hi = max(y.max(), hline or -1e9) + pad
            ax.set_ylim(lo, hi)
            if key == "depth":   # show the commanded reference too
                ax.plot(t, log["ref_depth"], ":", color="#aaaaaa", lw=1.1)
            if hline is not None:
                ax.axhline(hline, ls="--", color="#ff5555", lw=1.1)
                ax.text(T_END * 0.985, hline, f" {hline:.0f} N cap",
                        color="#ff7777", fontsize=7.5, va="bottom", ha="right")
            (ln,) = ax.plot([], [], color="#7ee0ff", lw=1.6)
            self.lines.append(ln)
            self.markers.append(ax.axvline(0.0, color="#ffffff", lw=0.9, alpha=0.6))
        for ln, (_, _, c, _) in zip(self.lines, specs):
            ln.set_color(c)
        axes[0].set_title("live response", color="#dfe5ee", fontsize=10)
        axes[-1].set_xlabel("time (s)", color="#dfe5ee", fontsize=9)
        self.keys = [s[0] for s in specs]

    def frame(self, i):
        t = self.log["t"][:i + 1]
        for ln, key in zip(self.lines, self.keys):
            ln.set_data(t, self.log[key][:i + 1])
        for mk in self.markers:
            mk.set_xdata([self.log["t"][i], self.log["t"][i]])
        self.fig.canvas.draw()
        buf = np.asarray(self.fig.canvas.buffer_rgba())[..., :3]
        return buf.copy()

    def close(self):
        plt.close(self.fig)


# --------------------------------------------------------------------------
def make_video(mode="mpc_kalman", out=None, fps=FPS):
    env = LapEnv()
    params = LapCtrlParams()
    ctrl = LapController(mode, env, params)
    dt = env.dt
    u_lat = lateral_axis(env.s0)
    R_d = env.R0
    n_steps = int(T_END / dt)

    model, data = env.env.model, env.env.data

    # ---- phase 1: roll the MuJoCo sim out, logging curves + render snapshots
    log = {k: [] for k in ("t", "tip_err", "F_tissue", "depth", "ref_depth",
                           "rcm")}
    snaps = []                                    # per-rendered-frame state
    for k in range(n_steps):
        t = k * dt
        dyn, st = env.get_dynamics_and_state()
        p_d, dp_d, ddp_d = references(env, t, dt)
        resolve = (k % DT_DECIM == 0)
        tau, info = ctrl.control(dyn, st, p_d, dp_d, ddp_d, R_d,
                                 env.p_rcm0, resolve=resolve)
        env.apply_torque(tau)

        p_tip, s_hat, v_tip, _ = env.tip_state(dyn, st)
        F_tis, delta = env.tissue_force(p_tip, v_tip, t)
        env.apply_point_force(st, F_tis, p_tip)
        F_tro, e_rcm = env.trocar_force(st, env.p_rcm0)
        env.apply_point_force(st, F_tro, env.p_rcm0)
        pushing = PUSH_WIN[0] <= t < PUSH_WIN[1]
        if pushing:
            env.apply_point_force(st, PUSH_MAG * u_lat, p_tip)

        log["t"].append(t)
        log["tip_err"].append(float(np.linalg.norm(p_d - p_tip)) * 1e3)
        log["F_tissue"].append(float(np.linalg.norm(F_tis)))
        log["depth"].append(float((p_tip - env.p_tip0) @ env.s0) * 1e3)
        log["ref_depth"].append(s_ref(t) * 1e3)
        log["rcm"].append(float(np.linalg.norm(e_rcm)) * 1e3)

        if k % RENDER_EVERY == 0:
            snaps.append(dict(qpos=data.qpos.copy(), ee_pos=st.ee_pos.copy(),
                              p_tip=p_tip.copy(), s_hat=s_hat.copy(),
                              F_tis=F_tis.copy(), e_rcm=e_rcm.copy(),
                              t=t, pushing=pushing, idx=k))
        env.step()

    log = {k: np.asarray(v) for k, v in log.items()}

    # ---- phase 2: render the 3D MuJoCo scene + the synced curve panel
    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = mujoco.MjvCamera()
    # frame the whole arm (base at origin up to the trocar/tip workspace)
    cam.lookat[:] = [0.22, 0.0, 0.34]
    cam.distance = 1.05
    cam.azimuth = 135.0
    cam.elevation = -14.0
    opt = mujoco.MjvOption()
    panel = LivePlot(log)

    out = out or str(Path(__file__).parent / f"lap_video_{mode}.mp4")
    writer = imageio.get_writer(out, fps=fps, codec="libx264",
                                quality=8, macro_block_size=8)

    tip_hist = []
    for s in snaps:
        tip_hist.append(s["p_tip"])
        data.qpos[:] = s["qpos"]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam, scene_option=opt)
        _overlay_from_snap(renderer.scene, env, s, tip_hist)
        rgb = renderer.render()
        rgb = add_hud(rgb, mode, s["t"],
                      float(np.linalg.norm(s["p_tip"] - references(env, s["t"], dt)[0])) * 1e3,
                      float(np.linalg.norm(s["F_tis"])),
                      float(np.linalg.norm(s["e_rcm"])) * 1e3,
                      float((s["p_tip"] - env.p_tip0) @ env.s0) * 1e3,
                      s["pushing"])
        curves = panel.frame(s["idx"])
        frame = np.hstack([rgb, curves])
        writer.append_data(frame)

    writer.close()
    renderer.close()
    panel.close()
    print(f"[video] {len(snaps)} frames @ {fps} fps -> {out}  ({W + PANEL_W}x{H})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="mpc_kalman",
                    choices=["impedance", "mpc", "mpc_kalman"])
    ap.add_argument("--out", default=None)
    ap.add_argument("--fps", type=int, default=FPS)
    a = ap.parse_args()
    make_video(a.mode, a.out, a.fps)


if __name__ == "__main__":
    main()
