#!/usr/bin/env python3
"""
rosbag_gui.py — Interfaz grafica para rosbag_analyzer.py

    source /opt/ros/jazzy/setup.bash
    source ~/ws/install/setup.bash
    python3 rosbag_gui.py [/ruta/al/bag | /ruta/al/fichero.mcap]

Requiere bag_analayzer.py en la misma carpeta, y tkinter
(sudo apt install python3-tk).

  * Ventana principal: lista de topics (con filtro y ordenacion), informe de
    texto y resumen general.
  * Cada grafica se abre en su propia ventana con barra de zoom/pan/guardar
    (y rotacion con el raton en las figuras 3D).
  * Seccion "Solo IMU" y "Solo GPS": dashboard, selector de canales y 3D.
  * "Solo IMU + GPS" al cargar: lee unicamente esos topics (carga rapida).
  * Ventana temporal (inicio/fin en segundos) aplicada a todas las graficas.
  * Pestana "Regrabar con nodo": reproduce un bag, ejecuta el nodo elegido
    y graba un bag nuevo con los topics que selecciones.
  * Tema claro/oscuro (Ctrl+D), atajos de teclado y menu contextual.
  * Regrabacion: crea la carpeta de salida si no existe, barra de progreso con
    porcentaje y ETA, resumen del bag nuevo, ajustes recordados entre sesiones.

Atajos:  Ctrl+O abrir carpeta · Ctrl+Shift+O abrir fichero · Ctrl+F filtrar
         Ctrl+D tema · Enter graficar seleccion · Clic derecho: menu
         F5 regrabar (reproducir + nodo + grabar) · F6 detener
"""

from __future__ import annotations

import contextlib
import copy
import io
import json
import math
import os
import queue
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    from tkinter import font as tkfont
except ImportError:
    raise SystemExit("\nFalta tkinter:  sudo apt install python3-tk\n")

import bag_analayzer as A

A.setup_style(True)          # backend Agg: las figuras se incrustan a mano en Tk
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_tkagg import (  # noqa: E402
    FigureCanvasTkAgg, NavigationToolbar2Tk)

# --------------------------------------------------------------------------- #
#  Tema (claro / oscuro)
# --------------------------------------------------------------------------- #

PALETTES = {
    "light": dict(bg="#f3f4f7", card="#ffffff", fg="#1f2430", muted="#6b7280",
                  border="#d9dce3", accent="#2563eb", accent_fg="#ffffff",
                  accent_hover="#1d4ed8", sel="#dbe7ff", stripe="#f7f8fa",
                  danger="#dc2626", danger_hover="#b91c1c", field="#ffffff"),
    "dark": dict(bg="#14171c", card="#1c2027", fg="#e6e8ec", muted="#8b93a1",
                 border="#2e3440", accent="#3b82f6", accent_fg="#ffffff",
                 accent_hover="#2563eb", sel="#23406b", stripe="#20242c",
                 danger="#ef4444", danger_hover="#dc2626", field="#232832"),
}
PAL: Dict[str, str] = dict(PALETTES["light"])


def _pick_font(root, prefs, default):
    have = set(tkfont.families(root))
    return next((f for f in prefs if f in have), default)


def apply_theme(root, mode: str) -> Dict[str, str]:
    """Aplica un tema plano y moderno a ttk y a los widgets clasicos de Tk."""
    p = PALETTES[mode]
    PAL.clear()
    PAL.update(p)
    ui = _pick_font(root, ("Inter", "Segoe UI", "SF Pro Text", "Noto Sans",
                           "Ubuntu", "Cantarell", "DejaVu Sans"), "TkDefaultFont")
    mono = _pick_font(root, ("JetBrains Mono", "Cascadia Mono", "Fira Code",
                             "Ubuntu Mono", "DejaVu Sans Mono", "Consolas"),
                      "TkFixedFont")
    PAL["ui"], PAL["mono"] = ui, mono
    for name, fam, size in (("TkDefaultFont", ui, 10), ("TkTextFont", ui, 10),
                            ("TkMenuFont", ui, 10), ("TkHeadingFont", ui, 10),
                            ("TkFixedFont", mono, 10)):
        try:
            tkfont.nametofont(name).configure(family=fam, size=size)
        except tk.TclError:
            pass

    root.configure(bg=p["bg"])
    # widgets clasicos (Toplevel, Listbox, Menu, Text) heredan estos valores
    root.option_add("*Toplevel.background", p["bg"])
    root.option_add("*Listbox.background", p["card"])
    root.option_add("*Listbox.foreground", p["fg"])
    root.option_add("*Listbox.selectBackground", p["sel"])
    root.option_add("*Listbox.selectForeground", p["fg"])
    root.option_add("*Menu.background", p["card"])
    root.option_add("*Menu.foreground", p["fg"])
    root.option_add("*Menu.activeBackground", p["accent"])
    root.option_add("*Menu.activeForeground", p["accent_fg"])
    root.option_add("*Menu.borderWidth", 0)
    root.option_add("*Menu.relief", "flat")

    st = ttk.Style(root)
    st.theme_use("clam")
    st.configure(".", background=p["bg"], foreground=p["fg"],
                 fieldbackground=p["field"], bordercolor=p["border"],
                 lightcolor=p["bg"], darkcolor=p["bg"], troughcolor=p["border"],
                 focuscolor=p["bg"], insertcolor=p["fg"], font=(ui, 10))

    st.configure("TLabel", background=p["bg"], foreground=p["fg"])
    st.configure("Muted.TLabel", foreground=p["muted"])
    st.configure("Title.TLabel", font=(ui, 11, "bold"))
    st.configure("Status.TLabel", background=p["card"], foreground=p["muted"],
                 padding=(12, 5))

    st.configure("TLabelframe", background=p["bg"], bordercolor=p["border"],
                 relief="solid", borderwidth=1)
    st.configure("TLabelframe.Label", background=p["bg"],
                 foreground=p["muted"], font=(ui, 9, "bold"))

    for sty, bg, hov, fg in (("TButton", p["card"], p["sel"], p["fg"]),
                             ("Accent.TButton", p["accent"], p["accent_hover"],
                              p["accent_fg"]),
                             ("Danger.TButton", p["danger"], p["danger_hover"],
                              p["accent_fg"])):
        border = p["border"] if sty == "TButton" else bg
        st.configure(sty, padding=(12, 6), relief="flat", borderwidth=1,
                     background=bg, foreground=fg, bordercolor=border,
                     lightcolor=bg, darkcolor=bg, focusthickness=0,
                     focuscolor=bg)
        st.map(sty,
               background=[("disabled", p["bg"]), ("pressed", hov),
                           ("active", hov)],
               foreground=[("disabled", p["muted"])],
               bordercolor=[("disabled", p["border"])],
               lightcolor=[("disabled", p["bg"]), ("active", hov)],
               darkcolor=[("disabled", p["bg"]), ("active", hov)])

    st.configure("TEntry", padding=(6, 5), fieldbackground=p["field"],
                 foreground=p["fg"], bordercolor=p["border"],
                 lightcolor=p["border"], darkcolor=p["border"])
    st.map("TEntry", bordercolor=[("focus", p["accent"])],
           lightcolor=[("focus", p["accent"])],
           darkcolor=[("focus", p["accent"])])

    st.configure("TCheckbutton", background=p["bg"], foreground=p["fg"],
                 indicatorbackground=p["field"], indicatorforeground=p["accent_fg"])
    st.map("TCheckbutton", background=[("active", p["bg"])],
           indicatorbackground=[("selected", p["accent"]),
                                ("pressed", p["accent"])])

    st.configure("Treeview", background=p["card"], fieldbackground=p["card"],
                 foreground=p["fg"], rowheight=27, borderwidth=0,
                 lightcolor=p["card"], darkcolor=p["card"])
    st.map("Treeview", background=[("selected", p["sel"])],
           foreground=[("selected", p["fg"])])
    st.configure("Treeview.Heading", background=p["bg"], foreground=p["muted"],
                 relief="flat", padding=(8, 6), font=(ui, 9, "bold"))
    st.map("Treeview.Heading", background=[("active", p["card"])])

    st.configure("TNotebook", background=p["bg"], borderwidth=0,
                 tabmargins=(0, 2, 0, 0))
    st.configure("TNotebook.Tab", padding=(16, 8), background=p["bg"],
                 foreground=p["muted"], borderwidth=0)
    st.map("TNotebook.Tab",
           background=[("selected", p["card"]), ("active", p["stripe"])],
           foreground=[("selected", p["accent"]), ("active", p["fg"])])

    st.configure("TScrollbar", background=p["border"], troughcolor=p["bg"],
                 bordercolor=p["bg"], lightcolor=p["border"],
                 darkcolor=p["border"], arrowcolor=p["muted"], relief="flat",
                 gripcount=0)
    st.map("TScrollbar", background=[("active", p["muted"])])

    st.configure("Horizontal.TProgressbar", background=p["accent"],
                 troughcolor=p["border"], bordercolor=p["bg"],
                 lightcolor=p["accent"], darkcolor=p["accent"], thickness=5)
    st.configure("Big.Horizontal.TProgressbar", background=p["accent"],
                 troughcolor=p["border"], bordercolor=p["bg"],
                 lightcolor=p["accent"], darkcolor=p["accent"], thickness=16)
    st.configure("TPanedwindow", background=p["bg"])
    st.configure("TSeparator", background=p["border"])
    return PAL


def style_text(txt: tk.Text) -> None:
    txt.configure(bg=PAL["card"], fg=PAL["fg"], insertbackground=PAL["fg"],
                  selectbackground=PAL["sel"], selectforeground=PAL["fg"],
                  relief="flat", borderwidth=0, highlightthickness=0,
                  padx=14, pady=10, spacing1=1, spacing3=1)


SETTINGS_PATH = os.path.join(os.path.expanduser("~"), ".config", "rosbag_gui",
                             "settings.json")


def load_settings() -> Dict:
    try:
        with open(SETTINGS_PATH, encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def save_settings(d: Dict) -> None:
    try:
        os.makedirs(os.path.dirname(SETTINGS_PATH), exist_ok=True)
        with open(SETTINGS_PATH, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=2, ensure_ascii=False)
    except OSError:
        pass


def center_on(win, master) -> None:
    win.update_idletasks()
    x = master.winfo_rootx() + (master.winfo_width() - win.winfo_reqwidth()) // 2
    y = master.winfo_rooty() + (master.winfo_height() - win.winfo_reqheight()) // 3
    win.geometry(f"+{max(x, 0)}+{max(y, 0)}")


# --------------------------------------------------------------------------- #
#  Figuras "solo canales elegidos" (IMU y GPS)
# --------------------------------------------------------------------------- #

IMU_GROUPS = [
    ("Aceleración", [("acc_x", "x", True), ("acc_y", "y", True),
                     ("acc_z", "z", True), ("acc_norm", "|a|", False)]),
    ("Velocidad angular", [("gyr_x", "x", True), ("gyr_y", "y", True),
                           ("gyr_z", "z", True), ("gyr_norm", "|ω|", False)]),
    ("Orientación", [("rpy_roll", "roll", False), ("rpy_pitch", "pitch", False),
                     ("rpy_yaw", "yaw", False)]),
    ("Espectros", [("psd_acc", "PSD aceleración", False),
                   ("psd_gyr", "PSD giroscopio", False)]),
    ("Opciones", [("demean", "restar la media (ver ruido)", False)]),
]

GPS_GROUPS = [
    ("Paneles", [("track", "Traza E–N", True), ("alt", "Altitud", True),
                 ("radial", "Distancia radial", False),
                 ("speed", "Velocidad (derivada)", False)]),
    ("Datos crudos", [("lat", "Latitud", False), ("lon", "Longitud", False),
                      ("status", "Estado del fix", False),
                      ("sigma", "σ de la covarianza", False)]),
]


def fig_imu_channels(c, b: Dict, t0: float, keys: List[str]):
    from scipy.signal import welch
    n = min(c.t.size, c.accel.shape[0])
    if n < 3:
        return None
    t = c.t[:n] - t0
    a, g = c.accel[:n], np.degrees(c.gyro[:n])
    dm = "demean" in keys
    mean_lab = " − media" if dm else ""

    def prep(y):
        return y - np.nanmean(y) if dm else y

    panels: List[Tuple[str, str, list]] = []
    s = [(k, prep(a[:, i]), A.AXIS_COLOR[k]) for i, k in enumerate("xyz")
         if f"acc_{k}" in keys]
    if "acc_norm" in keys:
        s.append(("|a|", prep(np.linalg.norm(a, axis=1)), A.INK_SOFT))
    if s:
        panels.append(("Aceleración lineal", "m/s²" + mean_lab, s))
    s = [(k, prep(g[:, i]), A.AXIS_COLOR[k]) for i, k in enumerate("xyz")
         if f"gyr_{k}" in keys]
    if "gyr_norm" in keys:
        s.append(("|ω|", prep(np.linalg.norm(g, axis=1)), A.INK_SOFT))
    if s:
        panels.append(("Velocidad angular", "°/s" + mean_lab, s))
    if any(k.startswith("rpy_") for k in keys) and np.any(np.abs(c.quat[:n])):
        r = np.degrees(A.rpy_from_quat(c.quat[:n]))
        r[:, 2] = np.degrees(np.unwrap(np.radians(r[:, 2])))
        s = [(lab, prep(r[:, i]), col) for i, (lab, col) in enumerate(
            (("roll", A.BLUE), ("pitch", A.ORANGE), ("yaw", A.AQUA)))
            if f"rpy_{lab}" in keys]
        if s:
            panels.append(("Orientación", "grados" + mean_lab, s))

    fs = b["hz"] if math.isfinite(b["hz"]) and b["hz"] > 0 else 100.0
    psd = []
    if n >= 64:
        for key, arr, spec, ttl, unit, pref in (
                ("psd_acc", a, A.SPEC_ACCEL_DENSITY, "Espectro de aceleración",
                 "m/s² / √Hz", "acc"),
                ("psd_gyr", g, A.SPEC_GYRO_DENSITY, "Espectro de velocidad angular",
                 "°/s / √Hz", "gyr")):
            if key in keys:
                axes = [k for k in "xyz" if f"{pref}_{k}" in keys] or list("xyz")
                psd.append((arr, spec, ttl, unit, axes))

    rows = len(panels) + (1 if psd else 0)
    if rows == 0:
        return None
    fig = plt.figure(figsize=(13.5, max(3.2, 2.6 * len(panels) + (3.4 if psd else 0))),
                     layout="constrained")
    ratios = [1.0] * len(panels) + ([1.3] if psd else [])
    gs = fig.add_gridspec(rows, 2, height_ratios=ratios)
    first = None
    for i, (title, ylab, series) in enumerate(panels):
        ax = fig.add_subplot(gs[i, :], sharex=first)
        if first is None:
            first = ax
        for lab, y, col in series:
            A.line(ax, t, y, col, lab)
        A.finish(ax, ylab, title)
        A.stats_text(ax, "σ  " + "  ".join(f"{lab} {np.nanstd(y):.4g}"
                                            for lab, y, _ in series))
    if first is not None:
        fig.axes[len(panels) - 1].set_xlabel("tiempo [s]")
    for j, (arr, spec, ttl, unit, axes) in enumerate(psd):
        ax = fig.add_subplot(gs[len(panels), :] if len(psd) == 1
                             else gs[len(panels), j])
        nper = int(min(n, max(256, fs * 8)))
        for k in axes:
            i = "xyz".index(k)
            f, p = welch(arr[:, i] - arr[:, i].mean(), fs=fs, nperseg=nper)
            ax.loglog(f[1:], np.sqrt(p[1:]), color=A.AXIS_COLOR[k], lw=1.1, label=k)
        ax.axhline(spec, color=A.INK_MUTED, lw=1.0, ls="--", label="spec")
        ax.set_xlabel("frecuencia [Hz]")
        ax.grid(True, which="both", alpha=0.9)
        A.finish(ax, unit, ttl)
    fig.suptitle(f"{c.topic} — canales elegidos   ({n} muestras, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def fig_gps_channels(c, b: Dict, t0: float, keys: List[str]):
    n = min(c.t.size, c.lat.size)
    ok = (np.isfinite(c.lat[:n]) & np.isfinite(c.lon[:n])
          & (np.abs(c.lat[:n]) > 1e-9))
    if ok.sum() < 3:
        return None
    t = (c.t[:n] - t0)[ok]
    lat, lon, alt = c.lat[:n][ok], c.lon[:n][ok], c.alt[:n][ok]
    stat, cov = c.status[:n][ok], c.cov[:n][ok]
    north, east, _, _ = A.lla_to_ne(lat, lon)
    r = np.hypot(north, east)

    # (titulo, unidad, [(etiqueta, y, color)], eje t propio o None, escalon?)
    panels: list = []
    if "alt" in keys:
        panels.append(("Altitud", "m", [("altitud", alt, A.BLUE)], None, False))
    if "lat" in keys:
        panels.append(("Latitud", "°", [("lat", lat, A.BLUE)], None, False))
    if "lon" in keys:
        panels.append(("Longitud", "°", [("lon", lon, A.ORANGE)], None, False))
    if "radial" in keys:
        static = r.max() < 10.0
        panels.append(("Dispersión radial" if static else "Distancia al centroide",
                       "m", [("radio", r, A.ORANGE)], None, False))
    if "speed" in keys:
        dt = np.diff(t)
        d = np.hypot(np.diff(north), np.diff(east))
        sp = np.where(dt > 0, d / np.where(dt > 0, dt, 1.0), np.nan)
        panels.append(("Velocidad (derivada de las posiciones)", "m/s",
                       [("v", sp, A.AQUA)], t[1:], False))
    if "status" in keys:
        panels.append(("Estado del fix", "status",
                       [("status", stat.astype(float), A.AQUA)], None, True))
    if "sigma" in keys:
        sg = np.sqrt(np.clip(cov, 0, None))
        panels.append(("σ de posición (covarianza)", "m",
                       [("este", sg[:, 0], A.BLUE), ("norte", sg[:, 1], A.ORANGE),
                        ("arriba", sg[:, 2], A.AQUA)], None, False))

    has_track = "track" in keys
    rows = max(len(panels), 1)
    if not panels and not has_track:
        return None
    fig = plt.figure(figsize=(13.5 if has_track else 11.0, max(4.6, 2.2 * rows + 1)),
                     layout="constrained")
    ncol = 2 if has_track else 1
    gs = fig.add_gridspec(rows, ncol, width_ratios=[1, 1.3] if has_track else [1])
    col0 = 1 if has_track else 0
    if has_track:
        at = fig.add_subplot(gs[:, 0])
        sc = at.scatter(east, north, c=t, cmap=A.blue_cmap(), s=4, linewidths=0)
        at.plot(east[0], north[0], "o", ms=8, mfc="none", mec=A.INK, mew=1.4)
        at.set_aspect("equal", adjustable="datalim")
        at.set_xlabel("este [m]")
        at.set_ylabel("norte [m]")
        at.set_title("Traza relativa al centroide", loc="left", pad=6)
        cb = fig.colorbar(sc, ax=at, pad=0.02, fraction=0.045)
        cb.set_label("tiempo [s]", color=A.INK_SOFT, fontsize=8.5)
        cb.outline.set_visible(False)
    first = None
    for i, (title, yl, series, tv, step) in enumerate(panels):
        ax = fig.add_subplot(gs[i, col0], sharex=first)
        if first is None:
            first = ax
        tt = t if tv is None else tv
        for lab, y, colr in series:
            if step:
                ax.step(tt, y, where="post", color=colr, lw=1.2, label=lab)
            else:
                A.line(ax, tt, y, colr, lab)
        A.finish(ax, yl, title, legend=len(series) > 1)
        yy = series[0][1]
        yy = yy[np.isfinite(yy)]
        if yy.size:
            A.stats_text(ax, f"media {yy.mean():.6g}   σ {yy.std():.4g}")
    if first is not None:
        fig.axes[-1].set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic} — canales elegidos   ({int(ok.sum())} fixes, "
                 f"{b['hz']:.1f} Hz)", x=0.01, ha="left")
    return fig


def align_time(c) -> None:
    """Si el tope --sample corto la deserializacion, alinea t con los datos."""
    name = {"imu": "accel", "odom": "pos", "fix": "lat"}.get(c.kind)
    if name is None:
        return
    m = len(getattr(c, name))
    if c.t.size != m:
        use_hdr = c.t_hdr.size >= m and m > 0 and np.any(c.t_hdr[:m] > 0)
        c.t = c.t_hdr[:m] if use_hdr else c.t_bag[:m]


def has_data(c) -> bool:
    k = c.kind
    if k == "imu":
        return c.accel.size > 0
    if k in ("odom", "car", "pose"):
        return c.pos.size > 0
    if k == "twist":
        return c.vel.size > 0
    if k == "fix":
        return c.lat.size > 0
    if k == "cloud":
        return c.t_bag.size > 0
    if k == "scan":
        return c.t_bag.size > 0
    if k == "image":
        return c.t_bag.size > 0
    if k == "range":
        return c.range.size > 0
    if k == "battery":
        return c.data.size > 0
    if k == "joint":
        return bool(c.names)
    if k == "diag":
        return bool(c.rows)
    if k == "clock":
        return c.clock.size > 0
    if k == "scalar":
        return c.value.size > 0
    if k in ("posearray", "path"):
        v = getattr(c, "counts", None)
        return v is not None and v.size > 0
    if k == "tf":
        return bool(c.tf)
    if k == "log":
        return bool(c.rows)
    return bool(getattr(c, "data", None))


# --------------------------------------------------------------------------- #
#  Dialogos y ventana de grafica
# --------------------------------------------------------------------------- #

class PlotWindow(tk.Toplevel):
    """Una figura de matplotlib en su propia ventana, con barra de herramientas."""

    _count = 0

    def __init__(self, master, fig, title: str):
        super().__init__(master)
        self.title(title)
        self.fig = fig
        wi, hi = fig.get_size_inches()
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w = min(1300, sw - 80)
        h = min(int(w * hi / wi) + 50, sh - 120)
        off = 28 * (PlotWindow._count % 8)       # cascada: no se tapan entre si
        PlotWindow._count += 1
        self.geometry(f"{w}x{max(h, 380)}+{50 + off}+{30 + off}")
        canvas = FigureCanvasTkAgg(fig, master=self)
        bar = NavigationToolbar2Tk(canvas, self, pack_toolbar=False)
        bar.update()
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        canvas.draw_idle()
        self.canvas = canvas
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.bind("<Escape>", lambda _e: self._close())
        self.bind("<Control-w>", lambda _e: self._close())
        self.focus_set()

    def _close(self) -> None:
        plt.close(self.fig)
        self.destroy()


class ListDialog(tk.Toplevel):
    def __init__(self, master, title: str, items: List[str], multi: bool = False):
        super().__init__(master)
        self.title(title)
        self.result: List[str] = []
        self.transient(master)
        body = ttk.Frame(self, padding=14)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(body, text=title, style="Title.TLabel").pack(anchor="w", pady=(0, 8))
        self.lb = tk.Listbox(body, selectmode=tk.EXTENDED if multi else tk.SINGLE,
                             width=64, height=min(12, max(3, len(items))),
                             exportselection=False, activestyle="none",
                             relief="flat", borderwidth=0, highlightthickness=1,
                             highlightbackground=PAL["border"],
                             highlightcolor=PAL["accent"],
                             bg=PAL["card"], fg=PAL["fg"],
                             selectbackground=PAL["sel"], selectforeground=PAL["fg"])
        for it in items:
            self.lb.insert(tk.END, it)
        self.lb.selection_set(0)
        self.lb.pack(fill=tk.BOTH, expand=True)
        self.lb.bind("<Double-Button-1>", lambda _e: self._ok())
        self.lb.focus_set()
        row = ttk.Frame(body)
        row.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(row, text="Aceptar", style="Accent.TButton",
                   command=self._ok).pack(side=tk.RIGHT)
        ttk.Button(row, text="Cancelar", command=self.destroy).pack(
            side=tk.RIGHT, padx=(0, 6))
        self.items = items
        self.bind("<Return>", lambda _e: self._ok())
        self.bind("<Escape>", lambda _e: self.destroy())
        center_on(self, master)
        self._modal()

    def _ok(self) -> None:
        self.result = [self.items[i] for i in self.lb.curselection()]
        self.destroy()

    def _modal(self) -> None:
        try:
            self.wait_visibility()
            self.grab_set()
        except tk.TclError:
            pass
        self.wait_window(self)


class CheckDialog(tk.Toplevel):
    def __init__(self, master, title: str, groups):
        super().__init__(master)
        self.title(title)
        self.transient(master)
        self.result: Optional[List[str]] = None
        self.vars: Dict[str, tk.BooleanVar] = {}
        body = ttk.Frame(self, padding=14)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(body, text=title, style="Title.TLabel").pack(anchor="w", pady=(0, 8))
        for gname, items in groups:
            lf = ttk.LabelFrame(body, text=gname, padding=8)
            lf.pack(fill=tk.X, pady=4)
            for key, label, default in items:
                v = tk.BooleanVar(value=default)
                self.vars[key] = v
                ttk.Checkbutton(lf, text=label, variable=v).pack(side=tk.LEFT, padx=6)
        row = ttk.Frame(body)
        row.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(row, text="Todos", command=lambda: self._set_all(True)).pack(
            side=tk.LEFT)
        ttk.Button(row, text="Ninguno", command=lambda: self._set_all(False)).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(row, text="Graficar", style="Accent.TButton",
                   command=self._ok).pack(side=tk.RIGHT)
        ttk.Button(row, text="Cancelar", command=self.destroy).pack(
            side=tk.RIGHT, padx=(0, 6))
        self.bind("<Return>", lambda _e: self._ok())
        self.bind("<Escape>", lambda _e: self.destroy())
        center_on(self, master)
        try:
            self.wait_visibility()
            self.grab_set()
        except tk.TclError:
            pass
        self.wait_window(self)

    def _set_all(self, value: bool) -> None:
        for v in self.vars.values():
            v.set(value)

    def _ok(self) -> None:
        self.result = [k for k, v in self.vars.items() if v.get()]
        self.destroy()


# Configuracion calibrada con la que se regraban los bags de estado.
# Son los mismos parametros que usan las misiones (brake_test, trackdrive,
# skidpad, autocross) para mirena_state; se aplican por defecto al abrir la
# pestana "Regrabar con nodo". use_sim_time lo anade la casilla correspondiente.
STATE_NODE_ARGS = (
    "--ros-args "
    "-p odom_timeout_sec:=0.5 "
    "-p imu_timeout_sec:=0.2 "
    "-p max_predict_dt_sec:=0.1 "
    "-p process_noise_diag:='[0.01, 0.01, 0.005, 0.5, 0.5, 0.5]' "
    "-p odom_meas_floor_diag:='[1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]' "
    "-p init_cov_floor_diag:='[0.01, 0.01, 0.001, 0.01, 0.01, 0.001]' "
    "-p gyro_meas_var:=1.0e-5 "
    "-p ins_reanchor_m:=12.0 "
    "-p use_bicycle_model:=true "
    "-p steering_degrees:=true "
    "-p steering_sign:=1.0 "
    "-p steering_ratio:=1.35 "
    "-p steering_offset_rad:=0.030 "
    "-p wheel_radius:=0.191"
)

# Topics que consume mirena_state y que hay que reproducir (no hace falta
# reproducir lidar, camaras, etc. para regrabar /state/car).
STATE_PLAY_TOPICS = ("/sensors/imu /sensors/imu/odom /sensors/wheel_speeds "
                     "/control/can")

# Script de regrabacion calibrado. La GUI lo ejecuta tal cual para que el
# orden (record -> nodo -> play pausado -> resume) y el reloj por mensaje sean
# exactamente los del guion que genera bags correctos.
RERECORD_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "rerecord_state.sh")


class RecordPanel(ttk.Frame):
    """Reproduce un bag, ejecuta un nodo y graba un bag nuevo con el resultado.

    Orden: arranca `ros2 bag record` -> arranca el nodo -> espera -> lanza
    `ros2 bag play` -> al terminar la reproduccion espera un poco, y cierra
    play, nodo y record (en este orden, con SIGINT) para que el bag nuevo
    quede bien cerrado. Todo corre en procesos aparte; la GUI no se bloquea.
    """

    def __init__(self, master):
        super().__init__(master, padding=12)
        self.running = False
        self.procs: Dict[str, Optional[subprocess.Popen]] = {}
        self.logq: "queue.Queue" = queue.Queue()
        self.pull_selection = None       # callable -> topics elegidos en el arbol
        self.open_cb = None              # callable(path) -> abrir bag nuevo
        self._after: Optional[str] = None
        self._stage = ""
        self._t_stage = 0.0
        self._t_play = 0.0
        self._wait = 2.0
        self._play_cmd_v: Optional[List[str]] = None
        self._node_cmd_v: Optional[List[str]] = None
        self._out = ""
        self._last_out = ""
        self._final_msg: Optional[str] = None
        self._manual = False
        self._stop_done = False
        self._warned_node = False
        self._out_touched = False
        self._topics: List[str] = []
        self._all_nodes: List[str] = []
        self._hint = None                # (ruta, duracion s) calculada por la app
        self._span: Optional[float] = None
        self._rate = 1.0
        self._t_size = 0.0
        self._size = 0
        self.progress_cb = None          # callable(pct | None)
        self._term: Optional[tk.Toplevel] = None
        self._term_text: Optional[tk.Text] = None

        self.bag = tk.StringVar()
        self.rate = tk.StringVar(value="1.0")
        self.start_s = tk.StringVar()
        self.duration = tk.StringVar()
        self.play_only = tk.StringVar(value=STATE_PLAY_TOPICS)
        self.clock = tk.BooleanVar(value=True)
        self.fast = tk.BooleanVar(value=True)
        self.queue_sz = tk.StringVar(value="10000")
        self.cache_mb = tk.StringVar(value="256")
        self.node_simtime = tk.BooleanVar(value=True)
        self.speed_hint = tk.StringVar(value="")
        self.node = tk.StringVar(value="mirena_state mirena_state")
        self.args = tk.StringVar(value=STATE_NODE_ARGS)
        self.no_node = tk.BooleanVar(value=False)
        self.node_wait = tk.StringVar(value="3.0")
        self.tail = tk.StringVar(value="3.0")
        self.out_dir = tk.StringVar(value=os.path.expanduser("~/mirena_ws/bags"))
        self.out_name = tk.StringVar()
        self.storage = tk.StringVar(value="mcap")
        self.all_topics = tk.BooleanVar(value=False)
        self.extra = tk.StringVar(value="/state/car /tf /health")
        self.use_script = tk.BooleanVar(value=True)
        self.script = tk.StringVar(value=RERECORD_SCRIPT)
        self.auto_open = tk.BooleanVar(value=False)
        self.pct = tk.StringVar(value="")
        self.status = tk.StringVar(
            value="Elige un bag de entrada, un nodo y pulsa «Reproducir y grabar».")

        self._build()
        for v in (self.rate, self.bag, self.clock):
            v.trace_add("write", self._update_hint)
        self._update_hint()
        self.after(600, self._find_nodes)
        self._after = self.after(150, self._tick)

    # ---- construccion ----------------------------------------------------- #

    def _build(self) -> None:
        top = ttk.Frame(self)
        top.pack(fill=tk.X)
        left = ttk.Frame(top)
        left.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # 1 · bag de entrada
        f1 = ttk.LabelFrame(left, text="1 · Bag de entrada (se reproduce)", padding=10)
        f1.pack(fill=tk.X)
        ttk.Label(f1, text="Bag").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Entry(f1, textvariable=self.bag).grid(
            row=0, column=1, columnspan=5, sticky="we", padx=6)
        b = ttk.Frame(f1)
        b.grid(row=0, column=6, sticky="e")
        ttk.Button(b, text="Carpeta…", width=8, command=self._ask_folder).pack(side=tk.LEFT)
        ttk.Button(b, text="Fichero…", width=8, command=self._ask_file).pack(
            side=tk.LEFT, padx=(4, 0))
        ttk.Label(f1, text="Velocidad").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(f1, textvariable=self.rate, width=7).grid(row=1, column=1, sticky="w", padx=6)
        ttk.Label(f1, text="Inicio [s]").grid(row=1, column=2, sticky="w", padx=(8, 0))
        self.start_entry = ttk.Entry(f1, textvariable=self.start_s, width=7)
        self.start_entry.grid(row=1, column=3, sticky="w", padx=6)
        ttk.Label(f1, text="Duración [s]").grid(row=1, column=4, sticky="w", padx=(8, 0))
        ttk.Entry(f1, textvariable=self.duration, width=7).grid(row=1, column=5, sticky="w", padx=6)
        ttk.Label(f1, text="Rápido").grid(row=2, column=0, sticky="w", pady=3)
        pre = ttk.Frame(f1)
        pre.grid(row=2, column=1, columnspan=6, sticky="w", padx=6)
        for txt, val in (("×1", "1"), ("×2", "2"), ("×5", "5"), ("×10", "10"),
                         ("×20", "20"), ("×50", "50"), ("×100", "100"), ("MÁX", "1000")):
            ttk.Button(pre, text=txt, width=5,
                       command=lambda v=val: self.rate.set(v)).pack(side=tk.LEFT, padx=(0, 3))
        ttk.Label(f1, text="Solo topics").grid(row=3, column=0, sticky="w", pady=3)
        self.play_only_entry = ttk.Entry(f1, textvariable=self.play_only)
        self.play_only_entry.grid(row=3, column=1, columnspan=5, sticky="we", padx=6)
        self.clock_chk = ttk.Checkbutton(f1, text="Reloj simulado (/clock)",
                                         variable=self.clock, command=self._on_clock)
        self.clock_chk.grid(row=3, column=6, sticky="e")
        fast = ttk.Frame(f1)
        fast.grid(row=4, column=0, columnspan=7, sticky="w", pady=(4, 0))
        ttk.Checkbutton(fast, text="Optimizar para velocidad", variable=self.fast).pack(
            side=tk.LEFT)
        ttk.Label(fast, text="Cola lectura", style="Muted.TLabel").pack(
            side=tk.LEFT, padx=(16, 4))
        ttk.Entry(fast, textvariable=self.queue_sz, width=7).pack(side=tk.LEFT)
        ttk.Label(fast, text="Caché [MB]", style="Muted.TLabel").pack(
            side=tk.LEFT, padx=(16, 4))
        ttk.Entry(fast, textvariable=self.cache_mb, width=6).pack(side=tk.LEFT)
        ttk.Label(f1, textvariable=self.speed_hint, style="Muted.TLabel",
                  wraplength=600, justify="left").grid(
            row=5, column=0, columnspan=7, sticky="w", pady=(4, 0))
        f1.columnconfigure(1, weight=1)

        # 2 · nodo
        f2 = ttk.LabelFrame(left, text="2 · Nodo a ejecutar", padding=10)
        f2.pack(fill=tk.X, pady=(8, 0))

        srow = ttk.Frame(f2)
        srow.grid(row=0, column=0, columnspan=5, sticky="we", pady=(0, 4))
        self.script_chk = ttk.Checkbutton(
            srow, text="Usar script calibrado (rerecord_state.sh)",
            variable=self.use_script, command=self._toggle_script)
        self.script_chk.pack(side=tk.LEFT)
        self.script_entry = ttk.Entry(srow, textvariable=self.script)
        self.script_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(srow, text="…", width=3, command=self._ask_script).pack(side=tk.LEFT)

        ttk.Label(f2, text="Nodo").grid(row=1, column=0, sticky="w", pady=3)
        self.node_cb = ttk.Combobox(f2, textvariable=self.node, values=[])
        self.node_cb.grid(row=1, column=1, columnspan=3, sticky="we", padx=6)
        self.node_cb.bind("<KeyRelease>", self._filter_nodes)
        ttk.Button(f2, text="Buscar nodos", command=self._find_nodes).grid(
            row=1, column=4, sticky="e")
        ttk.Label(f2, text="Argumentos").grid(row=2, column=0, sticky="w", pady=3)
        self.args_entry = ttk.Entry(f2, textvariable=self.args)
        self.args_entry.grid(row=2, column=1, columnspan=4, sticky="we", padx=6)
        opts = ttk.Frame(f2)
        opts.grid(row=3, column=0, columnspan=5, sticky="w", pady=(4, 0))
        self.no_node_chk = ttk.Checkbutton(
            opts, text="Sin nodo (solo regrabar)", variable=self.no_node,
            command=self._toggle_node)
        self.no_node_chk.pack(side=tk.LEFT)
        self.simtime_chk = ttk.Checkbutton(
            opts, text="use_sim_time", variable=self.node_simtime)
        self.simtime_chk.pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(opts, text="Espera al nodo [s]", style="Muted.TLabel").pack(
            side=tk.LEFT, padx=(16, 4))
        self.wait_entry = ttk.Entry(opts, textvariable=self.node_wait, width=6)
        self.wait_entry.pack(side=tk.LEFT)
        ttk.Label(opts, text="Cola final [s]", style="Muted.TLabel").pack(
            side=tk.LEFT, padx=(16, 4))
        self.tail_entry = ttk.Entry(opts, textvariable=self.tail, width=6)
        self.tail_entry.pack(side=tk.LEFT)
        f2.columnconfigure(1, weight=1)

        # 3 · grabacion
        f3 = ttk.LabelFrame(left, text="3 · Bag nuevo (se graba)", padding=10)
        f3.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(f3, text="Carpeta").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Entry(f3, textvariable=self.out_dir).grid(
            row=0, column=1, columnspan=3, sticky="we", padx=6)
        ttk.Button(f3, text="…", width=3, command=self._ask_out).grid(row=0, column=4)
        ttk.Label(f3, text="Nombre").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(f3, textvariable=self.out_name).grid(row=1, column=1, sticky="we", padx=6)
        ttk.Label(f3, text="Formato").grid(row=1, column=2, sticky="w", padx=(8, 0))
        ttk.Combobox(f3, textvariable=self.storage, values=["mcap", "sqlite3"],
                     width=8, state="readonly").grid(row=1, column=3, sticky="w", padx=6)
        ttk.Label(f3, text="Topics extra").grid(row=2, column=0, sticky="w", pady=3)
        self.extra_entry = ttk.Entry(f3, textvariable=self.extra)
        self.extra_entry.grid(row=2, column=1, columnspan=3, sticky="we", padx=6)
        self.all_topics_chk = ttk.Checkbutton(
            f3, text="Grabar todos (-a)", variable=self.all_topics,
            command=self._toggle_all)
        self.all_topics_chk.grid(row=3, column=1, sticky="w", pady=(4, 0))
        ttk.Label(f3, text="Nombre vacío = automático · sin selección en la lista "
                           "se graban solo los «Topics extra» (salidas del nodo, "
                           "ej. /state/car)", style="Muted.TLabel").grid(
            row=4, column=0, columnspan=5, sticky="w", pady=(4, 0))
        f3.columnconfigure(1, weight=1)

        # topics del bag
        right = ttk.LabelFrame(top, text="Topics del bag a grabar", padding=8)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        lbox = ttk.Frame(right)
        lbox.pack(fill=tk.BOTH, expand=True)
        self.lb = tk.Listbox(lbox, selectmode=tk.EXTENDED, width=30, height=10,
                             exportselection=False, activestyle="none",
                             relief="flat", borderwidth=0, highlightthickness=1)
        sy = ttk.Scrollbar(lbox, orient="vertical", command=self.lb.yview)
        self.lb.configure(yscrollcommand=sy.set)
        sy.pack(side=tk.RIGHT, fill=tk.Y)
        self.lb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        lb = ttk.Frame(right)
        lb.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(lb, text="Todos", width=6,
                   command=lambda: self.lb.selection_set(0, tk.END)).pack(side=tk.LEFT)
        ttk.Button(lb, text="Ninguno", width=7,
                   command=lambda: self.lb.selection_clear(0, tk.END)).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(lb, text="Del árbol", command=self._from_tree).pack(side=tk.LEFT)

        row = ttk.Frame(self)
        row.pack(fill=tk.X, pady=10)
        self.btn_start = ttk.Button(row, text="▶  Reproducir y grabar",
                                    style="Accent.TButton", command=self.start)
        self.btn_start.pack(side=tk.LEFT)
        self.btn_stop = ttk.Button(row, text="■  Detener", style="Danger.TButton",
                                   command=self.stop, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=6)
        self.btn_open = ttk.Button(row, text="Abrir resultado", command=self._open_result,
                                   state=tk.DISABLED)
        self.btn_open.pack(side=tk.LEFT)
        ttk.Checkbutton(row, text="Abrir al terminar", variable=self.auto_open).pack(
            side=tk.LEFT, padx=(14, 0))
        ttk.Label(row, textvariable=self.status, style="Muted.TLabel").pack(
            side=tk.LEFT, padx=14)

        prow = ttk.Frame(self)
        prow.pack(fill=tk.X, pady=(0, 8))
        self.prog = ttk.Progressbar(prow, style="Big.Horizontal.TProgressbar",
                                    mode="determinate", maximum=100)
        self.prog.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(prow, textvariable=self.pct, width=36, anchor="e").pack(
            side=tk.LEFT, padx=(10, 0))

        logbox = ttk.LabelFrame(self, text="Registro", padding=4)
        logbox.pack(fill=tk.BOTH, expand=True)
        tools = ttk.Frame(logbox)
        tools.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(tools, text="Copiar comandos", command=self.copy_cmds).pack(side=tk.LEFT)
        ttk.Button(tools, text="Terminal", command=self._open_terminal).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(tools, text="Guardar registro…", command=self.save_log).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(tools, text="Limpiar", command=self._clear_log).pack(side=tk.LEFT)
        self.log = make_text(logbox)
        self.restyle()

    def restyle(self) -> None:
        style_text(self.log)
        self.log.tag_configure("err", foreground=PAL["danger"])
        self.log.tag_configure("warn", foreground="#d97706")
        self.log.tag_configure("gui", foreground=PAL["muted"])
        self.log.tag_configure("cmd", foreground=PAL["accent"])
        self.lb.configure(bg=PAL["card"], fg=PAL["fg"], selectbackground=PAL["sel"],
                          selectforeground=PAL["fg"], highlightbackground=PAL["border"],
                          highlightcolor=PAL["accent"])

    # ---- entradas --------------------------------------------------------- #

    def _toggle_node(self) -> None:
        st = tk.DISABLED if (self.no_node.get() or self.use_script.get()) else tk.NORMAL
        self.node_cb.configure(state=st)

    def _toggle_all(self) -> None:
        disabled = self.all_topics.get() or self.use_script.get()
        self.lb.configure(state=tk.DISABLED if disabled else tk.NORMAL)

    def _toggle_script(self) -> None:
        scripted = self.use_script.get()
        st = tk.DISABLED if scripted else tk.NORMAL
        if scripted:
            self.start_s.set("")
        for w in (self.args_entry, self.no_node_chk, self.simtime_chk,
                  self.wait_entry, self.tail_entry, self.play_only_entry,
                  self.clock_chk, self.start_entry, self.extra_entry,
                  self.all_topics_chk):
            w.configure(state=st)
        self.script_entry.configure(state=tk.NORMAL if scripted else tk.DISABLED)
        self.btn_start.configure(
            text="▶  Reproducir y grabar (script)" if scripted
            else "▶  Reproducir y grabar")
        self._toggle_node()
        self._toggle_all()
        if scripted:
            self.status.set("Modo script: rerecord_state.sh fija nodo, topics y reloj; "
                            "la salida se ve en «Registro».")
        else:
            self.status.set("Modo manual: usa los campos Nodo / Topics.")

    def _ask_script(self) -> None:
        p = filedialog.askopenfilename(
            title="Script de regrabación",
            initialdir=os.path.dirname(os.path.expanduser(self.script.get())) or None,
            filetypes=[("shell", "*.sh"), ("todos", "*.*")])
        if p:
            self.script.set(p)

    def _ask_folder(self) -> None:
        p = filedialog.askdirectory(title="Carpeta del bag (con metadata.yaml)")
        if p:
            self.set_bag(p)

    def _ask_file(self) -> None:
        p = filedialog.askopenfilename(
            title="Fichero de bag", filetypes=[("rosbag2", "*.mcap *.db3"), ("todos", "*.*")])
        if p:
            self.set_bag(p)

    def _ask_out(self) -> None:
        p = filedialog.askdirectory(title="Carpeta donde crear el bag nuevo",
                                    initialdir=self.out_dir.get() or None)
        if p:
            self._out_touched = True
            self.out_dir.set(p)

    def _from_tree(self) -> None:
        if self.pull_selection is None or self.all_topics.get():
            return
        sel = set(self.pull_selection())
        self.lb.selection_clear(0, tk.END)
        for i, n in enumerate(self._topics):
            if n in sel:
                self.lb.selection_set(i)

    def set_bag(self, path: str) -> None:
        if not path or self.running:
            return
        self.bag.set(path)
        if not self._out_touched:
            parent = os.path.dirname(path.rstrip("/\\")) or os.path.expanduser("~")
            self.out_dir.set(parent)

    _STR = ("rate", "play_only", "node", "args", "node_wait", "tail", "storage", "extra",
            "queue_sz", "cache_mb", "script")
    _BOOL = ("clock", "no_node", "all_topics", "auto_open", "fast", "node_simtime",
             "use_script")

    def get_settings(self) -> Dict:
        d = {k: getattr(self, k).get() for k in self._STR + self._BOOL}
        if self._out_touched:
            d["out_dir"] = self.out_dir.get()
        return d

    def apply_settings(self, d: Dict) -> None:
        for k in self._STR + self._BOOL:
            if k in d:
                try:
                    getattr(self, k).set(d[k])
                except (tk.TclError, ValueError):
                    pass
        if d.get("out_dir"):
            self.out_dir.set(d["out_dir"])
            self._out_touched = True
        self._toggle_script()

    def set_duration_hint(self, path: str, secs: float) -> None:
        self._hint = (path, secs)
        self._update_hint()

    @staticmethod
    def _path_size(path: str) -> int:
        try:
            if os.path.isfile(path):
                return os.path.getsize(path)
        except OSError:
            return 0
        total = 0
        for root, _d, files in os.walk(path):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
        return total

    @staticmethod
    def _bag_info(path: str):
        try:
            with open(os.path.join(path, "metadata.yaml"), encoding="utf-8") as f:
                txt = f.read()
        except OSError:
            return None

        def num(pat: str) -> int:
            m = re.search(pat, txt, re.M)
            return int(m.group(1)) if m else 0
        return {"msgs": num(r"^\s{2}message_count:\s*(\d+)"),
                "topics": len(re.findall(r"^\s*-?\s*topic_metadata:", txt, re.M)),
                "dur": num(r"^\s{2}duration:\s*\n\s+nanoseconds:\s*(\d+)") / 1e9}

    def _read_duration(self, bag: str) -> Optional[float]:
        d = bag if os.path.isdir(bag) else os.path.dirname(bag)
        info = self._bag_info(d) if d else None
        if info and info["dur"] > 0:
            return info["dur"]
        if self._hint and os.path.expanduser(self._hint[0]) == bag and self._hint[1] > 0:
            return self._hint[1]
        return None

    @staticmethod
    def _fmt_t(sec: float) -> str:
        sec = int(max(0, sec))
        return f"{sec // 60}:{sec % 60:02d}"

    def _set_progress(self, pct: Optional[float], text: str = "") -> None:
        if pct is not None and str(self.prog.cget("mode")) == "determinate":
            self.prog["value"] = pct
        self.pct.set(text)
        if self.progress_cb:
            self.progress_cb(pct)

    def _update_progress(self, el: float, now: float) -> None:
        if now - self._t_size >= 1.0:
            self._size, self._t_size = self._path_size(self._out), now
        mb = f"{self._size / 1e6:.1f} MB"
        if self._span:
            pct = min(99.0, el * self._rate / self._span * 100.0)
            eta = max(0.0, self._span / self._rate - el)
            tail = "finalizando…" if pct >= 99 else f"quedan ~{self._fmt_t(eta)}"
            self._set_progress(pct, f"{pct:.0f} %  ·  {tail}  ·  {mb}")
        else:
            self._set_progress(None, f"{self._fmt_t(el)}  ·  {mb}  (duración desconocida)")

    def copy_cmds(self) -> None:
        try:
            bag = os.path.expanduser(self.bag.get().strip())
            if not bag:
                raise ValueError("Elige un bag de entrada.")
            rate = self._num(self.rate, "velocidad", 1.0)
            out = os.path.join(os.path.expanduser(self.out_dir.get().strip()),
                               self.out_name.get().strip() or "<nombre>")
            if self.use_script.get():
                cmds = [self._script_cmd(out, bag, rate,
                                         self._num(self.start_s, "inicio"),
                                         self._num(self.duration, "duración"))]
            else:
                cmds = [self._record_cmd(out)]
                if not self.no_node.get():
                    cmds.append(self._node_cmd())
                cmds.append(self._play_cmd(bag, rate, self._num(self.start_s, "inicio"),
                                           self._num(self.duration, "duración")))
        except ValueError as exc:
            messagebox.showerror("Configuración no válida", str(exc))
            return
        self.clipboard_clear()
        self.clipboard_append("\n".join(" ".join(shlex.quote(c) for c in cmd)
                                        for cmd in cmds))
        if self.use_script.get():
            self.status.set("Comando del script copiado; ejecútalo en una terminal.")
        else:
            self.status.set("Comandos copiados (orden: record, nodo, play; uno por terminal).")

    def save_log(self) -> None:
        p = filedialog.asksaveasfilename(
            title="Guardar registro", defaultextension=".log",
            filetypes=[("log", "*.log"), ("texto", "*.txt")])
        if p:
            try:
                with open(p, "w", encoding="utf-8") as f:
                    f.write(self.log.get("1.0", "end-1c"))
                self.status.set(f"Registro guardado en {p}")
            except OSError as exc:
                messagebox.showerror("Error", str(exc))

    def set_topics(self, names: List[str]) -> None:
        self._topics = list(names)
        self.lb.configure(state=tk.NORMAL)
        self.lb.delete(0, tk.END)
        for n in self._topics:
            self.lb.insert(tk.END, n)
        # Sin seleccion por defecto: al regrabar con un nodo lo normal es
        # grabar solo sus salidas ("Topics extra") y no duplicar el bag de
        # entrada. «Todos» sigue estando a un clic.
        self.lb.selection_clear(0, tk.END)
        self._toggle_all()

    @staticmethod
    def _num(var: tk.StringVar, name: str, default=None):
        s = var.get().strip()
        if not s:
            return default
        try:
            return float(s)
        except ValueError as exc:
            raise ValueError(f"«{name}» debe ser numérico.") from exc

    @staticmethod
    def _bag_target(path: str):
        if os.path.isdir(path):
            return path, None
        ext = os.path.splitext(path)[1].lower()
        return path, {".mcap": "mcap", ".db3": "sqlite3"}.get(ext)

    # ---- nodos disponibles ------------------------------------------------ #

    def _find_nodes(self) -> None:
        if shutil.which("ros2") is None:
            self._log("[gui] «ros2» no está en el PATH: sourcea ROS 2 para listar nodos.")
            return

        def work() -> None:
            try:
                out = subprocess.run(["ros2", "pkg", "executables"], capture_output=True,
                                     text=True, timeout=90).stdout
                self.logq.put(("nodes", [ln.strip() for ln in out.splitlines()
                                         if ln.strip()]))
            except Exception as exc:  # noqa: BLE001
                self.logq.put(f"[gui] no pude listar nodos: {exc}")
        threading.Thread(target=work, daemon=True).start()

    def _filter_nodes(self, _e=None) -> None:
        q = self.node.get().strip().lower()
        self.node_cb.configure(values=[n for n in self._all_nodes if q in n.lower()]
                               if q else self._all_nodes)

    # ---- comandos --------------------------------------------------------- #

    def _node_cmd(self) -> List[str]:
        txt = self.node.get().strip()
        if not txt:
            raise ValueError("Elige un nodo (paquete ejecutable) o marca «Sin nodo».")
        parts = shlex.split(txt)
        if parts[0] != "ros2":
            parts = ["ros2", "run"] + parts
        parts += shlex.split(self.args.get().strip())
        if self.node_simtime.get() and "use_sim_time" not in " ".join(parts):
            parts += ["--ros-args", "-p", "use_sim_time:=true"]
        return parts

    def _script_cmd(self, out: str, bag: str, rate: float, start, duration) -> List[str]:
        script = os.path.expanduser(self.script.get().strip())
        if not script or not os.path.isfile(script):
            raise ValueError("No encuentro el script «rerecord_state.sh».")
        if start is not None:
            raise ValueError("El modo script no admite «Inicio [s]»; déjalo vacío.")
        # El script recibe: <out_bag> [rate] [duration_s] [in_bag]. Se pasa una
        # cadena vacía para la duración cuando no se ha indicado, para que el
        # bag de entrada no se confunda con el tercer argumento.
        return ["bash", script, out, f"{rate:g}",
                (f"{duration:g}" if duration is not None else ""), bag]

    def _on_clock(self) -> None:
        if self.clock.get():
            self.node_simtime.set(True)

    def _fast_params(self):
        """(cola de lectura, caché en bytes) si hay que optimizar; None si no."""
        if not self.fast.get():
            return None
        try:
            rate = self._num(self.rate, "velocidad", 1.0)
        except ValueError:
            return None
        if rate <= 1:
            return None
        try:
            q = int(float(self.queue_sz.get().strip() or 0))
            c = int(float(self.cache_mb.get().strip() or 0) * 1e6)
        except ValueError:
            raise ValueError("«Cola lectura» y «Caché» deben ser numéricos.")
        return q, c

    def _update_hint(self, *_a) -> None:
        try:
            rate = float(self.rate.get().strip() or 1)
        except ValueError:
            self.speed_hint.set("La velocidad debe ser numérica.")
            return
        if rate <= 0:
            self.speed_hint.set("La velocidad debe ser > 0.")
            return
        bag = os.path.expanduser(self.bag.get().strip())
        total = self._read_duration(bag) if bag and os.path.exists(bag) else None
        parts = [f"Bag de {self._fmt_t(total)} → ~{self._fmt_t(total / rate)} a ×{rate:g} "
                 "(si el nodo y el disco aguantan)." if total else f"Velocidad ×{rate:g}."]
        if rate != 1 and not self.clock.get():
            parts.append(f"⚠ Sin «Reloj simulado» el bag nuevo tendrá los tiempos "
                         f"comprimidos ×{rate:g}.")
        if rate > 20:
            parts.append("A velocidades muy altas pueden perderse mensajes si el nodo "
                         "no da abasto.")
        self.speed_hint.set("  ".join(parts))

    def _record_cmd(self, out: str) -> List[str]:
        cmd = ["ros2", "bag", "record", "-o", out, "--storage", self.storage.get()]
        if self.clock.get():
            cmd.append("--use-sim-time")
        fp = self._fast_params()
        if fp and fp[1] > 0:
            cmd += ["--max-cache-size", str(fp[1])]
        if self.all_topics.get():
            return cmd + ["-a"]
        topics = [self._topics[i] for i in self.lb.curselection()]
        topics += shlex.split(self.extra.get().strip())
        topics = list(dict.fromkeys(topics))
        if not topics:
            raise ValueError("Selecciona topics a grabar, añade «Topics extra» "
                             "o marca «Grabar todos (-a)».")
        return cmd + topics

    def _play_cmd(self, bag: str, rate: float, start, duration) -> List[str]:
        uri, storage = self._bag_target(bag)
        cmd = ["ros2", "bag", "play", uri, "--rate", f"{rate:g}"]
        if storage:
            cmd += ["--storage", storage]
        if start is not None:
            cmd += ["--start-offset", f"{start:g}"]
        if duration is not None:
            cmd += ["--playback-duration", f"{duration:g}"]
        if self.clock.get():
            cmd.append("--clock")
        fp = self._fast_params()
        if fp and fp[0] > 0:
            cmd += ["--read-ahead-queue-size", str(fp[0])]
        cmd.append("--disable-keyboard-controls")
        only = shlex.split(self.play_only.get().strip())
        if only:
            cmd += ["--topics"] + only
        return cmd

    # ---- procesos --------------------------------------------------------- #

    def _spawn(self, cmd: List[str], tag: str):
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        self._log("$ " + " ".join(shlex.quote(c) for c in cmd))
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, bufsize=1, env=env, start_new_session=True)
        except (FileNotFoundError, OSError) as exc:
            self._log(f"[{tag}] no pude lanzar el proceso: {exc}")
            return None
        threading.Thread(target=self._reader, args=(p, tag), daemon=True).start()
        return p

    def _reader(self, p, tag: str) -> None:
        try:
            for line in p.stdout:
                self.logq.put(f"[{tag}] {line.rstrip()}")
        except Exception:  # noqa: BLE001
            pass

    @staticmethod
    def _kill(p, timeout: float) -> None:
        if p is None or p.poll() is not None:
            return
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3),
                          (signal.SIGKILL, 3)):
            try:
                os.killpg(p.pid, sig)
            except (ProcessLookupError, PermissionError):
                return
            try:
                p.wait(wait)
                return
            except subprocess.TimeoutExpired:
                continue

    # ---- ciclo de vida ---------------------------------------------------- #

    def start(self) -> None:
        if self.running:
            return
        bag = os.path.expanduser(self.bag.get().strip())
        if not bag or not os.path.exists(bag):
            messagebox.showerror("Bag no válido",
                                 "Elige la carpeta del bag o un fichero .mcap existente.")
            return
        if os.path.isfile(bag) and not bag.lower().endswith((".mcap", ".db3")):
            messagebox.showerror("Bag no válido", "El fichero debe ser .mcap o .db3.")
            return
        if shutil.which("ros2") is None:
            messagebox.showerror("Falta ROS 2",
                                 "No encuentro «ros2». Sourcea ROS 2 antes de abrir la GUI:\n\n"
                                 "    source /opt/ros/jazzy/setup.bash")
            return
        try:
            rate = self._num(self.rate, "velocidad", 1.0)
            start = self._num(self.start_s, "inicio")
            dur = self._num(self.duration, "duración")
            wait = self._num(self.node_wait, "espera al nodo", 2.0)
            tail = self._num(self.tail, "cola final", 2.0)
        except ValueError as exc:
            messagebox.showerror("Valor no válido", str(exc))
            return
        if rate <= 0 or wait < 0 or tail < 0:
            messagebox.showerror("Valor no válido",
                                 "La velocidad debe ser > 0 y las esperas ≥ 0.")
            return
        out_dir = os.path.expanduser(self.out_dir.get().strip())
        notes: List[str] = []
        if not out_dir:
            messagebox.showerror("Carpeta no válida", "Indica una carpeta de salida.")
            return
        if not os.path.isdir(out_dir):
            try:
                os.makedirs(out_dir, exist_ok=True)
            except OSError as exc:
                messagebox.showerror("No pude crear la carpeta", f"{out_dir}\n\n{exc}")
                return
            notes.append(f"[gui] carpeta creada: {out_dir}")
        name = self.out_name.get().strip()
        if not name:
            base = os.path.basename(bag.rstrip("/\\"))
            if os.path.isfile(bag):
                base = os.path.splitext(base)[0]
            name = f"{base}_regrabado_{time.strftime('%Y%m%d_%H%M%S')}"
        out = os.path.join(out_dir, name)
        if os.path.exists(out):
            k = 2
            while os.path.exists(f"{out}_{k}"):
                k += 1
            notes.append(f"[gui] «{os.path.basename(out)}» ya existe; "
                         f"se usará «{os.path.basename(out)}_{k}»")
            out = f"{out}_{k}"
        try:
            if self.use_script.get():
                cmd = self._script_cmd(out, bag, rate, start, dur)
                rec = node = play = None
            else:
                cmd = None
                rec = self._record_cmd(out)
                node = None if self.no_node.get() else self._node_cmd()
                play = self._play_cmd(bag, rate, start, dur)
        except ValueError as exc:
            messagebox.showerror("Configuración no válida", str(exc))
            return

        need = self._path_size(bag)
        try:
            free = shutil.disk_usage(out_dir).free
        except OSError:
            free = None
        if free is not None and need and free < need * 1.1:
            if not messagebox.askyesno(
                    "Poco espacio en disco",
                    f"El bag de entrada ocupa {need / 1e6:.0f} MB y quedan "
                    f"{free / 1e6:.0f} MB libres en la carpeta de salida.\n\n"
                    "¿Continuar igualmente?"):
                return
        self._clear_log()
        for n in notes:
            self._log(n)
        self._out, self._wait, self._tail_v = out, wait, tail
        total = self._read_duration(bag)
        span = None
        if total:
            span = max(0.0, total - (start or 0.0))
            if dur:
                span = min(span, dur)
        self._span, self._rate = (span or None), rate
        self._t_size, self._size = 0.0, 0
        self.prog.stop()
        self.prog.configure(mode="determinate")
        self.prog["value"] = 0
        self._set_progress(0, "preparando…")
        self._node_cmd_v, self._play_cmd_v = node, play
        self._final_msg, self._manual = None, False
        self._stop_done, self._warned_node = False, False
        self.procs = {}
        if cmd is not None:
            self._open_terminal()
            p = self._spawn(cmd, "script")
            if p is None:
                self.status.set("No pude arrancar el script.")
                return
            self.procs["script"] = p
            self.running = True
            self.btn_start.configure(state=tk.DISABLED)
            self.btn_stop.configure(state=tk.NORMAL)
            self.btn_open.configure(state=tk.DISABLED)
            self._stage, self._t_stage = "script", time.monotonic()
            self.status.set("Ejecutando rerecord_state.sh…")
            return
        p = self._spawn(rec, "rec")
        if p is None:
            self.status.set("No pude arrancar la grabación.")
            return
        self.procs["rec"] = p
        self.running = True
        self.btn_start.configure(state=tk.DISABLED)
        self.btn_stop.configure(state=tk.NORMAL)
        self.btn_open.configure(state=tk.DISABLED)
        self._stage, self._t_stage = "rec_wait", time.monotonic()
        self.status.set("Iniciando la grabación…")

    def stop(self) -> None:
        if self.running:
            self._begin_stop(manual=True)

    def _begin_stop(self, msg: Optional[str] = None, manual: bool = False) -> None:
        if self._stage == "stopping":
            return
        self._stage, self._final_msg, self._manual = "stopping", msg, manual
        self.btn_stop.configure(state=tk.DISABLED)
        self.status.set("Cerrando procesos y finalizando el bag…")

        def work() -> None:
            for tag, t in (("script", 15), ("play", 6), ("node", 8), ("rec", 15)):
                self._kill(self.procs.get(tag), t)
            self._stop_done = True
        threading.Thread(target=work, daemon=True).start()

    def _finalize(self) -> None:
        self.running = False
        self._stage = ""
        self.procs = {}
        self.btn_start.configure(state=tk.NORMAL)
        self.btn_stop.configure(state=tk.DISABLED)
        self.prog.stop()
        self.prog.configure(mode="determinate")
        info = self._bag_info(self._out)
        ok = info is not None
        size = self._path_size(self._out) / 1e6
        failed = bool(self._final_msg)
        if ok:
            self._last_out = self._out
            self.btn_open.configure(state=tk.NORMAL)
            self._log(f"[gui] resumen · {info['msgs']} mensajes · {info['topics']} topics · "
                      f"{info['dur']:.1f} s · {size:.1f} MB")
        if failed:
            self.status.set(self._final_msg)
        elif ok:
            self.status.set(("Detenido" if self._manual else "Listo")
                            + f" · bag guardado en {self._out}")
        else:
            self.status.set("No se generó el bag nuevo (¿no llegó ningún mensaje?).")
        if ok and not failed and not self._manual:
            self.prog["value"] = 100
            self.pct.set(f"Completada · {info['msgs']} msgs · {size:.1f} MB")
        else:
            self.pct.set("Interrumpida" + (f" · {size:.1f} MB" if ok else ""))
        if self.progress_cb:
            self.progress_cb(None)
        self._log(f"[gui] fin · {'bag en ' + self._out if ok else 'sin bag de salida'}")
        self.bell()
        if ok and self.auto_open.get() and self.open_cb:
            self.open_cb(self._out)

    def _open_result(self) -> None:
        if self.open_cb and self._last_out:
            self.open_cb(self._last_out)

    def _start_play(self, now: float) -> None:
        p = self._spawn(self._play_cmd_v, "play")
        if p is None:
            self._begin_stop("No pude lanzar «ros2 bag play».")
            return
        self.procs["play"] = p
        self._stage, self._t_play = "playing", now
        if not self._span:
            self.prog.configure(mode="indeterminate")
            self.prog.start(15)
        self.status.set("Reproduciendo y grabando…")

    def _tick(self) -> None:
        self._after = None
        try:
            self._drain()
            if self.running:
                self._advance()
        except Exception:  # noqa: BLE001
            traceback.print_exc()
        self._after = self.after(150, self._tick)

    def _advance(self) -> None:
        now, st = time.monotonic(), self._stage
        rec = self.procs.get("rec")
        if st != "stopping" and rec is not None and rec.poll() is not None:
            self._begin_stop("La grabación terminó de forma inesperada; revisa el registro.")
            return
        if st == "script":
            p = self.procs.get("script")
            if p is not None and p.poll() is not None:
                if p.returncode != 0:
                    self._begin_stop(f"El script terminó con código {p.returncode}; "
                                     "revisa el registro.")
                else:
                    self._begin_stop()
                return
            self.status.set("Ejecutando rerecord_state.sh…")
            self._update_progress(now - self._t_stage, now)
            return
        if st == "rec_wait" and now - self._t_stage >= 1.5:
            if self._node_cmd_v:
                p = self._spawn(self._node_cmd_v, "nodo")
                if p is None:
                    self._begin_stop("No pude arrancar el nodo.")
                    return
                self.procs["node"] = p
                self._stage, self._t_stage = "node_wait", now
                self.status.set(f"Esperando {self._wait:g} s a que arranque el nodo…")
            else:
                self._start_play(now)
        elif st == "node_wait" and now - self._t_stage >= self._wait:
            self._start_play(now)
        elif st == "playing":
            node = self.procs.get("node")
            if node is not None and node.poll() is not None and not self._warned_node:
                self._warned_node = True
                self._log(f"[gui] el nodo terminó antes de tiempo (código {node.returncode})")
            self.status.set("Reproduciendo y grabando…")
            self._update_progress(now - self._t_play, now)
            rc = self.procs["play"].poll()
            if rc is not None:
                if rc != 0:
                    self._begin_stop(f"«ros2 bag play» terminó con código {rc}; "
                                     "revisa el registro.")
                else:
                    self._stage, self._t_stage = "tail", now
                    self.prog.stop()
                    self.prog.configure(mode="determinate")
                    self._set_progress(100, f"100 %  ·  {self._size / 1e6:.1f} MB")
                    if self._span:
                        self._log(f"[gui] velocidad efectiva ≈ ×"
                                  f"{self._span / max(now - self._t_play, 1e-3):.1f} "
                                  f"(pedida ×{self._rate:g})")
                    self.status.set("Reproducción terminada · vaciando cola final…")
        elif st == "tail" and now - self._t_stage >= self._tail_v:
            self._begin_stop()
        elif st == "stopping" and self._stop_done:
            self._finalize()

    # ---- registro --------------------------------------------------------- #

    def _log(self, line: str) -> None:
        self.logq.put(line)

    def _clear_log(self) -> None:
        set_text(self.log, "")
        if self._term_text is not None:
            set_text(self._term_text, "")

    def _open_terminal(self) -> None:
        if self._term is not None and self._term.winfo_exists():
            self._term.deiconify()
            self._term.lift()
            return
        win = tk.Toplevel(self)
        win.title("Terminal · salida del script")
        win.geometry("1000x460")
        self._term_text = make_text(win)
        self._term_text.configure(state="normal")
        self._term_text.insert("1.0", self.log.get("1.0", "end-1c"))
        self._term_text.see(tk.END)
        self._term_text.configure(state="disabled")
        self._term = win
        win.protocol("WM_DELETE_WINDOW", self._close_terminal)

    def _close_terminal(self) -> None:
        if self._term is not None:
            try:
                self._term.destroy()
            except tk.TclError:
                pass
        self._term = None
        self._term_text = None

    def _drain(self) -> None:
        lines: List[str] = []
        try:
            while True:
                item = self.logq.get_nowait()
                if isinstance(item, tuple):
                    self._all_nodes = sorted(item[1])
                    self.node_cb.configure(values=self._all_nodes)
                    lines.append(f"[gui] {len(self._all_nodes)} ejecutables ROS 2 disponibles")
                else:
                    lines.append(item)
        except queue.Empty:
            pass
        if not lines:
            return
        self.log.configure(state="normal")
        for ln in lines:
            low = ln.lower()
            tag = ("gui" if ln.startswith("[gui]") else "cmd" if ln.startswith("$ ")
                   else "err" if ("error" in low or "traceback" in low or "fatal" in low)
                   else "warn" if "warn" in low else "")
            self.log.insert(tk.END, ln + "\n", (tag,) if tag else ())
        extra = int(self.log.index("end-1c").split(".")[0]) - 3000
        if extra > 0:
            self.log.delete("1.0", f"{extra + 1}.0")
        self.log.see(tk.END)
        self.log.configure(state="disabled")
        if self._term_text is not None:
            try:
                self._term_text.configure(state="normal")
                for ln in lines:
                    self._term_text.insert(tk.END, ln + "\n")
                extra = int(self._term_text.index("end-1c").split(".")[0]) - 3000
                if extra > 0:
                    self._term_text.delete("1.0", f"{extra + 1}.0")
                self._term_text.see(tk.END)
                self._term_text.configure(state="disabled")
            except tk.TclError:
                self._term_text = None

    def shutdown(self) -> None:
        if self._after is not None:
            try:
                self.after_cancel(self._after)
            except Exception:  # noqa: BLE001
                pass
            self._after = None
        if self.running:
            for tag, t in (("script", 15), ("play", 3), ("node", 4), ("rec", 8)):
                self._kill(self.procs.get(tag), t)
            self.running = False
        self._close_terminal()


def make_text(parent) -> tk.Text:
    frame = ttk.Frame(parent)
    frame.pack(fill=tk.BOTH, expand=True)
    txt = tk.Text(frame, wrap="none", font="TkFixedFont", state="disabled")
    style_text(txt)
    sy = ttk.Scrollbar(frame, orient="vertical", command=txt.yview)
    sx = ttk.Scrollbar(frame, orient="horizontal", command=txt.xview)
    txt.configure(yscrollcommand=sy.set, xscrollcommand=sx.set)
    sy.pack(side=tk.RIGHT, fill=tk.Y)
    sx.pack(side=tk.BOTTOM, fill=tk.X)
    txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    return txt


def set_text(txt: tk.Text, s: str) -> None:
    txt.configure(state="normal")
    txt.delete("1.0", tk.END)
    txt.insert("1.0", s)
    txt.configure(state="disabled")


# --------------------------------------------------------------------------- #
#  Aplicacion
# --------------------------------------------------------------------------- #

class App(tk.Tk):
    HEADS = {"#0": "Topic", "type": "Tipo", "msgs": "Msgs", "hz": "Hz",
             "gaps": "Huecos"}

    def __init__(self, path: Optional[str] = None):
        super().__init__()
        self.title("ROS 2 bag analyzer")
        self.geometry("1360x820")
        self.minsize(1040, 640)
        self.col: Dict[str, A.Base] = {}
        self.qos: Dict[str, str] = {}
        self.st: Dict[str, Dict] = {}
        self.t0 = 0.0
        self.path = ""
        self.q: "queue.Queue" = queue.Queue()
        self.loading = False
        self._poll_id: Optional[str] = None
        self._sel_after: Optional[str] = None
        self._sort: Tuple[str, bool] = ("#0", False)
        self.last_dir = os.path.expanduser("~")

        self.settings = load_settings()
        self.mode = self.settings.get("mode", "light")
        if self.mode not in PALETTES:
            self.mode = "light"
        ld = self.settings.get("last_dir")
        if isinstance(ld, str) and os.path.isdir(ld):
            self.last_dir = ld
        self.pal = apply_theme(self, self.mode)

        self.only_nav = tk.BooleanVar(value=bool(self.settings.get("only_nav", False)))
        self.sample = tk.StringVar(value="1000000")
        self.t_start = tk.StringVar()
        self.t_end = tk.StringVar()
        self.filter = tk.StringVar()
        self.count = tk.StringVar(value="0 topics")
        self.status = tk.StringVar(value="Abre un bag para empezar (Ctrl+O).")

        self._menu()
        self._ui()
        self._bind_keys()
        self.filter.trace_add("write", lambda *_: self._fill_tree())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_id = self.after(100, self._poll)
        if path:
            self.after(200, lambda: self.open_bag(path))

    # ---- construccion ----------------------------------------------------- #

    def _menu(self) -> None:
        m = tk.Menu(self)
        f = tk.Menu(m, tearoff=0)
        f.add_command(label="Abrir carpeta de bag…", accelerator="Ctrl+O",
                      command=self.ask_folder)
        f.add_command(label="Abrir fichero .mcap / .db3…",
                      accelerator="Ctrl+Shift+O", command=self.ask_file)
        f.add_separator()
        f.add_command(label="Exportar CSV de la selección…", command=self.export_csv)
        f.add_separator()
        f.add_command(label="Salir", accelerator="Ctrl+Q", command=self._on_close)
        m.add_cascade(label="Archivo", menu=f)
        v = tk.Menu(m, tearoff=0)
        v.add_command(label="Cambiar tema claro / oscuro", accelerator="Ctrl+D",
                      command=self.toggle_theme)
        v.add_command(label="Filtrar topics", accelerator="Ctrl+F",
                      command=self._focus_search)
        m.add_cascade(label="Ver", menu=v)
        self.config(menu=m)

        # menu contextual del arbol de topics
        self.ctx = tk.Menu(self, tearoff=0)
        self.ctx.add_command(label="Graficar", command=self.plot_selected)
        self.ctx.add_command(label="Vista 3D", command=self.plot_3d_selected)
        self.ctx.add_command(label="Temporización", command=self.plot_timing)
        self.ctx.add_separator()
        self.ctx.add_command(label="Exportar CSV…", command=self.export_csv)

    def _btns(self, parent, items, cols: int = 3) -> None:
        for i, (txt, cmd) in enumerate(items):
            ttk.Button(parent, text=txt, command=cmd).grid(
                row=i // cols, column=i % cols, sticky="we", padx=2, pady=2)
        for c in range(cols):
            parent.columnconfigure(c, weight=1, uniform="b")

    def _ui(self) -> None:
        top = ttk.Frame(self, padding=(12, 10))
        top.pack(fill=tk.X)
        ttk.Button(top, text="Abrir bag", style="Accent.TButton",
                   command=self.ask_folder).pack(side=tk.LEFT)
        ttk.Button(top, text="Fichero…", command=self.ask_file).pack(
            side=tk.LEFT, padx=(6, 18))
        ttk.Checkbutton(top, text="Solo IMU + GPS (carga rápida)",
                        variable=self.only_nav).pack(side=tk.LEFT)
        ttk.Label(top, text="Máx. msgs/topic", style="Muted.TLabel").pack(
            side=tk.LEFT, padx=(18, 6))
        ttk.Entry(top, textvariable=self.sample, width=9).pack(side=tk.LEFT)
        ttk.Button(top, text="◐", width=3, command=self.toggle_theme).pack(
            side=tk.RIGHT)
        self.prog = ttk.Progressbar(top, mode="indeterminate", length=130)
        self.prog.pack(side=tk.RIGHT, padx=12)
        ttk.Separator(self).pack(fill=tk.X)

        ttk.Label(self, textvariable=self.status, style="Status.TLabel",
                  anchor="w").pack(fill=tk.X, side=tk.BOTTOM)

        pw = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        pw.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)

        # --- izquierda: topics + acciones
        left = ttk.Frame(pw)
        pw.add(left, weight=2)

        head = ttk.Frame(left)
        head.pack(fill=tk.X)
        ttk.Label(head, text="Topics", style="Title.TLabel").pack(side=tk.LEFT)
        ttk.Label(head, textvariable=self.count, style="Muted.TLabel").pack(
            side=tk.RIGHT)
        srow = ttk.Frame(left)
        srow.pack(fill=tk.X, pady=(6, 6))
        ttk.Label(srow, text="Filtrar", style="Muted.TLabel").pack(side=tk.LEFT)
        self.search = ttk.Entry(srow, textvariable=self.filter)
        self.search.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0))

        acts = ttk.Frame(left)
        acts.pack(fill=tk.X, side=tk.BOTTOM, pady=(8, 0))

        tree_box = ttk.Frame(left)
        tree_box.pack(fill=tk.BOTH, expand=True)
        cols = ("type", "msgs", "hz", "gaps")
        self.tree = ttk.Treeview(tree_box, columns=cols, show="tree headings",
                                 selectmode="extended", height=12)
        self.tree.heading("#0", text=self.HEADS["#0"], anchor="w",
                          command=lambda: self._sort_by("#0"))
        self.tree.column("#0", width=230)
        for k, w in (("type", 110), ("msgs", 62), ("hz", 55), ("gaps", 62)):
            self.tree.heading(k, text=self.HEADS[k], command=lambda k=k: self._sort_by(k))
            self.tree.column(k, width=w, anchor="e" if k != "type" else "w")
        ys = ttk.Scrollbar(tree_box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=ys.set)
        ys.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self._tree_tags()
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self.on_select())
        self.tree.bind("<Double-Button-1>", self._on_double)

        imu = ttk.LabelFrame(acts, text="Solo IMU", padding=6)
        imu.pack(fill=tk.X, pady=3)
        self._btns(imu, [("Dashboard", self.imu_dashboard),
                         ("Canales…", self.imu_channels),
                         ("Gravedad 3D", self.imu_3d)])

        gps = ttk.LabelFrame(acts, text="Solo GPS", padding=6)
        gps.pack(fill=tk.X, pady=3)
        self._btns(gps, [("Dashboard", self.gps_dashboard),
                         ("Canales…", self.gps_channels),
                         ("Traza 3D", self.gps_3d)])

        sel = ttk.LabelFrame(acts, text="Topic seleccionado", padding=6)
        sel.pack(fill=tk.X, pady=3)
        self._btns(sel, [("Graficar", self.plot_selected),
                         ("3D", self.plot_3d_selected),
                         ("Temporización", self.plot_timing),
                         ("CSV…", self.export_csv)], cols=4)

        glob = ttk.LabelFrame(acts, text="Global", padding=6)
        glob.pack(fill=tk.X, pady=3)
        self._btns(glob, [("Tasas y ancho de banda", self.plot_global),
                          ("Comparar IMU", self.compare_imus),
                          ("Regrabar con nodo", self.goto_record)], cols=3)

        win = ttk.LabelFrame(acts, text="Ventana temporal [s desde el inicio]",
                             padding=6)
        win.pack(fill=tk.X, pady=3)
        ttk.Label(win, text="inicio").pack(side=tk.LEFT)
        ttk.Entry(win, textvariable=self.t_start, width=8).pack(side=tk.LEFT, padx=6)
        ttk.Label(win, text="fin").pack(side=tk.LEFT)
        ttk.Entry(win, textvariable=self.t_end, width=8).pack(side=tk.LEFT, padx=6)
        ttk.Button(win, text="Limpiar", command=self._clear_window).pack(
            side=tk.RIGHT)
        ttk.Label(win, text="(vacío = todo)", style="Muted.TLabel").pack(
            side=tk.RIGHT, padx=8)

        # --- derecha: informes
        right = ttk.Notebook(pw)
        pw.add(right, weight=3)
        f1, f2 = ttk.Frame(right), ttk.Frame(right)
        right.add(f1, text="Informe del topic")
        right.add(f2, text="Resumen general")
        self.txt_topic = make_text(f1)
        self.txt_over = make_text(f2)
        self.rec_panel = RecordPanel(right)
        self.rec_panel.pull_selection = lambda: list(self.tree.selection())
        self.rec_panel.open_cb = self.open_bag
        self.rec_panel.progress_cb = self._on_rec_progress
        self.rec_panel.apply_settings(self.settings.get("record", {}))
        right.add(self.rec_panel, text="Regrabar con nodo")
        self.nb = right

    def _bind_keys(self) -> None:
        self.bind("<Control-o>", lambda _e: self.ask_folder())
        self.bind("<Control-O>", lambda _e: self.ask_file())
        self.bind("<Control-d>", lambda _e: self.toggle_theme())
        self.bind("<Control-f>", lambda _e: self._focus_search())
        self.bind("<Control-q>", lambda _e: self._on_close())
        self.bind("<F5>", lambda _e: (self.nb.select(self.rec_panel),
                                      self.rec_panel.start()))
        self.bind("<F6>", lambda _e: self.rec_panel.stop())
        self.search.bind("<Escape>", lambda _e: self.filter.set(""))
        self.tree.bind("<Return>", lambda _e: self.plot_selected())
        self.tree.bind("<Control-a>", self._select_all)
        for seq in ("<Button-3>", "<Button-2>", "<Control-Button-1>"):
            self.tree.bind(seq, self._popup)

    def _focus_search(self) -> None:
        self.search.focus_set()
        self.search.select_range(0, tk.END)

    def _select_all(self, _e=None):
        self.tree.selection_set(self.tree.get_children())
        return "break"

    def _popup(self, e) -> None:
        iid = self.tree.identify_row(e.y)
        if iid and iid not in self.tree.selection():
            self.tree.selection_set(iid)
        if not self.tree.selection():
            return
        try:
            self.ctx.tk_popup(e.x_root, e.y_root)
        finally:
            self.ctx.grab_release()

    def _on_double(self, e) -> None:
        if self.tree.identify_region(e.x, e.y) in ("cell", "tree"):
            self.plot_selected()

    # ---- tema ------------------------------------------------------------- #

    def toggle_theme(self) -> None:
        self.mode = "dark" if self.mode == "light" else "light"
        self.pal = apply_theme(self, self.mode)
        self._menu()
        for t in (self.txt_topic, self.txt_over):
            style_text(t)
        self.rec_panel.restyle()
        self._tree_tags()
        self.status.set("Tema oscuro" if self.mode == "dark" else "Tema claro")

    def _tree_tags(self) -> None:
        self.tree.tag_configure("odd", background=PAL["stripe"])
        self.tree.tag_configure("even", background=PAL["card"])

    # ---- lista de topics (filtro / orden) --------------------------------- #

    def _key(self, n: str):
        c, b = self.col[n], self.st[n]
        k = self._sort[0]
        if k == "#0":
            return n.lower()
        if k == "type":
            return c.type
        v = b["n"] if k == "msgs" else b["hz"] if k == "hz" else b["gaps"]
        try:
            return -1.0 if not math.isfinite(float(v)) else float(v)
        except (TypeError, ValueError):
            return -1.0

    def _sort_by(self, k: str) -> None:
        rev = (not self._sort[1]) if self._sort[0] == k else False
        self._sort = (k, rev)
        for key, txt in self.HEADS.items():
            arrow = (" ▼" if rev else " ▲") if key == k else ""
            self.tree.heading(key, text=txt + arrow)
        self._fill_tree()

    def _fill_tree(self) -> None:
        if not hasattr(self, "tree"):
            return
        q = self.filter.get().strip().lower()
        keep = set(self.tree.selection())
        self.tree.delete(*self.tree.get_children())
        i = 0
        for n in sorted(self.col, key=self._key, reverse=self._sort[1]):
            c, b = self.col[n], self.st[n]
            if q and q not in n.lower() and q not in c.type.lower():
                continue
            hz = f"{b['hz']:.1f}" if math.isfinite(b["hz"]) else "-"
            self.tree.insert("", "end", iid=n, text=n,
                             values=(c.type.split("/")[-1], b["n"], hz, b["gaps"]),
                             tags=("odd" if i % 2 else "even",))
            i += 1
        kept = [n for n in keep if self.tree.exists(n)]
        if kept:
            self.tree.selection_set(kept)
        self.count.set(f"{i} de {len(self.col)} topics" if q else f"{i} topics")

    def _clear_window(self) -> None:
        self.t_start.set("")
        self.t_end.set("")

    # ---- carga ------------------------------------------------------------ #

    def ask_folder(self) -> None:
        p = filedialog.askdirectory(title="Carpeta del bag (con metadata.yaml)",
                                    initialdir=self.last_dir)
        if p:
            self.last_dir = os.path.dirname(p) or p
            self.open_bag(p)

    def ask_file(self) -> None:
        p = filedialog.askopenfilename(
            title="Fichero de bag", initialdir=self.last_dir,
            filetypes=[("rosbag2", "*.mcap *.db3"), ("todos", "*.*")])
        if p:
            self.last_dir = os.path.dirname(p)
            self.open_bag(p)

    def open_bag(self, path: str) -> None:
        if self.loading:
            return
        try:
            sample = max(1000, int(self.sample.get()))
        except ValueError:
            messagebox.showerror("Valor no válido", "«máx. msgs/topic» debe ser un entero.")
            return
        self.loading = True
        self.path = path
        self.status.set(f"Leyendo {path} …")
        self.config(cursor="watch")
        self.prog.start(12)
        threading.Thread(target=self._load_worker,
                         args=(path, self.only_nav.get(), sample), daemon=True).start()

    def _load_worker(self, path: str, only_nav: bool, sample: int) -> None:
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                wanted = None
                if only_nav:
                    r = A.open_reader(path)
                    wanted = [t.name for t in r.get_all_topics_and_types()
                              if t.type in (A.IMU_TYPE, A.FIX_TYPE)]
                    del r
                    if not wanted:
                        raise RuntimeError("El bag no tiene topics Imu ni NavSatFix.")
                col, qos = A.scan(path, wanted, sample)
                if not col:
                    raise RuntimeError("El bag no tiene topics legibles.")
                for c in col.values():
                    align_time(c)
                t0 = min((c.t_bag[0] for c in col.values() if c.t_bag.size), default=0.0)
                st = A.report_overview(col, qos)
                for c in col.values():
                    if c.kind == "log":
                        A.report_log(c, 60)
            self.q.put(("ok", (col, qos, st, t0, buf.getvalue())))
        except SystemExit as exc:
            self.q.put(("err", str(exc).strip() or "No se pudo abrir el bag."))
        except Exception:  # noqa: BLE001
            self.q.put(("err", traceback.format_exc()))

    def _poll(self) -> None:
        try:
            while True:
                kind, payload = self.q.get_nowait()
                self.prog.stop()
                self.config(cursor="")
                self.loading = False
                if kind == "err":
                    self.status.set("Error al cargar.")
                    messagebox.showerror("No se pudo cargar el bag", payload)
                    continue
                self.col, self.qos, self.st, self.t0, over = payload
                set_text(self.txt_over, over)
                set_text(self.txt_topic, "Selecciona un topic de la lista.")
                self.filter.set("")
                self._fill_tree()
                imus = sum(c.kind == "imu" for c in self.col.values())
                gps = sum(c.kind == "fix" for c in self.col.values())
                self.rec_panel.set_bag(self.path)
                self.rec_panel.set_topics(sorted(self.col))
                hi = max((float(c.t_bag[-1]) for c in self.col.values() if c.t_bag.size),
                         default=self.t0)
                self.rec_panel.set_duration_hint(self.path, hi - self.t0)
                self.status.set(f"{self.path}   ·   {len(self.col)} topics   ·   "
                                f"{imus} IMU, {gps} GPS")
        except queue.Empty:
            pass
        self._poll_id = self.after(100, self._poll)

    # ---- utilidades ------------------------------------------------------- #

    def _need_bag(self) -> bool:
        if not self.col:
            messagebox.showinfo("Sin datos", "Abre un bag primero.")
            return False
        return True

    def _window(self) -> Tuple[Optional[float], Optional[float]]:
        def val(s: str) -> Optional[float]:
            s = s.strip()
            return float(s) if s else None
        try:
            return val(self.t_start.get()), val(self.t_end.get())
        except ValueError:
            raise ValueError("La ventana temporal debe ser numérica (segundos).")

    def view(self, name: str):
        """Colector y estadisticas, recortados a la ventana temporal si hay."""
        c, b = self.col[name], self.st[name]
        s, e = self._window()
        if s is None and e is None:
            return c, b
        c2 = copy.copy(c)
        A.window(c2, self.t0, s, e)
        if c2.t.size < 2:
            raise ValueError(f"No hay datos de {name} en esa ventana temporal.")
        return c2, A.base_stats(c2)

    def pick(self, kind: str, multi: bool = False) -> List[str]:
        if not self._need_bag():
            return []
        cands = [n for n, c in self.col.items() if c.kind == kind and has_data(c)]
        label = {"imu": "IMU", "fix": "GPS"}.get(kind, kind)
        if not cands:
            messagebox.showinfo("Sin topics", f"Este bag no tiene topics {label} con datos.")
            return []
        sel = [n for n in self.tree.selection() if n in cands]
        if sel:
            return sel if multi else sel[:1]
        if len(cands) == 1:
            return cands
        return ListDialog(self, f"Elige topic {label}", cands, multi).result

    def show(self, builder, title: str) -> None:
        self.config(cursor="watch")
        self.status.set(f"Generando «{title}»…")
        self.update_idletasks()
        try:
            fig = builder()
            if fig is None:
                messagebox.showinfo("Sin gráfica", "No hay datos suficientes para esa gráfica.")
                return
            PlotWindow(self, fig, title)
            self.status.set(f"Listo · {title}")
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            self.status.set("Error al graficar.")
            messagebox.showerror("Error al graficar", f"{type(exc).__name__}: {exc}")
        finally:
            self.config(cursor="")

    def captured(self, fn, *a, **kw):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            r = fn(*a, **kw)
        return r, buf.getvalue()

    # ---- informe ---------------------------------------------------------- #

    def on_select(self) -> None:
        """Debounce: al recorrer la lista con las flechas solo se informa del ultimo."""
        if self._sel_after is not None:
            try:
                self.after_cancel(self._sel_after)
            except Exception:  # noqa: BLE001
                pass
        self._sel_after = self.after(80, self._on_select_now)

    def _on_select_now(self) -> None:
        self._sel_after = None
        sel = self.tree.selection()
        if not sel:
            return
        n = sel[0]
        try:
            c, b = self.col[n], self.st[n]
        except KeyError:
            return
        _, text = self.captured(self._report, c, b)
        set_text(self.txt_topic, text)
        self.nb.select(0)

    @staticmethod
    def _report(c, b) -> None:
        if not has_data(c) and c.kind != "cloud":
            print(f"{c.topic}: {b['n']} mensajes, {b['hz']:.2f} Hz. "
                  f"Sin campos deserializables.")
        elif c.kind == "imu":
            A.report_imu(c, b, A.imu_stats(c))
        elif c.kind == "odom":
            A.report_odom(c, b)
        elif c.kind == "car":
            A.report_car(c, b)
        elif c.kind == "pose":
            A.report_pose(c, b)
        elif c.kind == "twist":
            A.report_twist(c, b)
        elif c.kind == "fix":
            A.report_fix(c, b)
        elif c.kind == "cloud":
            A.report_cloud(c, b)
        elif c.kind == "scan":
            A.report_scan(c, b)
        elif c.kind == "image":
            A.report_image(c, b)
        elif c.kind == "range":
            A.report_range(c, b)
        elif c.kind == "battery":
            A.report_battery(c, b)
        elif c.kind == "joint":
            A.report_joint(c, b)
        elif c.kind == "diag":
            A.report_diag(c, b)
        elif c.kind == "clock":
            A.report_clock(c, b)
        elif c.kind == "scalar":
            A.report_scalar(c, b)
        elif c.kind == "posearray":
            A.report_posearray(c, b)
        elif c.kind == "path":
            A.report_path(c, b)
        elif c.kind == "tf":
            A.report_tf(c)
        elif c.kind == "log":
            A.report_log(c, 60)
        else:
            A.report_generic(c, b)

    # ---- IMU -------------------------------------------------------------- #

    def imu_dashboard(self) -> None:
        for n in self.pick("imu", multi=True):
            def build(n=n):
                c, b = self.view(n)
                return A.fig_imu(c, b, A.imu_stats(c), self.t0)
            self.show(build, f"IMU — {n}")

    def imu_channels(self) -> None:
        for n in self.pick("imu"):
            dlg = CheckDialog(self, f"Canales de {n}", IMU_GROUPS)
            if not dlg.result:
                return
            keys = dlg.result

            def build(n=n, keys=keys):
                c, b = self.view(n)
                return fig_imu_channels(c, b, self.t0, keys)
            self.show(build, f"IMU canales — {n}")

    def imu_3d(self) -> None:
        names = self.pick("imu", multi=True)
        if not names:
            return

        def build():
            imus, stats = {}, {}
            for n in names:
                c, _ = self.view(n)
                imus[n], stats[n] = c, A.imu_stats(c)
            return A.fig_imu3d(imus, stats)
        self.show(build, "IMU — gravedad 3D")

    def compare_imus(self) -> None:
        if not self._need_bag():
            return
        cands = [n for n, c in self.col.items() if c.kind == "imu" and has_data(c)]
        if len(cands) < 2:
            messagebox.showinfo("Comparar IMU", "Hacen falta al menos dos topics IMU.")
            return
        sel = [n for n in self.tree.selection() if n in cands]
        names = sel if len(sel) >= 2 else ListDialog(
            self, "Elige dos o más IMU", cands, multi=True).result
        if len(names) < 2:
            return
        self.config(cursor="watch")
        self.status.set("Comparando IMU…")
        self.update_idletasks()
        try:
            imus, stats = {}, {}
            for n in names:
                c, _ = self.view(n)
                imus[n], stats[n] = c, A.imu_stats(c)
            rot: Dict = {}
            notes, text = self.captured(A.report_imu_compare, imus, stats, rot)
            set_text(self.txt_topic, text)
            self.nb.select(0)
            PlotWindow(self, A.fig_imu_compare(imus, self.t0, notes), "Comparación de IMU")
            PlotWindow(self, A.fig_imu3d(imus, stats, rot or None), "IMU — 3D")
            self.status.set("Listo · comparación de IMU")
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            messagebox.showerror("Error", f"{type(exc).__name__}: {exc}")
        finally:
            self.config(cursor="")

    # ---- regrabacion ------------------------------------------------------ #

    def goto_record(self) -> None:
        if not self._need_bag():
            return
        self.rec_panel.set_bag(self.path)
        self.rec_panel._from_tree()
        self.nb.select(self.rec_panel)
        self.status.set("Elige el nodo y pulsa «Reproducir y grabar».")

    def _on_rec_progress(self, pct) -> None:
        base = "ROS 2 bag analyzer"
        self.title(base if pct is None else f"{base} — regrabando {pct:.0f} %")

    def _on_close(self) -> None:
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:  # noqa: BLE001
                pass
            self._poll_id = None
        try:
            self.settings.update(mode=self.mode, last_dir=self.last_dir,
                                 only_nav=bool(self.only_nav.get()),
                                 record=self.rec_panel.get_settings())
            save_settings(self.settings)
            self.rec_panel.shutdown()
        finally:
            self.destroy()

    # ---- GPS -------------------------------------------------------------- #

    def gps_dashboard(self) -> None:
        for n in self.pick("fix", multi=True):
            def build(n=n):
                c, b = self.view(n)
                return A.fig_fix(c, b, self.t0)
            self.show(build, f"GPS — {n}")

    def gps_channels(self) -> None:
        for n in self.pick("fix"):
            dlg = CheckDialog(self, f"Canales de {n}", GPS_GROUPS)
            if not dlg.result:
                return
            keys = dlg.result

            def build(n=n, keys=keys):
                c, b = self.view(n)
                return fig_gps_channels(c, b, self.t0, keys)
            self.show(build, f"GPS canales — {n}")

    def gps_3d(self) -> None:
        for n in self.pick("fix", multi=True):
            def build(n=n):
                c, b = self.view(n)
                return A.fig_fix3d(c, b, self.t0)
            self.show(build, f"GPS 3D — {n}")

    # ---- topic seleccionado / global ------------------------------------- #

    def _selected(self) -> List[str]:
        if not self._need_bag():
            return []
        sel = list(self.tree.selection())
        if not sel:
            messagebox.showinfo("Selecciona un topic", "Elige uno o más topics de la lista.")
        return sel

    def plot_selected(self) -> None:
        for n in self._selected():
            def build(n=n):
                c, b = self.view(n)
                if c.kind == "imu" and has_data(c):
                    return A.fig_imu(c, b, A.imu_stats(c), self.t0)
                if c.kind == "odom" and has_data(c):
                    cons, _ = self.captured(A.report_odom, c, b)
                    return A.fig_odom(c, b, self.t0, cons)
                if c.kind == "car" and has_data(c):
                    return A.fig_car(c, b, self.t0)
                if c.kind == "pose" and has_data(c):
                    return A.fig_pose(c, b, self.t0)
                if c.kind == "twist" and has_data(c):
                    return A.fig_twist(c, b, self.t0)
                if c.kind == "fix" and has_data(c):
                    return A.fig_fix(c, b, self.t0)
                if c.kind == "cloud":
                    return A.fig_cloud(c, b, self.t0)
                if c.kind == "scan":
                    return A.fig_scan(c, b, self.t0)
                if c.kind == "image":
                    return A.fig_image(c, b, self.t0)
                if c.kind == "range":
                    return A.fig_range(c, b, self.t0)
                if c.kind == "battery":
                    return A.fig_battery(c, b, self.t0)
                if c.kind == "joint":
                    return A.fig_joint(c, b, self.t0)
                if c.kind == "diag":
                    return A.fig_diag(c, b, self.t0)
                if c.kind == "clock":
                    return A.fig_clock(c, b, self.t0)
                if c.kind == "scalar":
                    return A.fig_scalar(c, b, self.t0)
                if c.kind == "posearray":
                    return A.fig_posearray(c, b, self.t0)
                if c.kind == "path":
                    return A.fig_path(c, b, self.t0)
                if c.kind == "tf":
                    return A.fig_tf3d(c)
                if c.kind == "log":
                    raise ValueError("/rosout no tiene gráfica; mira el resumen general.")
                if getattr(c, "data", None):
                    return A.fig_generic(c, b, self.t0)
                return None
            self.show(build, n)

    def plot_3d_selected(self) -> None:
        for n in self._selected():
            def build(n=n):
                c, b = self.view(n)
                if c.kind == "odom" and has_data(c):
                    return A.fig_traj3d(c, b, self.t0)
                if c.kind in ("car", "pose") and has_data(c):
                    return A.fig_traj3d(c, b, self.t0)
                if c.kind == "fix" and has_data(c):
                    return A.fig_fix3d(c, b, self.t0)
                if c.kind == "cloud":
                    return A.fig_cloud3d(c, b)
                if c.kind == "tf":
                    return A.fig_tf3d(c)
                if c.kind == "imu" and has_data(c):
                    return A.fig_imu3d({n: c}, {n: A.imu_stats(c)})
                raise ValueError(f"No hay vista 3D para {c.type}.")
            self.show(build, f"3D — {n}")

    def plot_timing(self) -> None:
        for n in self._selected():
            def build(n=n):
                c, _ = self.view(n)
                return A.fig_timing(c, self.t0)
            self.show(build, f"Temporización — {n}")

    def plot_global(self) -> None:
        if not self._need_bag():
            return
        self.show(lambda: A.fig_global(self.col, self.st, self.t0),
                  "Tasas y ancho de banda")

    def export_csv(self) -> None:
        sel = self._selected()
        if not sel:
            return
        prefix = filedialog.asksaveasfilename(
            title="Prefijo del CSV (se añade _topic.csv)", initialfile="bag",
            initialdir=self.last_dir)
        if not prefix:
            return
        try:
            out = ""
            for n in sel:
                c, _ = self.view(n)
                _, text = self.captured(A.export_csv, c, prefix)
                out += text
            messagebox.showinfo("CSV exportado", out.strip() or
                                "Esos topics no tienen series numéricas exportables.")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Error", f"{type(exc).__name__}: {exc}")


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else None
    App(path).mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())