#!/usr/bin/env python3
"""
bag_replay.py — Reproduce un bag, arranca un nodo y compara en vivo el estado.

    source /opt/ros/jazzy/setup.bash
    source ~/mirena_ws/install/setup.bash
    python3 bag_replay.py /ruta/al/bag

Arranca el nodo (por defecto `ros2 run mirena_state mirena_state`), espera a
que se suscriba y lanza `ros2 bag play`. Mientras se reproduce, dibuja en una
ventana el estado que sale SOLO del sensor y el que sale del filtro de Kalman:

  * trayectoria XY superpuesta,
  * x, y, yaw, velocidad y velocidad angular frente al tiempo,
  * error de posicion instantaneo,
  * covarianza del EKF (elipse 2 sigma) en la posicion actual.

El sensor se alinea con el primer estado del filtro con una rototraslacion
rigida; a partir de ahi la deriva que se ve es la que corrige el EKF.

Ejemplos:
    # bag con /sensors/imu/odom y /state/car
    python3 bag_replay.py bag/ --rate 1.0

    # tambien acepta un fichero .mcap suelto
    python3 bag_replay.py grabacion.mcap

    # otro nodo y otros topics
    python3 bag_replay.py bag/ --node-cmd "ros2 run mi_pkg mi_nodo" \
        --raw /sensors/imu/odom --filtered /state/car

    # sin nodo (el bag ya trae /state/car), o sin reproducir (ya esta corriendo)
    python3 bag_replay.py bag/ --no-node
    python3 bag_replay.py bag/ --no-play

    # guardar una captura al terminar
    python3 bag_replay.py bag/ --save /tmp/kalman.png

Notas:
  * `--rate 0.5` reproduce a media velocidad (util para ver la grafica).
  * `--delay` son los segundos que se espera al nodo antes de darle datos.
  * Ctrl-C para terminar; se cierran el bag y el nodo.

Tambien se puede importar desde rosbag_gui.py, que incrusta este dashboard
en una pestana de la interfaz en lugar de abrir otra ventana.

Requiere rclpy, rosbag2 y matplotlib. El nodo debe publicar
`mirena_common/msg/Car` o `nav_msgs/msg/Odometry` en el topic filtrado.
"""

from __future__ import annotations

import argparse
import math
import os
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

SURFACE, INK, INK_SOFT, INK_MUTED, GRID = (
    "#fcfcfb", "#0b0b0b", "#52514e", "#8a8880", "#e4e3df")
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"

DEFAULT_NODE = "ros2 run mirena_state mirena_state"
DEFAULT_RAW = "/sensors/imu/odom"
DEFAULT_FILTERED = "/state/car"


def bag_target(path: str):
    """Devuelve (uri, storage_id) que entiende `ros2 bag play`.

    Acepta la carpeta del bag o un fichero .mcap / .db3 suelto. Los plugins
    de almacenamiento abren el fichero directamente si se les da su id.
    """
    path = os.path.expanduser(path)
    if os.path.isdir(path):
        return path, None
    low = path.lower()
    if low.endswith(".mcap"):
        return path, "mcap"
    if low.endswith(".db3"):
        return path, "sqlite3"
    return path, None


# --------------------------------------------------------------------------- #
#  Almacen de datos en vivo
# --------------------------------------------------------------------------- #

def stamp_of(msg) -> Optional[float]:
    try:
        st = msg.header.stamp
        t = int(st.sec) + int(st.nanosec) * 1e-9
        return t if t > 0 else None
    except AttributeError:
        return None


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class Samples:
    t: List[float] = field(default_factory=list)
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    yaw: List[float] = field(default_factory=list)
    u: List[float] = field(default_factory=list)
    v: List[float] = field(default_factory=list)
    w: List[float] = field(default_factory=list)
    sx: List[float] = field(default_factory=list)
    sy: List[float] = field(default_factory=list)
    syaw: List[float] = field(default_factory=list)

    def add(self, t, x, y, yaw, u, v, w, cov=None):
        self.t.append(float(t))
        self.x.append(float(x))
        self.y.append(float(y))
        self.yaw.append(float(yaw))
        self.u.append(float(u))
        self.v.append(float(v))
        self.w.append(float(w))
        if cov is None:
            self.sx.append(float("nan"))
            self.sy.append(float("nan"))
            self.syaw.append(float("nan"))
        else:
            self.sx.append(float(math.sqrt(max(cov[0], 0.0))))
            self.sy.append(float(math.sqrt(max(cov[1], 0.0))))
            self.syaw.append(float(math.sqrt(max(cov[2], 0.0))))

    def __len__(self) -> int:
        return len(self.t)

    def arrays(self) -> Dict[str, np.ndarray]:
        return {k: np.asarray(v, float) for k, v in vars(self).items()}


class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw = Samples()
        self.filt = Samples()
        self.t0: Optional[float] = None
        self.started = time.monotonic()
        self.last_msg = time.monotonic()

    def _bump(self, t):
        # Un salto hacia atras grande (p. ej. --loop) reinicia las series para
        # que la trayectoria no se dibuje como un zigzag entre vueltas.
        last = None
        if self.raw.t:
            last = self.raw.t[-1]
        if self.filt.t:
            last = self.filt.t[-1] if last is None else max(last, self.filt.t[-1])
        if last is not None and t < last - 1.0:
            self.raw = Samples()
            self.filt = Samples()
            self.t0 = t
        if self.t0 is None:
            self.t0 = t
        self.last_msg = time.monotonic()

    def add_raw(self, t, *a, **kw):
        with self.lock:
            self._bump(t)
            self.raw.add(t, *a, **kw)

    def add_filt(self, t, *a, **kw):
        with self.lock:
            self._bump(t)
            self.filt.add(t, *a, **kw)

    def snapshot(self):
        with self.lock:
            return (self.raw.arrays(), self.filt.arrays(), self.t0,
                    time.monotonic() - self.last_msg)


# --------------------------------------------------------------------------- #
#  Nodo ROS 2
# --------------------------------------------------------------------------- #

class LiveNode:
    """Suscripciones minimas: odometria cruda y estado filtrado."""

    def __init__(self, store: Store, raw_topic: str, filt_topic: str):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy)
        from nav_msgs.msg import Odometry

        self.rclpy = rclpy
        self.store = store
        self.node = Node("bag_replay_monitor")
        qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)

        def on_odom(msg):
            t = stamp_of(msg) or self.node.get_clock().now().nanoseconds * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            tw = msg.twist.twist
            cov = np.asarray(msg.pose.covariance, float)
            self.store.add_raw(t, p.x, p.y, yaw_from_quat(q),
                               tw.linear.x, tw.linear.y, tw.angular.z,
                               cov[[0, 7, 35]] if cov.size >= 36 else None)

        self.node.create_subscription(Odometry, raw_topic, on_odom, qos)

        try:
            from mirena_common.msg import Car

            def on_car_raw(msg):
                t = stamp_of(msg) or self.node.get_clock().now().nanoseconds * 1e-9
                cov = np.asarray(msg.covariance, float)
                self.store.add_raw(t, msg.x, msg.y, msg.psi, msg.u, msg.v,
                                   msg.omega, cov[[0, 7, 35]]
                                   if cov.size >= 36 else None)

            self.node.create_subscription(Car, raw_topic, on_car_raw, qos)
        except Exception:  # noqa: BLE001
            pass

        # El estado filtrado puede ser mirena_common/Car o nav_msgs/Odometry.
        # Dos suscripciones sobre el mismo topic: solo la que coincide recibe.
        def on_car(msg):
            t = stamp_of(msg) or self.node.get_clock().now().nanoseconds * 1e-9
            cov = np.asarray(msg.covariance, float)
            self.store.add_filt(t, msg.x, msg.y, msg.psi, msg.u, msg.v,
                                msg.omega, cov[[0, 7, 35]]
                                if cov.size >= 36 else None)

        def on_odom_state(msg):
            t = stamp_of(msg) or self.node.get_clock().now().nanoseconds * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            tw = msg.twist.twist
            cov = np.asarray(msg.pose.covariance, float)
            self.store.add_filt(t, p.x, p.y, yaw_from_quat(q),
                                tw.linear.x, tw.linear.y, tw.angular.z,
                                cov[[0, 7, 35]] if cov.size >= 36 else None)

        try:
            from mirena_common.msg import Car
            self.node.create_subscription(Car, filt_topic, on_car, qos)
        except Exception:  # noqa: BLE001
            pass
        self.node.create_subscription(Odometry, filt_topic, on_odom_state, qos)

    def spin(self, stop: threading.Event):
        while not stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_node()


# --------------------------------------------------------------------------- #
#  Procesos externos
# --------------------------------------------------------------------------- #

class Pump(threading.Thread):
    """Lee la salida de un subproceso y la imprime con prefijo."""

    def __init__(self, proc: subprocess.Popen, prefix: str):
        super().__init__(daemon=True)
        self.proc, self.prefix = proc, prefix

    def run(self):
        if self.proc.stdout is None:
            return
        for line in self.proc.stdout:
            print(f"[{self.prefix}] {line.rstrip()}")
        self.proc.stdout.close()


def spawn(cmd: List[str], name: str) -> Optional[subprocess.Popen]:
    print(f"+ {shlex.join(cmd)}")
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1)
    except FileNotFoundError as exc:
        print(f"  no pude arrancar {name}: {exc}", file=sys.stderr)
        return None
    Pump(p, name).start()
    return p


def stop_proc(p: Optional[subprocess.Popen], name: str) -> None:
    if p is None or p.poll() is not None:
        return
    try:
        p.terminate()
        p.wait(timeout=4.0)
    except subprocess.TimeoutExpired:
        print(f"  {name} no termina; lo mato", file=sys.stderr)
        p.kill()
    except Exception:  # noqa: BLE001
        p.kill()


# --------------------------------------------------------------------------- #
#  Dibujo en vivo
# --------------------------------------------------------------------------- #

def setup_style() -> None:
    import matplotlib
    if not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE, "axes.edgecolor": GRID,
        "axes.labelcolor": INK_SOFT, "axes.titlecolor": INK,
        "axes.titlesize": 10.5, "axes.titleweight": "semibold",
        "axes.labelsize": 9, "axes.grid": True, "axes.axisbelow": True,
        "axes.spines.top": False, "axes.spines.right": False,
        "grid.color": GRID, "grid.linewidth": 0.8,
        "xtick.color": INK_MUTED, "ytick.color": INK_MUTED,
        "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
        "legend.frameon": False, "legend.fontsize": 8.5,
        "legend.labelcolor": INK_SOFT, "figure.titlesize": 12,
        "figure.titleweight": "semibold", "lines.linewidth": 1.0,
        "font.size": 9, "axes.formatter.useoffset": False,
    })


def align_first(raw: Dict[str, np.ndarray], filt: Dict[str, np.ndarray],
                enabled: bool = True):
    """Devuelve la posicion del sensor llevada al frame del primer EKF."""
    if not enabled or raw["t"].size == 0 or filt["t"].size == 0:
        return raw["x"], raw["y"], raw["yaw"]
    t0 = max(raw["t"][0], filt["t"][0])
    if t0 > min(raw["t"][-1], filt["t"][-1]):
        return raw["x"], raw["y"], raw["yaw"]
    rx0 = np.interp(t0, raw["t"], raw["x"])
    ry0 = np.interp(t0, raw["t"], raw["y"])
    ryaw0 = np.interp(t0, raw["t"], raw["yaw"])
    fx0 = np.interp(t0, filt["t"], filt["x"])
    fy0 = np.interp(t0, filt["t"], filt["y"])
    fyaw0 = np.interp(t0, filt["t"], filt["yaw"])
    dy = fyaw0 - ryaw0
    c, s = math.cos(dy), math.sin(dy)
    dx, dy_ = raw["x"] - rx0, raw["y"] - ry0
    return (c * dx - s * dy_ + fx0,
            s * dx + c * dy_ + fy0,
            raw["yaw"] + dy)


def thin(t, y, n=4000):
    if t.size <= n:
        return t, y
    i = np.linspace(0, t.size - 1, n).astype(int)
    return t[i], y[i]


class Dashboard:
    def __init__(self, raw_topic: str, filt_topic: str, align: bool = True):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.align = align
        self.raw_label, self.filt_label = raw_topic, filt_topic
        self.fig = plt.figure(figsize=(14.0, 9.0), layout="constrained")
        gs = self.fig.add_gridspec(3, 3, height_ratios=[2.3, 1.3, 1.3])
        self.ax_xy = self.fig.add_subplot(gs[0, 0])
        self.ax_err = self.fig.add_subplot(gs[0, 1:])
        self.ax_pos = self.fig.add_subplot(gs[1, :])
        self.ax_yaw = self.fig.add_subplot(gs[2, 0])
        self.ax_vel = self.fig.add_subplot(gs[2, 1])
        self.ax_om = self.fig.add_subplot(gs[2, 2])
        self.ax_xy.set_aspect("equal", adjustable="datalim")
        self.ax_xy.set_xlabel("x [m]")
        self.ax_xy.set_ylabel("y [m]")
        self.ax_xy.set_title("Trayectoria: sensor solo vs EKF", loc="left", pad=6)
        self.ax_err.set_xlabel("tiempo [s]")
        self.ax_err.set_ylabel("m")
        self.ax_err.set_title("Error de posición EKF − sensor", loc="left", pad=6)
        self.ax_pos.set_xlabel("tiempo [s]")
        self.ax_pos.set_ylabel("m")
        self.ax_pos.set_title("Posición", loc="left", pad=6)
        self.ax_yaw.set_xlabel("tiempo [s]")
        self.ax_yaw.set_ylabel("grados")
        self.ax_yaw.set_title("Yaw", loc="left", pad=6)
        self.ax_vel.set_xlabel("tiempo [s]")
        self.ax_vel.set_ylabel("m/s")
        self.ax_vel.set_title("Velocidad", loc="left", pad=6)
        self.ax_om.set_xlabel("tiempo [s]")
        self.ax_om.set_ylabel("grados/s")
        self.ax_om.set_title("Velocidad angular", loc="left", pad=6)

        self.l_raw, = self.ax_xy.plot([], [], color=ORANGE, ls="--", lw=1.1,
                                      label="sensor solo")
        self.l_filt, = self.ax_xy.plot([], [], color=BLUE, lw=1.2, label="EKF")
        self.p_raw, = self.ax_xy.plot([], [], "o", color=ORANGE, ms=4)
        self.p_filt, = self.ax_xy.plot([], [], "o", color=BLUE, ms=4)
        self.ell = None
        self.ax_xy.legend(loc="best")

        self.l_ex, = self.ax_err.plot([], [], color=ORANGE, lw=1.0,
                                      label="|EKF − sensor|")
        self.ax_err.legend(loc="upper right")
        self.txt_err = self.ax_err.text(0.01, 0.95, "", transform=self.ax_err.transAxes,
                                        va="top", fontsize=8.5, color=INK_SOFT)

        self.l_x, = self.ax_pos.plot([], [], color=BLUE, label="EKF x")
        self.l_y, = self.ax_pos.plot([], [], color=BLUE, ls="--", label="EKF y")
        self.l_rx, = self.ax_pos.plot([], [], color=ORANGE, label="sensor x")
        self.l_ry, = self.ax_pos.plot([], [], color=ORANGE, ls="--", label="sensor y")
        self.ax_pos.legend(loc="upper left", ncol=4)

        self.l_yr, = self.ax_yaw.plot([], [], color=ORANGE, label="sensor")
        self.l_yf, = self.ax_yaw.plot([], [], color=BLUE, label="EKF")
        self.ax_yaw.legend(loc="upper left")

        self.l_vu, = self.ax_vel.plot([], [], color=BLUE, label="EKF |v|")
        self.l_vr, = self.ax_vel.plot([], [], color=ORANGE, ls="--", label="sensor |v|")
        self.ax_vel.legend(loc="upper left")

        self.l_w, = self.ax_om.plot([], [], color=BLUE, label="EKF ω")
        self.l_wr, = self.ax_om.plot([], [], color=ORANGE, ls="--", label="sensor ω")
        self.ax_om.legend(loc="upper left")

        self.title = self.fig.suptitle(
            f"sensor: {raw_topic}    ·    EKF: {filt_topic}", x=0.01, ha="left")
        self._last_draw = 0.0

    def update(self, raw, filt, t0, idle, force=False):
        if raw["t"].size == 0 and filt["t"].size == 0:
            self.fig.suptitle("esperando mensajes…", x=0.01, ha="left")
            return
        now = time.monotonic()
        if not force and now - self._last_draw < 0.06:
            return
        self._last_draw = now
        tr = raw["t"] - (t0 or 0.0)
        tf = filt["t"] - (t0 or 0.0)
        rx, ry, ryaw = align_first(raw, filt, self.align)

        # Yaw desenrollado de cada fuente y, si hace falta, corregido en
        # multiplos de 360 para que arranquen en la misma vuelta.
        ryaw_u = np.degrees(np.unwrap(ryaw)) if tr.size else np.zeros(0)
        fyaw_u = np.degrees(np.unwrap(filt["yaw"])) if tf.size else np.zeros(0)
        if tr.size and tf.size:
            lo = max(tr[0], tf[0])
            if lo <= min(tr[-1], tf[-1]):
                a = np.interp(lo, tr, ryaw_u)
                b = np.interp(lo, tf, fyaw_u)
                ryaw_u = ryaw_u + 360.0 * round((b - a) / 360.0)

        # trayectoria (con muestreo para bags largos)
        if tr.size:
            i = np.linspace(0, tr.size - 1, min(tr.size, 5000)).astype(int)
            self.l_raw.set_data(rx[i], ry[i])
            self.p_raw.set_data([rx[-1]], [ry[-1]])
        if tf.size:
            i = np.linspace(0, tf.size - 1, min(tf.size, 5000)).astype(int)
            self.l_filt.set_data(filt["x"][i], filt["y"][i])
            self.p_filt.set_data([filt["x"][-1]], [filt["y"][-1]])

        # error sobre rejilla comun
        err = None
        if tr.size > 1 and tf.size > 1:
            lo = max(tr[0], tf[0])
            hi = min(tr[-1], tf[-1])
            if hi > lo:
                g = np.linspace(lo, hi, max(2, int((hi - lo) * 20)))
                cx = np.interp(g, tf, filt["x"])
                cy = np.interp(g, tf, filt["y"])
                rxi = np.interp(g, tr, rx)
                ryi = np.interp(g, tr, ry)
                err = np.hypot(cx - rxi, cy - ryi)
                # error yaw con signo envuelto
                ryaw_i = np.interp(g, tr, ryaw)
                fyaw = np.interp(g, tf, filt["yaw"])
                ye = (np.degrees(fyaw - ryaw_i) + 180.0) % 360.0 - 180.0
                self.l_ex.set_data(g, err)
                self.ax_err.set_xlim(max(0.0, g[0] - 1.0), max(5.0, g[-1] + 1.0))
                self.ax_err.set_ylim(0.0, max(0.5, float(err.max()) * 1.15))
                self.txt_err.set_text(
                    f"RMS {np.sqrt(np.mean(err ** 2)):.2f} m   "
                    f"max {err.max():.2f} m   "
                    f"actual {err[-1]:.2f} m\n"
                    f"yaw RMS {np.sqrt(np.mean(ye ** 2)):.1f}°   "
                    f"actual {ye[-1]:+.1f}°")

        if tf.size:
            tx, vx = thin(tf, filt["x"])
            ty, vy = thin(tf, filt["y"])
            _, vyaw = thin(tf, fyaw_u)
            _, vv = thin(tf, np.hypot(filt["u"], filt["v"]))
            _, vw = thin(tf, np.degrees(filt["w"]))
            self.l_x.set_data(tx, vx)
            self.l_y.set_data(tx, vy)
            self.l_yf.set_data(tx, vyaw)
            self.l_vu.set_data(tx, vv)
            self.l_w.set_data(tx, vw)
        if tr.size:
            tx, vx = thin(tr, rx)
            ty, vy = thin(tr, ry)
            _, vyaw = thin(tr, ryaw_u)
            _, vv = thin(tr, np.hypot(raw["u"], raw["v"]))
            _, vw = thin(tr, np.degrees(raw["w"]))
            self.l_rx.set_data(tx, vx)
            self.l_ry.set_data(ty, vy)
            self.l_yr.set_data(tx, vyaw)
            self.l_vr.set_data(tx, vv)
            self.l_wr.set_data(tx, vw)

        # limites temporales
        hi = max(tr[-1] if tr.size else 0.0, tf[-1] if tf.size else 0.0)
        lo = min(tr[0] if tr.size else 0.0, tf[0] if tf.size else 0.0)
        for ax in (self.ax_pos, self.ax_yaw, self.ax_vel, self.ax_om):
            ax.set_xlim(max(0.0, lo), max(5.0, hi + 1.0))

        # elipse 2 sigma de la posicion actual del EKF
        if tf.size and filt["sx"].size and np.isfinite(filt["sx"][-1]):
            if self.ell is not None:
                self.ell.remove()
            from matplotlib.patches import Ellipse
            self.ell = Ellipse((filt["x"][-1], filt["y"][-1]),
                               width=4 * filt["sx"][-1],
                               height=4 * filt["sy"][-1],
                               angle=math.degrees(filt["yaw"][-1]),
                               facecolor=BLUE, alpha=0.12, edgecolor=BLUE,
                               lw=0.8, zorder=0)
            self.ax_xy.add_patch(self.ell)
        self.ax_xy.relim()
        self.ax_xy.autoscale_view()
        # un poco de aire para que la elipse no quede cortada
        for ax in (self.ax_xy,):
            ax.margins(0.08)

        status = "reproduciendo" if idle < 2.0 else "en espera"
        nr, nf = raw["t"].size, filt["t"].size
        self.title.set_text(
            f"sensor: {self.raw_label} ({nr})    ·    "
            f"EKF: {self.filt_label} ({nf})    ·    {status}")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def save_dashboard(dash: Dashboard, path: str) -> None:
    try:
        dash.fig.savefig(path, dpi=140, bbox_inches="tight")
        print(f"  guardado {path}")
    except Exception as exc:  # noqa: BLE001
        print(f"  no pude guardar {path}: {exc}", file=sys.stderr)


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Reproduce un bag, arranca un nodo y compara sensor vs EKF.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("bag", help="carpeta del bag (con metadata.yaml) o fichero .mcap")
    p.add_argument("--node-cmd", default=DEFAULT_NODE,
                   help=f"comando del nodo (def. '{DEFAULT_NODE}')")
    p.add_argument("--no-node", action="store_true",
                   help="no arranca ningún nodo")
    p.add_argument("--raw", default=DEFAULT_RAW,
                   help=f"topic del sensor solo (def. {DEFAULT_RAW})")
    p.add_argument("--filtered", default=DEFAULT_FILTERED,
                   help=f"topic del estado filtrado (def. {DEFAULT_FILTERED})")
    p.add_argument("--rate", type=float, default=1.0,
                   help="velocidad de reproducción (def. 1.0)")
    p.add_argument("--start", type=float, default=None,
                   help="segundos desde el inicio del bag")
    p.add_argument("--duration", type=float, default=None,
                   help="segundos a reproducir")
    p.add_argument("--delay", type=float, default=2.0,
                   help="espera antes de reproducir, para que el nodo arranque")
    p.add_argument("--loop", action="store_true", help="reproducir en bucle")
    p.add_argument("--no-play", action="store_true",
                   help="no lanza ros2 bag play (solo escucha)")
    p.add_argument("--no-align", action="store_true",
                   help="no alinear el sensor con el primer estado del EKF")
    p.add_argument("--save", default="", metavar="PNG",
                   help="guarda el dashboard al terminar")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if not os.path.exists(args.bag):
        raise SystemExit(f"\nNo existe el bag: {args.bag}\n")
    bag_uri, bag_storage = bag_target(args.bag)
    if not os.path.exists(bag_uri):
        raise SystemExit(f"\nNo existe la ruta del bag: {bag_uri}\n")

    try:
        import rclpy
    except ImportError as exc:
        raise SystemExit("\nNo encuentro rclpy. Sourcea ROS 2:\n"
                         "    source /opt/ros/jazzy/setup.bash\n"
                         "    source ~/mirena_ws/install/setup.bash\n") from exc

    setup_style()
    import matplotlib.pyplot as plt

    store = Store()
    rclpy.init()
    live = LiveNode(store, args.raw, args.filtered)
    dash = Dashboard(args.raw, args.filtered, align=not args.no_align)

    stop = threading.Event()
    spin = threading.Thread(target=live.spin, args=(stop,), daemon=True)
    spin.start()

    node_proc = None
    play_proc = None
    play_cmd = []
    if not args.no_node:
        cmd = shlex.split(args.node_cmd)
        node_proc = spawn(cmd, "nodo")
        if node_proc is None:
            stop.set()
            rclpy.shutdown()
            return 1
        if args.delay > 0:
            print(f"  esperando {args.delay:.1f} s al nodo…")
            time.sleep(args.delay)

    if not args.no_play:
        play_cmd = ["ros2", "bag", "play", bag_uri, "--rate", str(args.rate)]
        if bag_storage:
            play_cmd += ["--storage", bag_storage]
        if args.start is not None:
            play_cmd += ["--start-offset", str(args.start)]
        if args.duration is not None:
            play_cmd += ["--playback-duration", str(args.duration)]
        if args.loop:
            play_cmd += ["--loop"]
        play_proc = spawn(play_cmd, "bag")
        if play_proc is None:
            stop_proc(node_proc, "nodo")
            stop.set()
            rclpy.shutdown()
            return 1

    print("\n  Ctrl-C para terminar.")
    interrupted = False
    try:
        while not stop.is_set():
            raw, filt, t0, idle = store.snapshot()
            dash.update(raw, filt, t0, idle)
            # termina solo cuando el bag ya acabo y no llegan mensajes
            if play_proc is not None and play_proc.poll() is not None and \
                    idle > 2.0 and not args.loop:
                break
            if args.no_play and not args.no_node and idle > 30.0:
                print("  30 s sin mensajes; termino.")
                break
            plt.pause(0.05)
    except KeyboardInterrupt:
        interrupted = True
        print("\n  interrumpido")
    finally:
        stop.set()
        stop_proc(play_proc, "bag")
        stop_proc(node_proc, "nodo")
        spin.join(timeout=2.0)
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    raw, filt, t0, _ = store.snapshot()
    if args.save:
        dash.update(raw, filt, t0, 0.0, force=True)
        save_dashboard(dash, args.save)
    print(f"\n  sensor: {raw['t'].size} muestras   EKF: {filt['t'].size} muestras")
    if raw["t"].size > 1 and filt["t"].size > 1:
        rx, ry, ryaw = align_first(raw, filt, not args.no_align)
        lo = max(raw["t"][0], filt["t"][0])
        hi = min(raw["t"][-1], filt["t"][-1])
        if hi > lo:
            g = np.linspace(lo, hi, max(2, int((hi - lo) * 20)))
            cx = np.interp(g, filt["t"], filt["x"])
            cy = np.interp(g, filt["t"], filt["y"])
            rxi = np.interp(g, raw["t"], rx)
            ryi = np.interp(g, raw["t"], ry)
            err = np.hypot(cx - rxi, cy - ryi)
            print(f"  error de posición: RMS {np.sqrt(np.mean(err**2)):.3f} m   "
                  f"max {err.max():.3f} m   final {err[-1]:.3f} m")
            fy = np.interp(g, filt["t"], filt["yaw"])
            ryr = np.interp(g, raw["t"], ryaw)
            ye = (np.degrees(fy - ryr) + 180.0) % 360.0 - 180.0
            print(f"  error de yaw:      RMS {np.sqrt(np.mean(ye**2)):.3f} deg   "
                  f"max {np.abs(ye).max():.3f} deg")
    if not interrupted and not args.no_play:
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
