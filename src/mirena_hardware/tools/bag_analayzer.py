
#!/usr/bin/env python3
"""
rosbag_analyzer.py — Analisis profundo de TODOS los topics de una rosbag ROS 2.

    source /opt/ros/jazzy/setup.bash
    source ~/ws/install/setup.bash          # para los mensajes propios
    python3 bag_analayzer.py /ruta/al/bag --save figs/
    python3 bag_analayzer.py /ruta/grabacion.mcap --save figs/

Una sola pasada del fichero. Cada topic recibe el mismo nivel de analisis que
la IMU, segun su tipo:

  Imu            ruido, bias, inclinacion de montaje desde la gravedad, RPY,
                 espectros contra la spec, temporizacion. Con dos o mas IMU:
                 desfase de relojes y ROTACION RELATIVA entre ellas por Kabsch.
  Odometry       trayectoria, velocidades, y CONSISTENCIA DE FRAMES: d(yaw)/dt
                 contra angular.z, y la direccion de (dx,dy) contra R(yaw)*v.
  Car            salida de mirena_state: trayectoria, covarianza y, si hay
                 odometria en el mismo bag, COMPARACION sensor solo vs EKF.
  Pose / Twist   PoseStamped, PoseWithCovariance, TwistStamped, TwistWithCov:
                 trayectoria, orientacion, velocidades y covarianzas.
  PoseArray      numero de poses, centroide y dispersion.
  Path           numero de poses y longitud del camino.
  NavSatFix      dispersion, CEP o recorrido, altitud, estado del fix.
  PointCloud2    tasa, tamano por nube, puntos y layout de campos, sin decodificar.
  LaserScan      puntos por barrido, haces validos, rango min/medio/max y
                 dibujo XY del ultimo barrido.
  Image          resolucion, encoding, bytes/frame, brillo y miniaturas
                 (CompressedImage se decodifica si OpenCV esta instalado).
  Range          distancia, limites y FOV.
  BatteryState   voltaje, corriente, carga, porcentaje, temperatura y celdas.
  JointState     posicion, velocidad y esfuerzo por joint.
  Diagnostic     estados OK/WARN/ERROR agrupados por nombre.
  Clock          avance del reloj, saltos y deriva frente al bag.
  Temperature, FluidPressure, MagneticField, Illuminance, RelativeHumidity
                 series escalares con sus unidades.
  TFMessage      arbol de transformadas con traslacion y RPY.
  Log            WARN/ERROR agrupados por nodo y mensaje.
  cualquier otro estadistica y series temporales de todos sus campos numericos
                 por introspeccion, sin conocer la definicion del mensaje.

Ademas, figuras globales: tasa de cada topic normalizada a su mediana (donde
se ve exactamente cuando y quien perdio mensajes) y reparto de ancho de banda.
Si el bag trae a la vez /state/car y la odometria del sensor, se genera una
figura de comparacion (EKF frente a sensor solo) con error de posicion y yaw.
Para reproducir el bag EN VIVO contra un nodo, usa bag_replay.py.

Opciones:
    --topic A [B ...]  solo esos topics
    --save DIR         guarda PNG (si no, abre ventanas)
    --start / --end    recorta a una ventana en segundos
    --sample N         tope de mensajes a deserializar por topic (def. 200000)
    --csv PREFIJO      exporta las series numericas a CSV
    --max-warn N       lineas de /rosout (def. 60)
    --no-figs          solo el informe de texto
    --quiet            solo la tabla general

Autocontenido: numpy, scipy, matplotlib y el entorno ROS 2.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# --------------------------------------------------------------------------- #
#  Tipos y constantes
# --------------------------------------------------------------------------- #

IMU_TYPE = "sensor_msgs/msg/Imu"
FIX_TYPE = "sensor_msgs/msg/NavSatFix"
ODOM_TYPE = "nav_msgs/msg/Odometry"
CLOUD_TYPE = "sensor_msgs/msg/PointCloud2"
SCAN_TYPE = "sensor_msgs/msg/LaserScan"
IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage"
RANGE_TYPE = "sensor_msgs/msg/Range"
BATTERY_TYPE = "sensor_msgs/msg/BatteryState"
JOINT_TYPE = "sensor_msgs/msg/JointState"
DIAG_TYPE = "diagnostic_msgs/msg/DiagnosticArray"
CLOCK_TYPE = "rosgraph_msgs/msg/Clock"
TEMPERATURE_TYPE = "sensor_msgs/msg/Temperature"
PRESSURE_TYPE = "sensor_msgs/msg/FluidPressure"
MAGNETIC_TYPE = "sensor_msgs/msg/MagneticField"
ILLUMINANCE_TYPE = "sensor_msgs/msg/Illuminance"
HUMIDITY_TYPE = "sensor_msgs/msg/RelativeHumidity"
CAR_TYPE = "mirena_common/msg/Car"
POSE_TYPES = ("geometry_msgs/msg/PoseStamped",
              "geometry_msgs/msg/PoseWithCovarianceStamped")
POSE_ARRAY_TYPE = "geometry_msgs/msg/PoseArray"
TWIST_TYPES = ("geometry_msgs/msg/TwistStamped",
               "geometry_msgs/msg/TwistWithCovarianceStamped")
PATH_TYPE = "nav_msgs/msg/Path"
TF_TYPES = ("tf2_msgs/msg/TFMessage",)
LOG_TYPE = "rcl_interfaces/msg/Log"
HEAVY_TYPES = (CLOUD_TYPE,)
HEAVY_SAMPLE = 25          # de las nubes basta con unas pocas
MEDIA_TYPES = (IMAGE_TYPE, COMPRESSED_TYPE)
MEDIA_SAMPLE = 80          # metadatos de imagen; solo se decodifican 6
MEDIA_THUMBS = 6

SEV = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# Paleta validada para daltonismo (CVD dE 9.2, vision normal 24.0, all-pairs).
AXIS_COLOR = {"x": "#2a78d6", "y": "#eb6834", "z": "#1baf7a"}
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
BLUE, ORANGE, AQUA = SERIES
BLUE_RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7",
             "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281",
             "#0d366b"]

SURFACE, INK, INK_SOFT, INK_MUTED, GRID = (
    "#fcfcfb", "#0b0b0b", "#52514e", "#8a8880", "#e4e3df")

G0 = 9.80665
SPEC_ACCEL_DENSITY = 0.14e-3 * G0       # VN-300: 0.14 mg/sqrt(Hz)
SPEC_GYRO_DENSITY = 0.0035              # VN-300: 0.0035 deg/s/sqrt(Hz)
STATIC_GYRO_DPS = 1.0
STATIC_ACC_TOL = 0.5


def slug(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", s).strip("_") or "topic"


def human(n: float) -> str:
    for u in ("B", "kB", "MB", "GB"):
        if n < 1024 or u == "GB":
            return f"{n:.1f} {u}"
        n /= 1024
    return f"{n:.1f} GB"


# --------------------------------------------------------------------------- #
#  Matematica
# --------------------------------------------------------------------------- #

def rpy_from_quat(q: np.ndarray) -> np.ndarray:
    """RPY intrinseco ZYX en radianes, desde cuaterniones (x, y, z, w)."""
    q = np.atleast_2d(q)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n == 0] = 1.0
    x, y, z, w = (q / n).T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def quat_to_matrix(q) -> np.ndarray:
    """Matriz de rotacion desde un cuaternion (x, y, z, w)."""
    x, y, z, w = np.asarray(q, float)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_from_matrix(R: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(R[2, 0]) < 0.99999:
        return np.array([np.arctan2(R[2, 1], R[2, 2]), pitch,
                         np.arctan2(R[1, 0], R[0, 0])])
    return np.array([np.arctan2(-R[1, 2], R[1, 1]), pitch, 0.0])


def circ_stats(err: np.ndarray) -> Tuple[float, float]:
    """
    Media y concentracion R de errores angulares (radianes).

    R cerca de 1 = errores concentrados; cerca de 0 = repartidos. Decidir por
    la media aritmetica es un error: una hipotesis falsa puede dar media ~0
    por puro reparto uniforme y colarse como buena.
    """
    if err.size == 0:
        return float("nan"), 0.0
    c, s = float(np.mean(np.cos(err))), float(np.mean(np.sin(err)))
    return math.degrees(math.atan2(s, c)), float(math.hypot(c, s))


def lla_to_ne(lat, lon) -> Tuple[np.ndarray, np.ndarray, float, float]:
    lat0, lon0 = float(np.mean(lat)), float(np.mean(lon))
    phi = math.radians(lat0)
    mlat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    mlon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return (lat - lat0) * mlat, (lon - lon0) * mlon, lat0, lon0


def smooth(v: np.ndarray, k: int) -> np.ndarray:
    k = max(1, int(k))
    if k <= 1 or v.size < 3 * k:
        return v
    return np.convolve(v, np.ones(k) / k, mode="same")


# --------------------------------------------------------------------------- #
#  Colectores: extraen a arrays sobre la marcha, sin guardar los mensajes.
#  Un bag con nubes de puntos no cabe en memoria si se guardan los objetos.
# --------------------------------------------------------------------------- #

def header_stamp(msg) -> Optional[float]:
    try:
        st = msg.header.stamp
        return int(st.sec) + int(st.nanosec) * 1e-9
    except AttributeError:
        return None


class Base:
    kind = "generic"

    def __init__(self, topic: str, ty: str):
        self.topic, self.type = topic, ty
        self.t_bag: List[float] = []
        self.t_msg: List[float] = []      # stamps de los mensajes deserializados
        self.t_hdr: List[float] = []
        self.sizes: List[int] = []

    def add_raw(self, nbytes: int, t_bag_ns: int) -> None:
        self.t_bag.append(t_bag_ns * 1e-9)
        self.sizes.append(nbytes)

    def add(self, msg) -> None:
        hs = header_stamp(msg)
        # La marca del mensaje real, no la de recepcion: si --sample corta la
        # deserializacion, t_bag tiene mas entradas que los arrays de datos y
        # todas las graficas se desplazan. Con t_msg las longitudes coinciden.
        self.t_msg.append(hs if hs is not None else
                          (self.t_bag[-1] if self.t_bag else 0.0))
        self.t_hdr.append(hs if hs is not None else 0.0)

    def finish(self) -> None:
        self.t_bag = np.asarray(self.t_bag, dtype=float)
        self.sizes = np.asarray(self.sizes, dtype=float)
        self.t_hdr = np.asarray(self.t_hdr, dtype=float)
        self.t_msg = np.asarray(self.t_msg, dtype=float)
        # Eje de tiempos de los datos: header si existe, si no recepcion.
        if self.t_msg.size:
            self.t = self.t_msg
        else:
            self.t = self.t_bag
        del self.t_msg


class ImuC(Base):
    kind = "imu"

    def __init__(self, *a):
        super().__init__(*a)
        self.a, self.g, self.q = [], [], []

    def add(self, m):
        super().add(m)
        la, av, qq = m.linear_acceleration, m.angular_velocity, m.orientation
        self.a.append((la.x, la.y, la.z))
        self.g.append((av.x, av.y, av.z))
        self.q.append((qq.x, qq.y, qq.z, qq.w))

    def finish(self):
        super().finish()
        self.accel = np.asarray(self.a, float)
        self.gyro = np.asarray(self.g, float)
        self.quat = np.asarray(self.q, float)
        del self.a, self.g, self.q


class OdomC(Base):
    kind = "odom"

    def __init__(self, *a):
        super().__init__(*a)
        self.p, self.q, self.v, self.w = [], [], [], []
        self.pcov, self.tcov = [], []
        self.frame = self.child = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame, self.child = m.header.frame_id, m.child_frame_id
        po, o = m.pose.pose.position, m.pose.pose.orientation
        tl, ta = m.twist.twist.linear, m.twist.twist.angular
        self.p.append((po.x, po.y, po.z))
        self.q.append((o.x, o.y, o.z, o.w))
        self.v.append((tl.x, tl.y, tl.z))
        self.w.append((ta.x, ta.y, ta.z))
        pc = np.asarray(m.pose.covariance, float)
        tc = np.asarray(m.twist.covariance, float)
        self.pcov.append(pc[[0, 7, 35]])       # x, y, yaw
        self.tcov.append(tc[[0, 7, 35]])

    def finish(self):
        super().finish()
        self.pos = np.asarray(self.p, float)
        self.quat = np.asarray(self.q, float)
        self.vel = np.asarray(self.v, float)
        self.omega = np.asarray(self.w, float)
        self.pose_cov = np.asarray(self.pcov, float)
        self.twist_cov = np.asarray(self.tcov, float)
        del self.p, self.q, self.v, self.w, self.pcov, self.tcov


class PoseC(Base):
    """PoseStamped / PoseWithCovarianceStamped: trayectoria y orientacion."""

    kind = "pose"

    def __init__(self, *a):
        super().__init__(*a)
        self.p, self.q, self.pc = [], [], []
        self.frame = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        pose = m.pose
        if hasattr(pose, "pose"):                 # PoseWithCovariance
            cov = np.asarray(m.pose.covariance, float)
            self.pc.append(cov[[0, 7, 35]])
            pose = pose.pose
        else:
            self.pc.append((float("nan"),) * 3)
        po, o = pose.position, pose.orientation
        self.p.append((po.x, po.y, po.z))
        self.q.append((o.x, o.y, o.z, o.w))

    def finish(self):
        super().finish()
        self.pos = np.asarray(self.p, float)
        self.quat = np.asarray(self.q, float)
        self.cov = np.asarray(self.pc, float)
        del self.p, self.q, self.pc


class TwistC(Base):
    """TwistStamped / TwistWithCovarianceStamped: velocidades lineales y angulares."""

    kind = "twist"

    def __init__(self, *a):
        super().__init__(*a)
        self.v, self.w, self.vc = [], [], []
        self.frame = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        tw = m.twist
        if hasattr(tw, "twist"):                  # TwistWithCovariance
            cov = np.asarray(m.twist.covariance, float)
            self.vc.append(cov[[0, 7, 35]])
            tw = tw.twist
        else:
            self.vc.append((float("nan"),) * 3)
        tl, ta = tw.linear, tw.angular
        self.v.append((tl.x, tl.y, tl.z))
        self.w.append((ta.x, ta.y, ta.z))

    def finish(self):
        super().finish()
        self.vel = np.asarray(self.v, float)
        self.omega = np.asarray(self.w, float)
        self.cov = np.asarray(self.vc, float)
        del self.v, self.w, self.vc


class CarC(Base):
    """mirena_common/Car: la salida del filtro de estado."""

    kind = "car"

    def __init__(self, *a):
        super().__init__(*a)
        self.p, self.q, self.v, self.w, self.cv = [], [], [], [], []
        self.frame = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        self.p.append((m.x, m.y, 0.0))
        self.q.append((0.0, 0.0, math.sin(0.5 * m.psi), math.cos(0.5 * m.psi)))
        self.v.append((m.u, m.v, 0.0))
        self.w.append((0.0, 0.0, m.omega))
        c = np.asarray(m.covariance, float)
        self.cv.append(c[[0, 7, 35]] if c.size >= 36 else (float("nan"),) * 3)

    def finish(self):
        super().finish()
        self.pos = np.asarray(self.p, float)
        self.quat = np.asarray(self.q, float)
        self.vel = np.asarray(self.v, float)
        self.omega = np.asarray(self.w, float)
        self.pose_cov = np.asarray(self.cv, float)
        del self.p, self.q, self.v, self.w, self.cv


class PoseArrayC(Base):
    kind = "posearray"

    def __init__(self, *a):
        super().__init__(*a)
        self.count, self.cx, self.cy, self.span = [], [], [], []
        self.last = None
        self.frame = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        pts = np.asarray([(p.position.x, p.position.y) for p in m.poses], float)
        self.count.append(len(pts))
        if pts.size:
            c = pts.mean(axis=0)
            self.cx.append(float(c[0]))
            self.cy.append(float(c[1]))
            self.span.append(float(np.hypot(pts[:, 0] - c[0],
                                            pts[:, 1] - c[1]).max()))
            self.last = pts[:5000]
        else:
            self.cx.append(float("nan"))
            self.cy.append(float("nan"))
            self.span.append(0.0)

    def finish(self):
        super().finish()
        self.counts = np.asarray(self.count, float)
        self.centroid = np.column_stack([self.cx, self.cy])
        self.spread = np.asarray(self.span, float)
        del self.count, self.cx, self.cy, self.span


class PathC(Base):
    kind = "path"

    def __init__(self, *a):
        super().__init__(*a)
        self.count, self.length = [], []
        self.last = None
        self.frame = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        pts = np.asarray([(p.pose.position.x, p.pose.position.y)
                          for p in m.poses], float)
        self.count.append(len(pts))
        if pts.shape[0] > 1:
            self.length.append(float(np.hypot(*np.diff(pts, axis=0).T).sum()))
            self.last = pts
        else:
            self.length.append(0.0)
            self.last = pts if pts.size else self.last

    def finish(self):
        super().finish()
        self.counts = np.asarray(self.count, float)
        self.lengths = np.asarray(self.length, float)
        del self.count, self.length


class FixC(Base):
    kind = "fix"

    def __init__(self, *a):
        super().__init__(*a)
        self.la, self.lo, self.al, self.st, self.cv = [], [], [], [], []

    def add(self, m):
        super().add(m)
        self.la.append(m.latitude)
        self.lo.append(m.longitude)
        self.al.append(m.altitude)
        self.st.append(int(m.status.status))
        c = np.asarray(m.position_covariance, float)
        self.cv.append(c[[0, 4, 8]])

    def finish(self):
        super().finish()
        self.lat = np.asarray(self.la, float)
        self.lon = np.asarray(self.lo, float)
        self.alt = np.asarray(self.al, float)
        self.status = np.asarray(self.st, int)
        self.cov = np.asarray(self.cv, float)
        del self.la, self.lo, self.al, self.st, self.cv


class CloudC(Base):
    kind = "cloud"

    def __init__(self, *a):
        super().__init__(*a)
        self.shape: List[Tuple[int, int, int]] = []
        self.fields: List[str] = []
        self.frame = ""
        # Una nube cruda guardada para poder dibujarla en 3D. Solo una: dos
        # megas caben, seis gigas no.
        self.sample: Optional[Dict] = None

    def add(self, m):
        super().add(m)
        if not self.fields:
            self.fields = [f"{f.name}({f.datatype})" for f in m.fields]
            self.frame = m.header.frame_id
        self.shape.append((m.width, m.height, m.point_step))
        self.sample = {
            "data": bytes(m.data), "point_step": int(m.point_step),
            "width": int(m.width), "height": int(m.height),
            "big": bool(m.is_bigendian),
            "fields": [(f.name, int(f.offset), int(f.datatype), int(f.count))
                       for f in m.fields]}

    def finish(self):
        super().finish()
        self.shp = np.asarray(self.shape, float) if self.shape else np.zeros((0, 3))


# Codigos de PointField -> dtype de numpy.
PF_DTYPE = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4",
            7: "f4", 8: "f8"}


def decode_cloud(s: Dict, max_points: int = 60000) -> Optional[Dict]:
    """
    Decodifica una PointCloud2 a arrays, sin ros2_numpy ni sensor_msgs_py.

    Construye el dtype estructurado desde los PointField (offset, datatype) y
    lee el buffer de una pasada. Submuestrea para dibujar.
    """
    try:
        order = ">" if s["big"] else "<"
        names, formats, offsets = [], [], []
        for name, off, dt, cnt in s["fields"]:
            base = PF_DTYPE.get(dt)
            if base is None or cnt != 1 or name in names:
                continue
            names.append(name)
            formats.append(order + base)
            offsets.append(off)
        if not {"x", "y", "z"} <= set(names):
            return None
        dtype = np.dtype({"names": names, "formats": formats,
                          "offsets": offsets, "itemsize": s["point_step"]})
        n = len(s["data"]) // s["point_step"]
        arr = np.frombuffer(s["data"][:n * s["point_step"]], dtype=dtype)
        xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(float)
        extra = {k: np.asarray(arr[k], float) for k in names
                 if k not in ("x", "y", "z")}
        ok = np.all(np.isfinite(xyz), axis=1) & (np.abs(xyz).max(axis=1) > 1e-6)
        xyz = xyz[ok]
        extra = {k: v[ok] for k, v in extra.items()}
        if xyz.shape[0] > max_points:
            idx = np.linspace(0, xyz.shape[0] - 1, max_points).astype(int)
            xyz = xyz[idx]
            extra = {k: v[idx] for k, v in extra.items()}
        return {"xyz": xyz, "extra": extra, "n_total": int(n)}
    except Exception:  # noqa: BLE001
        return None


class ScanC(Base):
    """LaserScan: resumen por barrido y ultimo barrido completo para dibujar."""

    kind = "scan"

    def __init__(self, *a):
        super().__init__(*a)
        self.rows: List[Tuple[float, ...]] = []
        self.last: Optional[Dict] = None
        self.frame = ""
        self.angle_min = self.angle_max = self.angle_inc = float("nan")
        self.range_min = self.range_max = float("nan")
        self.scan_time = float("nan")

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
            self.angle_min, self.angle_max = float(m.angle_min), float(m.angle_max)
            self.angle_inc = float(m.angle_increment)
            self.range_min, self.range_max = float(m.range_min), float(m.range_max)
            self.scan_time = float(m.scan_time)
        r = np.asarray(m.ranges, float)
        valid = np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)
        rv = r[valid]
        self.rows.append((
            float(r.size), float(rv.size),
            float(rv.min()) if rv.size else float("nan"),
            float(rv.mean()) if rv.size else float("nan"),
            float(np.median(rv)) if rv.size else float("nan"),
            float(rv.max()) if rv.size else float("nan")))
        # Guarda el ultimo barrido, submuestreado si es enorme.
        step = max(1, int(math.ceil(r.size / 4000.0))) if r.size else 1
        ang = self.angle_min + self.angle_inc * np.arange(r.size)
        self.last = {"angle": ang[::step].copy(), "range": r[::step].copy(),
                     "valid": valid[::step].copy()}

    def finish(self):
        super().finish()
        self.stats = (np.asarray(self.rows, float) if self.rows
                      else np.zeros((0, 6)))
        del self.rows


# Codificaciones de sensor_msgs/Image que se pueden interpretar sin cv2.
IMG_ENCODINGS = {
    "mono8": ("u1", 1), "8uc1": ("u1", 1), "mono16": ("u2", 1),
    "16uc1": ("u2", 1), "32fc1": ("f4", 1),
    "rgb8": ("u1", 3), "bgr8": ("u1", 3), "rgba8": ("u1", 4),
    "bgra8": ("u1", 4),
}


def image_to_arrays(m) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Devuelve (gris float, rgb uint8) de un sensor_msgs/Image, o (None, None)."""
    spec = IMG_ENCODINGS.get((m.encoding or "").lower())
    if spec is None:
        return None, None
    dt, ch = spec
    h, w, step = int(m.height), int(m.width), int(m.step)
    itemsize = np.dtype(dt).itemsize
    if h <= 0 or w <= 0 or step < w * ch * itemsize:
        return None, None
    try:
        flat = np.frombuffer(m.data, dtype=dt, count=h * step // itemsize)
        arr = flat.reshape(h, step // itemsize)[:, :w * ch].reshape(h, w, ch)
    except Exception:  # noqa: BLE001
        return None, None
    if ch >= 3:
        rgb = arr[..., :3]
        if (m.encoding or "").lower() in ("bgr8", "bgra8"):
            rgb = rgb[..., ::-1]
        gray = rgb.astype(np.float32).mean(axis=2)
        return gray, np.ascontiguousarray(rgb)
    gray = arr[..., 0].astype(np.float32)
    if (m.encoding or "").lower() in ("mono16", "16uc1") and \
            gray.size and float(np.nanmax(gray)) > 255.0:
        gray /= 256.0
    return gray, None


def compressed_to_arrays(m) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Decodifica un CompressedImage si OpenCV esta disponible."""
    try:
        import cv2
    except ImportError:
        return None, None
    try:
        buf = np.frombuffer(m.data, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    except Exception:  # noqa: BLE001
        return None, None
    if img is None:
        return None, None
    if img.ndim == 2:
        return img.astype(np.float32), None
    rgb = img[..., 2::-1] if img.shape[2] >= 3 else img[..., :3]
    gray = np.asarray(rgb, np.float32).mean(axis=2)
    return gray, np.ascontiguousarray(rgb)


def thumbnail(gray: Optional[np.ndarray], rgb: Optional[np.ndarray],
              max_side: int = 260):
    if gray is None or gray.size == 0:
        return None, None
    h, w = gray.shape[:2]
    step = max(1, int(math.ceil(max(h, w) / float(max_side))))
    g = gray[::step, ::step]
    r = rgb[::step, ::step] if rgb is not None else None
    return g, r


class ImageC(Base):
    kind = "image"

    def __init__(self, *a):
        super().__init__(*a)
        self.meta: List[Tuple] = []
        self.stat: List[Tuple[float, float]] = []
        self.thumbs: List[Tuple[float, np.ndarray, Optional[np.ndarray]]] = []
        self._seen = 0
        self._step = max(1, MEDIA_SAMPLE // MEDIA_THUMBS)
        self.frame = ""
        self.encoding = ""
        self.format = ""

    def arrays(self, m):
        return image_to_arrays(m)

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        if hasattr(m, "width"):                    # Image
            self.meta.append((int(m.width), int(m.height), int(m.step),
                              len(m.data)))
            if not self.encoding:
                self.encoding = str(m.encoding)
        else:                                      # CompressedImage
            self.meta.append((0, 0, 0, len(m.data)))
            if not self.format:
                self.format = str(m.format)
        gray, rgb = self.arrays(m)
        if gray is not None and gray.size:
            finite = gray[np.isfinite(gray)]
            self.stat.append((float(finite.mean()) if finite.size else
                              float("nan"),
                              float(finite.std()) if finite.size else
                              float("nan")))
        else:
            self.stat.append((float("nan"), float("nan")))
        self._seen += 1
        if gray is not None and len(self.thumbs) < MEDIA_THUMBS and \
                (self._seen <= MEDIA_THUMBS or self._seen % self._step == 0):
            g, r = thumbnail(gray, rgb)
            if g is not None:
                self.thumbs.append((self.t_msg[-1], g, r))

    def finish(self):
        super().finish()
        self.metadata = (np.asarray(self.meta, float) if self.meta
                         else np.zeros((0, 4)))
        self.image_stat = (np.asarray(self.stat, float) if self.stat
                           else np.zeros((0, 2)))
        del self.meta, self.stat


class CompressedImageC(ImageC):
    kind = "image"

    def arrays(self, m):
        return compressed_to_arrays(m)


class TfC(Base):
    kind = "tf"

    def __init__(self, *a):
        super().__init__(*a)
        self.tf: Dict[Tuple[str, str], Tuple] = {}
        self.quat: Dict[Tuple[str, str], Tuple] = {}

    def add(self, m):
        super().add(m)
        for tr in m.transforms:
            t, q = tr.transform.translation, tr.transform.rotation
            key = (tr.header.frame_id, tr.child_frame_id)
            self.tf[key] = (t.x, t.y, t.z, tuple(np.degrees(
                rpy_from_quat(np.array([[q.x, q.y, q.z, q.w]]))[0])))
            self.quat[key] = (q.x, q.y, q.z, q.w)


class LogC(Base):
    kind = "log"

    def __init__(self, *a):
        super().__init__(*a)
        self.rows: List[Tuple[int, str, str, float]] = []

    def add(self, m):
        super().add(m)
        self.rows.append((int(m.level), m.name, m.msg.strip(),
                          self.t_msg[-1] if self.t_msg else 0.0))


def _get_path(msg, path: str) -> float:
    v = msg
    for part in path.split("."):
        v = getattr(v, part)
    return float(v)


class ScalarC(Base):
    """Una o varias magnitudes escalares (temperatura, presion, campo...)."""

    kind = "scalar"
    fields: Tuple[str, ...] = ("data",)
    labels: Tuple[str, ...] = ("valor",)
    unit = ""

    def __init__(self, *a):
        super().__init__(*a)
        self.vals: List[Tuple[float, ...]] = []

    def add(self, m):
        super().add(m)
        try:
            self.vals.append(tuple(_get_path(m, f) for f in self.fields))
        except (AttributeError, TypeError, ValueError):
            self.vals.append((float("nan"),) * len(self.fields))

    def finish(self):
        super().finish()
        self.value = (np.asarray(self.vals, float) if self.vals
                      else np.zeros((0, len(self.fields))))
        del self.vals


class TemperatureC(ScalarC):
    fields, labels, unit = ("temperature",), ("temperatura",), "degC"


class PressureC(ScalarC):
    fields, labels, unit = ("fluid_pressure",), ("presion",), "Pa"


class IlluminanceC(ScalarC):
    fields, labels, unit = ("illuminance",), ("iluminancia",), "lux"


class HumidityC(ScalarC):
    fields, labels, unit = ("relative_humidity",), ("humedad",), "%"


class MagneticC(ScalarC):
    fields = ("magnetic_field.x", "magnetic_field.y", "magnetic_field.z")
    labels = ("x", "y", "z")
    unit = "T"


class RangeC(Base):
    kind = "range"

    def __init__(self, *a):
        super().__init__(*a)
        self.r, self.lo, self.hi, self.fov, self.rad = [], [], [], [], []
        self.frame = ""

    def add(self, m):
        super().add(m)
        if not self.frame:
            self.frame = m.header.frame_id
        self.r.append(float(m.range))
        self.lo.append(float(m.min_range))
        self.hi.append(float(m.max_range))
        self.fov.append(float(m.field_of_view))
        self.rad.append(int(m.radiation_type))

    def finish(self):
        super().finish()
        self.range = np.asarray(self.r, float)
        self.min_range = np.asarray(self.lo, float)
        self.max_range = np.asarray(self.hi, float)
        self.field_of_view = np.asarray(self.fov, float)
        self.radiation = np.asarray(self.rad, int)
        del self.r, self.lo, self.hi, self.fov, self.rad


class BatteryC(Base):
    kind = "battery"

    FIELDS = ("voltage", "temperature", "current", "charge", "capacity",
              "design_capacity", "percentage", "power_supply_status",
              "power_supply_health", "power_supply_technology", "present")

    def __init__(self, *a):
        super().__init__(*a)
        self.vals: List[Tuple[float, ...]] = []
        self.cell_v: List[float] = []
        self.cell_t: List[float] = []

    def add(self, m):
        super().add(m)
        self.vals.append(tuple(float(getattr(m, f)) for f in self.FIELDS))
        cv = np.asarray(m.cell_voltage, float)
        ct = np.asarray(m.cell_temperature, float)
        self.cell_v.append(float(np.nanmean(cv)) if cv.size else float("nan"))
        self.cell_t.append(float(np.nanmean(ct)) if ct.size else float("nan"))

    def finish(self):
        super().finish()
        self.data = (np.asarray(self.vals, float) if self.vals
                     else np.zeros((0, len(self.FIELDS))))
        self.cell_voltage = np.asarray(self.cell_v, float)
        self.cell_temperature = np.asarray(self.cell_t, float)
        del self.vals, self.cell_v, self.cell_t


class JointC(Base):
    kind = "joint"

    def __init__(self, *a):
        super().__init__(*a)
        self.rows: List[Dict[str, Tuple[float, float, float]]] = []

    def add(self, m):
        super().add(m)
        row: Dict[str, Tuple[float, float, float]] = {}
        for i, name in enumerate(m.name):
            row[name] = (
                float(m.position[i]) if i < len(m.position) else float("nan"),
                float(m.velocity[i]) if i < len(m.velocity) else float("nan"),
                float(m.effort[i]) if i < len(m.effort) else float("nan"))
        self.rows.append(row)

    def finish(self):
        super().finish()
        names: List[str] = []
        for r in self.rows:
            for n in r:
                if n not in names:
                    names.append(n)
        self.names = names
        self.position = {n: np.asarray([r.get(n, (np.nan,) * 3)[0]
                                        for r in self.rows], float) for n in names}
        self.velocity = {n: np.asarray([r.get(n, (np.nan,) * 3)[1]
                                        for r in self.rows], float) for n in names}
        self.effort = {n: np.asarray([r.get(n, (np.nan,) * 3)[2]
                                      for r in self.rows], float) for n in names}
        del self.rows


class DiagC(Base):
    kind = "diag"

    def __init__(self, *a):
        super().__init__(*a)
        self.rows: List[Tuple[float, int, str, str, str]] = []
        self.levels: List[int] = []
        self.latest: Dict[str, Tuple[int, str, List[Tuple[str, str, str]]]] = {}

    def add(self, m):
        super().add(m)
        worst = 0
        for st in m.status:
            lv = int(st.level)
            worst = max(worst, lv)
            self.rows.append((self.t_msg[-1], lv, st.name, st.message,
                              st.hardware_id))
            self.latest[st.name] = (lv, st.message,
                                    [(v.key, v.value) for v in st.values])
        self.levels.append(worst)

    def finish(self):
        super().finish()
        self.level = np.asarray(self.levels, int)
        del self.levels


class ClockC(Base):
    kind = "clock"

    def __init__(self, *a):
        super().__init__(*a)
        self.sec, self.nsec = [], []

    def add(self, m):
        super().add(m)
        self.sec.append(int(m.clock.sec))
        self.nsec.append(int(m.clock.nanosec))

    def finish(self):
        super().finish()
        self.clock = np.asarray(self.sec, float) + \
            np.asarray(self.nsec, float) * 1e-9
        del self.sec, self.nsec


class GenericC(Base):
    kind = "generic"

    def __init__(self, *a):
        super().__init__(*a)
        self.cols: Dict[str, List[float]] = defaultdict(list)
        self.k = 0

    def add(self, m):
        super().add(m)
        self.k += 1
        for key, v in flatten_numeric(m).items():
            c = self.cols[key]
            # Rellena huecos si un campo aparece tarde, para que todas las
            # columnas tengan la misma longitud y se puedan graficar.
            if len(c) < self.k - 1:
                c.extend([float("nan")] * (self.k - 1 - len(c)))
            c.append(v)

    def finish(self):
        super().finish()
        self.data = {}
        for key, c in self.cols.items():
            if len(c) < self.k:
                c.extend([float("nan")] * (self.k - len(c)))
            self.data[key] = np.asarray(c, float)
        del self.cols


def flatten_numeric(msg, prefix: str = "", depth: int = 0) -> Dict[str, float]:
    """Campos numericos escalares de cualquier mensaje, por introspeccion."""
    out: Dict[str, float] = {}
    if depth > 3:
        return out
    getf = getattr(msg, "get_fields_and_field_types", None)
    if getf is None:
        return out
    for name in getf():
        if name == "header":
            continue
        try:
            v = getattr(msg, name)
        except AttributeError:
            continue
        key = f"{prefix}{name}"
        if isinstance(v, bool) or isinstance(v, (int, float)):
            out[key] = float(v)
        elif isinstance(v, (str, bytes)):
            continue
        elif hasattr(v, "get_fields_and_field_types"):
            out.update(flatten_numeric(v, key + ".", depth + 1))
        elif hasattr(v, "__len__") and 0 < len(v) <= 16:
            try:
                for i, e in enumerate(v):
                    if isinstance(e, bool) or isinstance(e, (int, float)):
                        out[f"{key}[{i}]"] = float(e)
                    elif hasattr(e, "get_fields_and_field_types") and i < 4:
                        out.update(flatten_numeric(e, f"{key}[{i}].", depth + 1))
            except TypeError:
                pass
        elif hasattr(v, "__len__") and len(v) > 16:
            # Arrays grandes (covarianzas, listas de puntos, espectros): en vez
            # de descartarlos, se resume su contenido. Asi un mensaje propio
            # desconocido sigue produciendo series utiles.
            try:
                arr = np.asarray(v, dtype=float)
                if arr.size and np.any(np.isfinite(arr)):
                    out[f"{key}.n"] = float(arr.size)
                    out[f"{key}.mean"] = float(np.nanmean(arr))
                    out[f"{key}.std"] = float(np.nanstd(arr))
                    out[f"{key}.min"] = float(np.nanmin(arr))
                    out[f"{key}.max"] = float(np.nanmax(arr))
            except (TypeError, ValueError):
                pass
    return out


COLLECTORS = {IMU_TYPE: ImuC, ODOM_TYPE: OdomC, FIX_TYPE: FixC,
              CLOUD_TYPE: CloudC, LOG_TYPE: LogC, SCAN_TYPE: ScanC,
              IMAGE_TYPE: ImageC, COMPRESSED_TYPE: CompressedImageC,
              RANGE_TYPE: RangeC, BATTERY_TYPE: BatteryC,
              JOINT_TYPE: JointC, DIAG_TYPE: DiagC, CLOCK_TYPE: ClockC,
              TEMPERATURE_TYPE: TemperatureC, PRESSURE_TYPE: PressureC,
              MAGNETIC_TYPE: MagneticC, ILLUMINANCE_TYPE: IlluminanceC,
              HUMIDITY_TYPE: HumidityC, CAR_TYPE: CarC,
              POSE_ARRAY_TYPE: PoseArrayC, PATH_TYPE: PathC}
for _t in TF_TYPES:
    COLLECTORS[_t] = TfC
for _t in POSE_TYPES:
    COLLECTORS[_t] = PoseC
for _t in TWIST_TYPES:
    COLLECTORS[_t] = TwistC


# --------------------------------------------------------------------------- #
#  Lectura del bag
# --------------------------------------------------------------------------- #

def open_reader(path: str):
    try:
        import rosbag2_py  # noqa: F401
    except ImportError as exc:
        raise SystemExit("\nNo encuentro rosbag2_py. Sourcea ROS 2:\n"
                         "    source /opt/ros/jazzy/setup.bash\n") from exc
    import rosbag2_py
    if not os.path.exists(path):
        raise SystemExit(f"\nNo existe la ruta: {path}")
    # Un .mcap/.db3 suelto se abre directamente con su plugin; para una
    # carpeta conviene dejar que rosbag2 detecte el almacenamiento
    # (metadata.yaml).
    low = path.lower()
    if low.endswith(".mcap"):
        ids = ("mcap", "sqlite3")
    elif low.endswith(".db3"):
        ids = ("sqlite3", "mcap")
    else:
        ids = ("", "mcap", "sqlite3")
    last = None
    for sid in ids:
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=sid),
                   rosbag2_py.ConverterOptions(
                       input_serialization_format="cdr",
                       output_serialization_format="cdr"))
            return r
        except Exception as exc:  # noqa: BLE001
            last = exc
    raise SystemExit(f"\nNo pude abrir el bag '{path}':\n  {last}")


def qos_of(reader) -> Dict[str, str]:
    out: Dict[str, str] = {}
    try:
        for t in reader.get_metadata().topics_with_message_count:
            prof = getattr(t.topic_metadata, "offered_qos_profiles", None)
            if not prof:
                continue
            if isinstance(prof, str):
                out[t.topic_metadata.name] = ("best_effort" if "best_effort"
                                              in prof else "reliable")
            else:
                out[t.topic_metadata.name] = str(
                    getattr(prof[0], "reliability", "?")).split(".")[-1].lower()
    except Exception:  # noqa: BLE001
        pass
    return out


def scan(path: str, wanted: Optional[Sequence[str]], sample: int
         ) -> Tuple[Dict[str, Base], Dict[str, str]]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    qos = qos_of(reader)
    if wanted:
        missing = [t for t in wanted if t not in types]
        if missing:
            raise SystemExit(f"\nNo estan en el bag: {', '.join(missing)}\n")
        types = {k: v for k, v in types.items() if k in wanted}
        import rosbag2_py
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))

    col: Dict[str, Base] = {}
    cls: Dict[str, object] = {}
    cap: Dict[str, int] = {}
    for n, ty in types.items():
        col[n] = COLLECTORS.get(ty, GenericC)(n, ty)
        if ty in HEAVY_TYPES:
            cap[n] = HEAVY_SAMPLE
        elif ty in MEDIA_TYPES:
            cap[n] = min(sample, MEDIA_SAMPLE)
        else:
            cap[n] = sample
        try:
            cls[n] = get_message(ty)
        except Exception as exc:  # noqa: BLE001
            print(f"  [aviso] tipo '{ty}' de {n} no resoluble ({exc}); "
                  f"solo tasas y tamanos.")

    print(f"\nLeyendo {path} ...")
    ndes = defaultdict(int)
    total = 0
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        c = col.get(topic)
        if c is None:
            continue
        total += 1
        c.add_raw(len(raw), stamp)
        k = cls.get(topic)
        if k is None or ndes[topic] >= cap[topic]:
            continue
        try:
            c.add(deserialize_message(raw, k))
            ndes[topic] += 1
        except Exception:  # noqa: BLE001
            pass

    for c in col.values():
        c.finish()
    print(f"  {total} mensajes en {len(col)} topics")
    return col, qos


def window(c: Base, t0: float, start: Optional[float], end: Optional[float]) -> None:
    """Recorta en sitio todos los arrays alineados con las muestras."""
    if start is None and end is None:
        return

    def keep(tr: np.ndarray) -> np.ndarray:
        m = np.ones_like(tr, dtype=bool)
        if start is not None:
            m &= tr >= start
        if end is not None:
            m &= tr <= end
        return m

    md = keep(c.t - t0) if c.t.size else np.zeros(0, bool)
    mr = keep(c.t_bag - t0) if c.t_bag.size else np.zeros(0, bool)
    nd, nr = len(md), len(mr)

    def cut(list_like):
        if len(list_like) == nd:
            return [v for v, k in zip(list_like, md) if k]
        return list_like

    for k, v in list(vars(c).items()):
        if isinstance(v, np.ndarray) and v.ndim >= 1:
            if k in ("t_bag", "sizes") and v.shape[0] == nr:
                setattr(c, k, v[mr])
            elif v.shape[0] == nd:
                setattr(c, k, v[md])
            elif v.shape[0] == nr:
                setattr(c, k, v[mr])
        elif isinstance(v, dict) and v and \
                isinstance(next(iter(v.values())), np.ndarray) and \
                next(iter(v.values())).shape[0] == nd:
            setattr(c, k, {kk: vv[md] for kk, vv in v.items()
                           if isinstance(vv, np.ndarray) and vv.shape[0] == nd})
        elif isinstance(v, list) and v and len(v) == nd and \
                k not in ("thumbs", "rows", "latest", "names", "fields"):
            setattr(c, k, cut(v))
    if c.kind == "log":
        c.rows = [r for r, k in zip(c.rows, md) if k]
    elif c.kind == "diag":
        lo = (t0 + start) if start is not None else -float("inf")
        hi = (t0 + end) if end is not None else float("inf")
        c.rows = [r for r in c.rows if lo <= r[0] <= hi]


# --------------------------------------------------------------------------- #
#  Estadistica general
# --------------------------------------------------------------------------- #

def base_stats(c: Base) -> Dict:
    t, b = c.t_bag, c.sizes
    o = {"n": int(t.size), "bytes": float(b.sum()) if b.size else 0.0}
    if t.size < 2:
        o.update(dur=0.0, hz=float("nan"), jit=float("nan"), gaps=0, back=0,
                 bw=0.0, lat=float("nan"), latstep=float("nan"))
        return o
    dt = np.diff(c.t if c.t.size == t.size else t)
    pos = dt[dt > 0]
    med = float(np.median(pos)) if pos.size else float("nan")
    dur = float(t[-1] - t[0])
    o.update(dur=dur, hz=1.0 / med if med > 0 else float("nan"),
             jit=float(np.std(pos) * 1e3) if pos.size else float("nan"),
             gaps=int(np.sum(dt > 3.0 * med)) if med > 0 else 0,
             back=int(np.sum(dt < 0)),
             bw=float(b.sum() / dur) if dur > 0 else 0.0)
    if c.t_hdr.size == t.size and np.any(c.t_hdr > 0):
        lat = (t - c.t_hdr) * 1e3
        o["lat"] = float(np.median(lat))
        if lat.size > 200:
            k = max(20, lat.size // 50)
            run = np.convolve(lat, np.ones(k) / k, mode="valid")
            o["latstep"] = float(np.max(np.abs(np.diff(run)))) if run.size > 1 else 0.0
        else:
            o["latstep"] = 0.0
    else:
        o["lat"] = o["latstep"] = float("nan")
    return o


def report_overview(col: Dict[str, Base], qos: Dict[str, str]) -> Dict[str, Dict]:
    st = {n: base_stats(c) for n, c in col.items()}
    order = sorted(st, key=lambda n: -st[n]["bytes"])
    w = max(len(n) for n in order) + 1
    tot = sum(s["bw"] for s in st.values())
    print("\n" + "=" * 120)
    print("  TODOS LOS TOPICS")
    print("=" * 120)
    print(f"{'topic':<{w}}{'msgs':>8}{'Hz':>8}{'jit[ms]':>9}{'huecos':>8}"
          f"{'volumen':>11}{'kB/s':>9}{'%BW':>7}{'lat[ms]':>9}  {'QoS':<12}tipo")
    for n in order:
        s, q = st[n], qos.get(n, "?")
        flag = " *" if (s["gaps"] and q == "best_effort") else ""
        lat = f"{s['lat']:.1f}" if math.isfinite(s["lat"]) else "-"
        pct = 100 * s["bw"] / tot if tot > 0 else 0
        print(f"{n:<{w}}{s['n']:>8}{s['hz']:>8.2f}{s['jit']:>9.2f}{s['gaps']:>8}"
              f"{human(s['bytes']):>11}{s['bw']/1024:>9.1f}{pct:>7.1f}{lat:>9}"
              f"  {q:<12}{col[n].type}{flag}")
    print(f"\n  ancho de banda total: {tot/1024/1024:.2f} MB/s")
    if any(st[n]["gaps"] and qos.get(n) == "best_effort" for n in order):
        print("  (*) huecos Y QoS best_effort: los mensajes se pueden estar")
        print("      perdiendo en el grabador, no en el nodo que publica.")
    for n in order:
        if st[n]["back"]:
            print(f"  ATENCION  {n}: {st[n]['back']} stamps van hacia atras")
        if math.isfinite(st[n]["latstep"]) and st[n]["latstep"] > 20:
            print(f"  ATENCION  {n}: escalon de latencia de "
                  f"{st[n]['latstep']:.1f} ms (cambio de fuente de reloj)")
    return st


# --------------------------------------------------------------------------- #
#  Analisis por tipo — texto
# --------------------------------------------------------------------------- #

def imu_stats(c: ImuC) -> Dict:
    a, g = c.accel, np.degrees(c.gyro)
    gd = np.degrees(np.linalg.norm(c.gyro, axis=1))
    an = np.linalg.norm(a, axis=1)
    sm = (gd < STATIC_GYRO_DPS) & (np.abs(an - np.median(an)) < STATIC_ACC_TOL)
    a_st = a[sm] if sm.sum() > 20 else a
    am = a_st.mean(axis=0)
    gmag = float(np.linalg.norm(am))
    if gmag > 1e-6:
        v = am if am[2] < 0 else np.array([am[0], -am[1], -am[2]])
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, v[0] / gmag))))
        roll = math.degrees(math.atan2(-v[1], -v[2]))
    else:
        pitch = roll = float("nan")
    return {"sigma_a": a.std(axis=0), "sigma_g": g.std(axis=0),
            "bias_g": g[sm].mean(axis=0) if sm.sum() > 20 else g.mean(axis=0),
            "g_mean": float(an.mean()), "g_static": gmag, "a_static": am,
            "n_static": int(sm.sum()),
            "pitch": pitch, "roll": roll, "az_mean": float(a[:, 2].mean()),
            "has_quat": bool(np.any(np.abs(c.quat)))}


def report_imu(c: ImuC, b: Dict, s: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Imu")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['dur']:.1f} s, {b['hz']:.2f} Hz, "
          f"jitter {b['jit']:.3f} ms, {b['gaps']} huecos")
    print("  sigma accel x {:.5f}  y {:.5f}  z {:.5f}  m/s2".format(*s["sigma_a"]))
    print("  sigma giro  x {:.5f}  y {:.5f}  z {:.5f}  deg/s".format(*s["sigma_g"]))
    print("  bias giro   x {:+.5f} y {:+.5f} z {:+.5f}  deg/s".format(*s["bias_g"]))
    print(f"  |a| media   {s['g_mean']:.4f} m/s2")
    if s["n_static"] > 20:
        print(f"  en reposo   {s['n_static']} muestras   |g| {s['g_static']:.4f}   "
              f"pitch {s['pitch']:+.3f}   roll {s['roll']:+.3f} deg")
    if abs(s["g_mean"] - G0) > 1.0:
        print("  ATENCION    |a| deberia rondar 9.807 en cualquier orientacion;")
        print(f"              {s['g_mean']:.3f} sugiere gravedad restada o datos invalidos")
    if abs(abs(s["az_mean"]) - G0) < 0.5 and s["az_mean"] < 0:
        print("  ATENCION    accel z negativa nivelado; en ROS (FLU) seria +9.81")
    if not s["has_quat"]:
        print("  Sin cuaternion de orientacion en el mensaje.")


def estimate_offset(A: ImuC, B: ImuC, fs=100.0, max_lag=2.0) -> Tuple[float, float]:
    """Desfase de relojes por correlacion del modulo del giroscopo."""
    from scipy.signal import correlate
    t0, t1 = max(A.t[0], B.t[0]), min(A.t[-1], B.t[-1])
    if t1 - t0 < 5.0:
        return float("nan"), 0.0
    grid = np.arange(t0, t1, 1.0 / fs)
    gA = np.interp(grid, A.t, np.linalg.norm(A.gyro, axis=1))
    gB = np.interp(grid, B.t, np.linalg.norm(B.gyro, axis=1))
    gA -= gA.mean()
    gB -= gB.mean()
    if gA.std() < 1e-9 or gB.std() < 1e-9:
        return float("nan"), 0.0
    c = correlate(gA, gB, mode="full")
    lags = np.arange(-len(gB) + 1, len(gA))
    keep = np.abs(lags) <= int(max_lag * fs)
    ck, lk = c[keep], lags[keep]
    k = int(np.argmax(ck))
    frac = 0.0
    if 0 < k < len(ck) - 1:
        y0, y1, y2 = float(ck[k - 1]), float(ck[k]), float(ck[k + 1])
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            frac = float(np.clip(0.5 * (y0 - y2) / den, -1.0, 1.0))
    nrm = float(np.sqrt(np.sum(gA ** 2) * np.sum(gB ** 2)))
    return float((lk[k] + frac) / fs), (float(ck[k] / nrm) if nrm > 0 else 0.0)


def estimate_rotation(A: ImuC, B: ImuC, dt: float, fs=100.0) -> Optional[Dict]:
    """Rotacion R con gyro_A ~= R @ gyro_B, por Kabsch/SVD."""
    t0, t1 = max(A.t[0], B.t[0] + dt), min(A.t[-1], B.t[-1] + dt)
    if t1 - t0 < 5.0:
        return None
    grid = np.arange(t0, t1, 1.0 / fs)
    Ga = np.stack([np.interp(grid, A.t, A.gyro[:, i]) for i in range(3)], 1)
    Gb = np.stack([np.interp(grid, B.t + dt, B.gyro[:, i]) for i in range(3)], 1)
    m = (np.linalg.norm(Ga, axis=1) > math.radians(5.0)) & \
        (np.linalg.norm(Gb, axis=1) > math.radians(5.0))
    if m.sum() < 200:
        return {"insufficient": True, "n": int(m.sum())}
    Ga, Gb = Ga[m], Gb[m]
    U, _, Vt = np.linalg.svd(Gb.T @ Ga)
    D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(Vt.T @ U.T)))])
    R = Vt.T @ D @ U.T
    res = Ga - (R @ Gb.T).T
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    scale = float(np.sqrt(np.mean(np.sum(Ga ** 2, axis=1))))
    return {"R": R, "rpy": np.degrees(rpy_from_matrix(R)),
            "rms": math.degrees(rms), "rel": rms / scale if scale > 0 else float("nan"),
            "n": int(len(Ga)), "insufficient": False}


def report_imu_compare(imus: Dict[str, ImuC], stats: Dict[str, Dict],
                       out_rot: Optional[Dict] = None) -> List[str]:
    names = list(imus)
    notes: List[str] = []
    print("\n" + "=" * 72)
    print("  COMPARACION ENTRE IMU")
    print("=" * 72)
    for n in names:
        s = stats[n]
        line = (f"{n}: |g| {s['g_static']:.4f}   pitch {s['pitch']:+.2f}   "
                f"roll {s['roll']:+.2f} deg")
        print("  " + line)
        notes.append(line)
    for i in range(len(names) - 1):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            print(f"\n  {a}  vs  {b}")
            off, q = estimate_offset(imus[a], imus[b])
            if math.isfinite(off):
                print(f"    desfase de reloj {off*1e3:+.1f} ms (sumar a {b})   "
                      f"correlacion {q:.3f}")
                if q < 0.7:
                    print("      correlacion baja: puede que no midan lo mismo")
                else:
                    print("      la precision del desfase depende de lo rapido que")
                    print("      sea el movimiento; manda el residuo de abajo")
                notes.append(f"desfase: {off*1e3:+.1f} ms (corr {q:.2f})")
            else:
                off = 0.0
                print("    desfase no estimable (sin movimiento comun)")
            rot = estimate_rotation(imus[a], imus[b], off)
            if rot is None or rot.get("insufficient"):
                print("    rotacion relativa no estimable: hace falta un tramo")
                print("    con giro en varios ejes (ochos, slalom)")
                notes.append("rotacion: sin datos dinamicos")
                continue
            r, p, y = rot["rpy"]
            print(f"    rotacion relativa (gyro_A = R @ gyro_B), {rot['n']} muestras:")
            print(f"      roll {r:+8.3f}  pitch {p:+8.3f}  yaw {y:+8.3f} deg")
            print(f"      residuo RMS {rot['rms']:.4f} deg/s "
                  f"({100*rot['rel']:.1f} % de la senal)")
            if rot["rel"] > 0.15:
                print("      residuo alto: relojes mal alineados o una de las dos")
                print("      IMU no esta midiendo bien")
            notes.append(f"R: [{r:+.1f} {p:+.1f} {y:+.1f}] deg, "
                         f"res {100*rot['rel']:.0f}%")
            if out_rot is not None and "R" not in out_rot:
                out_rot.update(rot)          # la primera pareja alimenta el 3D
    return notes


def report_odom(c: OdomC, b: Dict) -> Dict:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Odometry")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, {b['gaps']} huecos")
    print(f"  frame_id '{c.frame}'   child_frame_id '{c.child}'")
    x, y = c.pos[:, 0], c.pos[:, 1]
    print(f"  recorrido {np.hypot(x - x[0], y - y[0]).max():.1f} m   "
          f"|v| max {np.hypot(c.vel[:, 0], c.vel[:, 1]).max():.2f} m/s   "
          f"|omega| max {math.degrees(np.abs(c.omega[:, 2]).max()):.1f} deg/s")
    if np.all(c.pose_cov == 0):
        print("  ATENCION: covarianza de pose TODA A CERO — el estimador que")
        print("            consuma esto no puede ponderarlo")
    if np.all(c.twist_cov == 0):
        print("  ATENCION: covarianza de twist TODA A CERO")

    out = {"slope": float("nan"), "best": None, "res": {}}
    t = c.t
    if t.size < 50 or not np.all(np.diff(t) > 0):
        print("  stamps no monotonos: no evaluo consistencia de frames")
        return out
    dt = float(np.median(np.diff(t)))
    k = max(5, int(round(1.0 / (3.0 * dt))))
    yaw = rpy_from_quat(c.quat)[:, 2]
    a = smooth(np.gradient(np.unwrap(yaw), dt), k)
    w = smooth(c.omega[:, 2], k)
    act = np.abs(w) > math.radians(3.0)
    if act.sum() > 100:
        den = float(np.sum(w[act] ** 2))
        out["slope"] = float(np.sum(a[act] * w[act]) / den) if den > 0 else float("nan")
        print(f"  (a) pendiente d(yaw)/dt vs angular.z = {out['slope']:+.3f}"
              f"   (debe ser +1.000)")
        if out["slope"] < -0.5:
            print("      pendiente ~ -1: yaw y angular.z con signos opuestos")
    else:
        print("  (a) sin giro suficiente para evaluar")

    vx, vy = smooth(np.gradient(x, dt), k), smooth(np.gradient(y, dt), k)
    m = np.hypot(vx, vy) > 2.0
    if m.sum() < 100:
        print("  (b) sin velocidad suficiente para evaluar")
        return out
    course = np.arctan2(vy[m], vx[m])
    for sgn, lab in ((+1.0, "yaw tal cual"), (-1.0, "yaw negado")):
        ps = sgn * yaw[m]
        bx = c.vel[m, 0] * np.cos(ps) - c.vel[m, 1] * np.sin(ps)
        by = c.vel[m, 0] * np.sin(ps) + c.vel[m, 1] * np.cos(ps)
        mu, R = circ_stats(np.arctan2(by, bx) - course)
        out["res"][lab] = (mu, R)
        print(f"  (b) error angular, {lab:<14}: {mu:+7.2f} deg   concentracion R {R:.3f}")
    out["best"] = max(out["res"], key=lambda kk: out["res"][kk][1])
    if out["best"] == "yaw tal cual" and abs(out["res"][out["best"]][0]) < 2.0:
        print("  -> pose y twist son coherentes entre si")
    else:
        print(f"  -> INCOHERENTE: la hipotesis que cuadra es '{out['best']}'.")
        print("     Revisa los signos de yaw y angular.z frente a x/y y linear.")
    return out


def report_fix(c: FixC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — NavSatFix")
    print("=" * 72)
    print(f"  {b['n']} mensajes, {b['hz']:.2f} Hz, {b['gaps']} huecos")
    ok = np.isfinite(c.lat) & np.isfinite(c.lon) & (np.abs(c.lat) > 1e-9)
    v, n = np.unique(c.status, return_counts=True)
    print(f"  fixes validos {int(ok.sum())} de {c.lat.size}")
    print("  status: " + "  ".join(f"{int(a)}:{int(b_)}" for a, b_ in zip(v, n))
          + "   (-1 sin fix, 0 fix, 1 SBAS, 2 GBAS)")
    if np.all(c.cov == 0):
        print("  ATENCION: position_covariance TODA A CERO")
    if ok.sum() < 5:
        return
    north, east, lat0, lon0 = lla_to_ne(c.lat[ok], c.lon[ok])
    r = np.hypot(north, east)
    print(f"  centro {lat0:.7f}, {lon0:.7f}   altitud media {c.alt[ok].mean():.2f} m")
    print(f"  sigma norte/este {north.std():.3f} / {east.std():.3f} m   "
          f"sigma altitud {c.alt[ok].std():.3f} m")
    if r.max() < 10.0:
        print("  bag estatico: CEP50 {:.3f}   CEP95 {:.3f} m".format(
            *np.percentile(r, [50, 95])))
    else:
        print(f"  bag en movimiento: recorrido {r.max():.0f} m, el CEP no aplica")


def report_cloud(c: CloudC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — PointCloud2")
    print("=" * 72)
    print(f"  {b['n']} nubes, {b['hz']:.2f} Hz, {b['gaps']} huecos, "
          f"{human(b['bytes'])} en total ({b['bw']/1024/1024:.2f} MB/s)")
    print(f"  tamano por nube: mediana {human(float(np.median(c.sizes)))}, "
          f"max {human(float(c.sizes.max()))}")
    if c.shp.size:
        w_, h_, ps = c.shp[:, 0], c.shp[:, 1], c.shp[:, 2]
        print(f"  puntos por nube: mediana {np.median(w_ * h_):.0f}   "
              f"point_step {int(np.median(ps))} B   frame '{c.frame}'")
        print(f"  campos: {', '.join(c.fields)}")
    print("  (no se decodifica el payload: solo tasa, tamano y layout)")


def report_pose(c: PoseC, b: Dict) -> Dict:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Pose")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, {b['gaps']} huecos, "
          f"frame '{c.frame}'")
    out = {}
    if not c.pos.size:
        return out
    x, y = c.pos[:, 0], c.pos[:, 1]
    dist = float(np.hypot(np.diff(x), np.diff(y)).sum())
    yaw = np.degrees(np.unwrap(rpy_from_quat(c.quat)[:, 2]))
    print(f"  recorrido {dist:.1f} m   yaw [{yaw.min():+.1f}, {yaw.max():+.1f}] deg")
    if c.t.size > 3 and np.all(np.diff(c.t) > 0):
        v = np.hypot(np.gradient(x, c.t), np.gradient(y, c.t))
        print(f"  velocidad derivada: media {v.mean():.2f}  max {v.max():.2f} m/s")
        out["speed"] = v
    if np.all(c.cov == 0):
        print("  ATENCION: covarianza de pose TODA A CERO")
    return out


def report_twist(c: TwistC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Twist")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, {b['gaps']} huecos, "
          f"frame '{c.frame}'")
    if not c.vel.size:
        return
    vn = np.linalg.norm(c.vel, axis=1)
    wn = np.degrees(np.linalg.norm(c.omega, axis=1))
    print(f"  |v| media {vn.mean():.3f}  max {vn.max():.3f} m/s   "
          f"|w| media {wn.mean():.2f}  max {wn.max():.2f} deg/s")
    for i, k in enumerate("xyz"):
        print(f"  v{k} sigma {c.vel[:, i].std():.4f} m/s   "
              f"w{k} sigma {np.degrees(c.omega[:, i].std()):.4f} deg/s")
    if np.all(c.cov == 0):
        print("  ATENCION: covarianza de twist TODA A CERO")


def report_car(c: CarC, b: Dict) -> Dict:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Car (salida del filtro de estado)")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, {b['gaps']} huecos, "
          f"frame '{c.frame}'")
    out = {}
    if not c.pos.size:
        return out
    x, y = c.pos[:, 0], c.pos[:, 1]
    yaw = np.degrees(np.unwrap(rpy_from_quat(c.quat)[:, 2]))
    print(f"  recorrido {np.hypot(x - x[0], y - y[0]).max():.1f} m   "
          f"|v| max {np.hypot(c.vel[:, 0], c.vel[:, 1]).max():.2f} m/s   "
          f"|omega| max {math.degrees(np.abs(c.omega[:, 2]).max()):.1f} deg/s")
    sd = np.sqrt(np.maximum(c.pose_cov, 0.0))
    if np.all(c.pose_cov == 0):
        print("  ATENCION: covarianza TODA A CERO")
    else:
        print(f"  sigma final x {sd[-1, 0]:.3f} m  y {sd[-1, 1]:.3f} m  "
              f"yaw {math.degrees(sd[-1, 2]):.3f} deg")
    return out


def report_state_compare(car: CarC, odom: OdomC) -> None:
    """Texto del EKF frente al sensor solo, con alineacion en el primer estado."""
    if not car.pos.size or not odom.pos.size:
        return
    lo, hi = max(car.t[0], odom.t[0]), min(car.t[-1], odom.t[-1])
    if hi - lo < 0.5:
        return
    yc0 = rpy_from_quat(car.quat[:1])[0, 2]
    yo0 = rpy_from_quat(odom.quat[:1])[0, 2]
    dy = yc0 - yo0
    c, s = math.cos(dy), math.sin(dy)
    raw = odom.pos[:, :2].copy() - odom.pos[0, :2]
    raw = np.column_stack([c * raw[:, 0] - s * raw[:, 1],
                           s * raw[:, 0] + c * raw[:, 1]]) + car.pos[0, :2]
    g = np.linspace(lo, hi, max(2, int((hi - lo) * 20)))
    cx = np.interp(g, car.t, car.pos[:, 0])
    cy = np.interp(g, car.t, car.pos[:, 1])
    rx = np.interp(g, odom.t, raw[:, 0])
    ry = np.interp(g, odom.t, raw[:, 1])
    err = np.hypot(cx - rx, cy - ry)
    yc = np.interp(g, car.t, _yaw_deg(car))
    yo = np.interp(g, odom.t,
                   np.degrees(np.unwrap(rpy_from_quat(odom.quat)[:, 2]))) + \
        math.degrees(dy)
    ye = (yc - yo + 180.0) % 360.0 - 180.0
    print("\n" + "=" * 72)
    print(f"  EKF {car.topic}  vs  sensor solo {odom.topic}")
    print("=" * 72)
    print(f"  {g.size} muestras comunes, {hi-lo:.1f} s")
    print(f"  error de posicion: RMS {np.sqrt(np.mean(err**2)):.3f} m   "
          f"max {err.max():.3f} m   final {err[-1]:.3f} m")
    print(f"  error de yaw:      RMS {np.sqrt(np.mean(ye**2)):.3f} deg   "
          f"max {np.abs(ye).max():.3f} deg")
    print("  (el sensor se alinea con el primer estado del EKF; la deriva")
    print("   acumulada es lo que corrige el filtro)")


def report_posearray(c: PoseArrayC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — PoseArray")
    print("=" * 72)
    print(f"  {b['n']} mensajes, {b['hz']:.2f} Hz, frame '{c.frame}'")
    if not c.counts.size:
        return
    print(f"  poses por mensaje: mediana {np.median(c.counts):.0f}  "
          f"max {int(c.counts.max())}")
    ok = np.isfinite(c.centroid[:, 0])
    if ok.any():
        print(f"  centroide medio ({c.centroid[ok, 0].mean():+.2f}, "
              f"{c.centroid[ok, 1].mean():+.2f}) m   "
              f"dispersion max {c.spread[ok].max():.2f} m")


def report_path(c: PathC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Path")
    print("=" * 72)
    print(f"  {b['n']} mensajes, {b['hz']:.2f} Hz, frame '{c.frame}'")
    if c.counts.size:
        print(f"  poses por path: mediana {np.median(c.counts):.0f}  "
              f"max {int(c.counts.max())}")
        print(f"  longitud del path: max {c.lengths.max():.1f} m")


def report_scan(c: ScanC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — LaserScan")
    print("=" * 72)
    print(f"  {b['n']} barridos, {b['hz']:.2f} Hz, {b['gaps']} huecos, "
          f"{human(b['bytes'])} ({b['bw']/1024/1024:.2f} MB/s)")
    print(f"  frame '{c.frame}'   FOV {math.degrees(c.angle_max-c.angle_min):.1f} deg   "
          f"incremento {math.degrees(c.angle_inc):.3f} deg")
    print(f"  rango declarado [{c.range_min:.3f}, {c.range_max:.3f}] m   "
          f"scan_time {c.scan_time*1e3:.1f} ms")
    if not c.stats.size:
        return
    s = c.stats
    valid_ratio = np.divide(s[:, 1], s[:, 0], out=np.zeros_like(s[:, 1]),
                            where=s[:, 0] > 0)
    print(f"  puntos por barrido: mediana {np.median(s[:, 0]):.0f}")
    print(f"  puntos validos: mediana {np.median(s[:, 1]):.0f} "
          f"({100*np.median(valid_ratio):.1f} %)")
    ok = np.isfinite(s[:, 3])
    if ok.any():
        print(f"  rango medio {np.nanmedian(s[ok, 3]):.3f} m   "
              f"min {np.nanmin(s[ok, 2]):.3f}   max {np.nanmax(s[ok, 5]):.3f}")
    if np.median(valid_ratio) < 0.05:
        print("  ATENCION: casi todos los haces son inf/NaN. Puede ser un")
        print("            escaner sin retorno, un frame mal montado o datos en crudo.")


def report_image(c: ImageC, b: Dict) -> None:
    kind = "CompressedImage" if isinstance(c, CompressedImageC) else "Image"
    print("\n" + "=" * 72)
    print(f"  {c.topic} — {kind}")
    print("=" * 72)
    print(f"  {b['n']} mensajes, {b['hz']:.2f} Hz, {b['gaps']} huecos, "
          f"{human(b['bytes'])} ({b['bw']/1024/1024:.2f} MB/s)")
    if c.metadata.size:
        w, h = c.metadata[:, 0], c.metadata[:, 1]
        bw = c.metadata[:, 3]
        if np.any(w > 0):
            print(f"  resolucion {int(np.median(w))}x{int(np.median(h))}   "
                  f"encoding '{c.encoding or c.format}'")
        print(f"  tamano por frame: mediana {human(float(np.median(bw)))}, "
              f"max {human(float(bw.max()))}")
    n_dec = len(c.thumbs)
    print(f"  frames decodificados para vista previa: {n_dec}")
    if c.image_stat.size and np.isfinite(c.image_stat[:, 0]).any():
        m = c.image_stat[:, 0]
        print(f"  brillo medio {np.nanmean(m):.1f}   "
              f"contraste (sigma) {np.nanmean(c.image_stat[:, 1]):.1f}")
    if n_dec < 2 and isinstance(c, CompressedImageC):
        print("  (instala opencv-python para decodificar JPEG/PNG)")


def report_range(c: RangeC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Range")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, frame '{c.frame}'")
    if not c.range.size:
        return
    r = c.range
    valid = np.isfinite(r) & (r >= c.min_range) & (r <= c.max_range)
    print(f"  rango valido {int(valid.sum())}/{r.size}   "
          f"min {np.nanmin(r):.3f}  media {np.nanmean(r):.3f}  "
          f"max {np.nanmax(r):.3f} m")
    print(f"  limites declarados [{c.min_range[0]:.3f}, {c.max_range[0]:.3f}] m   "
          f"FOV {math.degrees(c.field_of_view[0]):.1f} deg   "
          f"tipo {int(c.radiation[0])}")


def report_battery(c: BatteryC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — BatteryState")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz")
    if not c.data.size:
        return
    labels = ("voltaje [V]", "temperatura [degC]", "corriente [A]",
              "carga [Ah]", "capacidad [Ah]", "capacidad diseno [Ah]",
              "porcentaje [0-1]", "estado", "salud", "tecnologia", "presente")
    for i, lab in enumerate(labels):
        v = c.data[:, i]
        if np.all(np.isnan(v)):
            continue
        print(f"  {lab:<22} media {np.nanmean(v):>10.4g}   "
              f"min {np.nanmin(v):>10.4g}   max {np.nanmax(v):>10.4g}")
    if c.cell_voltage.size and np.isfinite(c.cell_voltage).any():
        print(f"  celda: voltaje medio {np.nanmean(c.cell_voltage):.3f} V   "
              f"temperatura media {np.nanmean(c.cell_temperature):.2f} degC")
    pct = c.data[:, 6]
    if np.isfinite(pct).any() and np.nanmax(pct) <= 1.0:
        print("  (el porcentaje viene en 0..1, no en 0..100)")


def report_joint(c: JointC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — JointState")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, {b['gaps']} huecos")
    if not c.names:
        return
    print(f"  {len(c.names)} joints")
    w = max(len(n) for n in c.names) + 2
    print(f"  {'joint':<{w}}{'pos min':>10}{'pos max':>10}"
          f"{'vel max':>10}{'eff max':>10}")
    for n in c.names:
        p = c.position[n]
        v = c.velocity[n]
        e = c.effort[n]
        rng = (f"{np.nanmin(p):>10.4g}{np.nanmax(p):>10.4g}" if np.isfinite(p).any()
               else f"{'-':>10}{'-':>10}")
        vm = f"{np.nanmax(np.abs(v)):>10.4g}" if np.isfinite(v).any() else f"{'-':>10}"
        em = f"{np.nanmax(np.abs(e)):>10.4g}" if np.isfinite(e).any() else f"{'-':>10}"
        print(f"  {n:<{w}}{rng}{vm}{em}")


def report_diag(c: DiagC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — DiagnosticArray")
    print("=" * 72)
    print(f"  {b['n']} mensajes, {b['hz']:.2f} Hz")
    if not c.rows:
        return
    lv = np.asarray([r[1] for r in c.rows])
    names = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
    counts = {int(k): int((lv == k).sum()) for k in np.unique(lv)}
    print("  estados: " + "  ".join(f"{names.get(k, k)} {v}" for k, v in counts.items()))
    for name, (level, message, values) in sorted(c.latest.items()):
        flag = names.get(level, str(level))
        print(f"  [{flag:<5}] {name}: {message[:120]}")
        if level >= 1 and values:
            print("          " + "  ".join(f"{k}={v}" for k, v in values[:6]))


def report_scalar(c: ScalarC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — {c.type.split('/')[-1]}")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz, {b['gaps']} huecos")
    if not c.value.size:
        return
    for i, lab in enumerate(c.labels):
        v = c.value[:, i]
        v = v[np.isfinite(v)]
        if not v.size:
            continue
        note = "  (constante)" if v.std() == 0 else ""
        print(f"  {lab:<16} media {v.mean():>12.5g}   min {v.min():>12.5g}   "
              f"max {v.max():>12.5g}   sigma {v.std():>10.4g} {c.unit}{note}")
    if c.value.shape[1] == 3:
        mag = np.linalg.norm(c.value, axis=1)
        print(f"  |vector| media {np.nanmean(mag):.5g}   max {np.nanmax(mag):.5g} {c.unit}")


def report_clock(c: ClockC, b: Dict) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — Clock")
    print("=" * 72)
    print(f"  {b['n']} muestras, {b['hz']:.2f} Hz")
    if c.clock.size < 2:
        return
    d = np.diff(c.clock)
    print(f"  avance del reloj {d.sum():.3f} s   "
          f"intervalo mediano {np.median(d)*1e3:.3f} ms")
    if np.any(d < 0):
        print(f"  ATENCION: {int((d < 0).sum())} saltos hacia atras")
    if np.any(d == 0):
        print(f"  ATENCION: {int((d == 0).sum())} repeticiones de reloj")
    if c.t.size == c.clock.size:
        ratio = (c.clock[-1] - c.clock[0]) / (c.t[-1] - c.t[0]) if c.t[-1] > c.t[0] else float("nan")
        print(f"  reloj / tiempo del bag = {ratio:.6f} (1.0 = sin deriva)")


def report_tf(c: TfC) -> None:
    print("\n" + "=" * 72)
    print(f"  {c.topic} — {len(c.tf)} transformadas")
    print("=" * 72)
    for (par, chi), (x, y, z, rpy) in sorted(c.tf.items()):
        print(f"  {par} -> {chi}")
        print(f"      xyz [{x:+.4f} {y:+.4f} {z:+.4f}] m   "
              f"rpy [{rpy[0]:+.2f} {rpy[1]:+.2f} {rpy[2]:+.2f}] deg")
    print("\n  El TF de ROS va en FLU (X adelante, Y izquierda, Z arriba) y los")
    print("  registros del VN-300 en FRD: al pasar un offset de un sitio al")
    print("  otro cambian de signo la Y y la Z.")


def report_log(c: LogC, limit: int) -> None:
    bad = [r for r in c.rows if r[0] >= 30]
    print("\n" + "=" * 72)
    print(f"  {c.topic} — {len(bad)} avisos de {len(c.rows)} mensajes")
    print("=" * 72)
    if not bad:
        print("  Sin WARN ni ERROR.")
        return
    agg: Dict[Tuple[str, str, str], int] = defaultdict(int)
    for lv, nm, ms, _ in bad:
        agg[(SEV.get(lv, str(lv)), nm, ms)] += 1
    for (sev, node, txt), k in sorted(agg.items(), key=lambda kv: -kv[1])[:limit]:
        print(f"  [{sev:<5}] {node}{f' x{k}' if k > 1 else ''}")
        print(f"          {txt[:160]}")


def report_generic(c: GenericC, b: Dict) -> None:
    if not c.data:
        return
    print("\n" + "=" * 72)
    print(f"  {c.topic} — {c.type}")
    print("=" * 72)
    print(f"  {b['n']} mensajes, {b['hz']:.2f} Hz, {b['gaps']} huecos "
          f"({len(next(iter(c.data.values())))} deserializados)")
    w = max(len(k) for k in c.data) + 2
    print(f"  {'campo':<{w}}{'media':>13}{'min':>13}{'max':>13}{'sigma':>12}")
    for k in sorted(c.data):
        a = c.data[k]
        a = a[np.isfinite(a)]
        if a.size == 0:
            continue
        note = ""
        if a.std() == 0:
            note = "  (constante)"
            if a[0] == 0:
                note = "  (SIEMPRE CERO)"
        print(f"  {k:<{w}}{a.mean():>13.5g}{a.min():>13.5g}{a.max():>13.5g}"
              f"{a.std():>12.5g}{note}")


# --------------------------------------------------------------------------- #
#  Figuras
# --------------------------------------------------------------------------- #

def setup_style(headless: bool) -> None:
    import matplotlib
    if headless or (sys.platform.startswith("linux") and not os.environ.get("DISPLAY")):
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


def blue_cmap():
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list("vn_blue", BLUE_RAMP)


def envelope(t: np.ndarray, y: np.ndarray, target: int = 3000):
    """
    Reduce series largas conservando minimo y maximo de cada tramo.

    Submuestrear uno de cada N esconde los picos, que en un log de vibracion
    son justo lo que interesa ver.
    """
    n = t.size
    if n <= target * 2:
        return t, y
    e = np.linspace(0, n, target + 1).astype(int)
    ts, ys = [], []
    for a, b in zip(e[:-1], e[1:]):
        if b <= a:
            continue
        lo = a + int(np.nanargmin(y[a:b])) if np.any(np.isfinite(y[a:b])) else a
        hi = a + int(np.nanargmax(y[a:b])) if np.any(np.isfinite(y[a:b])) else a
        i, j = (lo, hi) if lo <= hi else (hi, lo)
        ts.extend((t[i], t[j]))
        ys.extend((y[i], y[j]))
    return np.asarray(ts), np.asarray(ys)


def line(ax, t, y, color, label, lw=1.0):
    te, ye = envelope(t, y)
    ax.plot(te, ye, color=color, lw=lw, label=label)


def finish(ax, ylabel, title, legend=True):
    """La leyenda va fuera del area de datos: con trazas densas cualquier
    posicion interior acaba tapando senal."""
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title, loc="left", pad=6)
    if legend:
        ax.legend(loc="center left", bbox_to_anchor=(1.005, 0.5),
                  handlelength=1.6, borderaxespad=0.0)


def stats_text(ax, text):
    """A la derecha, en la fila del titulo, para que nunca colisionen."""
    ax.text(1.0, 1.02, text, transform=ax.transAxes, fontsize=8,
            color=INK_MUTED, va="bottom", ha="right")


def pad_ylim(ax, y, span):
    lo, hi = float(np.nanmin(y)), float(np.nanmax(y))
    if hi - lo < span:
        c = 0.5 * (lo + hi)
        ax.set_ylim(c - span / 2, c + span / 2)


def fig_imu(c: ImuC, b: Dict, s: Dict, t0: float):
    import matplotlib.pyplot as plt
    from scipy.signal import welch
    t, a, g = c.t - t0, c.accel, np.degrees(c.gyro)
    fs = b["hz"] if math.isfinite(b["hz"]) else 100.0

    fig = plt.figure(figsize=(13.5, 13.0), layout="constrained")
    gs = fig.add_gridspec(4, 2, height_ratios=[3, 3, 2.2, 2.6])
    ax_a, ax_g = fig.add_subplot(gs[0, :]), None
    ax_g = fig.add_subplot(gs[1, :], sharex=ax_a)
    ax_n = fig.add_subplot(gs[2, 0], sharex=ax_a)
    ax_r = fig.add_subplot(gs[2, 1], sharex=ax_a)
    ax_pa, ax_pg = fig.add_subplot(gs[3, 0]), fig.add_subplot(gs[3, 1])

    for i, k in enumerate("xyz"):
        line(ax_a, t, a[:, i], AXIS_COLOR[k], k)
    finish(ax_a, "m/s²", "Aceleración lineal")
    stats_text(ax_a, "sigma  x {:.4f}  y {:.4f}  z {:.4f}  m/s²".format(*s["sigma_a"]))
    for i, k in enumerate("xyz"):
        line(ax_g, t, g[:, i], AXIS_COLOR[k], k)
    finish(ax_g, "°/s", "Velocidad angular")
    stats_text(ax_g, "sigma  x {:.4f}  y {:.4f}  z {:.4f}  °/s".format(*s["sigma_g"]))

    nrm = np.linalg.norm(a, axis=1)
    line(ax_n, t, nrm, BLUE, "|a|")
    ax_n.axhline(G0, color=INK_MUTED, lw=1.0, ls="--", label="9.807")
    finish(ax_n, "m/s²", "Módulo de la aceleración")
    stats_text(ax_n, f"media {nrm.mean():.4f}")
    ax_n.set_xlabel("tiempo [s]")

    if s["has_quat"]:
        rpy = np.degrees(rpy_from_quat(c.quat))
        line(ax_r, t, rpy[:, 0], BLUE, "roll")
        line(ax_r, t, rpy[:, 1], ORANGE, "pitch")
        line(ax_r, t, np.degrees(np.unwrap(np.radians(rpy[:, 2]))), AQUA, "yaw")
        finish(ax_r, "grados", "Orientación (yaw desenrollado)")
    else:
        ax_r.set_title("Orientación", loc="left", pad=6)
        ax_r.text(0.5, 0.5, "el mensaje no trae cuaternión", ha="center",
                  va="center", transform=ax_r.transAxes, color=INK_MUTED)
        ax_r.set_yticks([])
    ax_r.set_xlabel("tiempo [s]")

    nper = int(min(len(t), max(256, fs * 8)))
    for i, k in enumerate("xyz"):
        f, p = welch(a[:, i] - a[:, i].mean(), fs=fs, nperseg=nper)
        ax_pa.loglog(f[1:], np.sqrt(p[1:]), color=AXIS_COLOR[k], lw=1.1, label=k)
        f, p = welch(g[:, i] - g[:, i].mean(), fs=fs, nperseg=nper)
        ax_pg.loglog(f[1:], np.sqrt(p[1:]), color=AXIS_COLOR[k], lw=1.1, label=k)
    ax_pa.axhline(SPEC_ACCEL_DENSITY, color=INK_MUTED, lw=1.0, ls="--",
                  label="spec 0.14 mg/√Hz")
    ax_pg.axhline(SPEC_GYRO_DENSITY, color=INK_MUTED, lw=1.0, ls="--",
                  label="spec 0.0035 °/s/√Hz")
    for axp, lab, ti in ((ax_pa, "m/s² / √Hz", "Espectro de aceleración"),
                         (ax_pg, "°/s / √Hz", "Espectro de velocidad angular")):
        axp.set_xlabel("frecuencia [Hz]")
        axp.grid(True, which="both", alpha=0.9)
        finish(axp, lab, ti)
    stats_text(ax_pg, f"Nyquist {fs/2:.0f} Hz")

    warn = f"   ·   {b['gaps']} huecos" if b["gaps"] else ""
    fig.suptitle(f"{c.topic}   ({b['n']} muestras, {b['hz']:.1f} Hz, "
                 f"{b['dur']:.0f} s){warn}", x=0.01, ha="left")
    return fig


def fig_timing(c: Base, t0: float):
    import matplotlib.pyplot as plt
    if c.t_hdr.size != c.t_bag.size or not np.any(c.t_hdr > 0):
        return None
    dt_ms = np.diff(c.t_hdr) * 1e3
    lat_ms = (c.t_bag - c.t_hdr) * 1e3
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.2), layout="constrained")
    span = float(np.max(dt_ms) - np.min(dt_ms))
    ax[0].hist(dt_ms, bins=200 if span > 1e-6 else 11, color=BLUE, edgecolor="none")
    ax[0].set_yscale("log")
    ax[0].set_xlabel("intervalo entre stamps [ms]")
    ax[0].set_ylabel("cuentas")
    ax[0].set_title("Distribución de intervalos", loc="left", pad=6)
    stats_text(ax[0], f"mediana {np.median(dt_ms):.3f}  sigma {np.std(dt_ms):.3f} ms")
    line(ax[1], c.t - t0, lat_ms, ORANGE, "latencia")
    pad_ylim(ax[1], lat_ms, 1.0)
    ax[1].set_xlabel("tiempo [s]")
    ax[1].set_ylabel("ms")
    ax[1].set_title("Latencia stamp → recepción", loc="left", pad=6)
    stats_text(ax[1], "un escalón delata el cambio de reloj")
    fig.suptitle(f"Temporización — {c.topic}", x=0.01, ha="left")
    return fig


def fig_imu_compare(imus: Dict[str, ImuC], t0: float, notes: Sequence[str]):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(13.5, 9.5), layout="constrained")
    gs = fig.add_gridspec(3, 1, height_ratios=[3, 3, 2])
    ax_n, ax_g = fig.add_subplot(gs[0]), None
    ax_g = fig.add_subplot(gs[1], sharex=ax_n)
    ax_t = fig.add_subplot(gs[2])
    # Grosor decreciente: si dos IMU coinciden, la de encima taparia a la otra
    # y no se sabria si hay una o dos. Asi la de abajo asoma como halo.
    for i, (n, c) in enumerate(imus.items()):
        col, lw = SERIES[i % len(SERIES)], max(0.8, 1.8 - 0.5 * i)
        line(ax_n, c.t - t0, np.linalg.norm(c.accel, axis=1), col, n, lw=lw)
        line(ax_g, c.t - t0, np.degrees(np.linalg.norm(c.gyro, axis=1)), col, n, lw=lw)
    ax_n.axhline(G0, color=INK_MUTED, lw=1.0, ls="--", label="9.807")
    finish(ax_n, "m/s²", "Módulo de la aceleración — todas las IMU")
    stats_text(ax_n, "todas deben rondar 9.807 en reposo")
    finish(ax_g, "°/s", "Módulo de la velocidad angular — todas las IMU")
    stats_text(ax_g, "si una va adelantada, los relojes no están alineados")
    ax_g.set_xlabel("tiempo [s] desde el inicio del bag")
    ax_t.axis("off")
    ax_t.set_title("Resultados", loc="left", pad=6)
    ax_t.text(0.0, 0.95, "\n".join(notes), transform=ax_t.transAxes, va="top",
              ha="left", fontsize=9, color=INK_SOFT, family="monospace")
    fig.suptitle(f"Comparación entre {len(imus)} IMU", x=0.01, ha="left")
    return fig


def fig_odom(c: OdomC, b: Dict, t0: float, cons: Dict):
    import matplotlib.pyplot as plt
    t = c.t - t0
    x, y = c.pos[:, 0], c.pos[:, 1]
    yaw = np.degrees(np.unwrap(rpy_from_quat(c.quat)[:, 2]))

    fig = plt.figure(figsize=(13.5, 9.0), layout="constrained")
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.15])
    a_tr = fig.add_subplot(gs[:, 0])
    a_y = fig.add_subplot(gs[0, 1])
    a_v = fig.add_subplot(gs[1, 1], sharex=a_y)
    a_w = fig.add_subplot(gs[2, 1], sharex=a_y)

    sc = a_tr.scatter(y, x, c=t, cmap=blue_cmap(), s=3, linewidths=0)
    a_tr.plot(y[0], x[0], "o", ms=8, mfc="none", mec=INK, mew=1.4)
    a_tr.annotate("inicio", (y[0], x[0]), textcoords="offset points",
                  xytext=(10, 6), fontsize=8.5, color=INK_SOFT)
    a_tr.set_aspect("equal", adjustable="datalim")
    a_tr.set_xlabel("y [m]")
    a_tr.set_ylabel("x [m]")
    a_tr.set_title(f"Trayectoria en '{c.frame}'", loc="left", pad=6)
    cb = fig.colorbar(sc, ax=a_tr, pad=0.02, fraction=0.045)
    cb.set_label("tiempo [s]", color=INK_SOFT, fontsize=8.5)
    cb.outline.set_visible(False)

    line(a_y, t, yaw, BLUE, "yaw")
    finish(a_y, "grados", "Rumbo (desenrollado)", legend=False)
    line(a_v, t, c.vel[:, 0], BLUE, "linear.x")
    line(a_v, t, c.vel[:, 1], ORANGE, "linear.y")
    finish(a_v, "m/s", "Velocidad en el cuerpo")
    line(a_w, t, np.degrees(c.omega[:, 2]), AQUA, "angular.z")
    finish(a_w, "°/s", "Velocidad de guiñada", legend=False)
    a_w.set_xlabel("tiempo [s]")

    msg = ""
    if cons.get("best"):
        ok = cons["best"] == "yaw tal cual"
        msg = ("   ·   frames coherentes" if ok else
               f"   ·   INCOHERENTE: cuadra '{cons['best']}'")
    fig.suptitle(f"{c.topic}   ({b['n']} muestras, {b['hz']:.1f} Hz){msg}",
                 x=0.01, ha="left")
    return fig


def fig_fix(c: FixC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    ok = np.isfinite(c.lat) & np.isfinite(c.lon) & (np.abs(c.lat) > 1e-9)
    if ok.sum() < 5:
        return None
    north, east, _, _ = lla_to_ne(c.lat[ok], c.lon[ok])
    t = c.t[ok] - t0
    fig = plt.figure(figsize=(13.5, 6.4), layout="constrained")
    gs = fig.add_gridspec(3, 2, width_ratios=[1.05, 1])
    a_tr = fig.add_subplot(gs[:, 0])
    a_al = fig.add_subplot(gs[0, 1])
    a_r = fig.add_subplot(gs[1, 1], sharex=a_al)
    a_st = fig.add_subplot(gs[2, 1], sharex=a_al)

    sc = a_tr.scatter(east, north, c=t, cmap=blue_cmap(), s=3, linewidths=0)
    a_tr.set_aspect("equal", adjustable="datalim")
    a_tr.set_xlabel("este [m]")
    a_tr.set_ylabel("norte [m]")
    a_tr.set_title("Trayectoria relativa al centroide", loc="left", pad=6)
    cb = fig.colorbar(sc, ax=a_tr, pad=0.02, fraction=0.045)
    cb.set_label("tiempo [s]", color=INK_SOFT, fontsize=8.5)
    cb.outline.set_visible(False)

    line(a_al, t, c.alt[ok], BLUE, "altitud")
    finish(a_al, "m", "Altitud", legend=False)
    stats_text(a_al, f"sigma {c.alt[ok].std():.3f} m")
    r = np.hypot(north, east)
    line(a_r, t, r, ORANGE, "radio")
    static = r.max() < 10.0
    finish(a_r, "m", "Dispersión radial" if static else "Distancia al centroide",
           legend=False)
    stats_text(a_r, "CEP50 {:.3f}  CEP95 {:.3f} m".format(*np.percentile(r, [50, 95]))
               if static else f"recorrido {r.max():.0f} m — el CEP no aplica")
    a_st.step(c.t - t0, c.status, where="post", color=AQUA, lw=1.2)
    finish(a_st, "status", "Estado del fix", legend=False)
    a_st.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} fixes, {b['hz']:.1f} Hz)", x=0.01, ha="left")
    return fig


def fig_cloud(c: CloudC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.2), layout="constrained")
    line(ax[0], c.t_bag - t0, c.sizes / 1024 / 1024, BLUE, "tamaño")
    finish(ax[0], "MB", "Tamaño por nube", legend=False)
    stats_text(ax[0], f"mediana {np.median(c.sizes)/1024/1024:.2f} MB")
    ax[0].set_xlabel("tiempo [s]")
    dt = np.diff(c.t_bag) * 1e3
    ax[1].hist(dt, bins=100, color=ORANGE, edgecolor="none")
    ax[1].set_yscale("log")
    ax[1].set_xlabel("intervalo entre nubes [ms]")
    ax[1].set_ylabel("cuentas")
    ax[1].set_title("Distribución de intervalos", loc="left", pad=6)
    stats_text(ax[1], f"mediana {np.median(dt):.1f} ms")
    fig.suptitle(f"{c.topic}   ({b['n']} nubes, {b['hz']:.1f} Hz, "
                 f"{b['bw']/1024/1024:.2f} MB/s)", x=0.01, ha="left")
    return fig


def _yaw_deg(c) -> Optional[np.ndarray]:
    if getattr(c, "quat", None) is None or not c.quat.size:
        return None
    return np.degrees(np.unwrap(rpy_from_quat(c.quat)[:, 2]))


def fig_pose(c: PoseC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig = plt.figure(figsize=(13.5, 8.6), layout="constrained")
    gs = fig.add_gridspec(2, 2)
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_t = fig.add_subplot(gs[0, 1])
    ax_y = fig.add_subplot(gs[1, 0])
    ax_v = fig.add_subplot(gs[1, 1])

    x, y = c.pos[:, 0], c.pos[:, 1]
    ax_xy.plot(x, y, color=BLUE, lw=1.0)
    ax_xy.scatter([x[0]], [y[0]], color=AQUA, s=30, marker="o", label="inicio")
    ax_xy.scatter([x[-1]], [y[-1]], color=ORANGE, s=30, marker="s", label="fin")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Trayectoria", loc="left", pad=6)
    ax_xy.legend(loc="best")

    line(ax_t, t, x, BLUE, "x")
    line(ax_t, t, y, ORANGE, "y")
    finish(ax_t, "m", "Posición")
    ax_t.set_xlabel("tiempo [s]")

    yaw = _yaw_deg(c)
    if yaw is not None:
        line(ax_y, t, yaw, AQUA, "yaw")
        finish(ax_y, "grados", "Yaw (desenrollado)")
    else:
        ax_y.set_visible(False)
    ax_y.set_xlabel("tiempo [s]")

    if c.t.size > 3 and np.all(np.diff(c.t) > 0):
        v = np.hypot(np.gradient(x, c.t), np.gradient(y, c.t))
        line(ax_v, t, v, BLUE, "|v| derivada")
        finish(ax_v, "m/s", "Velocidad")
        stats_text(ax_v, f"max {v.max():.2f} m/s")
    else:
        ax_v.set_visible(False)
    ax_v.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} muestras, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def fig_twist(c: TwistC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(2, 2, figsize=(13.5, 7.0), layout="constrained")
    for i, k in enumerate("xyz"):
        line(ax[0, 0], t, c.vel[:, i], AXIS_COLOR[k], f"v{k}")
        line(ax[1, 0], t, np.degrees(c.omega[:, i]), AXIS_COLOR[k], f"w{k}")
    finish(ax[0, 0], "m/s", "Velocidad lineal")
    finish(ax[1, 0], "grados/s", "Velocidad angular")
    line(ax[0, 1], t, np.linalg.norm(c.vel, axis=1), BLUE, "|v|")
    finish(ax[0, 1], "m/s", "Módulo lineal")
    line(ax[1, 1], t, np.degrees(np.linalg.norm(c.omega, axis=1)), ORANGE, "|w|")
    finish(ax[1, 1], "grados/s", "Módulo angular")
    for a in ax.ravel():
        a.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} muestras, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def _fig_state(c, b: Dict, t0: float, title: str):
    import matplotlib.pyplot as plt
    t = c.t - t0
    x, y = c.pos[:, 0], c.pos[:, 1]
    fig = plt.figure(figsize=(13.5, 8.6), layout="constrained")
    gs = fig.add_gridspec(2, 2)
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_t = fig.add_subplot(gs[0, 1])
    ax_y = fig.add_subplot(gs[1, 0])
    ax_v = fig.add_subplot(gs[1, 1])

    ax_xy.plot(x, y, color=BLUE, lw=1.0)
    ax_xy.scatter([x[0]], [y[0]], color=AQUA, s=30, marker="o", label="inicio")
    ax_xy.scatter([x[-1]], [y[-1]], color=ORANGE, s=30, marker="s", label="fin")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Trayectoria", loc="left", pad=6)
    ax_xy.legend(loc="best")

    line(ax_t, t, x, BLUE, "x")
    line(ax_t, t, y, ORANGE, "y")
    cov = getattr(c, "pose_cov", None)
    if cov is not None and cov.size and np.any(np.isfinite(cov)):
        sx = 2.0 * np.sqrt(np.maximum(cov[:, 0], 0.0))
        sy = 2.0 * np.sqrt(np.maximum(cov[:, 1], 0.0))
        ax_t.fill_between(t, x - sx, x + sx, color=BLUE, alpha=0.10, lw=0)
        ax_t.fill_between(t, y - sy, y + sy, color=ORANGE, alpha=0.10, lw=0)
        stats_text(ax_t, "banda: ±2σ")
    finish(ax_t, "m", "Posición")
    ax_t.set_xlabel("tiempo [s]")

    yaw = _yaw_deg(c)
    if yaw is not None:
        line(ax_y, t, yaw, AQUA, "yaw")
    finish(ax_y, "grados", "Yaw (desenrollado)")
    ax_y.set_xlabel("tiempo [s]")

    line(ax_v, t, np.hypot(c.vel[:, 0], c.vel[:, 1]), BLUE, "|v|")
    finish(ax_v, "m/s", "Velocidad")
    ax_v.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic} — {title}   ({b['n']} muestras, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def fig_car(c: CarC, b: Dict, t0: float):
    return _fig_state(c, b, t0, "Car")


def fig_scan(c: ScanC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig = plt.figure(figsize=(13.5, 8.6), layout="constrained")
    gs = fig.add_gridspec(2, 2)
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_p = fig.add_subplot(gs[0, 1])
    ax_t = fig.add_subplot(gs[1, 0])
    ax_v = fig.add_subplot(gs[1, 1])

    if c.last is not None:
        ang, r, ok = c.last["angle"], c.last["range"], c.last["valid"]
        rr = r[ok]
        aa = ang[ok]
        ax_xy.plot(rr * np.cos(aa), rr * np.sin(aa), ".", color=BLUE, ms=2)
        ax_xy.scatter([0], [0], color=ORANGE, s=26, marker="^")
        ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Último barrido (vista XY)", loc="left", pad=6)

    if c.last is not None:
        ax_p.plot(np.degrees(c.last["angle"]), c.last["range"], ".", color=BLUE, ms=2)
    ax_p.set_xlabel("ángulo [grados]")
    ax_p.set_ylabel("rango [m]")
    ax_p.set_title("Rango por haz", loc="left", pad=6)

    if c.stats.size:
        s = c.stats
        line(ax_t, t, s[:, 3], BLUE, "medio")
        line(ax_t, t, s[:, 2], AQUA, "min")
        line(ax_t, t, s[:, 5], ORANGE, "max")
        finish(ax_t, "m", "Rango por barrido")
        valid_ratio = np.divide(s[:, 1], s[:, 0], out=np.zeros_like(s[:, 1]),
                                where=s[:, 0] > 0)
        line(ax_v, t, 100 * valid_ratio, ORANGE, "válidos [%]")
        finish(ax_v, "%", "Haces con retorno")
        stats_text(ax_v, f"mediana {100*np.median(valid_ratio):.1f} %")
    for a in (ax_t, ax_v):
        a.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} barridos, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def fig_image(c: ImageC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    n = max(1, len(c.thumbs))
    fig = plt.figure(figsize=(13.5, 8.0), layout="constrained")
    gs = fig.add_gridspec(2, n)
    for i, (ts, g, r) in enumerate(c.thumbs):
        ax = fig.add_subplot(gs[0, i])
        ax.imshow(r if r is not None else g,
                  cmap=None if r is not None else "gray")
        ax.set_title(f"t = {ts - t0:.2f} s", fontsize=9, loc="left", pad=3)
        ax.set_axis_off()
    if not c.thumbs:
        ax = fig.add_subplot(gs[0, :])
        ax.text(0.5, 0.5, "sin frames decodificables (instala opencv-python)",
                ha="center", va="center", transform=ax.transAxes, color=INK_MUTED)
        ax.set_axis_off()

    ax_b = fig.add_subplot(gs[1, :])
    t = c.t - t0
    if c.image_stat.size:
        line(ax_b, t, c.image_stat[:, 0], BLUE, "brillo medio")
        line(ax_b, t, c.image_stat[:, 1], ORANGE, "contraste (sigma)")
    finish(ax_b, "nivel", "Estadística de la imagen")
    ax_b.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} mensajes, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def fig_range(c: RangeC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.3), layout="constrained")
    line(ax[0], t, c.range, BLUE, "rango")
    ax[0].axhline(c.min_range[0], color=INK_MUTED, ls="--", lw=1.0, label="min")
    ax[0].axhline(c.max_range[0], color=INK_MUTED, ls=":", lw=1.0, label="max")
    finish(ax[0], "m", "Distancia")
    ax[0].set_xlabel("tiempo [s]")
    r = c.range[np.isfinite(c.range)]
    ax[1].hist(r, bins=100, color=BLUE, edgecolor="none")
    ax[1].set_xlabel("rango [m]")
    ax[1].set_ylabel("cuentas")
    ax[1].set_title("Distribución", loc="left", pad=6)
    fig.suptitle(f"{c.topic}   ({b['n']} muestras)", x=0.01, ha="left")
    return fig


def fig_battery(c: BatteryC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(2, 2, figsize=(13.5, 7.0), layout="constrained")
    for a, col, lab, ttl in (
            (ax[0, 0], 0, "V", "Voltaje"),
            (ax[0, 1], 2, "A", "Corriente"),
            (ax[1, 0], 6, "0..1", "Porcentaje"),
            (ax[1, 1], 1, "degC", "Temperatura")):
        line(a, t, c.data[:, col], BLUE, lab)
        finish(a, lab, ttl)
        a.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} muestras)", x=0.01, ha="left")
    return fig


def fig_joint(c: JointC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    names = c.names[:8]
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 5.0), layout="constrained")
    t = c.t - t0
    for i, n in enumerate(names):
        col = SERIES[i % len(SERIES)]
        line(ax[0], t, c.position[n], col, n)
        line(ax[1], t, c.velocity[n], col, n)
    finish(ax[0], "posición", "Posición por joint")
    finish(ax[1], "velocidad", "Velocidad por joint")
    for a in ax:
        a.set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({len(c.names)} joints)", x=0.01, ha="left")
    return fig


def fig_diag(c: DiagC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.3), layout="constrained")
    if c.level.size:
        ax[0].step(t, c.level, where="post", color=ORANGE)
        ax[0].set_yticks([0, 1, 2, 3])
        ax[0].set_yticklabels(["OK", "WARN", "ERROR", "STALE"])
    ax[0].set_xlabel("tiempo [s]")
    ax[0].set_title("Peor nivel del mensaje", loc="left", pad=6)
    if c.rows:
        lv = np.asarray([r[1] for r in c.rows])
        u, n = np.unique(lv, return_counts=True)
        ax[1].bar([str(int(x)) for x in u], n, color=BLUE)
    ax[1].set_xlabel("nivel")
    ax[1].set_ylabel("cuentas")
    ax[1].set_title("Estados por nivel", loc="left", pad=6)
    fig.suptitle(f"{c.topic}   ({b['n']} mensajes)", x=0.01, ha="left")
    return fig


def fig_scalar(c: ScalarC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(1, 1, figsize=(13.5, 4.2), layout="constrained")
    for i, lab in enumerate(c.labels):
        line(ax, t, c.value[:, i],
             SERIES[i % len(SERIES)] if c.value.shape[1] > 1 else BLUE, lab)
    ax.set_xlabel("tiempo [s]")
    finish(ax, c.unit, f"{c.type.split('/')[-1]}")
    fig.suptitle(f"{c.topic}   ({b['n']} muestras, {b['hz']:.1f} Hz)",
                 x=0.01, ha="left")
    return fig


def fig_clock(c: ClockC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.3), layout="constrained")
    if c.clock.size:
        ax[0].plot(t, c.clock - c.clock[0], color=BLUE, lw=1.0)
    ax[0].set_xlabel("tiempo del bag [s]")
    ax[0].set_ylabel("avance [s]")
    ax[0].set_title("Reloj frente al bag", loc="left", pad=6)
    if c.clock.size > 1:
        d = np.diff(c.clock)
        ax[1].hist(d * 1e3, bins=100, color=BLUE, edgecolor="none")
    ax[1].set_xlabel("intervalo [ms]")
    ax[1].set_ylabel("cuentas")
    ax[1].set_title("Intervalos del reloj", loc="left", pad=6)
    fig.suptitle(f"{c.topic}   ({b['n']} muestras)", x=0.01, ha="left")
    return fig


def fig_posearray(c: PoseArrayC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 5.0), layout="constrained")
    if c.last is not None and c.last.size:
        ax[0].scatter(c.last[:, 0], c.last[:, 1], s=8, color=BLUE)
        ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    ax[0].set_title("Último PoseArray", loc="left", pad=6)
    line(ax[1], t, c.counts, BLUE, "poses")
    finish(ax[1], "poses", "Elementos por mensaje")
    ax[1].set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} mensajes)", x=0.01, ha="left")
    return fig


def fig_path(c: PathC, b: Dict, t0: float):
    import matplotlib.pyplot as plt
    t = c.t - t0
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 5.0), layout="constrained")
    if c.last is not None and c.last.size:
        ax[0].plot(c.last[:, 0], c.last[:, 1], color=BLUE, lw=1.2)
        ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    ax[0].set_title("Último path", loc="left", pad=6)
    line(ax[1], t, c.lengths, BLUE, "longitud")
    finish(ax[1], "m", "Longitud del path")
    ax[1].set_xlabel("tiempo [s]")
    fig.suptitle(f"{c.topic}   ({b['n']} mensajes)", x=0.01, ha="left")
    return fig


def fig_state_compare(car: CarC, odom: OdomC, t0: float):
    """Compara en el mismo grafico la salida del EKF con el sensor solo."""
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(13.5, 9.0), layout="constrained")
    gs = fig.add_gridspec(2, 2)
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_p = fig.add_subplot(gs[0, 1])
    ax_y = fig.add_subplot(gs[1, 0])
    ax_e = fig.add_subplot(gs[1, 1])

    tc, to = car.t - t0, odom.t - t0
    # Alinea el sensor al primer estado con una rototraslacion rígida.
    yc0 = rpy_from_quat(car.quat[:1])[0, 2]
    yo0 = rpy_from_quat(odom.quat[:1])[0, 2]
    dy = yc0 - yo0
    c, s = math.cos(dy), math.sin(dy)
    raw = odom.pos[:, :2].copy()
    raw -= odom.pos[0, :2]
    raw = np.column_stack([c * raw[:, 0] - s * raw[:, 1],
                           s * raw[:, 0] + c * raw[:, 1]]) + car.pos[0, :2]

    ax_xy.plot(car.pos[:, 0], car.pos[:, 1], color=BLUE, lw=1.2, label="EKF")
    ax_xy.plot(raw[:, 0], raw[:, 1], color=ORANGE, lw=1.0, ls="--",
               label="sensor solo")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Trayectoria", loc="left", pad=6)
    ax_xy.legend(loc="best")

    line(ax_p, tc, car.pos[:, 0], BLUE, "EKF x")
    line(ax_p, tc, car.pos[:, 1], BLUE, "EKF y")
    line(ax_p, to, raw[:, 0], ORANGE, "sensor x")
    line(ax_p, to, raw[:, 1], ORANGE, "sensor y")
    finish(ax_p, "m", "Posición")
    ax_p.set_xlabel("tiempo [s]")

    yc = _yaw_deg(car)
    yo = np.degrees(np.unwrap(rpy_from_quat(odom.quat)[:, 2])) + math.degrees(dy)
    if yc is not None:
        line(ax_y, tc, yc, BLUE, "EKF")
    line(ax_y, to, yo, ORANGE, "sensor")
    finish(ax_y, "grados", "Yaw")
    ax_y.set_xlabel("tiempo [s]")

    # Error de posicion sobre una rejilla comun.
    lo, hi = max(tc[0], to[0]), min(tc[-1], to[-1])
    if hi > lo:
        g = np.linspace(lo, hi, max(2, int((hi - lo) * 20)))
        cx = np.interp(g, tc, car.pos[:, 0])
        cy = np.interp(g, tc, car.pos[:, 1])
        rx = np.interp(g, to, raw[:, 0])
        ry = np.interp(g, to, raw[:, 1])
        err = np.hypot(cx - rx, cy - ry)
        line(ax_e, g, err, ORANGE, "|EKF - sensor|")
        finish(ax_e, "m", "Error de posición")
        stats_text(ax_e, f"RMS {np.sqrt(np.mean(err**2)):.2f} m   "
                         f"max {err.max():.2f} m")
        ax_e.set_xlabel("tiempo [s]")
    fig.suptitle(f"EKF {car.topic}  vs  sensor {odom.topic}", x=0.01, ha="left")
    return fig


def fig_generic(c: GenericC, b: Dict, t0: float, max_fields: int = 12):
    """Series temporales de cada campo numerico, en pequenos multiplos."""
    import matplotlib.pyplot as plt
    keys = [k for k in sorted(c.data) if np.any(np.isfinite(c.data[k]))]
    # Los campos constantes no dan informacion en una serie temporal.
    var = [k for k in keys if np.nanstd(c.data[k]) > 0]
    keys = (var or keys)[:max_fields]
    if not keys:
        return None
    n = len(keys)
    t = c.t[:c.data[keys[0]].size] - t0
    fig, ax = plt.subplots(n, 1, figsize=(13.5, max(2.2, 1.35 * n)),
                           sharex=True, layout="constrained", squeeze=False)
    for i, k in enumerate(keys):
        a = ax[i][0]
        y = c.data[k]
        line(a, t[:y.size], y, SERIES[i % len(SERIES)], k)
        a.set_ylabel(k, fontsize=8, rotation=0, ha="right", va="center")
        yy = y[np.isfinite(y)]
        if yy.size:
            stats_text(a, f"media {yy.mean():.4g}   sigma {yy.std():.4g}")
            pad_ylim(a, yy, abs(yy.mean()) * 1e-3 + 1e-9)
    ax[-1][0].set_xlabel("tiempo [s]")
    extra = "" if len(var) <= max_fields else f" (los {max_fields} con variación)"
    fig.suptitle(f"{c.topic} — {c.type}   ({b['n']} msgs, {b['hz']:.1f} Hz)"
                 f"{extra}", x=0.01, ha="left")
    return fig


def fig_global(col: Dict[str, Base], st: Dict[str, Dict], t0: float):
    """
    Tasa de cada topic normalizada a su mediana, en pequenos multiplos.

    Normalizar deja todos los topics comparables en el mismo eje pese a ir de
    5 a 100 Hz: 1.0 es nominal y los valles son perdidas. Se usan multiplos
    pequenos y no una leyenda porque con diez series ninguna paleta mantiene
    la separacion de color.
    """
    import matplotlib.pyplot as plt
    names = [n for n in sorted(col, key=lambda k: -st[k]["bw"])
             if col[n].t_bag.size > 10]
    if not names:
        return None
    n = len(names)
    fig, ax = plt.subplots(n + 1, 1, figsize=(13.5, 1.15 * n + 3.2),
                           layout="constrained", squeeze=False,
                           gridspec_kw={"height_ratios": [1] * n + [2.4]})
    for i, name in enumerate(names):
        a = ax[i][0]
        c = col[name]
        t = c.t_bag - t0
        dt = np.diff(t)
        med = float(np.median(dt[dt > 0])) if np.any(dt > 0) else float("nan")
        rel = (med / dt) if med > 0 else np.ones_like(dt)
        line(a, t[1:], np.clip(rel, 0, 2), BLUE, name)
        a.axhline(1.0, color=INK_MUTED, lw=0.8, ls="--")
        a.set_ylim(0, 1.6)
        a.set_yticks([0, 1])
        a.set_ylabel(name, fontsize=7.5, rotation=0, ha="right", va="center")
        stats_text(a, f"{st[name]['hz']:.1f} Hz   {st[name]['gaps']} huecos")
        if i < n - 1:
            a.set_xticklabels([])
    ax[n - 1][0].set_xlabel("tiempo [s]")

    a = ax[n][0]
    bw = np.array([st[k]["bw"] / 1024 / 1024 for k in names])
    ypos = np.arange(len(names))[::-1]
    a.barh(ypos, bw, color=BLUE, height=0.65)
    a.set_yticks(ypos)
    a.set_yticklabels(names, fontsize=8)
    a.set_xlabel("MB/s")
    a.set_title("Ancho de banda por topic", loc="left", pad=6)
    a.grid(axis="y", alpha=0)
    for yv, v in zip(ypos, bw):
        a.text(v, yv, f"  {v:.3f}", va="center", fontsize=8, color=INK_SOFT)
    fig.suptitle("Tasa relativa a la mediana (1.0 = nominal) y ancho de banda",
                 x=0.01, ha="left")
    return fig


# --------------------------------------------------------------------------- #

# --------------------------------------------------------------------------- #
#  Figuras 3D
# --------------------------------------------------------------------------- #

def _ax3d(fig, pos, title: str):
    ax = fig.add_subplot(pos, projection="3d")
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", pad=6)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_facecolor(SURFACE)
        pane.pane.set_edgecolor(GRID)
        pane._axinfo["grid"]["color"] = GRID
    return ax


def _equal_3d(ax, X, Y, Z, min_ratio: float = 0.06) -> float:
    """
    Escala honesta para los tres ejes. Devuelve el factor de exageracion en Z.

    X e Y van SIEMPRE a la misma escala: en planta las distancias tienen que
    ser reales. Con Z hay un conflicto: un coche en llano o una nube de LiDAR
    tienen metros de altura contra cientos de metros de planta, y forzar la
    misma escala deja el panel 3D liso e inutil. Asi que si el recorrido
    vertical no llega al 6 % del horizontal, Z se escala aparte y se devuelve
    el factor para declararlo en la etiqueta: exagerar callando seria mentir
    sobre la forma.
    """
    X, Y, Z = np.asarray(X, float), np.asarray(Y, float), np.asarray(Z, float)
    if X.size == 0:
        return 1.0
    rxy = max(float(np.ptp(X)), float(np.ptp(Y)), 1e-3)
    rz = max(float(np.ptp(Z)), 1e-6)
    cx, cy, cz = X.mean(), Y.mean(), Z.mean()
    ax.set_xlim(cx - rxy / 2, cx + rxy / 2)
    ax.set_ylim(cy - rxy / 2, cy + rxy / 2)
    if rz >= min_ratio * rxy:
        ax.set_zlim(cz - rxy / 2, cz + rxy / 2)
        return 1.0
    pad = rz * 0.6
    ax.set_zlim(cz - pad, cz + pad)
    return rxy / (2 * pad)


def _zlabel(ax, base: str, exag: float) -> None:
    """Etiqueta del eje Z declarando la exageracion cuando la hay."""
    ax.set_zlabel(base if exag <= 1.01 else f"{base}   (escala Z ×{exag:.0f})")


def _triad(ax, origin, R, scale: float, label: str = "",
           color: Optional[str] = None, lw: float = 1.6,
           dz: float = 0.25) -> None:
    """
    Dibuja los tres ejes de un frame: X azul, Y naranja, Z aqua.

    Con `color` se pinta la triada entera de un solo tono: se usa para la
    triada de referencia, que si llevara los colores de eje competiria por
    significado con los datos dibujados en esos mismos colores.
    """
    o = np.asarray(origin, float)
    for i, k in enumerate("xyz"):
        v = R[:, i] * scale
        ax.quiver(o[0], o[1], o[2], v[0], v[1], v[2],
                  color=color or AXIS_COLOR[k], lw=lw, arrow_length_ratio=0.18)
    if label:
        ax.text(o[0], o[1], o[2] + scale * dz, label, fontsize=8,
                color=INK_SOFT)


def fig_traj3d(c: OdomC, b: Dict, t0: float):
    """Trayectoria 3D coloreada por tiempo, con triadas de actitud."""
    import matplotlib.pyplot as plt
    x, y, z = c.pos[:, 0], c.pos[:, 1], c.pos[:, 2]
    t = c.t - t0
    fig = plt.figure(figsize=(13.5, 6.4), layout="constrained")
    ax = _ax3d(fig, 121, f"Trayectoria 3D en '{c.frame}'")
    sc = ax.scatter(x, y, z, c=t, cmap=blue_cmap(), s=2, linewidths=0)
    ax.plot([x[0]], [y[0]], [z[0]], "o", ms=8, mfc="none", mec=INK, mew=1.4)
    ex = _equal_3d(ax, x, y, z)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    _zlabel(ax, "z [m]", ex)
    cb = fig.colorbar(sc, ax=ax, pad=0.08, fraction=0.03)
    cb.set_label("tiempo [s]", color=INK_SOFT, fontsize=8.5)
    cb.outline.set_visible(False)

    ax2 = _ax3d(fig, 122, "Actitud a lo largo del recorrido")
    ax2.plot(x, y, z, color=INK_MUTED, lw=0.7)
    span = max(float(np.ptp(x)), float(np.ptp(y)), 1.0)
    k = max(1, len(x) // 24)
    for i in range(0, len(x), k):
        _triad(ax2, (x[i], y[i], z[i]), quat_to_matrix(c.quat[i]), span * 0.035)
    ex2 = _equal_3d(ax2, x, y, z)
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    _zlabel(ax2, "z [m]", ex2)
    fig.suptitle(f"{c.topic} — 3D   (X azul, Y naranja, Z aqua)",
                 x=0.01, ha="left")
    return fig


def fig_fix3d(c: FixC, b: Dict, t0: float):
    """Traza GNSS en 3D: norte, este y altitud."""
    import matplotlib.pyplot as plt
    ok = np.isfinite(c.lat) & np.isfinite(c.lon) & (np.abs(c.lat) > 1e-9)
    if ok.sum() < 10:
        return None
    north, east, _, _ = lla_to_ne(c.lat[ok], c.lon[ok])
    alt = c.alt[ok] - float(np.median(c.alt[ok]))
    t = c.t[ok] - t0
    fig = plt.figure(figsize=(9.5, 7.0), layout="constrained")
    ax = _ax3d(fig, 111, "Traza GNSS relativa al centroide")
    sc = ax.scatter(east, north, alt, c=t, cmap=blue_cmap(), s=4, linewidths=0)
    ex = _equal_3d(ax, east, north, alt)
    ax.set_xlabel("este [m]")
    ax.set_ylabel("norte [m]")
    _zlabel(ax, "altitud rel. [m]", ex)
    cb = fig.colorbar(sc, ax=ax, pad=0.1, fraction=0.03)
    cb.set_label("tiempo [s]", color=INK_SOFT, fontsize=8.5)
    cb.outline.set_visible(False)
    fig.suptitle(f"{c.topic} — 3D   (ejes a la misma escala)", x=0.01, ha="left")
    return fig


def fig_cloud3d(c: CloudC, b: Dict):
    """Una nube del LiDAR decodificada y dibujada en 3D."""
    import matplotlib.pyplot as plt
    if c.sample is None:
        return None
    d = decode_cloud(c.sample)
    if d is None or d["xyz"].shape[0] < 10:
        return None
    xyz = d["xyz"]
    # Color por el campo mas informativo disponible; si no hay, por altura.
    key = next((k for k in ("intensity", "reflectivity", "ring", "tag")
                if k in d["extra"]), None)
    cvals, clab = (d["extra"][key], key) if key else (xyz[:, 2], "z [m]")
    lo, hi = np.percentile(cvals, [2, 98])

    fig = plt.figure(figsize=(13.5, 6.6), layout="constrained")
    ax = _ax3d(fig, 121, f"Nube en '{c.frame}'")
    sc = ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=np.clip(cvals, lo, hi),
                    cmap=blue_cmap(), s=0.7, linewidths=0)
    ex = _equal_3d(ax, xyz[:, 0], xyz[:, 1], xyz[:, 2])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    _zlabel(ax, "z [m]", ex)
    cb = fig.colorbar(sc, ax=ax, pad=0.08, fraction=0.03)
    cb.set_label(clab, color=INK_SOFT, fontsize=8.5)
    cb.outline.set_visible(False)

    ax2 = fig.add_subplot(122)
    ax2.scatter(xyz[:, 0], xyz[:, 1], c=np.clip(cvals, lo, hi),
                cmap=blue_cmap(), s=0.7, linewidths=0)
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    ax2.set_title("Vista cenital", loc="left", pad=6)
    rng = float(np.linalg.norm(xyz, axis=1).max())
    stats_text(ax2, f"{d['xyz'].shape[0]} de {d['n_total']} puntos   "
                    f"alcance {rng:.1f} m")
    fig.suptitle(f"{c.topic} — nube decodificada", x=0.01, ha="left")
    return fig


def fig_tf3d(c: TfC):
    """
    El arbol de transformadas dibujado en 3D.

    Compone las cadenas desde cada raiz, asi que ves la geometria real del
    coche: donde esta cada sensor y como esta orientado. Es la comprobacion
    visual del montaje que los numeros solos no dan.
    """
    import matplotlib.pyplot as plt
    if not c.tf:
        return None
    parent = {chi: par for (par, chi) in c.tf}
    roots = sorted({par for (par, _) in c.tf} - set(parent))
    if not roots:
        roots = [sorted(parent)[0]]

    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        r: (np.zeros(3), np.eye(3)) for r in roots}
    for _ in range(len(c.tf) + 1):                 # composicion iterativa
        for (par, chi), (x, y, z, _rpy) in c.tf.items():
            if par in poses and chi not in poses:
                p0, R0 = poses[par]
                Rc = quat_to_matrix(c.quat.get((par, chi), (0, 0, 0, 1)))
                poses[chi] = (p0 + R0 @ np.array([x, y, z]), R0 @ Rc)
    if len(poses) < 2:
        return None

    P = np.array([p for p, _ in poses.values()])
    span = max(float(np.ptp(P[:, 0])), float(np.ptp(P[:, 1])),
               float(np.ptp(P[:, 2])), 0.5)
    fig = plt.figure(figsize=(9.5, 7.6), layout="constrained")
    ax = _ax3d(fig, 111, "Árbol de transformadas")
    for chi, par in parent.items():
        if chi in poses and par in poses:
            a, b_ = poses[par][0], poses[chi][0]
            ax.plot([a[0], b_[0]], [a[1], b_[1]], [a[2], b_[2]],
                    color=INK_MUTED, lw=1.0, ls="--")
    for name, (p, R) in poses.items():
        _triad(ax, p, R, span * 0.13, name)
    ex = _equal_3d(ax, P[:, 0], P[:, 1], P[:, 2])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    _zlabel(ax, "z [m]", ex)
    fig.suptitle(f"{c.topic} — {len(poses)} frames   "
                 f"(X azul, Y naranja, Z aqua; FLU de ROS)", x=0.01, ha="left")
    return fig


def fig_imu3d(imus: Dict[str, ImuC], stats: Dict[str, Dict],
              rot: Optional[Dict] = None):
    """
    Gravedad medida por cada IMU y, si se pudo estimar, la rotacion relativa.

    El vector de gravedad en ejes propios es la forma mas directa de ver como
    esta montado cada sensor. Si ademas hay rotacion relativa estimada, el
    segundo panel muestra la triada de B rotada al frame de A: cuanto mas se
    parezca a la de A, mejor cuadran.
    """
    import matplotlib.pyplot as plt
    names = list(imus)
    ncol = 2 if rot and not rot.get("insufficient") else 1
    fig = plt.figure(figsize=(6.8 * ncol + 1.5, 6.6), layout="constrained")

    ax = _ax3d(fig, 100 + ncol * 10 + 1, "Gravedad en ejes de cada IMU")
    for i, n in enumerate(names):
        s = stats[n]
        g = np.asarray(s.get("a_static", [0, 0, 0]), float)
        if not np.any(g):
            continue
        g = g / (np.linalg.norm(g) or 1.0)
        col = SERIES[i % len(SERIES)]
        ax.quiver(0, 0, 0, g[0], g[1], g[2], color=col, lw=2.2,
                  arrow_length_ratio=0.15,
                  label=f"{n}  (pitch {s['pitch']:+.1f}, roll {s['roll']:+.1f})")
    _triad(ax, (0, 0, 0), np.eye(3), 0.45, color=INK_MUTED, lw=1.0)
    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_zlim(-1.1, 1.1)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 0.08), fontsize=8)

    if ncol == 2:
        r, p, y = rot["rpy"]
        ax2 = _ax3d(fig, 122, "Rotación relativa entre las dos IMU")
        _triad(ax2, (0, 0, 0), np.eye(3), 1.0, names[0], dz=0.30)
        _triad(ax2, (0, 0, 0), rot["R"], 0.72, names[1] + " rotada",
               lw=1.0, dz=-0.55)
        ax2.set_xlim(-1.1, 1.1)
        ax2.set_ylim(-1.1, 1.1)
        ax2.set_zlim(-1.1, 1.1)
        ax2.set_xlabel("x")
        ax2.set_ylabel("y")
        ax2.set_zlabel("z")
        ax2.text2D(0.02, 0.95, f"roll {r:+.2f}   pitch {p:+.2f}   yaw {y:+.2f} deg\n"
                               f"residuo {100*rot['rel']:.1f} %",
                   transform=ax2.transAxes, fontsize=9, color=INK_SOFT,
                   family="monospace")
    fig.suptitle("IMU en 3D   (X azul, Y naranja, Z aqua)", x=0.01, ha="left")
    return fig


def export_csv(c: Base, prefix: str) -> None:
    cols, hdr = [c.t], ["t_unix"]
    if c.kind == "imu":
        cols += [c.accel, c.gyro, c.quat]
        hdr += ["ax", "ay", "az", "gx", "gy", "gz", "qx", "qy", "qz", "qw"]
    elif c.kind == "odom":
        cols += [c.pos, c.quat, c.vel, c.omega]
        hdr += ["px", "py", "pz", "qx", "qy", "qz", "qw",
                "vx", "vy", "vz", "wx", "wy", "wz"]
    elif c.kind == "fix":
        cols += [c.lat, c.lon, c.alt, c.status]
        hdr += ["lat", "lon", "alt", "status"]
    elif c.kind in ("odom", "car", "pose"):
        cols += [c.pos, c.quat]
        hdr += ["px", "py", "pz", "qx", "qy", "qz", "qw"]
        if getattr(c, "vel", None) is not None and c.vel.size:
            cols += [c.vel, c.omega]
            hdr += ["vx", "vy", "vz", "wx", "wy", "wz"]
    elif c.kind == "twist":
        cols += [c.vel, c.omega]
        hdr += ["vx", "vy", "vz", "wx", "wy", "wz"]
    elif c.kind == "scan" and c.stats.size:
        cols += [c.stats]
        hdr += ["puntos", "validos", "r_min", "r_medio", "r_mediana", "r_max"]
    elif c.kind == "image":
        cols += [c.metadata, c.image_stat]
        hdr += ["ancho", "alto", "step", "bytes", "brillo", "contraste"]
    elif c.kind == "range":
        cols += [c.range, c.min_range, c.max_range, c.field_of_view]
        hdr += ["range", "min", "max", "fov"]
    elif c.kind == "battery":
        cols += [c.data, c.cell_voltage, c.cell_temperature]
        hdr += list(BatteryC.FIELDS) + ["cell_v", "cell_t"]
    elif c.kind == "joint":
        for n in c.names:
            cols += [c.position[n], c.velocity[n], c.effort[n]]
            hdr += [f"pos_{n}", f"vel_{n}", f"eff_{n}"]
    elif c.kind == "scalar":
        cols += [c.value]
        hdr += list(c.labels)
    elif c.kind == "clock":
        cols += [c.clock]
        hdr += ["clock"]
    elif c.kind in ("posearray", "path"):
        cols += [c.counts]
        hdr += ["count"]
        if c.kind == "posearray":
            cols += [c.centroid, c.spread]
            hdr += ["cx", "cy", "spread"]
        else:
            cols += [c.lengths]
            hdr += ["length"]
    elif c.kind == "generic" and c.data:
        ks = sorted(c.data)
        cols += [c.data[k] for k in ks]
        hdr += ks
    else:
        return
    try:
        M = np.column_stack([np.asarray(x, float) for x in cols])
    except ValueError:
        return
    path = f"{prefix}_{slug(c.topic)}.csv"
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    np.savetxt(path, M, delimiter=",", comments="", fmt="%.9g",
               header=",".join(hdr))
    print(f"  CSV -> {path}  ({M.shape[0]} filas, {M.shape[1]} columnas)")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Analisis profundo de todos los topics de una rosbag ROS 2.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("bag")
    p.add_argument("--topic", nargs="+", default=None)
    p.add_argument("--save", metavar="DIR", default="")
    p.add_argument("--start", type=float, default=None)
    p.add_argument("--end", type=float, default=None)
    p.add_argument("--sample", type=int, default=200000)
    p.add_argument("--csv", metavar="PREFIJO", default="")
    p.add_argument("--max-warn", type=int, default=60)
    p.add_argument("--no-figs", action="store_true")
    p.add_argument("--no-3d", action="store_true",
                   help="omite las figuras tridimensionales")
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--dpi", type=int, default=130)
    args = p.parse_args()

    col, qos = scan(args.bag, args.topic, args.sample)
    t0 = min((c.t_bag[0] for c in col.values() if c.t_bag.size), default=0.0)
    if args.start is not None or args.end is not None:
        for c in col.values():
            window(c, t0, args.start, args.end)
        col = {n: c for n, c in col.items() if c.t_bag.size > 1}

    st = report_overview(col, qos)
    if args.quiet:
        return 0

    # /rosout primero: suele explicar el resto del informe.
    for n, c in col.items():
        if c.kind == "log":
            report_log(c, args.max_warn)

    imus: Dict[str, ImuC] = {}
    imu_st: Dict[str, Dict] = {}
    cons: Dict[str, Dict] = {}
    cars: Dict[str, CarC] = {}
    odoms: Dict[str, OdomC] = {}
    for n in sorted(col):
        c, b = col[n], st[n]
        if c.t_bag.size == 0:
            continue
        if c.kind == "imu" and c.accel.size:
            s = imu_stats(c)
            report_imu(c, b, s)
            imus[n], imu_st[n] = c, s
        elif c.kind == "odom" and c.pos.size:
            cons[n] = report_odom(c, b)
            odoms[n] = c
        elif c.kind == "car" and c.pos.size:
            report_car(c, b)
            cars[n] = c
        elif c.kind == "pose" and c.pos.size:
            report_pose(c, b)
        elif c.kind == "twist" and c.vel.size:
            report_twist(c, b)
        elif c.kind == "fix" and c.lat.size:
            report_fix(c, b)
        elif c.kind == "cloud":
            report_cloud(c, b)
        elif c.kind == "scan":
            report_scan(c, b)
        elif c.kind == "image":
            report_image(c, b)
        elif c.kind == "range":
            report_range(c, b)
        elif c.kind == "battery":
            report_battery(c, b)
        elif c.kind == "joint":
            report_joint(c, b)
        elif c.kind == "diag":
            report_diag(c, b)
        elif c.kind == "clock":
            report_clock(c, b)
        elif c.kind == "scalar":
            report_scalar(c, b)
        elif c.kind == "posearray":
            report_posearray(c, b)
        elif c.kind == "path":
            report_path(c, b)
        elif c.kind == "tf" and c.tf:
            report_tf(c)
        elif c.kind == "generic":
            report_generic(c, b)

    for cn, cc in cars.items():
        best = None
        for on, oc in odoms.items():
            lo, hi = max(cc.t[0], oc.t[0]), min(cc.t[-1], oc.t[-1])
            if hi - lo > 0.5 and (best is None or hi - lo > best[0]):
                best = (hi - lo, on, oc)
        if best:
            report_state_compare(cc, best[2])

    notes: List[str] = []
    rel_rot: Dict = {}
    if len(imus) > 1:
        notes = report_imu_compare(imus, imu_st, rel_rot)

    if args.csv:
        print()
        for c in col.values():
            export_csv(c, args.csv)

    if args.no_figs:
        print()
        return 0

    setup_style(bool(args.save))
    import matplotlib.pyplot as plt
    figs: Dict[str, object] = {}
    g = fig_global(col, st, t0)
    if g is not None:
        figs["_global"] = g
    for n in sorted(col):
        c, b, sg = col[n], st[n], slug(n)
        try:
            if c.kind == "imu" and c.accel.size:
                figs[f"imu_{sg}"] = fig_imu(c, b, imu_st[n], t0)
            elif c.kind == "odom" and c.pos.size:
                figs[f"odom_{sg}"] = fig_odom(c, b, t0, cons.get(n, {}))
            elif c.kind == "car" and c.pos.size:
                figs[f"car_{sg}"] = fig_car(c, b, t0)
            elif c.kind == "pose" and c.pos.size:
                figs[f"pose_{sg}"] = fig_pose(c, b, t0)
            elif c.kind == "twist" and c.vel.size:
                figs[f"twist_{sg}"] = fig_twist(c, b, t0)
            elif c.kind == "fix" and c.lat.size:
                f = fig_fix(c, b, t0)
                if f is not None:
                    figs[f"fix_{sg}"] = f
            elif c.kind == "cloud" and c.t_bag.size > 2:
                figs[f"cloud_{sg}"] = fig_cloud(c, b, t0)
            elif c.kind == "scan" and c.t_bag.size > 2:
                figs[f"scan_{sg}"] = fig_scan(c, b, t0)
            elif c.kind == "image" and c.t_bag.size:
                figs[f"image_{sg}"] = fig_image(c, b, t0)
            elif c.kind == "range" and c.range.size:
                figs[f"range_{sg}"] = fig_range(c, b, t0)
            elif c.kind == "battery" and c.data.size:
                figs[f"battery_{sg}"] = fig_battery(c, b, t0)
            elif c.kind == "joint" and c.names:
                figs[f"joint_{sg}"] = fig_joint(c, b, t0)
            elif c.kind == "diag" and c.rows:
                figs[f"diag_{sg}"] = fig_diag(c, b, t0)
            elif c.kind == "clock" and c.clock.size:
                figs[f"clock_{sg}"] = fig_clock(c, b, t0)
            elif c.kind == "scalar" and c.value.size:
                figs[f"scalar_{sg}"] = fig_scalar(c, b, t0)
            elif c.kind == "posearray" and c.counts.size:
                figs[f"posearray_{sg}"] = fig_posearray(c, b, t0)
            elif c.kind == "path" and c.counts.size:
                figs[f"path_{sg}"] = fig_path(c, b, t0)
            elif c.kind == "generic" and c.data:
                f = fig_generic(c, b, t0)
                if f is not None:
                    figs[f"data_{sg}"] = f
            if not args.no_3d:
                f3 = None
                if c.kind == "odom" and c.pos.size:
                    f3 = fig_traj3d(c, b, t0)
                elif c.kind == "fix" and c.lat.size:
                    f3 = fig_fix3d(c, b, t0)
                elif c.kind == "cloud":
                    f3 = fig_cloud3d(c, b)
                elif c.kind == "tf" and c.tf:
                    f3 = fig_tf3d(c)
                if f3 is not None:
                    figs[f"3d_{sg}"] = f3
            if c.kind in ("imu", "odom"):
                tf = fig_timing(c, t0)
                if tf is not None:
                    figs[f"timing_{sg}"] = tf
        except Exception as exc:  # noqa: BLE001
            print(f"  [aviso] no pude dibujar {n}: {exc}")
    for cn, cc in cars.items():
        best = None
        for on, oc in odoms.items():
            lo = max(cc.t[0], oc.t[0])
            hi = min(cc.t[-1], oc.t[-1])
            if hi - lo > 0.5 and (best is None or hi - lo > best[0]):
                best = (hi - lo, on, oc)
        if best:
            figs[f"compare_{slug(cn)}"] = fig_state_compare(cc, best[2], t0)
    if len(imus) > 1:
        figs["imu_compare"] = fig_imu_compare(imus, t0, notes)
    if imus and not args.no_3d:
        try:
            figs["3d_imu"] = fig_imu3d(imus, imu_st, rel_rot or None)
        except Exception as exc:  # noqa: BLE001
            print(f"  [aviso] no pude dibujar el 3D de IMU: {exc}")

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        print()
        for name, fig in figs.items():
            out = os.path.join(args.save, f"{name}.png")
            fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
            print(f"  {out}")
        plt.close("all")
    else:
        plt.show()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
