#!/usr/bin/env python3
"""
plot_imu.py — Le das la ruta de una rosbag de ROS 2 y te grafica TODAS las IMU.

    source /opt/ros/jazzy/setup.bash
    python3 plot_imu.py /ruta/al/bag

Por defecto detecta todos los topics de tipo sensor_msgs/msg/Imu, los procesa
en una sola pasada del fichero y dibuja, para cada uno, seis paneles
(aceleracion, velocidad angular, modulo, RPY y los dos espectros) mas una
figura de temporizacion.

Con dos o mas IMU anade una figura de COMPARACION que es la parte util cuando
tienes un VN-300 y el IMU del LiDAR en el mismo coche:
  - modulo de la aceleracion de cada una sobre el mismo eje de tiempos
    absoluto, asi que un desfase de relojes se ve a simple vista
  - inclinacion de montaje de cada una, derivada de la gravedad en reposo
  - la ROTACION RELATIVA entre ellas, estimada por Kabsch sobre los
    giroscopos, con su residuo y el desfase temporal estimado

Opciones:
    --topic A [B ...]    procesa solo esos topics (por defecto, todos los Imu)
    --overview           tabla de tasas y huecos de TODOS los topics del bag
    --list               lista topics, tipos y numero de mensajes, y sale
    --save figs/         guarda PNG en vez de abrir ventanas
    --start 30 --end 90  recorta a una ventana en segundos
    --csv pref           exporta a pref_<topic>.csv
    --no-compare         no dibuja la figura de comparacion

Depende de numpy, scipy y matplotlib, ademas del entorno ROS 2 sourceado.
Es un archivo autocontenido: no necesita ningun otro modulo.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

IMU_TYPE = "sensor_msgs/msg/Imu"

# --------------------------------------------------------------------------- #
#  Paleta validada (CVD dE 9.2, vision normal dE 24.0, en todos los pares).
#  Dentro de una IMU: X azul, Y naranja, Z aqua.
#  Entre IMU distintas: mismo orden de slots, un color por topic.
# --------------------------------------------------------------------------- #

AXIS_COLOR = {"x": "#2a78d6", "y": "#eb6834", "z": "#1baf7a"}
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
BLUE, ORANGE, AQUA = SERIES

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SOFT = "#52514e"
INK_MUTED = "#8a8880"
GRID = "#e4e3df"

G0 = 9.80665
SPEC_ACCEL_DENSITY = 0.14e-3 * G0       # 0.14 mg/sqrt(Hz) del VN-300
SPEC_GYRO_DENSITY = 0.0035              # 0.0035 deg/s/sqrt(Hz) del VN-300

# Umbrales para considerar una muestra "en reposo".
STATIC_GYRO_DPS = 1.0
STATIC_ACC_TOL = 0.5


def slug(topic: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", topic).strip("_") or "topic"


# --------------------------------------------------------------------------- #
#  Lectura del bag
# --------------------------------------------------------------------------- #

def open_reader(path: str):
    """Abre el bag probando los storage_id habituales."""
    try:
        import rosbag2_py  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "\nNo encuentro rosbag2_py. Sourcea ROS 2 antes de ejecutar:\n"
            "    source /opt/ros/jazzy/setup.bash\n"
        ) from exc

    import rosbag2_py

    if not os.path.exists(path):
        raise SystemExit(f"\nNo existe la ruta: {path}")

    last = None
    for storage_id in ("", "mcap", "sqlite3"):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr"),
            )
            return reader
        except Exception as exc:  # noqa: BLE001
            last = exc
    raise SystemExit(f"\nNo pude abrir el bag '{path}':\n  {last}")


def topic_counts(reader) -> Dict[str, int]:
    """Numero de mensajes por topic, de los metadatos (sin leer el fichero)."""
    try:
        md = reader.get_metadata()
        return {t.topic_metadata.name: t.message_count
                for t in md.topics_with_message_count}
    except Exception:  # noqa: BLE001
        return {}


def list_topics(path: str) -> int:
    reader = open_reader(path)
    counts = topic_counts(reader)
    print(f"\nTopics en {path}:\n")
    for t in sorted(reader.get_all_topics_and_types(), key=lambda x: x.name):
        n = counts.get(t.name)
        mark = "   <-- IMU" if t.type == IMU_TYPE else ""
        cnt = f"{n:>8}" if n is not None else "       ?"
        print(f"  {cnt}  {t.name:<32} {t.type}{mark}")
    print()
    return 0


def find_imu_topics(reader, requested: Optional[Sequence[str]]) -> List[str]:
    """Los topics pedidos, o todos los de tipo Imu."""
    topics = reader.get_all_topics_and_types()
    available = {t.name: t.type for t in topics}

    if requested:
        missing = [t for t in requested if t not in available]
        if missing:
            raise SystemExit(
                f"\nNo estan en el bag: {', '.join(missing)}\n"
                f"Usa --list para ver los que hay.\n")
        return list(requested)

    found = sorted(t.name for t in topics if t.type == IMU_TYPE)
    if not found:
        raise SystemExit(
            f"\nEl bag no contiene ningun topic de tipo {IMU_TYPE}.\n"
            f"Usa --list para ver que hay, y --topic para forzar uno.\n")
    return found


def read_imu_topics(path: str, requested: Optional[Sequence[str]] = None
                    ) -> Dict[str, Dict]:
    """
    Extrae varios topics de IMU en UNA sola pasada del fichero.

    Leer el bag una vez por topic multiplica el tiempo por el numero de IMU,
    y en un bag con nubes de puntos eso son minutos.
    """
    reader = open_reader(path)

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    names = find_imu_topics(reader, requested)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {n: get_message(types[n]) for n in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=names))

    acc: Dict[str, Dict[str, list]] = {
        n: {k: [] for k in ("t_bag", "t_hdr", "a", "g", "q")} for n in names}

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        b = acc.get(topic)
        if b is None:
            continue
        m = deserialize_message(raw, classes[topic])
        b["t_bag"].append(stamp)
        b["t_hdr"].append(int(m.header.stamp.sec) * 1_000_000_000
                          + int(m.header.stamp.nanosec))
        la, av, q = m.linear_acceleration, m.angular_velocity, m.orientation
        b["a"].append((la.x, la.y, la.z))
        b["g"].append((av.x, av.y, av.z))
        b["q"].append((q.x, q.y, q.z, q.w))

    out: Dict[str, Dict] = {}
    for n in names:
        b = acc[n]
        if not b["t_bag"]:
            print(f"  [aviso] '{n}' esta vacio, se omite.")
            continue
        t_hdr = np.asarray(b["t_hdr"], dtype=np.int64)
        t_bag = np.asarray(b["t_bag"], dtype=np.int64)
        base = t_hdr if np.any(t_hdr > 0) else t_bag
        out[n] = {
            "topic": n,
            "t": base.astype(np.float64) * 1e-9,
            "t_bag": t_bag.astype(np.float64) * 1e-9,
            "t_hdr": t_hdr.astype(np.float64) * 1e-9,
            "accel": np.asarray(b["a"], dtype=np.float64),
            "gyro": np.asarray(b["g"], dtype=np.float64),     # rad/s
            "quat": np.asarray(b["q"], dtype=np.float64),
        }
    if not out:
        raise SystemExit("\nNinguno de los topics pedidos trae mensajes.\n")
    return out


def bag_overview(path: str) -> int:
    """
    Tasa, huecos y duracion de TODOS los topics, sin deserializar nada.

    Solo mira las marcas de tiempo del bag, asi que sirve para cualquier tipo
    de mensaje. Responde de un vistazo a por que un topic va a 88 Hz cuando
    deberia ir a 100.
    """
    import rosbag2_py  # noqa: F401

    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    stamps: Dict[str, list] = {n: [] for n in types}

    print(f"\nLeyendo marcas de tiempo de {path} (puede tardar)...")
    while reader.has_next():
        topic, _raw, stamp = reader.read_next()
        if topic in stamps:
            stamps[topic].append(stamp)

    rows = []
    for n, s in stamps.items():
        if len(s) < 2:
            rows.append((n, len(s), float("nan"), float("nan"), 0, types[n]))
            continue
        t = np.asarray(s, dtype=np.float64) * 1e-9
        dt = np.diff(t)
        med = float(np.median(dt[dt > 0])) if np.any(dt > 0) else float("nan")
        hz = 1.0 / med if med > 0 else float("nan")
        gaps = int(np.sum(dt > 3.0 * med)) if med > 0 else 0
        rows.append((n, len(s), t[-1] - t[0], hz, gaps, types[n]))

    rows.sort(key=lambda r: -r[1])
    w = max(len(r[0]) for r in rows) + 2
    print(f"\n{'topic':<{w}}{'msgs':>9}{'dur[s]':>9}{'Hz':>9}{'huecos':>9}   tipo")
    for n, c, dur, hz, gaps, ty in rows:
        print(f"{n:<{w}}{c:>9}{dur:>9.1f}{hz:>9.2f}{gaps:>9}   {ty}")
    print()
    return 0


def slice_window(d: Dict, start: Optional[float], end: Optional[float],
                 t0: Optional[float] = None) -> Optional[Dict]:
    if start is None and end is None:
        return d
    base = d["t"][0] if t0 is None else t0
    tr = d["t"] - base
    m = np.ones_like(tr, dtype=bool)
    if start is not None:
        m &= tr >= start
    if end is not None:
        m &= tr <= end
    if m.sum() < 3:
        return None
    out = dict(d)
    for k in ("t", "t_bag", "t_hdr", "accel", "gyro", "quat"):
        out[k] = d[k][m]
    return out


# --------------------------------------------------------------------------- #
#  Calculos
# --------------------------------------------------------------------------- #

def rpy_from_quat(q: np.ndarray) -> np.ndarray:
    """RPY intrinseco ZYX en radianes, desde cuaterniones (x, y, z, w)."""
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n == 0] = 1.0
    x, y, z, w = (q / n).T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def static_mask(d: Dict) -> np.ndarray:
    """Muestras en reposo: giroscopo quieto y modulo de aceleracion estable."""
    gd = np.degrees(np.linalg.norm(d["gyro"], axis=1))
    an = np.linalg.norm(d["accel"], axis=1)
    return (gd < STATIC_GYRO_DPS) & (np.abs(an - np.median(an)) < STATIC_ACC_TOL)


def tilt_from_gravity(a_mean: np.ndarray) -> Tuple[float, float, float]:
    """
    Inclinacion de montaje a partir del vector de gravedad medido en reposo.

    En FRD (X adelante, Y derecha, Z abajo) la fuerza especifica en reposo es
    (g sin0, -g cos0 sin@, -g cos0 cos@), de donde salen cabeceo y balanceo.
    En FLU (convencion ROS) los signos de Y y Z estan invertidos, y el mismo
    despeje sigue valiendo cambiandolos.
    """
    g = float(np.linalg.norm(a_mean))
    if g < 1e-6:
        return float("nan"), float("nan"), 0.0
    # Asumimos FLU si Z sale positiva (lo normal en ROS), FRD si sale negativa.
    v = a_mean if a_mean[2] < 0 else np.array([a_mean[0], -a_mean[1], -a_mean[2]])
    pitch = np.degrees(np.arcsin(np.clip(v[0] / g, -1.0, 1.0)))
    roll = np.degrees(np.arctan2(-v[1], -v[2]))
    return pitch, roll, g


def summary(d: Dict) -> Dict:
    t = d["t"]
    dt = np.diff(t)
    dt_pos = dt[dt > 0]
    med = float(np.median(dt_pos)) if dt_pos.size else float("nan")
    a, g = d["accel"], np.degrees(d["gyro"])
    sm = static_mask(d)
    a_st = a[sm] if sm.sum() > 20 else a
    pitch, roll, gmag = tilt_from_gravity(a_st.mean(axis=0))
    return {
        "n": len(t),
        "dur": float(t[-1] - t[0]),
        "rate": 1.0 / med if med > 0 else float("nan"),
        "jitter_ms": float(np.std(dt_pos) * 1e3) if dt_pos.size else float("nan"),
        "gaps": int(np.sum(dt > 3.0 * med)) if med > 0 else 0,
        "back": int(np.sum(dt < 0)),
        "sigma_a": a.std(axis=0),
        "sigma_g": g.std(axis=0),
        "bias_g": g[sm].mean(axis=0) if sm.sum() > 20 else g.mean(axis=0),
        "g_mean": float(np.mean(np.linalg.norm(a, axis=1))),
        "g_static": gmag,
        "a_static": a_st.mean(axis=0),
        "n_static": int(sm.sum()),
        "pitch": pitch,
        "roll": roll,
        "az_mean": float(a[:, 2].mean()),
        "has_quat": bool(np.any(np.abs(d["quat"]))),
    }


def print_summary(d: Dict, s: Dict) -> None:
    print(f"\n  ── {d['topic']}")
    print(f"     Muestras    {s['n']}  ({s['dur']:.1f} s)   "
          f"tasa {s['rate']:.2f} Hz   jitter {s['jitter_ms']:.3f} ms")
    if s["gaps"]:
        print(f"     Huecos      {s['gaps']} intervalos > 3 periodos")
    if s["back"]:
        print(f"     ATENCION    {s['back']} stamps van hacia atras")
    print("     sigma accel x {:.5f}  y {:.5f}  z {:.5f}  m/s2".format(*s["sigma_a"]))
    print("     sigma giro  x {:.5f}  y {:.5f}  z {:.5f}  deg/s".format(*s["sigma_g"]))
    print("     bias giro   x {:+.5f} y {:+.5f} z {:+.5f}  deg/s".format(*s["bias_g"]))
    print(f"     |a| media   {s['g_mean']:.4f} m/s2")
    if s["n_static"] > 20:
        print(f"     en reposo   {s['n_static']} muestras   |g| {s['g_static']:.4f}   "
              f"pitch {s['pitch']:+.3f}   roll {s['roll']:+.3f} deg")
    if abs(s["g_mean"] - G0) > 1.0:
        print(f"     ATENCION    |a| deberia rondar 9.807 en cualquier orientacion;")
        print(f"                 {s['g_mean']:.3f} sugiere gravedad restada o datos invalidos")
    if abs(abs(s["az_mean"]) - G0) < 0.5 and s["az_mean"] < 0:
        print("     ATENCION    accel z negativa en reposo nivelado;")
        print("                 en ROS (FLU, Z arriba) deberia ser +9.81")


# --------------------------------------------------------------------------- #
#  Comparacion entre IMU
# --------------------------------------------------------------------------- #

def estimate_time_offset(dA: Dict, dB: Dict, fs: float = 100.0,
                         max_lag_s: float = 2.0) -> float:
    """
    Desfase temporal entre dos IMU por correlacion cruzada del modulo del
    giroscopo. Devuelve los segundos que hay que SUMAR a los tiempos de B
    para alinearlo con A.

    Sin esto, dos sensores con relojes distintos dan una rotacion relativa
    sin sentido y nada avisa de ello.
    """
    from scipy.signal import correlate

    t0 = max(dA["t"][0], dB["t"][0])
    t1 = min(dA["t"][-1], dB["t"][-1])
    if t1 - t0 < 5.0:
        return float("nan")
    grid = np.arange(t0, t1, 1.0 / fs)
    gA = np.interp(grid, dA["t"], np.linalg.norm(dA["gyro"], axis=1))
    gB = np.interp(grid, dB["t"], np.linalg.norm(dB["gyro"], axis=1))
    gA -= gA.mean()
    gB -= gB.mean()
    if gA.std() < 1e-9 or gB.std() < 1e-9:
        return float("nan")           # sin movimiento no hay nada que alinear
    c = correlate(gA, gB, mode="full")
    lags = np.arange(-len(gB) + 1, len(gA))
    keep = np.abs(lags) <= int(max_lag_s * fs)
    return float(lags[keep][np.argmax(c[keep])] / fs)


def estimate_relative_rotation(dA: Dict, dB: Dict, fs: float = 100.0,
                               dt_offset: float = 0.0) -> Optional[Dict]:
    """
    Rotacion R tal que  gyro_A ~= R @ gyro_B,  por Kabsch/SVD.

    Necesita un tramo con movimiento en varios ejes: con el coche parado el
    problema esta indeterminado y el residuo lo delata. Da solo la rotacion,
    no la traslacion (para eso hacen falta las pruebas de pivote o skidpad).
    """
    t0 = max(dA["t"][0], dB["t"][0] + dt_offset)
    t1 = min(dA["t"][-1], dB["t"][-1] + dt_offset)
    if t1 - t0 < 5.0:
        return None
    grid = np.arange(t0, t1, 1.0 / fs)
    A = np.stack([np.interp(grid, dA["t"], dA["gyro"][:, i]) for i in range(3)], 1)
    B = np.stack([np.interp(grid, dB["t"] + dt_offset, dB["gyro"][:, i])
                  for i in range(3)], 1)

    # Solo muestras con rotacion apreciable: el reposo no aporta informacion
    # y solo mete ruido en la estimacion.
    m = (np.linalg.norm(A, axis=1) > np.radians(5.0)) & \
        (np.linalg.norm(B, axis=1) > np.radians(5.0))
    if m.sum() < 200:
        return {"insufficient": True, "n": int(m.sum())}
    A, B = A[m], B[m]

    H = B.T @ A
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(Vt.T @ U.T)))])
    R = Vt.T @ D @ U.T

    resid = A - (R @ B.T).T
    rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))
    scale = float(np.sqrt(np.mean(np.sum(A ** 2, axis=1))))
    rpy = np.degrees(rpy_from_matrix(R))
    return {"R": R, "rpy": rpy, "rms_dps": np.degrees(rms),
            "rel": rms / scale if scale > 0 else float("nan"),
            "n": int(len(A)), "insufficient": False}


def rpy_from_matrix(R: np.ndarray) -> np.ndarray:
    """RPY ZYX en radianes desde una matriz de rotacion."""
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(R[2, 0]) < 0.99999:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:                                   # gimbal lock
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def print_comparison(datas: Dict[str, Dict], sums: Dict[str, Dict]) -> List[str]:
    names = list(datas)
    lines: List[str] = []
    print("\n" + "=" * 72)
    print("  COMPARACION ENTRE IMU")
    print("=" * 72)
    for n in names:
        s = sums[n]
        txt = (f"{n}: |g| {s['g_static']:.4f} m/s2   "
               f"pitch {s['pitch']:+.2f}   roll {s['roll']:+.2f} deg")
        print("  " + txt)
        lines.append(txt)

    for i in range(len(names) - 1):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            print(f"\n  {a}  vs  {b}")
            off = estimate_time_offset(datas[a], datas[b])
            if np.isfinite(off):
                print(f"    desfase de reloj estimado: {off * 1e3:+.1f} ms "
                      f"(sumar a {b})")
                lines.append(f"desfase {a[:14]}/{b[:14]}: {off*1e3:+.1f} ms")
            else:
                off = 0.0
                print("    desfase no estimable (sin movimiento comun)")

            rot = estimate_relative_rotation(datas[a], datas[b], dt_offset=off)
            if rot is None or rot.get("insufficient"):
                print("    rotacion relativa: no estimable, hace falta un tramo")
                print("    con giro en varios ejes (ochos, slalom)")
                lines.append(f"rotacion {a[:14]}/{b[:14]}: sin datos dinamicos")
                continue
            r, p, y = rot["rpy"]
            print(f"    rotacion relativa (gyro_A = R @ gyro_B), {rot['n']} muestras:")
            print(f"      roll {r:+8.3f}   pitch {p:+8.3f}   yaw {y:+8.3f} deg")
            print(f"      residuo RMS {rot['rms_dps']:.4f} deg/s "
                  f"({100 * rot['rel']:.1f} % de la senal)")
            if rot["rel"] > 0.15:
                print("      residuo alto: relojes mal alineados, ejes distintos,")
                print("      o una de las dos IMU no esta midiendo bien")
            lines.append(f"R({a[:12]}<-{b[:12]}): "
                         f"[{r:+.1f} {p:+.1f} {y:+.1f}] deg, res {100*rot['rel']:.0f}%")
    print()
    return lines


# --------------------------------------------------------------------------- #
#  Estilo y ayudas de dibujo
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
        "legend.labelcolor": INK_SOFT,
        "figure.titlesize": 12, "figure.titleweight": "semibold",
        "lines.linewidth": 1.0, "font.size": 9,
        "axes.formatter.useoffset": False,
    })


def envelope(t: np.ndarray, y: np.ndarray, target: int = 3000):
    """
    Reduce series largas conservando minimo y maximo de cada tramo.

    Submuestrear uno de cada N esconde los picos, que en un log de vibracion
    son justo lo que interesa ver. Esto conserva la envolvente visual.
    """
    n = t.size
    if n <= target * 2:
        return t, y
    edges = np.linspace(0, n, target + 1).astype(int)
    ts, ys = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        if b <= a:
            continue
        lo = a + int(np.argmin(y[a:b]))
        hi = a + int(np.argmax(y[a:b]))
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
    ax.set_title(title, loc="left", pad=6)
    if legend:
        ax.legend(loc="center left", bbox_to_anchor=(1.005, 0.5),
                  handlelength=1.6, borderaxespad=0.0)


def stats_text(ax, text):
    """Alineado a la derecha, en la misma fila que el titulo, para no chocar."""
    ax.text(1.0, 1.02, text, transform=ax.transAxes, fontsize=8,
            color=INK_MUTED, va="bottom", ha="right")


def pad_ylim(ax, y, min_span):
    """Evita que datos casi constantes se pinten sobre ruido de coma flotante."""
    lo, hi = float(np.min(y)), float(np.max(y))
    if hi - lo < min_span:
        c = 0.5 * (lo + hi)
        ax.set_ylim(c - min_span / 2, c + min_span / 2)


# --------------------------------------------------------------------------- #
#  Figuras
# --------------------------------------------------------------------------- #

def make_figure(d: Dict, s: Dict, t0: float):
    import matplotlib.pyplot as plt
    from scipy.signal import welch

    t = d["t"] - t0
    a = d["accel"]
    g = np.degrees(d["gyro"])
    fs = s["rate"]

    fig = plt.figure(figsize=(13.5, 13.0), layout="constrained")
    gs = fig.add_gridspec(4, 2, height_ratios=[3, 3, 2.2, 2.6])
    ax_a = fig.add_subplot(gs[0, :])
    ax_g = fig.add_subplot(gs[1, :], sharex=ax_a)
    ax_n = fig.add_subplot(gs[2, 0], sharex=ax_a)
    ax_r = fig.add_subplot(gs[2, 1], sharex=ax_a)
    ax_pa = fig.add_subplot(gs[3, 0])
    ax_pg = fig.add_subplot(gs[3, 1])

    for i, k in enumerate("xyz"):
        line(ax_a, t, a[:, i], AXIS_COLOR[k], k)
    finish(ax_a, "m/s²", "Aceleración lineal")
    stats_text(ax_a, "sigma  x {:.4f}   y {:.4f}   z {:.4f}  m/s²".format(*s["sigma_a"]))

    for i, k in enumerate("xyz"):
        line(ax_g, t, g[:, i], AXIS_COLOR[k], k)
    finish(ax_g, "°/s", "Velocidad angular")
    stats_text(ax_g, "sigma  x {:.4f}   y {:.4f}   z {:.4f}  °/s".format(*s["sigma_g"]))

    norm = np.linalg.norm(a, axis=1)
    line(ax_n, t, norm, BLUE, "|a|")
    ax_n.axhline(G0, color=INK_MUTED, lw=1.0, ls="--", label="9.807")
    finish(ax_n, "m/s²", "Módulo de la aceleración")
    stats_text(ax_n, f"media {norm.mean():.4f}")
    ax_n.set_xlabel("tiempo [s]")

    if s["has_quat"]:
        rpy = np.degrees(rpy_from_quat(d["quat"]))
        line(ax_r, t, rpy[:, 0], BLUE, "roll")
        line(ax_r, t, rpy[:, 1], ORANGE, "pitch")
        line(ax_r, t, np.degrees(np.unwrap(np.radians(rpy[:, 2]))), AQUA, "yaw")
        finish(ax_r, "grados", "Orientación (yaw desenrollado)")
    else:
        ax_r.set_title("Orientación", loc="left", pad=6)
        ax_r.text(0.5, 0.5, "el mensaje no trae cuaternión",
                  transform=ax_r.transAxes, ha="center", va="center",
                  color=INK_MUTED, fontsize=9)
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
    for axp, lab, title in ((ax_pa, "m/s² / √Hz", "Espectro de aceleración"),
                            (ax_pg, "°/s / √Hz", "Espectro de velocidad angular")):
        axp.set_xlabel("frecuencia [Hz]")
        axp.grid(True, which="both", alpha=0.9)
        finish(axp, lab, title)
    stats_text(ax_pg, f"Nyquist {fs / 2:.0f} Hz")

    warn = f"   ·   {s['gaps']} huecos" if s["gaps"] else ""
    fig.suptitle(f"{d['topic']}   ({s['n']} muestras, {s['rate']:.1f} Hz, "
                 f"{s['dur']:.0f} s){warn}", x=0.01, ha="left")
    return fig


def make_timing_figure(d: Dict, s: Dict, t0: float):
    """Intervalos entre stamps y latencia hasta la recepcion."""
    import matplotlib.pyplot as plt

    if not np.any(d["t_hdr"] > 0):
        return None
    dt_ms = np.diff(d["t_hdr"]) * 1e3
    lat_ms = (d["t_bag"] - d["t_hdr"]) * 1e3
    t = d["t"] - t0

    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.2), layout="constrained")
    span = float(np.max(dt_ms) - np.min(dt_ms))
    ax[0].hist(dt_ms, bins=200 if span > 1e-6 else 11, color=BLUE, edgecolor="none")
    ax[0].set_yscale("log")
    ax[0].set_xlabel("intervalo entre stamps [ms]")
    ax[0].set_ylabel("cuentas")
    ax[0].set_title("Distribución de intervalos", loc="left", pad=6)
    stats_text(ax[0], f"mediana {np.median(dt_ms):.3f}   sigma {np.std(dt_ms):.3f} ms")

    line(ax[1], t, lat_ms, ORANGE, "latencia")
    pad_ylim(ax[1], lat_ms, 1.0)
    ax[1].set_xlabel("tiempo [s]")
    ax[1].set_ylabel("ms")
    ax[1].set_title("Latencia stamp → recepción", loc="left", pad=6)
    stats_text(ax[1], "un escalón delata el cambio de reloj")

    fig.suptitle(f"Temporización — {d['topic']}", x=0.01, ha="left")
    return fig


def make_compare_figure(datas: Dict[str, Dict], sums: Dict[str, Dict],
                        t0: float, notes: Sequence[str]):
    """Las IMU sobre el mismo eje de tiempos absoluto."""
    import matplotlib.pyplot as plt

    names = list(datas)
    fig = plt.figure(figsize=(13.5, 9.5), layout="constrained")
    gs = fig.add_gridspec(3, 1, height_ratios=[3, 3, 2])
    ax_n = fig.add_subplot(gs[0])
    ax_g = fig.add_subplot(gs[1], sharex=ax_n)
    ax_t = fig.add_subplot(gs[2])

    for i, n in enumerate(names):
        c = SERIES[i % len(SERIES)]
        d = datas[n]
        t = d["t"] - t0
        line(ax_n, t, np.linalg.norm(d["accel"], axis=1), c, n)
        line(ax_g, t, np.degrees(np.linalg.norm(d["gyro"], axis=1)), c, n)
    ax_n.axhline(G0, color=INK_MUTED, lw=1.0, ls="--", label="9.807")
    finish(ax_n, "m/s²", "Módulo de la aceleración — todas las IMU")
    stats_text(ax_n, "todas deben rondar 9.807 en reposo")
    finish(ax_g, "°/s", "Módulo de la velocidad angular — todas las IMU")
    stats_text(ax_g, "si una va adelantada, los relojes no están alineados")
    ax_g.set_xlabel("tiempo [s] desde el inicio del bag")

    ax_t.axis("off")
    ax_t.set_title("Resultados", loc="left", pad=6)
    ax_t.text(0.0, 0.95, "\n".join(notes), transform=ax_t.transAxes,
              va="top", ha="left", fontsize=9, color=INK_SOFT, family="monospace")

    fig.suptitle(f"Comparación entre {len(names)} IMU", x=0.01, ha="left")
    return fig


# --------------------------------------------------------------------------- #

def export_csv(d: Dict, path: str) -> None:
    cols = np.column_stack([d["t"], d["t"] - d["t"][0], d["accel"], d["gyro"],
                            d["quat"]])
    np.savetxt(path, cols, delimiter=",", comments="", fmt="%.9g",
               header="t_unix,t_rel,ax,ay,az,gx,gy,gz,qx,qy,qz,qw")
    print(f"  CSV -> {path}  ({len(cols)} filas)")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Extrae y grafica todas las IMU de una rosbag de ROS 2.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("bag", help="ruta de la carpeta del bag (o del .mcap / .db3)")
    p.add_argument("--topic", nargs="+", default=None,
                   help="topics a procesar (por defecto, todos los de tipo Imu)")
    p.add_argument("--save", metavar="DIR", default="", help="guarda PNG en DIR")
    p.add_argument("--start", type=float, default=None, help="segundos relativos")
    p.add_argument("--end", type=float, default=None, help="segundos relativos")
    p.add_argument("--csv", metavar="PREFIJO", default="",
                   help="exporta a PREFIJO_<topic>.csv")
    p.add_argument("--list", action="store_true", help="lista los topics y sale")
    p.add_argument("--overview", action="store_true",
                   help="tabla de tasas y huecos de todos los topics")
    p.add_argument("--no-compare", action="store_true",
                   help="no dibuja la figura de comparacion")
    p.add_argument("--dpi", type=int, default=140)
    args = p.parse_args()

    if args.list:
        return list_topics(args.bag)
    if args.overview:
        return bag_overview(args.bag)

    print(f"\nLeyendo {args.bag}")
    datas = read_imu_topics(args.bag, args.topic)
    print(f"  {len(datas)} topic(s) de IMU: {', '.join(datas)}")

    # Eje de tiempos comun: asi un desfase entre sensores se ve en la grafica.
    t0 = min(d["t"][0] for d in datas.values())

    if args.start is not None or args.end is not None:
        cut = {}
        for n, d in datas.items():
            w = slice_window(d, args.start, args.end, t0)
            if w is None:
                print(f"  [aviso] la ventana deja '{n}' sin muestras, se omite.")
            else:
                cut[n] = w
        if not cut:
            raise SystemExit("\nLa ventana --start/--end deja todo vacio.\n")
        datas = cut

    sums = {n: summary(d) for n, d in datas.items()}
    for n in datas:
        print_summary(datas[n], sums[n])

    if args.csv:
        for n, d in datas.items():
            export_csv(d, f"{args.csv}_{slug(n)}.csv")

    notes: List[str] = []
    if len(datas) > 1 and not args.no_compare:
        notes = print_comparison(datas, sums)

    setup_style(bool(args.save))
    import matplotlib.pyplot as plt

    figs: Dict[str, object] = {}
    for n, d in datas.items():
        sg = slug(n)
        figs[f"imu_{sg}"] = make_figure(d, sums[n], t0)
        tf = make_timing_figure(d, sums[n], t0)
        if tf is not None:
            figs[f"timing_{sg}"] = tf
    if len(datas) > 1 and not args.no_compare:
        figs["compare"] = make_compare_figure(datas, sums, t0, notes)

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        for name, fig in figs.items():
            out = os.path.join(args.save, f"{name}.png")
            fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
            print(f"  {out}")
        plt.close("all")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())