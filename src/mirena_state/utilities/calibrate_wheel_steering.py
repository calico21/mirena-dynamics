#!/usr/bin/env python3
"""Calibra el radio efectivo de rueda y la ganancia de dirección.

Lee una bolsa ROS 2 con:

  /sensors/imu          sensor_msgs/Imu
  /sensors/imu/odom     nav_msgs/Odometry   (INS, para la escala de rueda)
  /sensors/wheel_speeds mirena_common/WheelSpeeds
  /control/can          mirena_common/CarControl

y ajusta, por mínimos cuadrados robustos:

  u_encoder = k * u_INS                 -> wheel_radius = nominal / k
  omega_gyro = u * tan(a*delta + b) / L -> steering_ratio = 1/a

El ajuste de dirección se hace con |a_y| y |a_x| acotados para reducir el
efecto del deslizamiento. El resultado es una calibración de modelo, no una
verdad física: úsala como valor de partida y valida con una bolsa A/B.

Ejemplo:

  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  python3 src/mirena_state/utilities/calibrate_wheel_steering.py \
      rosbag2_2026_10_06-18_02_11
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mirena_common.msg import CarControl, WheelSpeeds

try:
    from scipy.optimize import least_squares
except ImportError as exc:  # pragma: no cover - depende del entorno
    raise SystemExit(
        "Este script necesita scipy (python3-scipy) para el ajuste robusto."
    ) from exc


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def load_topic(bag: str, topic: str, msg_type, storage_id: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    messages = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        messages.append(deserialize_message(data, msg_type))
    return messages


def wheel_radius(
    t_ins: np.ndarray,
    u_ins: np.ndarray,
    t_enc: np.ndarray,
    u_enc: np.ndarray,
    t_imu: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    nominal_radius: float,
    min_speed: float,
    max_abs_ax: float,
    max_abs_ay: float,
) -> Tuple[float, float, int]:
    """Ajusta u_enc = k*u_ins y devuelve (radio, k, número de muestras)."""
    u_enc_i = np.interp(t_ins, t_enc, u_enc)
    ax_i = np.interp(t_ins, t_imu, ax)
    ay_i = np.interp(t_ins, t_imu, ay)

    mask = (
        np.isfinite(u_ins)
        & np.isfinite(u_enc_i)
        & np.isfinite(ax_i)
        & np.isfinite(ay_i)
        & (np.abs(u_ins) >= min_speed)
        & (np.abs(ax_i) <= max_abs_ax)
        & (np.abs(ay_i) <= max_abs_ay)
    )
    n = int(mask.sum())
    if n < 20:
        raise SystemExit("Muy pocas muestras para calibrar la rueda.")

    # Sin intercepto: un error de radio es proporcional a la velocidad. El
    # término constante se deja fuera a propósito; si el encoder y el INS
    # tuvieran un desfase temporal grande, este ajuste lo repartiría mal.
    denominator = float(np.sum(u_ins[mask] * u_ins[mask]))
    k = float(np.sum(u_enc_i[mask] * u_ins[mask]) / denominator)
    if not math.isfinite(k) or k <= 0.0:
        raise SystemExit("Ajuste de rueda no válido.")

    return nominal_radius / k, k, n


def steering_calibration(
    t_imu: np.ndarray,
    omega: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    t_enc: np.ndarray,
    u_enc: np.ndarray,
    t_steer: np.ndarray,
    steer_deg: np.ndarray,
    wheelbase: float,
    min_speed: float,
    max_abs_ax: float,
    max_abs_ay: float,
    current_ratio: float,
    current_offset: float,
) -> Tuple[float, float, float, float, int]:
    """Ajusta omega = u*tan(a*delta+b)/L y devuelve (ratio, offset, rms, rms0, n)."""
    u = np.interp(t_imu, t_enc, u_enc)
    steer = np.interp(t_imu, t_steer, steer_deg)

    mask = (
        np.isfinite(u)
        & np.isfinite(steer)
        & np.isfinite(omega)
        & np.isfinite(ax)
        & np.isfinite(ay)
        & (np.abs(u) >= min_speed)
        & (np.abs(steer) <= 300.0)
        & (np.abs(ax) <= max_abs_ax)
        & (np.abs(ay) <= max_abs_ay)
    )
    n = int(mask.sum())
    if n < 20:
        raise SystemExit("Muy pocas muestras para calibrar la dirección.")

    u_m = u[mask]
    steer_m = steer[mask]
    omega_m = omega[mask]

    def residual(p: np.ndarray) -> np.ndarray:
        scale, offset = p
        delta = np.deg2rad(scale * steer_m) + offset
        return u_m * np.tan(delta) / wheelbase - omega_m

    fit = least_squares(
        residual,
        x0=np.array([1.0 / current_ratio, current_offset]),
        bounds=([0.4, -0.2], [1.5, 0.4]),
        loss="soft_l1",
        f_scale=0.15,
    )
    scale, offset = (float(fit.x[0]), float(fit.x[1]))
    if not math.isfinite(scale) or scale <= 0.0:
        raise SystemExit("Ajuste de dirección no válido.")

    rms = float(np.sqrt(np.mean(residual(fit.x) ** 2)))

    delta0 = np.deg2rad(steer_m / current_ratio) + current_offset
    rms0 = float(
        np.sqrt(np.mean((u_m * np.tan(delta0) / wheelbase - omega_m) ** 2))
    )

    return 1.0 / scale, offset, rms, rms0, n


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Ruta de la bolsa ROS 2")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--wheel-radius", type=float, default=0.20,
                        help="Radio nominal usado al calcular u_encoder (m)")
    parser.add_argument("--wheelbase", type=float, default=1.55, help="m")
    parser.add_argument("--min-speed", type=float, default=3.0, help="m/s")
    parser.add_argument("--max-abs-ax", type=float, default=3.0, help="m/s^2")
    parser.add_argument("--max-abs-ay", type=float, default=3.0, help="m/s^2")
    parser.add_argument("--current-ratio", type=float, default=1.0,
                        help="steering_ratio actual, solo para comparar")
    parser.add_argument("--current-offset", type=float, default=0.0361,
                        help="steering_offset_rad actual, solo para comparar")
    args = parser.parse_args()

    imu = load_topic(args.bag, "/sensors/imu", Imu, args.storage_id)
    ins = load_topic(args.bag, "/sensors/imu/odom", Odometry, args.storage_id)
    wheels = load_topic(args.bag, "/sensors/wheel_speeds", WheelSpeeds,
                        args.storage_id)
    steer = load_topic(args.bag, "/control/can", CarControl, args.storage_id)

    if not imu or not ins or not wheels or not steer:
        raise SystemExit("Falta alguno de los topics necesarios en la bolsa.")

    t_imu = np.array([stamp(m) for m in imu])
    omega = np.array([m.angular_velocity.z for m in imu])
    ax = np.array([m.linear_acceleration.x for m in imu])
    ay = np.array([m.linear_acceleration.y for m in imu])

    t_enc = np.array([stamp(m) for m in wheels])
    # Misma conversión que mirena_state.cpp: RPM -> m/s con el radio nominal.
    u_enc = np.array([
        0.5 * (m.rl + m.rr) * 2.0 * math.pi / 60.0 * args.wheel_radius
        for m in wheels
    ])

    t_ins = np.array([stamp(m) for m in ins])
    u_ins = np.array([m.twist.twist.linear.x for m in ins])

    t_steer = np.array([stamp(m) for m in steer])
    steer_deg = np.array([m.steer_angle for m in steer])

    radius, k, n_wheel = wheel_radius(
        t_ins, u_ins, t_enc, u_enc, t_imu, ax, ay,
        args.wheel_radius, args.min_speed, args.max_abs_ax, args.max_abs_ay,
    )
    ratio, offset, rms, rms0, n_steer = steering_calibration(
        t_imu, omega, ax, ay, t_enc, u_enc, t_steer, steer_deg,
        args.wheelbase, args.min_speed, args.max_abs_ax, args.max_abs_ay,
        args.current_ratio, args.current_offset,
    )

    result = {
        "wheel_radius": round(radius, 4),
        "wheel_scale": round(k, 4),
        "wheel_samples": n_wheel,
        "steering_ratio": round(ratio, 4),
        "steering_offset_rad": round(offset, 4),
        "steering_samples": n_steer,
        "steering_rms_rad_s": round(rms, 4),
        "steering_rms_before_rad_s": round(rms0, 4),
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
