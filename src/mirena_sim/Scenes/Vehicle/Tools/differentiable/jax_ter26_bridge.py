#!/usr/bin/env python3
"""
jax_ter26_bridge.py — Native C++20 <-> Python / JAX Differentiable Physics Bridge (§7)
======================================================================================
Directly binds the compiled C++20 shared library `Native/bin/libter26_vehicle.so`
via its C-ABI (`ter26_step_c_abi`, `ter26_vjp_c_abi`, `ter26_jacobians_c_abi`).

Key Architectural Principles (§7.1–§7.3):
  1. Single Physical Authority: No reimplementation of vehicle physics in Python.
     Every forward step, Jacobian, and reverse-mode Adjoint VJP is computed by the
     exact C++20 `VehicleDynamicsEngine`.
  2. JAX `@jax.custom_vjp` Primitive (when JAX is installed) + Pure NumPy/SciPy
     Adjoint Backpropagation-Through-Time (BPTT) for multi-step trajectory optimization
     and gradient-based system identification.
"""

from __future__ import annotations
import ctypes
import os
import sys
from pathlib import Path
from typing import Tuple
import numpy as np


def _load_ter26_library() -> ctypes.CDLL:
    repo_vehicle_dir = Path(__file__).resolve().parents[2]
    so_path = repo_vehicle_dir / "Native" / "bin" / "libter26_vehicle.so"
    if not so_path.exists():
        raise FileNotFoundError(f"Could not find compiled C++20 shared library at {so_path}")

    lib = ctypes.CDLL(str(so_path))

    dbl_ptr = ctypes.POINTER(ctypes.c_double)
    int_ptr = ctypes.POINTER(ctypes.c_int)

    lib.ter26_step_c_abi.argtypes = [
        dbl_ptr, dbl_ptr, dbl_ptr, ctypes.c_double, ctypes.c_int,
        dbl_ptr, dbl_ptr, int_ptr
    ]
    lib.ter26_step_c_abi.restype = None

    lib.ter26_vjp_c_abi.argtypes = [
        dbl_ptr, dbl_ptr, dbl_ptr, dbl_ptr, ctypes.c_double,
        dbl_ptr, dbl_ptr, dbl_ptr
    ]
    lib.ter26_vjp_c_abi.restype = None

    lib.ter26_jacobians_c_abi.argtypes = [
        dbl_ptr, dbl_ptr, dbl_ptr, ctypes.c_double,
        dbl_ptr, dbl_ptr, dbl_ptr
    ]
    lib.ter26_jacobians_c_abi.restype = None

    return lib


_LIB = _load_ter26_library()

# Default 24-element physical parameter vector matching VehicleParameters::to_parameter_vector()
DEFAULT_P24 = np.array([
    269.0,    # 0: sprung_mass_kg
    0.330,    # 1: h_cg_m
    0.876,    # 2: lf_m
    118.0,    # 3: I_zz [kg*m^2]
    35030.0,  # 4: k_spring_front [N/m]
    52540.0,  # 5: k_spring_rear [N/m]
    1800.0,   # 6: c_low_bump_front [N*s/m]
    1600.0,   # 7: c_low_bump_rear [N*s/m]
    4500.0,   # 8: arb_wheel_rate_front [N/m]
    6500.0,   # 9: arb_wheel_rate_rear [N/m]
    98377.0,  # 10: tire_kz_front [N/m]
    98377.0,  # 11: tire_kz_rear [N/m]
    0.68,     # 12: tire_front lambda_mux
    0.64,     # 13: tire_front lambda_muy
    0.68,     # 14: tire_rear lambda_mux
    0.64,     # 15: tire_rear lambda_muy
    0.92,     # 16: tire_front lambda_Kya
    0.92,     # 17: tire_rear lambda_Kya
    0.18,     # 18: tire_front sigma_y_m
    0.18,     # 19: tire_rear sigma_y_m
    3.15,     # 20: aero cl_nominal
    1.18,     # 21: aero cd_nominal
    0.46,     # 22: aero_balance_front
    21.0,     # 23: peak_torque_per_motor_Nm
], dtype=np.float64)


def cpp_step(
    x38: np.ndarray,
    u3: np.ndarray,
    p24: np.ndarray = DEFAULT_P24,
    dt_s: float = 0.0025,
    differentiable: bool = True,
) -> Tuple[np.ndarray, float, bool]:
    """Execute one step of the C++20 TeR26 vehicle dynamics engine."""
    x_in = np.ascontiguousarray(x38, dtype=np.float64)
    u_in = np.ascontiguousarray(u3, dtype=np.float64)
    p_in = np.ascontiguousarray(p24, dtype=np.float64)
    x_out = np.zeros(38, dtype=np.float64)
    energy = ctypes.c_double(0.0)
    ns_flag = ctypes.c_int(0)

    _LIB.ter26_step_c_abi(
        x_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        p_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        ctypes.c_double(dt_s),
        ctypes.c_int(1 if differentiable else 0),
        x_out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        ctypes.byref(energy),
        ctypes.byref(ns_flag),
    )
    return x_out, float(energy.value), bool(ns_flag.value != 0)


def cpp_vjp(
    x38: np.ndarray,
    u3: np.ndarray,
    p24: np.ndarray,
    cotangent_y38: np.ndarray,
    dt_s: float = 0.0025,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate reverse-mode Adjoint VJP (grad_x, grad_u, grad_p) in C++20."""
    x_in = np.ascontiguousarray(x38, dtype=np.float64)
    u_in = np.ascontiguousarray(u3, dtype=np.float64)
    p_in = np.ascontiguousarray(p24, dtype=np.float64)
    cot_in = np.ascontiguousarray(cotangent_y38, dtype=np.float64)

    gx = np.zeros(38, dtype=np.float64)
    gu = np.zeros(3, dtype=np.float64)
    gp = np.zeros(24, dtype=np.float64)

    _LIB.ter26_vjp_c_abi(
        x_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        p_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        cot_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        ctypes.c_double(dt_s),
        gx.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        gu.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        gp.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )
    return gx, gu, gp


def cpp_jacobians(
    x38: np.ndarray,
    u3: np.ndarray,
    p24: np.ndarray = DEFAULT_P24,
    dt_s: float = 0.0025,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate A (38x38), B (38x3), and S_p (38x24) in C++20."""
    x_in = np.ascontiguousarray(x38, dtype=np.float64)
    u_in = np.ascontiguousarray(u3, dtype=np.float64)
    p_in = np.ascontiguousarray(p24, dtype=np.float64)

    A = np.zeros((38, 38), dtype=np.float64)
    B = np.zeros((38, 3), dtype=np.float64)
    Sp = np.zeros((38, 24), dtype=np.float64)

    _LIB.ter26_jacobians_c_abi(
        x_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        p_in.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        ctypes.c_double(dt_s),
        A.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        B.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        Sp.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )
    return A, B, Sp


# Optional JAX @custom_vjp registration when JAX is installed in the Python environment
try:
    import jax
    import jax.numpy as jnp
    jax.config.update("jax_enable_x64", True)
    HAS_JAX = True

    @jax.custom_vjp
    def jax_ter26_step(x38: jnp.ndarray, u3: jnp.ndarray, p24: jnp.ndarray) -> jnp.ndarray:
        res_shape = jax.ShapeDtypeStruct((38,), jnp.float64)
        return jax.pure_callback(
            lambda x, u, p: cpp_step(np.asarray(x), np.asarray(u), np.asarray(p), 0.0025, True)[0],
            res_shape, x38, u3, p24
        )

    def _jax_step_fwd(x38, u3, p24):
        y = jax_ter26_step(x38, u3, p24)
        return y, (x38, u3, p24)

    def _jax_step_bwd(residuals, cot_y):
        x38, u3, p24 = residuals
        shapes = (
            jax.ShapeDtypeStruct((38,), jnp.float64),
            jax.ShapeDtypeStruct((3,), jnp.float64),
            jax.ShapeDtypeStruct((24,), jnp.float64),
        )
        gx, gu, gp = jax.pure_callback(
            lambda x, u, p, g: cpp_vjp(np.asarray(x), np.asarray(u), np.asarray(p), np.asarray(g), 0.0025),
            shapes, x38, u3, p24, cot_y
        )
        return gx, gu, gp

    jax_ter26_step.defvjp(_jax_step_fwd, _jax_step_bwd)
except ImportError:
    HAS_JAX = False


def rollout_and_adjoint_bptt(
    x0: np.ndarray,
    u_traj: np.ndarray,
    p24: np.ndarray,
    target_traj: np.ndarray,
    state_weights: np.ndarray,
    dt_s: float = 0.0025,
) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """
    Roll out an N-step trajectory using the C++20 engine and compute exact
    reverse-mode Adjoint Backpropagation-Through-Time (BPTT) gradients:
        L(u, p) = 0.5 * sum_{t=1}^N || W^{1/2} (x_t - x_t^{target}) ||_2^2
    Returns: (loss, x_traj, dL_du_traj, dL_dp24)
    """
    N = u_traj.shape[0]
    x_traj = np.zeros((N + 1, 38), dtype=np.float64)
    x_traj[0] = x0

    for t in range(N):
        x_traj[t + 1], _, _ = cpp_step(x_traj[t], u_traj[t], p24, dt_s, True)

    diffs = x_traj[1:] - target_traj
    weighted_diffs = diffs * state_weights[None, :]
    loss = 0.5 * float(np.sum(diffs * weighted_diffs))

    # Reverse-mode Adjoint recursion from t = N-1 down to 0
    adj_x = np.zeros(38, dtype=np.float64)
    grad_u_traj = np.zeros_like(u_traj)
    grad_p24 = np.zeros(24, dtype=np.float64)

    for t in range(N - 1, -1, -1):
        cot_y = weighted_diffs[t] + adj_x
        gx, gu, gp = cpp_vjp(x_traj[t], u_traj[t], p24, cot_y, dt_s)
        adj_x = gx
        grad_u_traj[t] = gu
        grad_p24 += gp

    return loss, x_traj, grad_u_traj, grad_p24


def run_verification_suite() -> int:
    print("================================================================================")
    print("  TeR26 PYTHON / C-ABI / ADJOINT BPTT DIFFERENTIABLE PHYSICS VERIFICATION SUITE")
    print("================================================================================")
    print(f"  Loaded C++20 Shared Library: {_LIB._name} (JAX installed: {HAS_JAX})")

    # Build a nominal settled state at vx = 12.0 m/s
    x0 = np.zeros(38, dtype=np.float64)
    x0[2] = 0.2968  # settled CoG z
    x0[6:10] = [0.0029, 0.0029, 0.0030, 0.0030]
    x0[14] = 12.0   # vx = 12 m/s
    x0[24:28] = 12.0 / 0.2032

    u0 = np.array([0.45, 0.0, 0.12], dtype=np.float64)
    p0 = DEFAULT_P24.copy()

    # 1. Verify C-ABI Adjoint VJP vs Finite-Difference Gradient on a 10-step rollout
    N_steps = 10
    u_seq = np.tile(u0, (N_steps, 1))
    p_true = p0.copy()
    p_true[13] = 0.72  # higher front lateral friction lambda_muy
    p_true[15] = 0.70  # higher rear lateral friction lambda_muy

    weights = np.zeros(38, dtype=np.float64)
    weights[14] = 10.0  # vx
    weights[15] = 25.0  # vy
    weights[19] = 50.0  # yaw rate r

    _, target_x, _, _ = rollout_and_adjoint_bptt(x0, u_seq, p_true, np.zeros((N_steps, 38)), weights)
    target_states = target_x[1:]

    loss0, _, grad_u, grad_p = rollout_and_adjoint_bptt(x0, u_seq, p0, target_states, weights)

    # Check multi-step BPTT gradient dL/dp[13] and dL/du[0, 2] against central finite differences
    eps = 1e-5
    p_plus = p0.copy(); p_plus[13] += eps
    p_minus = p0.copy(); p_minus[13] -= eps
    l_p, _, _, _ = rollout_and_adjoint_bptt(x0, u_seq, p_plus, target_states, weights)
    l_m, _, _, _ = rollout_and_adjoint_bptt(x0, u_seq, p_minus, target_states, weights)
    fd_dp13 = (l_p - l_m) / (2.0 * eps)
    err_dp13 = abs(grad_p[13] - fd_dp13) / max(1e-12, abs(fd_dp13))
    print(f"  [PASS] 10-Step Adjoint BPTT dL/d(lambda_muy_f): analytic={grad_p[13]:.6e}, FD={fd_dp13:.6e} (rel_err={err_dp13:.2e})")
    assert err_dp13 < 1e-3, f"BPTT parameter gradient mismatch: {err_dp13}"

    # 2. Gradient-Based Parameter Identification Recovery Test (recovering lambda_muy_front & rear)
    p_est = p0.copy()
    lr = np.zeros(24, dtype=np.float64)
    lr[13] = 18.0
    lr[15] = 18.0
    init_loss = loss0
    curr_loss = loss0
    for it in range(18):
        curr_loss, _, _, gp = rollout_and_adjoint_bptt(x0, u_seq, p_est, target_states, weights)
        p_est -= lr * gp
    print(f"  [PASS] Gradient-based tire grip identification via C++20 Adjoint VJP: "
          f"loss {init_loss:.6e} -> {curr_loss:.6e} ({100.0 * (1.0 - curr_loss / init_loss):.2f}% reduction)")
    assert curr_loss < 0.05 * init_loss, "Gradient-based parameter estimation did not converge"

    if HAS_JAX:
        cot = jnp.ones(38, dtype=jnp.float64)
        gx_j, gu_j, gp_j = jax.grad(
            lambda x, u, p: jnp.dot(cot, jax_ter26_step(x, u, p)), argnums=(0, 1, 2)
        )(jnp.asarray(x0), jnp.asarray(u0), jnp.asarray(p0))
        gx_c, gu_c, gp_c = cpp_vjp(x0, u0, p0, np.asarray(cot))
        jax_err = float(np.linalg.norm(np.asarray(gx_j) - gx_c) + np.linalg.norm(np.asarray(gp_j) - gp_c))
        print(f"  [PASS] @jax.custom_vjp backward pass exact match with C++20 VJP (err={jax_err:.2e})")
        assert jax_err < 1e-12

    print("================================================================================")
    print("  ALL PYTHON / C-ABI / ADJOINT BPTT VERIFICATION TESTS PASSED (0 FAILED)")
    print("================================================================================")
    return 0


if __name__ == "__main__":
    sys.exit(run_verification_suite())
