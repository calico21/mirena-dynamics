#!/usr/bin/env python3
"""Emit the acados C sources for the Mirena contouring-control OCP.

Run at CMake configure time; the C it produces is compiled straight into the
mirena_control_mpc executable, so no Python or CasADi is loaded at runtime.

    generate_solver.py --output <dir> [--path-points N] [--horizon N]
                       [--horizon-time T] [--solve-budget S]
"""

import argparse
import math
import os
import sys

import numpy as np
import casadi as ca

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

if 'ACADOS_SOURCE_DIR' not in os.environ:
    os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, ACADOS_INFTY  # noqa: E402
import dbm_model as dbm  # noqa: E402

RAD30 = 30.0 * math.pi / 180.0

MODEL_NAME = 'mpc_problem'


def build_ocp(path_points: int, horizon: int, horizon_time: float,
              solve_budget: float) -> AcadosOcp:
    # Variables de problema
    p = ca.MX.sym('p')    # Progreso
    vp = ca.MX.sym('vp')  # Velocidad de Progreso

    # Estado aumentado de 10 elementos: [x, y, phi, u, v, omega, g, delta, p, vp]
    X_P = ca.vertcat(
        dbm.X_CAR,  # Car state
        dbm.U_CAR,  # Control state
        p,          # Progress state
        vp          # Progress speed
    )
    X_P_DOT = ca.MX.sym('X_P_DOT', X_P.shape[0])

    # Trabajamos sobre las derivadas del control para garantizar suavidad
    vp_dot = ca.MX.sym('vp_dot')
    g_dot = ca.MX.sym('gas')
    d_dot = ca.MX.sym('steer_angle')
    U_P = ca.vertcat(g_dot, d_dot, vp_dot)

    F_P_dot = ca.Function('MPCDerivative',
                          [X_P, U_P],
                          [ca.vertcat(
                              dbm.F_BLEND(dbm.X_CAR, dbm.U_CAR),
                              g_dot,  # g
                              d_dot,  # d
                              vp,     # p
                              vp_dot  # vp
                          )])

    # Parámetros del spline (dimensión fija)
    path_pts_x = ca.MX.sym('ctrl_pts_x', path_points)
    path_pts_y = ca.MX.sym('ctrl_pts_y', path_points)
    path_length = ca.MX.sym('path_length')

    P_GLOBAL = ca.vertcat(path_pts_x, path_pts_y, path_length)

    # Normalized and clamped path (avoids solver errors on bad lookups)
    p_norm = ca.fmin(ca.fmax(p / path_length, 0.0), 1.0)

    norm_grid = np.linspace(0, 1, path_points)

    # Splines independientes para cada coordenada (p indexa la curva)
    path_x = ca.interpolant('path_x', 'bspline', [norm_grid])
    path_y = ca.interpolant('path_y', 'bspline', [norm_grid])

    x_ref = path_x(p_norm, path_pts_x)
    y_ref = path_y(p_norm, path_pts_y)

    # Derivadas para la tangente
    dx_dp = ca.jacobian(x_ref, p)
    dy_dp = ca.jacobian(y_ref, p)

    EPSILON_REG = 1e-4
    norm_factor = ca.sqrt(dx_dp**2 + dy_dp**2 + EPSILON_REG**2)

    tx = dx_dp / norm_factor
    ty = dy_dp / norm_factor

    # Virtual car pursuit: el producto escalar da 0 si alineado, 2 si desalineado
    e_psi = 1.0 - (ca.cos(dbm.phi) * tx + ca.sin(dbm.phi) * ty)
    e_d = (dbm.x - x_ref)**2 + (dbm.y - y_ref)**2

    # Maximizamos el progreso (vp) y penalizamos errores y cambios de control.
    cost_expr = (
        4 * e_d +
        3 * e_psi +
        -0.01 * vp +        # Maximizar progreso (MINIMIZAR -vp)
        0.002 * g_dot**2 +  # Penalización a tasa de gas
        0.001 * d_dot**2 +    # Penalización a tasa de volante
        0.001 * vp_dot**2   # Penalización a tasa de vp
    )



    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = X_P
    model.u = U_P
    model.p_global = P_GLOBAL
    model.f_expl_expr = F_P_dot(X_P, U_P)
    model.xdot = X_P_DOT
    model.f_impl_expr = X_P_DOT - F_P_dot(X_P, U_P)
    model.cost_expr_ext_cost = cost_expr

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = horizon
    ocp.solver_options.tf = horizon_time
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.hpipm_mode = 'SPEED'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.levenberg_marquardt = 0.01
    # The solver runs inside a 20 Hz loop, so its budget is the loop's period,
    # not "however long convergence takes". The iteration caps are what that
    # budget buys at this problem size; timeout_max_time is the hard backstop
    # for the iteration that turns out to be unusually expensive -- acados stops
    # before starting an iteration it predicts will overrun, and returns the
    # iterate it has. The node accepts that iterate on its residual rather than
    # discarding it, so a capped solve still drives the car; what it must not do
    # is hold the solver thread past the period and starve the next solve.
    ocp.solver_options.qp_solver_iter_max = 25
    ocp.solver_options.nlp_solver_max_iter = 15
    ocp.solver_options.timeout_max_time = solve_budget
    ocp.solver_options.timeout_heuristic = 'LAST'
    ocp.solver_options.globalization_fixed_step_length = 0.8
    ocp.solver_options.regularize_method = 'PROJECT'
    ocp.solver_options.tol = 1e-2
    ocp.cost.cost_type = 'EXTERNAL'

    # Merit backtracking with a slightly more robust line search
    ocp.solver_options.nlp_solver_globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.globalization_alpha_min = 0.05
    ocp.solver_options.globalization_line_search_use_sufficient_descent = 1

    nx = X_P.shape[0]
    nu = U_P.shape[0]

    # ---------------- State constraints (todos los pasos de tiempo) ----------------
    # Constrain everything except position and progress.
    ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8, 9])

    ocp.constraints.lbx = np.array([
        0,       # u (must be positive for dynamics)
        -2,      # v
        -3,      # omega
        -1,      # g (gas/throttle command)
        -RAD30,  # delta (steering angle)
        0,       # p (progress)
        -10,     # vp (progress velocity)
    ])

    ocp.constraints.ubx = np.array([
        10,             # u (max speed)
        2,              # v
        3,              # omega
        1,              # g (gas/throttle command)
        RAD30,          # delta (steering angle)
        ACADOS_INFTY,   # p (progress)
        10,             # vp (progress velocity)
    ])

    # ---------------- Control constraints (rate of change) ----------------
    ocp.constraints.idxbu = np.arange(nu)
    # [g_dot, delta_dot, vp_dot]
    ocp.constraints.lbu = np.array([-5, -5 * RAD30, -5])
    ocp.constraints.ubu = np.array([5, 5 * RAD30, 5])

    # Terminal bounds
    ocp.constraints.idxbx_e = ocp.constraints.idxbx
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx

    # Initial state constraint (overwritten every solve by the node)
    ocp.constraints.x0 = np.zeros(nx)

    ocp.p_global_values = np.zeros(P_GLOBAL.shape[0])
    return ocp


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='code export directory')
    parser.add_argument('--path-points', type=int, default=20)
    parser.add_argument('--horizon', type=int, default=20)
    parser.add_argument('--horizon-time', type=float, default=2.0)
    parser.add_argument('--solve-budget', type=float, default=0.04,
                        help='wall-clock seconds a single solve may take; '
                             'must stay under the node solver_period')
    args = parser.parse_args()

    out_dir = os.path.abspath(args.output)
    os.makedirs(out_dir, exist_ok=True)

    ocp = build_ocp(args.path_points, args.horizon, args.horizon_time,
                    args.solve_budget)
    ocp.code_export_directory = out_dir

    # generate() renders the templates and exports the CasADi functions but does
    # not invoke the compiler -- CMake owns the build of the emitted sources.
    AcadosOcpSolver.generate(ocp, json_file=os.path.join(out_dir, 'acados_ocp.json'),
                             verbose=False)

    header = os.path.join(out_dir, f'acados_solver_{MODEL_NAME}.h')
    if not os.path.isfile(header):
        print(f'error: acados did not produce {header}', file=sys.stderr)
        return 1

    print(f'mirena_control: acados solver generated in {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
