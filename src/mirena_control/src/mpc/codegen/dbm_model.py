"""Dynamic bicycle model used by the Mirena MPC.

This file is only executed at build time: acados turns the CasADi expressions
below into plain C that gets compiled into the control node.  Nothing here runs
while the car is driving.
"""

import casadi as ca

# Parametros de vehiculo
M = 230  # Masa
I_Z = 120  # Momento de Inercia
L_F = 0.806  # COG a Eje frontal
L_R = 0.744  # COG a Eje trasero


# Parametros powertrain
C_M = 500  # Coeficiente de comanda
C_R0 = 0  # Coeficiente de fricción
C_R2 = 0  # Coeficiente cuadratico de drag

# Parametros de Rueda (Pacejka@Fz=-1150)
B = 9
C = 1.4
D = 2.33

# Parametros elipsoide de rueda
P_LONG = 2
P_ELLIPSE = 4

# Parametro de relajación (Decaimiento de variables en el estado kinematico)
K_RELAX = 20

# Blending
U_BLEND_MIN = 3
U_BLEND_MAX = 4


# Vertical force
F_Z = -(M * 9.81) / 2


# Vector  extendido de 6 elementos: [x, y, phi, u, v, omega]
x = ca.MX.sym('x')
y = ca.MX.sym('y')
phi = ca.MX.sym('phi')
u = ca.MX.sym('u')
v = ca.MX.sym('v')
omega = ca.MX.sym('omega')

X_CAR = ca.vertcat(
    x,
    y,
    phi,
    u,
    v,
    omega
)  # Estado kinematico

# Vector de control
g = ca.MX.sym('g')  # Control de gas
d = ca.MX.sym('delta')  # Volante

U_CAR = ca.vertcat(
    g,  # Gas
    d  # Steer angle
)


# Modelo Fuerza en x del powertrain
# Incluye drag inicial y drag cuadratico por aero
f_x = C_M * g - C_R0 - C_R2 * u**2

epsilon = 0.001

# Definición de slip angles
# La raiz regularizada evita la división por cero a baja velocidad
alpha_r = ca.arctan((v - L_R * omega) / ca.sqrt(u**2 + epsilon**2))  # Rear slip angle
alpha_f = ca.arctan((v + L_F * omega) / ca.sqrt(u**2 + epsilon**2) - d)  # Front slip angle

# Definición de fuerzas en y de rueda (Lateral forces using Pacejka simplified model)
f_r_y = F_Z * D * ca.sin(C * ca.atan(B * alpha_r))
f_f_y = F_Z * D * ca.sin(C * ca.atan(B * alpha_f))

# Definición de las resultantes de fuerza al cuadrado
F_R_S = f_r_y**2 + (P_LONG * (C_M * g * 0.5))**2  # Maximal lateral force
F_F_S = f_f_y**2 + (P_LONG * 0)**2  # Maximal lateral force (Front axis has no traction)
F_S_MAX = (F_Z * D * P_ELLIPSE)**2  # Force Squared Max

# Lat speed target
omega_t = u * d / (L_F + L_R)

# Modelo cinemático
F_KIN = ca.Function('KinDerivative',
                    [X_CAR, U_CAR],
                    [ca.vertcat(
                        u * ca.cos(phi) - v * ca.sin(phi),
                        u * ca.sin(phi) + v * ca.cos(phi),
                        omega,
                        f_x / M,
                        K_RELAX * -v,  # Decays exiting lateral speed
                        K_RELAX * (omega_t - omega)  # Drives to target rate
                    )])

# Modelo dinámico
F_DYN = ca.Function('DynDerivative',
                    [X_CAR, U_CAR],
                    [ca.vertcat(
                        u * ca.cos(phi) - v * ca.sin(phi),
                        u * ca.sin(phi) + v * ca.cos(phi),
                        omega,
                        (1 / M) * (f_x - f_f_y * ca.sin(d) + M * v * omega),
                        (1 / M) * (f_r_y + f_f_y * ca.cos(d) - M * u * omega),
                        (1 / I_Z) * (f_f_y * L_F * ca.cos(d) - f_r_y * L_R)
                    )])

# Modelo blending (Soft)
K_SMOOTH = 2
U_TRANS = 3
blend = 0.5 * (1 + ca.tanh(K_SMOOTH * (ca.fabs(u) - U_TRANS)))
F_BLEND = ca.Function('BlendDerivative',
                      [X_CAR, U_CAR],
                      [blend * F_DYN(X_CAR, U_CAR) + (1 - blend) * F_KIN(X_CAR, U_CAR)])
