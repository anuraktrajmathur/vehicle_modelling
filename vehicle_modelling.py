### This Python script models a Single Track (Bicycle) Vehicle Dynamics Model.
### It simulates both linear and non-linear (Pacejka) tyre behavior to analyze and compare
### vehicle responses: lateral acceleration, lateral forces, yaw rate, and body slip angle.
### 
### The model is fully parameterized—vehicle and tyre parameters are defined using variables
### for clarity, reusability, and ease of modification. Each section is well-commented
### to support understanding and future development.
### 
### Comparative plots are generated to visualize the differences between the linear and
### non-linear tyre models under identical steering inputs. A commented out section at the 
### end of the script generates animated plots.
###
### Python version: 3.11+
### Author: Anurakt Raj Mathur, M.Sc.

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation
import numpy as np

# === Vehicle parameters ===
m = 1150.0       # Mass of the vehicle [in kg]
g = 9.81         # Acceleration due to gravity [in m/s2]
L = 2.66         # Wheelbase [in m]
a = 1.06         # Distance of front axle from CoG [in m]
b = L - a        # Distance of rear axle from CoG [in m]
J = 1850.0       # Yaw moment of inertia [in kg.m2]
eta = 0.03       # Understeer gradient

# == Inputs (steering angle and longitudinal velocity) == 

Vx = 80 / 3.6    # Longitudinal velocity [m/s]

# Steering input: 4 deg left turn ramp (left is positive) 
def delta(t):
    if t < 1.0:
        return 0.0
    elif t <= 1.02:
        return (t - 1.0) * (4.0 * np.pi / 180) / 0.02
    else:
        return 4.0 * np.pi / 180


# === Non-linear tyre data CF. Magic Formula model == 

# simplified model description: Fy = D.sin[CC.atan{B.x-E.(B.x-atac(B.x))}]
# Index: 1 --> front ; 2 --> rear
# Peak factor (D) is taken proportional to axle load

Fz0 = 8000.0           # Nominal load on axle [N]                      
Fz1 = m * g * b / L    # Front axle load [N]
Fz2 = m * g * a / L    # Rear axle load [N]

dFz1 = (Fz1 - Fz0) / Fz0    
dFz2 = (Fz2 - Fz0) / Fz0

mu1 = 0.9       # Front tyre-road friction coefficient 
mu2 = 0.9       # Rear tyre-road friction coefficient

D1 = mu1 * (-0.145 * dFz1 + 0.99) * Fz1     # Front peak factor [N]
D2 = mu2 * (-0.145 * dFz2 + 0.99) * Fz2     # Rear peak factor [N]

C1 = 1.19       # Front shape factor [N]
C2 = 1.19       # Rear shape factor [N]

K1 = 14.95 * Fz0 * np.sin(2 * np.arctan(Fz1 / 2.13 / Fz0))  
K2 = Fz2 * K1 / (Fz1 - eta * K1)

B1 = K1 / C1 / D1       # Front stiffness factor
B2 = K2 / C2 / D2       # Rear stiffness factor

E1 = -1.003 - 0.537 * dFz1      # Front curvature factor
E2 = -1.003 - 0.537 * dFz2      # Rear curvature factor

# === Linear tyre model == #

Cy1 = B1 * C1 * D1      # Front cornering stiffness [N/rad]
Cy2 = B2 * C2 * D2      # Rear cornering stiffness [N/rad]



# === Pacejka tire model formula ===
def pacejka(alpha, B, C, D, E):
    Fy = D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))
    return Fy

# === Non-linear model dynamics ===
def model_nonlinear_correct(t, y):
    v, r = y  # lateral velocity and yaw rate
    delta_t = delta(t)

    # Calculate slip angles
    alpha_f = delta_t - (v + a * r) / Vx
    alpha_r = - (v - b * r) / Vx

    # Calculate lateral tire forces using Pacejka formula
    Fyf = pacejka(alpha_f, B1, C1, D1, E1)
    Fyr = pacejka(alpha_r, B2, C2, D2, E2)

    # Equations of motion
    v_dot = (Fyf + Fyr) / m - Vx * r
    r_dot = (a * Fyf - b * Fyr) / J

    return [v_dot, r_dot]

# === Linear model dynamics (linear tire model) ===
def model_linear(t, y):
    v, r = y  # lateral velocity and yaw rate
    delta_t = delta(t)

    # Slip angles
    alpha_f = delta_t - (v + a * r) / Vx
    alpha_r = - (v - b * r) / Vx

    # Linear lateral forces
    Fy1 = Cy1 * alpha_f
    Fy2 = Cy2 * alpha_r

    Fy_total = Fy1 + Fy2
    Mz_total = a * Fy1 - b * Fy2

    # Equations of motion
    v_dot = Fy_total / m - Vx * r
    r_dot = Mz_total / J

    return [v_dot, r_dot]

# === Simulation parameters ===
y0 = [0.0, 0.0]
t_span = (0, 10)
t_eval = np.linspace(0, 10, 2000)

# Solve ODEs
sol_nl = solve_ivp(model_nonlinear_correct, t_span, y0, t_eval=t_eval, method='Radau', rtol=1e-6, atol=1e-9)
sol_l = solve_ivp(model_linear, t_span, y0, t_eval=t_eval, method='Radau', rtol=1e-6, atol=1e-9)

# === Post-processing ===
def process(sol, model='nonlinear'):
    t = sol.t
    ay = np.zeros_like(t)
    Fyf = np.zeros_like(t)
    Fyr = np.zeros_like(t)

    v = sol.y[0]
    r = sol.y[1]

    for i, ti in enumerate(t):
        delta_t = delta(ti)
        alpha_f = delta_t - (v[i] + a * r[i]) / Vx
        alpha_r = - (v[i] - b * r[i]) / Vx

        if model == 'linear':
            Fyf[i] = Cy1 * alpha_f
            Fyr[i] = Cy2 * alpha_r
        else:
            Fyf[i] = pacejka(alpha_f, B1, C1, D1, E1)
            Fyr[i] = pacejka(alpha_r, B2, C2, D2, E2)

        v_dot = (Fyf[i] + Fyr[i]) / m - Vx * r[i]
        ay[i] = (v_dot + Vx * r[i]) / g

    beta = v / Vx  # Slip angle β

    return beta, r, ay, Fyf, Fyr

# Process simulation results
beta_nl, r_nl, ay_nl, Fyf_nl, Fyr_nl = process(sol_nl, 'nonlinear')
beta_l, r_l, ay_l, Fyf_l, Fyr_l = process(sol_l, 'linear')

# === Plotting (static graphs) ===
plt.figure()
plt.plot(sol_l.t, np.rad2deg(r_l), label='Linear')
plt.plot(sol_nl.t, np.rad2deg(r_nl), label='Non-linear')
plt.ylabel('Yaw Rate (deg/s)')
plt.xlabel('Time (s)')
plt.title('Yaw Rate vs Time')
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(sol_l.t, np.rad2deg(beta_l),  label='Linear')
plt.plot(sol_nl.t, np.rad2deg(beta_nl), label='Non-linear')
plt.ylabel('Body Slip Angle β (deg)')
plt.xlabel('Time (s)')
plt.title('Body Slip Angle vs Time')
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(sol_l.t, ay_l, label='Linear')
plt.plot(sol_nl.t, ay_nl, label='Non-linear')
plt.ylabel('Lateral Acceleration (g)')
plt.xlabel('Time (s)')
plt.title('Lateral Acceleration vs Time')
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(sol_l.t, Fyf_l, label='Front Lateral Force, Linear')
plt.plot(sol_nl.t, Fyf_nl, label='Front Lateral Force, Non-linear')
plt.plot(sol_l.t, Fyr_l,  '--', label='Rear Lateral Force, Linear')
plt.plot(sol_nl.t, Fyr_nl, '--', label='Rear Lateral Force, Non-linear')
plt.ylabel('Lateral Force (N)')
plt.xlabel('Time (s)')
plt.title('Axle Lateral Forces vs Time')
plt.legend()
plt.grid(True)

plt.show()

# # === Plotting (animated graphs) ===

# # Time vector and data
# t = sol_l.t
# y_lin = np.rad2deg(r_l)
# y_nl  = np.rad2deg(r_nl)
# beta_lin = np.rad2deg(beta_l)
# beta_nl  = np.rad2deg(beta_nl)
# ay_lin = ay_l
# ay_nl  = ay_nl
# fyl_lin = Fyf_l
# fyl_nl  = Fyf_nl
# fyr_lin = Fyr_l
# fyr_nl  = Fyr_nl

# # Create subplots
# fig, axs = plt.subplots(2, 2, figsize=(14, 10), dpi=100, constrained_layout=True)

# # Set common x-limits
# for ax in axs.flat:
#     ax.set_xlim(0, 10.5)
#     ax.grid(True)

# # === Subplot 1: Yaw Rate ===
# ax1 = axs[0, 0]
# ax1.set_ylim(min(np.min(y_lin), np.min(y_nl)) - 2, max(np.max(y_lin), np.max(y_nl)) + 2)
# line1_1, = ax1.plot([], [], label='Linear')
# line1_2, = ax1.plot([], [], label='Non-linear')
# ax1.set_title("Yaw Rate vs Time")
# ax1.set_ylabel("Yaw Rate (deg/s)")
# ax1.legend(loc='center right')

# # === Subplot 2: Body Slip Angle β ===
# ax2 = axs[0, 1]
# ax2.set_ylim(min(np.min(beta_lin), np.min(beta_nl)) - 1, max(np.max(beta_lin), np.max(beta_nl)) + 1)
# line2_1, = ax2.plot([], [], label='Linear')
# line2_2, = ax2.plot([], [], label='Non-linear')
# ax2.set_title("Body Slip Angle vs Time")
# ax2.set_ylabel("Body Slip Angle β (deg)")
# ax2.legend(loc='center right')

# # === Subplot 3: Lateral Acceleration ===
# ax3 = axs[1, 0]
# ax3.set_ylim(min(np.min(ay_lin), np.min(ay_nl)) - 0.1, max(np.max(ay_lin), np.max(ay_nl)) + 0.1)
# line3_1, = ax3.plot([], [], label='Linear')
# line3_2, = ax3.plot([], [], label='Non-linear')
# ax3.set_title("Lateral Acceleration vs Time")
# ax3.set_ylabel("Lateral Acceleration (g)")
# ax3.legend(loc='center right')

# # === Subplot 4: Lateral Forces ===
# ax4 = axs[1, 1]
# combined_min = min(np.min(fyl_lin), np.min(fyl_nl), np.min(fyr_lin), np.min(fyr_nl)) - 500
# combined_max = max(np.max(fyl_lin), np.max(fyl_nl), np.max(fyr_lin), np.max(fyr_nl)) + 500
# ax4.set_ylim(combined_min, combined_max)
# line4_1, = ax4.plot([], [], label='Front Lateral Force, Linear')
# line4_2, = ax4.plot([], [], label='Front Lateral Force, Non-linear')
# line4_3, = ax4.plot([], [], '--', label='Rear Lateral Force Linear')
# line4_4, = ax4.plot([], [], '--', label='Rear Lateral Force Non-linear')
# ax4.set_title("Axle Lateral Forces vs Time")
# ax4.set_ylabel("Lateral Force (N)")
# ax4.legend(loc='lower right')

# # Set x-labels
# for ax in axs[1, :]:
#     ax.set_xlabel("Time (s)")

# plt.tight_layout()

# # === Animation function ===
# def animate(i):
#     # Yaw rate
#     line1_1.set_data(t[:i], y_lin[:i])
#     line1_2.set_data(t[:i], y_nl[:i])
    
#     # Slip angle
#     line2_1.set_data(t[:i], beta_lin[:i])
#     line2_2.set_data(t[:i], beta_nl[:i])
    
#     # Lateral acceleration
#     line3_1.set_data(t[:i], ay_lin[:i])
#     line3_2.set_data(t[:i], ay_nl[:i])
    
#     # Forces
#     line4_1.set_data(t[:i], fyl_lin[:i])
#     line4_2.set_data(t[:i], fyl_nl[:i])
#     line4_3.set_data(t[:i], fyr_lin[:i])
#     line4_4.set_data(t[:i], fyr_nl[:i])
    
#     return (
#         line1_1, line1_2, line2_1, line2_2,
#         line3_1, line3_2, line4_1, line4_2,
#         line4_3, line4_4
#     )

# # Create animation
# ani = FuncAnimation(
#     fig, animate, frames=len(t), interval=1, blit=True, repeat=False
# )

# plt.show()

# # Save animation
# ani.save("plots.gif", writer='pillow', fps=30, dpi=150)
