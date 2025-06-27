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

# === Vehicle parameters ===
m = 1150.0       # Mass of the vehicle [in kg]
g = 9.81         # Acceleration due to gravity [in m/s2]
L = 2.66         # Wheelbase [in m]
a = 1.06         # Distance of front axle from CoG [in m]
b = L - a        # Distance of rear axle from CoG [in m]
J = 1850.0       # Yaw moment of inertia [in kg.m2]
# eta = 0.03       # Understeer gradient

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

E1 = -1.003 - 0.537 * dFz1      # Front curvature factor
E2 = -1.003 - 0.537 * dFz2      # Rear curvature factor

# === Pacejka tire model formula ===
def pacejka(alpha, B, C, D, E):
    Fy = D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))
    return Fy

# === Simulation parameters ===
y0 = [0.0, 0.0]
t_span = (0, 10)
t_eval = np.linspace(0, 10, 2000)
eta_values = [0.05, 0.03, -0.02]           # Multiple values of understeer gradient for sensitivity analysis
eta_labels = ['Understeer (0.05)', 'Reference (0.03)', 'Oversteer (-0.02)']

results = {}

for eta in eta_values:
    K2 = Fz2 * K1 / (Fz1 - eta * K1)
    B1 = K1 / C1 / D1
    B2 = K2 / C2 / D2
    Cy1 = B1 * C1 * D1
    Cy2 = B2 * C2 * D2

    # Linear model
    def model_linear(t, y):
        v, r = y
        delta_t = delta(t)
        alpha_f = delta_t - (v + a * r) / Vx
        alpha_r = - (v - b * r) / Vx
        Fyf = Cy1 * alpha_f
        Fyr = Cy2 * alpha_r
        v_dot = (Fyf + Fyr) / m - Vx * r
        r_dot = (a * Fyf - b * Fyr) / J
        return [v_dot, r_dot]

    # Non-linear model
    def model_nonlinear(t, y):
        v, r = y
        delta_t = delta(t)
        alpha_f = delta_t - (v + a * r) / Vx
        alpha_r = - (v - b * r) / Vx
        Fyf = pacejka(alpha_f, B1, C1, D1, E1)
        Fyr = pacejka(alpha_r, B2, C2, D2, E2)
        v_dot = (Fyf + Fyr) / m - Vx * r
        r_dot = (a * Fyf - b * Fyr) / J
        return [v_dot, r_dot]

    sol_lin = solve_ivp(model_linear, t_span, y0, t_eval=t_eval, method='Radau', rtol=1e-6, atol=1e-9)
    sol_nl = solve_ivp(model_nonlinear, t_span, y0, t_eval=t_eval, method='Radau', rtol=1e-6, atol=1e-9)

    def process(sol, is_linear):
        v, r = sol.y
        beta = v / Vx
        ay = np.zeros_like(v)
        Fyf = np.zeros_like(v)
        Fyr = np.zeros_like(v)
        for i, ti in enumerate(sol.t):
            delta_t = delta(ti)
            alpha_f = delta_t - (v[i] + a * r[i]) / Vx
            alpha_r = - (v[i] - b * r[i]) / Vx
            if is_linear:
                Fyf[i] = Cy1 * alpha_f
                Fyr[i] = Cy2 * alpha_r
            else:
                Fyf[i] = pacejka(alpha_f, B1, C1, D1, E1)
                Fyr[i] = pacejka(alpha_r, B2, C2, D2, E2)
            v_dot = (Fyf[i] + Fyr[i]) / m - Vx * r[i]
            ay[i] = (v_dot + Vx * r[i]) / g
        return np.rad2deg(r), np.rad2deg(beta), ay, Fyf, Fyr

    results[(eta, 'linear')] = process(sol_lin, True)
    results[(eta, 'nonlinear')] = process(sol_nl, False)

# === Plotting ===
ref_eta = 0.03
if ref_eta in eta_values:
    ref_lin = results[(ref_eta, 'linear')]
    ref_nl = results[(ref_eta, 'nonlinear')]

    # === Plotting (static graphs) ===
    plt.figure()
    plt.plot(t_eval, ref_lin[0], label='Linear')
    plt.plot(t_eval, ref_nl[0], label='Non-linear')
    plt.ylabel('Yaw Rate (deg/s)')
    plt.xlabel('Time (s)')
    plt.title('Yaw Rate vs Time')
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t_eval, ref_lin[1], label='Linear')
    plt.plot(t_eval, ref_nl[1], label='Non-linear')
    plt.ylabel('Body Slip Angle β (deg)')
    plt.xlabel('Time (s)')
    plt.title('Body Slip Angle vs Time')
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t_eval, ref_lin[2], label='Linear')
    plt.plot(t_eval, ref_nl[2], label='Non-linear')
    plt.ylabel('Lateral Acceleration (g)')
    plt.xlabel('Time (s)')
    plt.title('Lateral Acceleration vs Time')
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t_eval, ref_lin[3], label='Front Lateral Force, Linear')
    plt.plot(t_eval, ref_nl[3], label='Front Lateral Force, Non-linear')
    plt.plot(t_eval, ref_lin[4], '--', label='Rear Lateral Force, Linear')
    plt.plot(t_eval, ref_nl[4], '--', label='Rear Lateral Force, Non-linear')
    plt.ylabel('Lateral Force (N)')
    plt.xlabel('Time (s)')
    plt.title('Axle Lateral Forces vs Time')
    plt.legend()
    plt.grid(True)

# Sensitivity analysis plots
metrics = [
    (0, 'Yaw Rate', 'deg/s'),
    (2, 'Lateral Acceleration', 'g'),
    (1, 'Body Slip Angle', 'deg')
]

for metric_idx, label, unit in metrics:
    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for eta in eta_values:
        axs[0].plot(t_eval, results[(eta, 'linear')][metric_idx], label=eta_labels[eta_values.index(eta)])
        axs[1].plot(t_eval, results[(eta, 'nonlinear')][metric_idx], label=eta_labels[eta_values.index(eta)])
    axs[0].set_title(f'{label} Sensitivity - Linear')
    axs[1].set_title(f'{label} Sensitivity - Nonlinear')
    for ax in axs:
        ax.set_ylabel(f'{label} ({unit})')
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.grid(True)
    plt.tight_layout()

# Tyre force sensitivity
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
axs[0].set_title('Tyre Force Sensitivity - Linear')
axs[1].set_title('Tyre Force Sensitivity - Nonlinear')

# Store handles in separate lists for ordering
front_handles_top, rear_handles_top = [], []
front_handles_bottom, rear_handles_bottom = [], []

# Plot and store front curves first
for eta, label in zip(eta_values, eta_labels):
    front_lin = results[(eta, 'linear')][3]
    front_nl = results[(eta, 'nonlinear')][3]

    h_front_lin, = axs[0].plot(t_eval, front_lin, label=f'Front, {label}')
    h_front_nl,  = axs[1].plot(t_eval, front_nl, label=f'Front, {label}')

    front_handles_top.append(h_front_lin)
    front_handles_bottom.append(h_front_nl)

# Plot and store rear curves after
for eta, label in zip(eta_values, eta_labels):
    rear_lin = results[(eta, 'linear')][4]
    rear_nl  = results[(eta, 'nonlinear')][4]

    h_rear_lin, = axs[0].plot(t_eval, rear_lin, '--', label=f'Rear, {label}')
    h_rear_nl,  = axs[1].plot(t_eval, rear_nl, '--', label=f'Rear, {label}')

    rear_handles_top.append(h_rear_lin)
    rear_handles_bottom.append(h_rear_nl)

# Set legends with correct order: fronts first, then rears
axs[0].legend(handles=front_handles_top + rear_handles_top)
axs[1].legend(handles=front_handles_bottom + rear_handles_bottom)

for ax in axs:
    ax.set_ylabel('Force (N)')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

plt.tight_layout()

for ax in axs:
    ax.set_ylabel('Force (N)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)

plt.tight_layout()
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

# # === Plotting Sensitivity Analysis (animated graphs) ===
# eta_colors = ['tab:red', 'tab:blue', 'tab:green']
# n_frames = len(t_eval)

# # === Helper function for per-model axis limits
# def get_model_axis_limits(index, model):
#     all_vals = []
#     for eta in eta_values:
#         all_vals.extend(results[(eta, model)][index])
#     min_val = min(all_vals)
#     max_val = max(all_vals)
#     margin = 0.1 * (max_val - min_val) if max_val != min_val else 1.0
#     return min_val - margin, max_val + margin

# # === Animate Yaw Rate, Lateral Acceleration, Slip Angle
# metrics = [
#     (0, 'Yaw Rate', 'deg/s'),
#     (2, 'Lateral Acceleration', 'g'),
#     (1, 'Body Slip Angle', 'deg')
# ]

# for metric_idx, label, unit in metrics:
#     fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
#     lines = [[], []]  # Linear, Nonlinear

#     # Set x and y limits separately
#     axs[0].set_xlim(t_eval[0], t_eval[-1])  # Linear
#     axs[1].set_xlim(t_eval[0], t_eval[-1])  # Nonlinear

#     ymin_lin, ymax_lin = get_model_axis_limits(metric_idx, 'linear')
#     ymin_nl,  ymax_nl  = get_model_axis_limits(metric_idx, 'nonlinear')

#     axs[0].set_ylim(ymin_lin, ymax_lin)
#     axs[1].set_ylim(ymin_nl,  ymax_nl)

#     # Labels and titles
#     axs[0].set_title(f'{label} Sensitivity - Linear')
#     axs[1].set_title(f'{label} Sensitivity - Nonlinear')
#     for ax in axs:
#         ax.set_ylabel(f'{label} ({unit})')
#         ax.set_xlabel('Time (s)')
#         ax.grid(True)

#     # Lines for each eta
#     for i, eta in enumerate(eta_values):
#         l1, = axs[0].plot([], [], label=eta_labels[i], color=eta_colors[i])
#         l2, = axs[1].plot([], [], label=eta_labels[i], color=eta_colors[i])
#         lines[0].append(l1)
#         lines[1].append(l2)

#     axs[0].legend(loc='lower right')
#     axs[1].legend(loc='lower right')
#     plt.tight_layout()

#     # Animation function
#     def animate(i):
#         for j, eta in enumerate(eta_values):
#             lines[0][j].set_data(t_eval[:i], results[(eta, 'linear')][metric_idx][:i])
#             lines[1][j].set_data(t_eval[:i], results[(eta, 'nonlinear')][metric_idx][:i])
#         return lines[0] + lines[1]

#     ani = FuncAnimation(fig, animate, frames=n_frames, interval=10, blit=True, repeat=False)
#     # Save animation as GIF named by the metric label, e.g. "yaw_rate_animation.gif"
#     filename = f"{label.lower().replace(' ', '_')}_animation.gif"
#     ani.save(filename, writer='pillow', fps=30)
#     plt.close(fig)  # Close the figure to prevent it from showing
#     # plt.show()

# # === Animate Tyre Force Sensitivity (Front & Rear)
# fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
# lines_force = [[], []]  # Linear, Nonlinear

# # Compute limits separately for front & rear per model
# fmin_lin = min(get_model_axis_limits(3, 'linear')[0], get_model_axis_limits(4, 'linear')[0])
# fmax_lin = max(get_model_axis_limits(3, 'linear')[1], get_model_axis_limits(4, 'linear')[1])
# fmin_nl  = min(get_model_axis_limits(3, 'nonlinear')[0], get_model_axis_limits(4, 'nonlinear')[0])
# fmax_nl  = max(get_model_axis_limits(3, 'nonlinear')[1], get_model_axis_limits(4, 'nonlinear')[1])

# axs[0].set_xlim(t_eval[0], t_eval[-1])
# axs[1].set_xlim(t_eval[0], t_eval[-1])
# axs[0].set_ylim(fmin_lin, fmax_lin)
# axs[1].set_ylim(fmin_nl, fmax_nl)

# axs[0].set_title('Tyre Force Sensitivity - Linear')
# axs[1].set_title('Tyre Force Sensitivity - Nonlinear')

# for ax in axs:
#     ax.set_ylabel('Force (N)')
#     ax.set_xlabel('Time (s)')
#     ax.grid(True)

# for i, (eta, label, color) in enumerate(zip(eta_values, eta_labels, eta_colors)):
#     # Linear
#     fl, = axs[0].plot([], [], label=f'Front, {label}', color=color)
#     rl, = axs[0].plot([], [], '--', label=f'Rear, {label}', color=color)
#     # Nonlinear
#     fn, = axs[1].plot([], [], label=f'Front, {label}', color=color)
#     rn, = axs[1].plot([], [], '--', label=f'Rear, {label}', color=color)
#     lines_force[0].extend([fl, rl])
#     lines_force[1].extend([fn, rn])

# axs[0].legend(loc='upper left')
# axs[1].legend(loc='lower right')
# plt.tight_layout()

# # Animation function for forces
# def animate_force(i):
#     for j, eta in enumerate(eta_values):
#         lines_force[0][2*j].set_data(t_eval[:i], results[(eta, 'linear')][3][:i])    # Front linear
#         lines_force[0][2*j+1].set_data(t_eval[:i], results[(eta, 'linear')][4][:i])  # Rear linear
#         lines_force[1][2*j].set_data(t_eval[:i], results[(eta, 'nonlinear')][3][:i]) # Front non-linear
#         lines_force[1][2*j+1].set_data(t_eval[:i], results[(eta, 'nonlinear')][4][:i]) # Rear non-linear
#     return lines_force[0] + lines_force[1]

# ani_force = FuncAnimation(fig, animate_force, frames=n_frames, interval=10, blit=True, repeat=False)
# ani_force.save('tyre_force_animation.gif', writer='pillow', fps=30)
# plt.close(fig)