"""
Bicycle model + Stanley controller
Nå med cubic spline mellom veipunkter
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# ---------------------------------------------------------------
# --- Vehicle Model ---
# ---------------------------------------------------------------
class BicycleModel:
    def __init__(self, L=2.7, dt=0.02, x0=0, y0=0, theta0=0):
        self.L = L
        self.dt = dt
        self.state = np.array([x0, y0, theta0])  # [x, y, theta]

    def derivatives(self, state, delta, v):
        x, y, th = state
        dx = v * np.cos(th)
        dy = v * np.sin(th)
        dth = v / self.L * np.tan(delta)
        return np.array([dx, dy, dth])

    def rk4_step(self, delta, v):
        s = self.state
        f = self.derivatives
        dt = self.dt

        k1 = f(s, delta, v)
        k2 = f(s + 0.5 * dt * k1, delta, v)
        k3 = f(s + 0.5 * dt * k2, delta, v)
        k4 = f(s + dt * k3, delta, v)
        self.state = s + dt / 6 * (k1 + 2*k2 + 2*k3 + k4)
        return self.state

# ---------------------------------------------------------------
# --- Stanley Controller ---
# ---------------------------------------------------------------
def stanley_control(x, y, theta, path_x, path_y, path_yaw, k, v):
    # finn nærmeste punkt
    dx = path_x - x
    dy = path_y - y
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    nearest_x = path_x[target_idx]
    nearest_y = path_y[target_idx]
    yaw_path = path_yaw[target_idx]

    # beregn tverrfeil
    map_xy = np.array([nearest_x, nearest_y])
    vec_path = np.array([np.cos(yaw_path), np.sin(yaw_path)])
    error_front_axle = np.dot(np.array([x - nearest_x, y - nearest_y]),
                              np.array([-np.sin(yaw_path), np.cos(yaw_path)]))

    # heading-feil
    theta_e = yaw_path - theta
    theta_e = np.arctan2(np.sin(theta_e), np.cos(theta_e))  # normaliser

    # Stanley formel
    delta = theta_e + np.arctan2(k * error_front_axle, v + 1e-5)
    return delta, target_idx, error_front_axle

# ---------------------------------------------------------------
# --- Generer bane med cubic spline ---
# ---------------------------------------------------------------
# Diskrete veipunkter
waypoints = np.array([
    [0, 0],
    [10, 0],
    [20, 10],
    [30, 10],
    [40, 0],
])
x_wp = waypoints[:, 0]
y_wp = waypoints[:, 1]

# Lag cubic spline
cs = CubicSpline(x_wp, y_wp)

# Generer tett bane
x_ref = np.linspace(x_wp[0], x_wp[-1], 800)
y_ref = cs(x_ref)
dy = cs(x_ref, 1)
path_yaw = np.arctan2(dy, np.ones_like(dy))  # tangentretning

# ---------------------------------------------------------------
# --- Simulering ---
# ---------------------------------------------------------------
model = BicycleModel(L=2.7, dt=0.02, x0=0, y0=0, theta0=0)
v = 10.0  # m/s
k = 1.0   # Stanley gain

history_x, history_y = [], []
N = 200

for i in range(N):
    x, y, th = model.state
    delta, target_idx, e_crosstrack = stanley_control(
        x, y, th, x_ref, y_ref, path_yaw, k, v)
    model.rk4_step(delta, v)
    history_x.append(x)
    history_y.append(y)

# ---------------------------------------------------------------
# --- Plot ---
# ---------------------------------------------------------------
plt.figure(figsize=(9, 5))
plt.plot(x_ref, y_ref, 'r--', label='Cubic spline path')
plt.plot(x_wp, y_wp, 'ko', label='Waypoints')
plt.plot(history_x, history_y, 'b', label='Actual trajectory')
plt.axis('equal')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Bicycle model + Stanley controller with cubic spline path')
plt.legend()
plt.grid(True)
plt.show()