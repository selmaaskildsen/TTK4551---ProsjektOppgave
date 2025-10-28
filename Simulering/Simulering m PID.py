import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# ======================================================
# --- 1. Generate Cubic Spline Path ---
# ======================================================
x_points = np.array([
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
    19, 18, 17, 16, 15, 14, 13, 12, 11, 10,
    9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
])
y_points = np.array([
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    9, 8, 7, 6, 5, 4, 3, 2, 1, 0
])

# Beregn kumulativ distanse
dist = np.zeros(len(x_points))
for i in range(1, len(x_points)):
    dist[i] = dist[i - 1] + np.hypot(
        x_points[i] - x_points[i - 1],
        y_points[i] - y_points[i - 1]
    )

# Lag cubic spline x(s), y(s)
spline_x = CubicSpline(dist, x_points)
spline_y = CubicSpline(dist, y_points)

# Evaluer banen tett
s_vals = np.linspace(0, dist[-1], 5000)
x_path = spline_x(s_vals)
y_path = spline_y(s_vals)
dx = spline_x.derivative()(s_vals)
dy = spline_y.derivative()(s_vals)
path_yaw = np.arctan2(dy, dx)

# ======================================================
# --- 2. Bicycle Model with RK4 integration ---
# ======================================================
class BicycleModel:
    def __init__(self, L=2.5, dt=0.02, x0=0, y0=0, yaw0=0):
        self.L = L
        self.dt = dt
        self.state = np.array([x0, y0, yaw0])

    def derivatives(self, state, delta, v):
        x, y, yaw = state
        dx = v * np.cos(yaw)
        dy = v * np.sin(yaw)
        dyaw = v / self.L * np.tan(delta)
        return np.array([dx, dy, dyaw])

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

# ======================================================
# --- 3. PID Controller for path following ---
# ======================================================
class PIDController:
    def __init__(self, Kp=100, Ki=0, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def pid_path_control(state, path_x, path_y, path_yaw, pid, dt):
    x, y, yaw = state

    # Finn nærmeste punkt
    distances = np.hypot(path_x - x, path_y - y)
    idx = np.argmin(distances)
    target_yaw = path_yaw[idx]

    # Feil i heading
    heading_error = np.arctan2(np.sin(target_yaw - yaw), np.cos(target_yaw - yaw))

    # Tverravstandsfeil (cross-track error)
    map_x, map_y = path_x[idx], path_y[idx]
    cte = np.sin(target_yaw) * (x - map_x) - np.cos(target_yaw) * (y - map_y)

    # Bruk PID på summen av heading + cte
    total_error = heading_error + 0.3 * cte
    delta = pid.control(total_error, dt)

    # Begrens styringen (for realisme)
    delta = np.clip(delta, -np.radians(30), np.radians(30))
    return delta, idx, heading_error, cte

# ======================================================
# --- 4. Simulation ---
# ======================================================
dt = 0.02
v = 4.0
model = BicycleModel(dt=dt)
pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.5)

x_hist, y_hist = [], []
cte_hist, heading_hist = [], []

max_time = dist[-1] / v * 1.2

for t in np.arange(0, max_time, dt):
    delta, idx, heading_error, cte = pid_path_control(model.state, x_path, y_path, path_yaw, pid, dt)
    model.rk4_step(delta, v)
    x_hist.append(model.state[0])
    y_hist.append(model.state[1])
    cte_hist.append(cte)
    heading_hist.append(heading_error)
    if idx >= len(x_path) - 2:
        break

# ======================================================
# --- 5. Plot Results ---
# ======================================================
plt.figure(figsize=(10, 6))
plt.plot(x_path, y_path, 'k--', label='Cubic Spline Path')
plt.plot(x_hist, y_hist, 'r-', label='Vehicle Trajectory (PID)')
plt.scatter(x_points, y_points, c='blue', s=40, label='Waypoints')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('PID Controller Path Following (Cubic Spline Path)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()

# ======================================================
# --- 6. Plot Cross-Track and Heading Errors ---
# ======================================================
time = np.arange(0, len(cte_hist)) * dt
plt.figure(figsize=(10, 4))
plt.subplot(2, 1, 1)
plt.plot(time, cte_hist)
plt.ylabel('Cross-track error [m]')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(time, np.rad2deg(heading_hist))
plt.ylabel('Heading error [deg]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.show()
