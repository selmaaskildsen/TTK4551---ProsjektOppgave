import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline   

x_points = np.array([0, 2.5, 5])
y_points = np.array([0, 1.5, 2])

# --- Make parameters---
dist = np.zeros(len(x_points))
for i in range(1, len(x_points)):
    dist[i] = dist[i-1] + np.hypot(x_points[i] - x_points[i-1], y_points[i] - y_points[i-1])

# --- Cubic spline interpolation for both x and y as a function of distance ---
spline_x = CubicSpline(dist, x_points)
spline_y = CubicSpline(dist, y_points)

# --- Evaluate the splines ---
s_vals = np.linspace(0, dist[-1], 500)
x_smooth = spline_x(s_vals)
y_smooth = spline_y(s_vals)


# --- Bicycle Model ---
def bicycle_derivatives(x, y, theta, v, delta, L):
    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    dtheta = v / L * np.tan(delta)
    return dx, dy, dtheta

# --- RK4 Integrator ---
def bicycle_model_rk4(x, y, theta, v, delta, L, dt):
    k1x, k1y, k1th = bicycle_derivatives(x, y, theta, v, delta, L)
    k2x, k2y, k2th = bicycle_derivatives(x + 0.5*dt*k1x, y + 0.5*dt*k1y, theta + 0.5*dt*k1th, v, delta, L)
    k3x, k3y, k3th = bicycle_derivatives(x + 0.5*dt*k2x, y + 0.5*dt*k2y, theta + 0.5*dt*k2th, v, delta, L)
    k4x, k4y, k4th = bicycle_derivatives(x + dt*k3x, y + dt*k3y, theta + dt*k3th, v, delta, L)
    x_next = x + (dt/6)*(k1x + 2*k2x + 2*k3x + k4x)
    y_next = y + (dt/6)*(k1y + 2*k2y + 2*k3y + k4y)
    theta_next = theta + (dt/6)*(k1th + 2*k2th + 2*k3th + k4th)
    return x_next, y_next, theta_next

# --- Stanley Controller ---
def stanley_control(phi, k, e, v):
    delta = phi + np.arctan2(k * e, v)
    return delta

# --- Cross track error ---
def cross_track_error(x, y, x_path, y_path):
