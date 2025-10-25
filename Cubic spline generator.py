import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# --- Define Waypoint ---
x_points = np.array([0, 5, 10, 15, 20, 20, 20, 20, 20, 15, 10, 5, 0, 0, 0, 0, 0])
y_points = np.array([0, 0, 0, 0, 0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 8, 6, 4, 2, 0])
#x_points = np.array([0, 10, 20, 30, 40, 50])
#y_points = np.array([0, 3, 20, 7, -1, 0])
#x_points = np.array([0, 10, 20, 30, 40, 50])
#y_points = np.array([0, 0, 0, 0, 0, 0])

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

# --- Plot ---
plt.figure(figsize=(8,5))
plt.plot(x_points, y_points, 'o', label='Waypoints')
plt.plot(x_smooth, y_smooth, '-', label='Cubic Spline Path')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Cubic Spline Interpolated Path')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
