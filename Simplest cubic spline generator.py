import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# --- Definer veipunkter ---
x_points = np.array([0, 10, 20, 30, 40, 50])
y_points = np.array([0, 5, 0, -5, 0, 5])

# --- Lag kontinuerlig spline-funksjon ---
cs = CubicSpline(x_points, y_points)

# --- Plot den kontinuerlige funksjonen ---
x_vals = np.linspace(x_points[0], x_points[-1], 400)
y_vals = cs(x_vals)

plt.figure(figsize=(8,4))
plt.plot(x_points, y_points, 'o', label='Waypoints')
plt.plot(x_vals, y_vals, '-', label='Cubic Spline (kontinuerlig)')
plt.title("Kontinuerlig bane generert med Cubic Spline")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
