import numpy as np
import matplotlib.pyplot as plt

# --- Define Waypoints ---
x_points = np.array([
0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0
])

y_points = np.array([
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
9, 8, 7, 6, 5, 4, 3, 2, 1, 0
])
n = len(x_points)

# --- Make parameters based on accumulated distance ---
s = np.zeros(n)
for i in range(1, n):
    s[i] = s[i-1] + np.hypot(x_points[i]-x_points[i-1], y_points[i]-y_points[i-1])
h = np.diff(s)

# --- Function solving cubic spline for one variable ---
def cubic_spline_coeff(y, h):
    n = len(y)
    A = np.zeros((n, n))
    b = np.zeros(n)
    A[0,0] = 1
    A[-1,-1] = 1
    for i in range(1, n-1):
        A[i, i-1] = h[i-1]
        A[i, i]   = 2*(h[i-1] + h[i])
        A[i, i+1] = h[i]
        b[i] = 6*((y[i+1]-y[i])/h[i] - (y[i]-y[i-1])/h[i-1])
    return np.linalg.solve(A, b)

# --- Solve for second derivatives ---
Mx = cubic_spline_coeff(x_points, h)
My = cubic_spline_coeff(y_points, h)

# --- Evaluate cubic spline at points with close distance ---
def eval_spline(s_query, s, y, M):
    """Evaluate cubic spline y(s)."""
    i = np.searchsorted(s, s_query) - 1
    i = np.clip(i, 0, len(s)-2)
    hi = s[i+1] - s[i]
    si, si1 = s[i], s[i+1]
    yi, yi1 = y[i], y[i+1]
    Mi, Mi1 = M[i], M[i+1]
    return (
        Mi*(si1 - s_query)**3/(6*hi)
        + Mi1*(s_query - si)**3/(6*hi)
        + (yi/hi - Mi*hi/6)*(si1 - s_query)
        + (yi1/hi - Mi1*hi/6)*(s_query - si)
    )

s_dense = np.linspace(0, s[-1], 1000)
x_smooth = np.array([eval_spline(si, s, x_points, Mx) for si in s_dense])
y_smooth = np.array([eval_spline(si, s, y_points, My) for si in s_dense])

# --- Plot ---
plt.figure(figsize=(8,6))
plt.plot(x_points, y_points, 'o', label='Waypoints')
plt.plot(x_smooth, y_smooth, '-', label='Cubic Spline (2D)')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('2D Cubic Spline Path (Manual Implementation)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
