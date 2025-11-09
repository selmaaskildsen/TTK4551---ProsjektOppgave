import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# --------------------------------
# 1. Veipunkter (kan være i vilkårlig rekkefølge)
# --------------------------------
"""
waypoints = np.array([
    [0, 0],[1, 0],[2, 0],[3, 0],[4, 0],[5, 0],[6, 0],[7, 0],[8, 0],[9, 0],[10, 0],
    [11, 0],[12, 0],[13, 0],[14, 0],[15, 0],[16, 0],[17, 0],[18, 0],[19, 0],[20, 0],
    [20, 1],[20, 2],[20, 3],[20, 4],[20, 5],[20, 6],[20, 7],[20, 8],[20, 9],[20, 10],
    [19, 10],[18, 10],[17, 10],[16, 10],[15, 10],[14, 10],[13, 10],[12, 10],[11, 10],[10, 10],
    [9, 10],[8, 10],[7, 10],[6, 10],[5, 10],[4, 10],[3, 10],[2, 10],[1, 10],[0, 10],
    [0, 9],[0, 8],[0, 7],[0, 6],[0, 5],[0, 4],[0, 3],[0, 2],[0, 1],[0, 0]
])
"""

waypoints = np.array([
    [0,0],[1,1],[2,2],[3,1],[4,0],[5,1],[6,2],[7,1],[8,0],[9,1],[10,2],
])
# Hvis du vil teste motsatt rekkefølge:
# waypoints = waypoints[::-1]

x = waypoints[:, 0]
y = waypoints[:, 1]

# --------------------------------
# 2. Lag B-spline gjennom punktene
# --------------------------------
# s=0 gir eksakt gjennom punktene (ingen glatting)
tck, u = splprep([x, y], s=0, k=3)

# --------------------------------
# 3. Evaluer spline med mange punkter
# --------------------------------
u_fine = np.linspace(0, 1, 200)
x_smooth, y_smooth = splev(u_fine, tck)

# --------------------------------
# 4. Plot resultatet
# --------------------------------
plt.figure(figsize=(7,4))       
plt.plot(x, y, 'ro--', label='Veipunkter')
plt.plot(x_smooth, y_smooth, 'b', label='B-spline bane')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('B-spline interpolasjon av veipunkter')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
