import os
import requests
import numpy as np
import matplotlib.pyplot as plt
from shapely import wkt
import pandas as pd
from scipy.interpolate import CubicSpline
import json

# -------------------------------------------------------
# 1. Hent vegdata fra NVDB
# -------------------------------------------------------
url = "https://nvdbapiles.atlas.vegvesen.no/vegnett/api/v4/veglenkesekvenser"
params = {
    "vegsystemreferanse": "KV1699",  # Kalkbrennerveien
    "kommune": "3201"                # Bærum
}

resp = requests.get(url, params=params)
resp.raise_for_status()
data = resp.json()

if "objekter" not in data or len(data["objekter"]) == 0:
    raise ValueError("Ingen objekter funnet. Sjekk vegsystemreferanse eller kommune.")

# -------------------------------------------------------
# 2. Hent og sorter veglenker etter startposisjon
# -------------------------------------------------------
alle_segmenter = []

for item in data["objekter"]:
    for veglenke in item["veglenker"]:
        geom = veglenke["geometri"]["wkt"]
        if not geom or not geom.startswith("LINESTRING"):
            continue

        startpos = veglenke.get("startposisjon", 0)
        line = wkt.loads(geom)
        x, y = line.xy
        segment = list(zip(x, y))
        alle_segmenter.append((startpos, segment))

# Sorter etter startposisjon langs vegen
alle_segmenter.sort(key=lambda s: s[0])

# Slå sammen alle punkter i rekkefølge
vei = [p for _, segment in alle_segmenter for p in segment]

# -------------------------------------------------------
# 3. Konverter til NumPy og normaliser (sett m0 = origo)
# -------------------------------------------------------
x = np.array([p[0] for p in vei])
y = np.array([p[1] for p in vei])

x0, y0 = x[0], y[0]
x -= x0
y -= y0

#print(f"Origo satt til (x0, y0) = ({x0:.1f}, {y0:.1f})")

# -------------------------------------------------------
# 4. Beregn avstand langs vegen og fjern duplikater
# -------------------------------------------------------

dist = np.zeros(len(x))
for i in range(1, len(x)):
    dist[i] = dist[i-1] + np.hypot(x[i] - x[i-1], y[i] - y[i-1])

"""
# --- Skriv ut euklidske avstander mellom punkter ---
print("\nEuklidsk avstand mellom etterfølgende punkter:")

tot = 0.0


for i in range(1, len(x)):
    d = np.hypot(x[i] - x[i-1], y[i] - y[i-1])
    tot += d
    print(f"Punkt {i-1:3d} → {i:3d}: {d:.3f} m")

print(f"Total euklidsk avstand: {tot:.3f} m\n")
"""

# Fjern punkter der avstanden ikke øker (duplikate punkter)
mask = np.diff(dist, prepend=-1) > 0
x = x[mask]
y = y[mask]
dist = dist[mask]

# -------------------------------------------------------
# 5. Cubic spline-interpolasjon
# -------------------------------------------------------
spline_x = CubicSpline(dist, x)
spline_y = CubicSpline(dist, y)

s_vals = np.linspace(0, dist[-1], 500)
x_smooth = spline_x(s_vals)
y_smooth = spline_y(s_vals)

x_smooth = spline_x(s_vals)
y_smooth = spline_y(s_vals)

# lag waypoints-array
waypoint = np.array(list(zip(x_smooth, y_smooth)))


# --- Vehicle Model ---

class BicycleModel:

    def __init__(self, L=2.7, dt=0.02, x0=0, y0=0, theta0=np.pi/2):
        self.L = L
        self.dt = dt
        self.state = np.array([x0, y0, theta0]) # [x, y, theta]

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
        k2 = f(s + 0.5*dt*k1, delta, v)
        k3 = f(s + 0.5*dt*k2, delta, v)
        k4 = f(s + dt*k3, delta, v)
        
        self.state = s + dt*(k1+2*k2+2*k3+k4)/6
        self.state[2] = (self.state[2] + np.pi) % (2*np.pi) - np.pi
        return self.state


# --- Stanley Controller ---

class StanleyController:
    def __init__(self, k=0.8):
        self.k = k

    def control(self, state, path, v):
        x, y, th = state
        
        # Finn nærmeste punkt
        dists = np.sum((path - np.array([x,y]))**2, axis=1)
        i = np.argmin(dists)
        target = path[min(i+1, len(path)-1)]
        path_dir = np.arctan2(target[1]-path[i,1], target[0]-path[i,0])
        
        # Cross-track error (sign via kryssprodukt)
        dx, dy = x-path[i,0], y-path[i,1]
        v_path = np.array([np.cos(path_dir), np.sin(path_dir)])
        e = -np.cross(np.r_[v_path,0], np.r_[dx,dy,0])[2]
        psi_err = (path_dir - th + np.pi)%(2*np.pi)-np.pi
        delta = psi_err + np.arctan2(self.k*e, v) 
        return delta, e, psi_err

# --- Demo ---

if __name__ == "__main__":

    # Lag veipunkter
    waypoints = waypoint
    model = BicycleModel()
    ctrl = StanleyController(k=1.2) 

    v = 1.0
    log = {"x":[],"y":[],"cte":[],"psi":[],"delta":[]}
    
    for _ in range(35000):
        delta, e, psi = ctrl.control(model.state, waypoints, v)
        x,y,th = model.rk4_step(delta, v)

        log["x"].append(x); 
        log["y"].append(y)
        log["cte"].append(e); 
        log["psi"].append(np.rad2deg(psi)); 
        log["delta"].append(np.rad2deg(delta))


# --- Plotting ---
plt.figure()
plt.plot(log["x"], log["y"],'b',label="Vehicle")
plt.plot(waypoints[:,0], waypoints[:,1],'r--',label="Path")
plt.legend()
plt.axis("equal")
fig_path = "Figures/trajectory.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')

plt.figure(); plt.plot(log["cte"]); plt.title("Cross-Track Error")
plt.xlabel("Tidssteg"); plt.ylabel("Avstand [m]")
fig_path = "Figures/cross_track_error.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')
plt.figure(); plt.plot(log["psi"]); plt.title("Heading Error [deg]")
plt.xlabel("Tidssteg"); plt.ylabel("Vinkel [deg]")
fig_path = "Figures/heading_error.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')
plt.figure(); plt.plot(log["delta"]); plt.title("Steering [deg]")
plt.xlabel("Tidssteg"); plt.ylabel("Vinkel [deg]")
fig_path = "Figures/steering.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')
plt.show()

