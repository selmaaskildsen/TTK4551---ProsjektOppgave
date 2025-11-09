import os
import requests
import numpy as np
import matplotlib.pyplot as plt
from shapely import wkt
import pandas as pd
from scipy.interpolate import CubicSpline
import json

# -------------------------------------------------------
# 1. API-kall til NVDB
# -------------------------------------------------------
url = "https://nvdbapiles.atlas.vegvesen.no/vegnett/api/v4/veglenkesekvenser"
params = {
    "vegsystemreferanse": "KV1531",  # eksempel: Granåsen
    "kommune": "3201"                # Bærum
}

resp = requests.get(url, params=params)
resp.raise_for_status()
data = resp.json()

if "objekter" not in data or len(data["objekter"]) == 0:
    raise ValueError("Ingen objekter funnet. Sjekk vegsystemreferanse eller kommune.")

# -------------------------------------------------------
# 2. Hent punkter med riktig kjøreretning, fjern overlapp
# -------------------------------------------------------
alle_segmenter = []

for item in data["objekter"]:
    for veglenke in item["veglenker"]:
        geom = veglenke["geometri"]["wkt"]
        if not geom or not geom.startswith("LINESTRING"):
            continue

        startpos = veglenke.get("startposisjon", 0)
        sluttpos = veglenke.get("sluttposisjon", 0)
        retning = veglenke.get("retning", "").upper()
        reversert = veglenke.get("reversert", False)

        # Hopp over hvis dette segmentet overlapper et tidligere segment
        overlap = False
        for s_start, s_slutt, _ in alle_segmenter:
            if not (sluttpos <= s_start or startpos >= s_slutt):
                overlap = True
                break
        if overlap:
            continue  # hopper over overlappende segment

        # Hent ut geometrien
        line = wkt.loads(geom)
        x, y = line.xy
        segment = list(zip(x, y))

        # Snur rekkefølge om retningen er MOT eller reversert=True
        if retning == "MOT" or reversert:
            segment = segment[::-1]

        alle_segmenter.append((startpos, sluttpos, segment))

if not alle_segmenter:
    raise ValueError("Ingen gyldige (ikke-overlappende) segmenter funnet i NVDB-data.")


# -------------------------------------------------------
# 3. Start ved 0 m (laveste startposisjon)
# -------------------------------------------------------
alle_segmenter.sort(key=lambda s: s[0])
start_segment = alle_segmenter[0][2]
startpunkt = start_segment[0]

# Slå sammen alle punkter etter riktig rekkefølge
alle_punkter = [p for _, _, seg in alle_segmenter for p in seg]
punkter = np.array(alle_punkter)

# -------------------------------------------------------
# 4. Sorter punkter etter nærmeste nabo, fra startpunkt
# -------------------------------------------------------
def sort_points_from_start(points, start_point):
    """Sorter punkter etter nærmeste nabo, men start ved kjent 0 m-punkt."""
    points = np.array(points)
    sorted_points = [np.array(start_point)]
    remaining = points.copy()

    # Fjern startpunkt fra kandidatlisten
    dists = np.linalg.norm(remaining - start_point, axis=1)
    idx_start = np.argmin(dists)
    remaining = np.delete(remaining, idx_start, axis=0)

    while len(remaining) > 0:
        last_point = sorted_points[-1]
        dists = np.linalg.norm(remaining - last_point, axis=1)
        idx_min = np.argmin(dists)
        sorted_points.append(remaining[idx_min])
        remaining = np.delete(remaining, idx_min, axis=0)

    return np.array(sorted_points)

sorted_pts = sort_points_from_start(punkter, startpunkt)
x, y = sorted_pts[:, 0], sorted_pts[:, 1]

# -------------------------------------------------------
# 5. Normaliser koordinater (sett 0 m som origo)
# -------------------------------------------------------
x -= startpunkt[0]
y -= startpunkt[1]

dist = np.zeros(len(x))
for i in range(1, len(x)):
    dist[i] = dist[i-1] + np.hypot(x[i] - x[i-1], y[i] - y[i-1])

# Fjern punkter der avstanden ikke øker (duplikate punkter)
mask = np.diff(dist, prepend=-1) > 0
x = x[mask]
y = y[mask]
dist = dist[mask]

# -------------------------------------------------------
# 6. Cubic spline-interpolasjon
# -------------------------------------------------------
spline_x = CubicSpline(dist, x)
spline_y = CubicSpline(dist, y)

s_vals = np.linspace(0, dist[-1], 500)
x_smooth = spline_x(s_vals)
y_smooth = spline_y(s_vals)

# Lag waypoints-array
waypoints = np.array(list(zip(x_smooth, y_smooth)))

# -------------------------------------------------------
# 7. Kjøretøymodell og Stanley-kontroller
# -------------------------------------------------------

class BicycleModel:
    def __init__(self, L=2.7, dt=0.1, x0=0, y0=0, theta0=np.pi/2):
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


class StanleyController:
    def __init__(self, k=1.2):
        self.k = k

    def control(self, state, path, v):
        x, y, th = state
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


# -------------------------------------------------------
# 8. Simulering (kjør til slutten av veien)
# -------------------------------------------------------
if __name__ == "__main__":

    model = BicycleModel()
    ctrl = StanleyController(k=1.2) 

    v = 5.0  # konstant hastighet [m/s]
    log = {"x":[],"y":[],"cte":[],"psi":[],"delta":[]}

    goal = waypoints[-1]
    tol = 1.0         # stopp når bilen er nærmere enn 1 meter fra mål
    max_steps = 100000
    step = 0

    while True:
        delta, e, psi = ctrl.control(model.state, waypoints, v)
        x, y, th = model.rk4_step(delta, v)

        log["x"].append(x)
        log["y"].append(y)
        log["cte"].append(e)
        log["psi"].append(np.rad2deg(psi))
        log["delta"].append(np.rad2deg(delta))

        dist_to_goal = np.hypot(goal[0] - x, goal[1] - y)
        if dist_to_goal < tol:
            print(f"Kjøretøyet har nådd enden av veien (innenfor {tol:.1f} m).")
            break

        step += 1
        if step > max_steps:
            print("Maksimalt antall steg nådd – avslutter simulering.")
            break

# -------------------------------------------------------
# 9. Plotting
# -------------------------------------------------------

plt.figure(figsize=(7,6))
plt.plot(sorted_pts[:,0], sorted_pts[:,1], 'k.', markersize=5, label="Veipunkter (NVDB)")       # svarte prikker
plt.plot(x_smooth, y_smooth, 'r-', linewidth=1.2, label="Cubic Spline-bane")     # rød jevn bane
plt.plot(log["x"], log["y"], 'b--', linewidth=1.0, label="Kjøretøyets bane")    # blå stiplet linje
plt.plot(goal[0], goal[1], 'ro', markersize=5, label="Sluttpunkt")               # grønt sluttpunkt
plt.legend()
plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Granåsen – Veipunkter, spline-bane og kjøretøyets bane")
plt.grid(True)
fig_path = "Figures/trajectory.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')

# --- Cross-track error ---
plt.figure()
plt.plot(log["cte"], color='purple')
plt.title("Cross-Track Error")
plt.xlabel("Tidssteg")
plt.ylabel("Avstand [m]")
plt.grid(True)
fig_path = "Figures/cross_track_error.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')

# --- Heading error ---
plt.figure()
plt.plot(log["psi"], color='darkorange')
plt.title("Heading Error [deg]")
plt.xlabel("Tidssteg")
plt.ylabel("Vinkel [deg]")
plt.grid(True)
fig_path = "Figures/heading_error.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')

# --- Steering angle ---
plt.figure()
plt.plot(log["delta"], color='teal')
plt.title("Steering Angle [deg]")
plt.xlabel("Tidssteg")
plt.ylabel("Vinkel [deg]")
plt.grid(True)
fig_path = "Figures/steering.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')

plt.show()
