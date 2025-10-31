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

print(f"Origo satt til (x0, y0) = ({x0:.1f}, {y0:.1f})")

# -------------------------------------------------------
# 4. Beregn avstand langs vegen og fjern duplikater
# -------------------------------------------------------
dist = np.zeros(len(x))
for i in range(1, len(x)):
    dist[i] = dist[i-1] + np.hypot(x[i] - x[i-1], y[i] - y[i-1])

# --- Skriv ut euklidske avstander mellom punkter ---
print("\nEuklidsk avstand mellom etterfølgende punkter:")

tot = 0.0

for i in range(1, len(x)):
    d = np.hypot(x[i] - x[i-1], y[i] - y[i-1])
    tot += d
    print(f"Punkt {i-1:3d} → {i:3d}: {d:.3f} m")

print(f"Total euklidsk avstand: {tot:.3f} m\n")

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

# -------------------------------------------------------
# 6. Lagre koordinater til CSV
# -------------------------------------------------------
os.makedirs("Figures", exist_ok=True)
df_raw = pd.DataFrame({"x": x, "y": y})
df_spline = pd.DataFrame({"x": x_smooth, "y": y_smooth})

df_raw.to_csv("Figures/kalkbrennerveien_points_raw.csv", index=False)
df_spline.to_csv("Figures/kalkbrennerveien_points_spline.csv", index=False)

print("Koordinater lagret til:")
print("  - Figures/kalkbrennerveien_points_raw.csv")
print("  - Figures/kalkbrennerveien_points_spline.csv")

# -------------------------------------------------------
# 7. Plot original og spline
# -------------------------------------------------------
plt.figure(figsize=(8,6))
plt.plot(x, y, 'o:', label='Original NVDB-punkter', alpha=0.5)
plt.plot(x_smooth, y_smooth, '-', linewidth=2, label='Cubic Spline')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Kalkbrennerveien – NVDB-data med Cubic Spline-interpolasjon")
plt.legend()
plt.axis('equal')
plt.grid(True)

fig_path = "Figures/kalkbrennerveien_spline_plot.png"
plt.savefig(fig_path, dpi=300, bbox_inches='tight')
plt.show()

print(f"Plot lagret til: {fig_path}")
