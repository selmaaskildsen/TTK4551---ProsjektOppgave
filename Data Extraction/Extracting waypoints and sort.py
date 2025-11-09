import os
import requests
import numpy as np
import matplotlib.pyplot as plt
from shapely import wkt
import pandas as pd

# -------------------------------------------------------
# 1. API-kall til NVDB
# -------------------------------------------------------
url = "https://nvdbapiles.atlas.vegvesen.no/vegnett/api/v4/veglenkesekvenser"
params = {
    "vegsystemreferanse": "KV1548",  # eksempel: Granåsen
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

print(f"Antall segmenter etter filtrering: {len(alle_segmenter)}")

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

print(f"Starter sortering fra NVDB-startpunkt (0 m): {startpunkt}")
sorted_pts = sort_points_from_start(punkter, startpunkt)
x, y = sorted_pts[:, 0], sorted_pts[:, 1]

# -------------------------------------------------------
# 5. Normaliser koordinater (sett 0 m som origo)
# -------------------------------------------------------
x -= startpunkt[0]
y -= startpunkt[1]
print(f"Origo satt til (x0, y0) = ({startpunkt[0]:.1f}, {startpunkt[1]:.1f})")

# -------------------------------------------------------
# 6. Lagre og plotte
# -------------------------------------------------------
os.makedirs("Figures", exist_ok=True)
df = pd.DataFrame({"x": x, "y": y})
csv_path = "Figures/granasen_uten_overlapp.csv"
df.to_csv(csv_path, index=False)
print(f"Koordinater lagret til: {csv_path}")

plt.figure(figsize=(7,6))
plt.plot(x, y, ':', linewidth=1.2)
plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Granåsen – uten overlappende segmenter")
plt.grid(True)
plt.show()
