import os
import requests
import numpy as np
import matplotlib.pyplot as plt
from shapely import wkt
import pandas as pd
import json

# -------------------------------------------------------
# 1. API-kall til NVDB
# -------------------------------------------------------
url = "https://nvdbapiles.atlas.vegvesen.no/vegnett/api/v4/veglenkesekvenser"
params = {
    "vegsystemreferanse": "KV1548",  # Kalkbrennerveien
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

        # hent startposisjon, default=0 om ikke finnes
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
# 4. Lagre koordinater til CSV
# -------------------------------------------------------
os.makedirs("Figures", exist_ok=True)
df = pd.DataFrame({"x": x, "y": y})
csv_path = "Figures/kalkbrennerveien_points.csv"
df.to_csv(csv_path, index=False)
print(f"Koordinater lagret til: {csv_path}")

# -------------------------------------------------------
# 5. Plot bane
# -------------------------------------------------------
plt.figure(figsize=(7,6))
plt.plot(x, y, '.', linewidth=1)
plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Bispeveien – NVDB-data (m0 som origo)")
plt.grid(True)

#fig_path = "Figures/kalkbrennerveien_plot.png"
#plt.savefig(fig_path, dpi=300, bbox_inches='tight')
plt.show()

#print(f"Plot lagret til: {fig_path}")
