import requests
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pandas as pd
import json

# -------------------------------
# 1. KONFIGURASJON
# -------------------------------
VEGSYSTEM = "KV1699"   # Kommunal vegnummer (f.eks. Kalkbrennerveien)
KOMMUNE = "3201"       # Oslo kommune
# -------------------------------

# --- NVDB API v4 endepunkt ---
#url = "https://nvdbapi.vegdata.no/vegnett/api/v4/veglenkesekvenser"
url = "https://nvdbapiles.atlas.vegvesen.no/vegnett/api/v4/veglenkesekvenser?vegsystemreferanse=KV1699&kommune=3201"
params = {
    "filter": [f"vegsystemreferanse:{VEGSYSTEM}", f"kommune:{KOMMUNE}"],
    "inkluder": "geometri"
}
headers = {
    "Accept": "application/vnd.vegvesen.nvdb-v4+json",
    "X-Client": "selmaaskildsen@stud.ntnu.no"
}

# --- 2. Hent veglenkesekvenser ---
resp = requests.get(url)#, params=params)#, headers=headers)
resp.raise_for_status()
data = resp.json()
#print(data["objekter"][0])
#print(json.dumps(data["objekter"][0], indent=2))
# --- 3. Hent ut alle koordinater fra alle lenker ---
coords = []
for entry in data["objekter"]:
    print(entry.keys())
    print(type(entry["veglenker"]))
    exit(0)
    """for seq in data["objekter"].get("veglenkesekvenser", []):
        for link in seq.get("veglenker", []):
            geom = link.get("geometri", {}).get("wkt")
            if geom and geom.startswith("LINESTRING"):
                pts = geom.replace("LINESTRING Z (", "").replace("LINESTRING (", "").replace(")", "").split(", ")
                for p in pts:
                    parts = p.split()
                    x, y = float(parts[0]), float(parts[1])
                    coords.append((x, y))"""

if not coords:
    raise RuntimeError("Fant ingen koordinater. Sjekk vegsystemreferanse eller kommune.")

coords = np.array(coords)

# --- 4. Sortér punkter etter x (for spline) ---
order = np.argsort(coords[:, 0])
x, y = coords[order, 0], coords[order, 1]

# --- 5. Generér jevn Cubic Spline ---
cs = CubicSpline(x, y)
x_smooth = np.linspace(x.min(), x.max(), 500)
y_smooth = cs(x_smooth)

# --- 6. Plot originaldata + spline ---
plt.figure(figsize=(7,5))
plt.plot(x, y, 'o', label='NVDB punkter')
plt.plot(x_smooth, y_smooth, '-', label='Cubic Spline (bane)')
plt.title(f"Bane generert fra NVDB-data ({VEGSYSTEM})")
plt.xlabel("x [m] (UTM33)")
plt.ylabel("y [m] (UTM33)")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()

# --- 7. (Valgfritt) lagre banen ---
path = np.column_stack((x_smooth, y_smooth))
pd.DataFrame(path, columns=["x", "y"]).to_csv("bane_nvdb.csv", index=False)
print(f"✅ Lagret {len(path)} punkter til 'bane_nvdb.csv'")
