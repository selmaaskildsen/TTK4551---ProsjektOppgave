import requests
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pandas as pd
import json


url = "https://nvdbapiles.atlas.vegvesen.no/vegnett/api/v4/veglenkesekvenser?vegsystemreferanse=KV1699&kommune=3201"

# --- 2. Hent veglenkesekvenser ---
resp = requests.get(url) 
resp.raise_for_status()
data = resp.json()

#print(data["objekter"][0])
#print(json.dumps(data["objekter"][0], indent=2))

vei = []

for item in data["objekter"]:
    veglenker = item["veglenker"]
    for veglenke in veglenker:
        geom = veglenke["geometri"]["wkt"]
        if geom and geom.startswith("LINESTRING"):
            pts = geom.replace("LINESTRING Z (", "").replace("LINESTRING (", "").replace(")", "").split(",")
            for p in pts:
                parts = p.split()
                x, y, z = parts
                vei.append((float(x), float(y), float(z)))
    #print(type(item["veglenker"]))
    #print(json.dumps(item["veglenker"][0]["geometri"]["wkt"], indent=2))
    #print(item["veglenker"][0]["geometri"]["wkt"])

for punkt in vei:
    x, y, z = punkt
    print(f"x: {x}, y: {y}, z: {z}")

x = np.array([punkt[0] for punkt in vei])
y = np.array([punkt[1] for punkt in vei])

plt.plot(x, y, '.')
plt.show()


