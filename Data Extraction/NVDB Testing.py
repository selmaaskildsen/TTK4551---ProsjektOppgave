# -------------------------------------------------------
# NVDB geometry over real map (OpenStreetMap)
# -------------------------------------------------------

import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
import contextily as ctx

# -------------------------------------------------------
# 1. Read NVDB coordinates from your CSV file
# -------------------------------------------------------
csv_path = "Figures/kalkbrennerveien_points.csv"
df = pd.read_csv(csv_path)

# The CSV file contains normalized coordinates (x, y) relative to m0 = (0, 0)
# To place them correctly on a map, we need to add back the original UTM offset.
# Replace these with the values printed from your earlier script:
x0 = 596437.4   # Example: original NVDB UTM-Easting of m0
y0 = 6642018.3  # Example: original NVDB UTM-Northing of m0

# Shift coordinates back to absolute UTM
df["x_abs"] = df["x"] + x0
df["y_abs"] = df["y"] + y0

# -------------------------------------------------------
# 2. Convert to GeoDataFrame (EPSG:25833 = UTM zone 33)
# -------------------------------------------------------
gdf = gpd.GeoDataFrame(
    df, geometry=gpd.points_from_xy(df["x_abs"], df["y_abs"]), crs="EPSG:25833"
)

# Optionally, connect points into a LineString for a cleaner plot
road_line = gdf.unary_union.convex_hull
gdf_line = gpd.GeoSeries([road_line], crs="EPSG:25833")

# -------------------------------------------------------
# 3. Plot over OpenStreetMap basemap
# -------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 8))

# Plot the NVDB road geometry
gdf.plot(ax=ax, color="red", markersize=6, label="NVDB points")
gdf_line.plot(ax=ax, color="darkred", linewidth=1.5, label="Interpolated road")

# Add OpenStreetMap background
ctx.add_basemap(ax, crs=gdf.crs.to_string(), source=ctx.providers.OpenStreetMap.Mapnik)

# Styling
ax.set_title("Kalkbrennerveien – NVDB geometry over OpenStreetMap", fontsize=12)
ax.set_xlabel("Easting [m]")
ax.set_ylabel("Northing [m]")
ax.legend()
plt.tight_layout()

# -------------------------------------------------------
# 4. Save figure
# -------------------------------------------------------
fig_path = "Figures/kalkbrennerveien_map.png"
#plt.savefig(fig_path, dpi=300, bbox_inches="tight")
plt.show()

print(f"Map figure saved to: {fig_path}")
