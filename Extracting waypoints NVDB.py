import requests
import numpy as np

def hent_veglenkesekvens(veglenkesekvensid: int):
    """
    Henter geometri (x, y) for en veglenkesekvens fra NVDB API v4.
    
    Parametre:
        veglenkesekvensid : int
            Unik ID for veglenkesekvensen (fra NVDB)
    
    Returnerer:
        numpy.ndarray med form (N, 2) der hver rad er [x, y]
    """
    
    url = f"https://nvdbapi.vegdata.no/v4/vegnett/veglenkesekvenser/{veglenkesekvensid}"
    headers = {
        "Accept": "application/vnd.vegvesen.nvdb-v4+json",
        "X-Client": "selma-prosjektoppgave@example.com"
    }

    resp = requests.get(url, headers=headers)
    if resp.status_code != 200:
        raise Exception(f"Feil ved henting ({resp.status_code}): {resp.text}")

    data = resp.json()
    coords = []

    # Hent alle geometrier for hver veglenke i sekvensen
    for link in data.get("veglenker", []):
        geom = link.get("geometri", {}).get("wkt")
        if geom and geom.startswith("LINESTRING"):
            # Fjern tekst og splitt koordinatene
            pts = geom.replace("LINESTRING Z (", "").replace("LINESTRING (", "").replace(")", "").split(", ")
            for p in pts:
                parts = p.split()
                x, y = float(parts[0]), float(parts[1])
                coords.append((x, y))

    if not coords:
        print(f"Ingen koordinater funnet for {veglenkesekvensid}.")
        return np.empty((0, 2))

    return np.array(coords)
