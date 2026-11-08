"""Simulator: autobuze cu bară LiDAR care circulă pe linii din Brașov și trimit date serverului.

Traseele urmează STRĂZI REALE: la prima rulare sunt cerute serviciului public de rutare
OSRM (profil auto) și salvate în routes_cache.json; de atunci simulatorul merge și offline.

Ce simulează, la fiecare trecere a unui autobuz:
  - bara cu 5 senzori LiDAR (la 20 cm unul de altul) vede doar ce e sub ea; autobuzul nu
    merge mereu exact pe aceeași linie, deci o groapă e prinsă la unele treceri, la altele nu;
  - pe ploaie, o groapă plină cu apă nu poate fi măsurată de LiDAR; dacă roata intră în ea,
    accelerometrul o simte ca „șoc” (fără dimensiuni);
  - erori de măsurare, eroare GPS, capace de canal și șine de tramvai/rosturi (alarme false).
La final compară gropile confirmate pe hartă cu gropile „reale” generate.

    python simulate.py                    # 6 autobuze, câte 6 treceri pe linie
    python simulate.py --passes 8 --seed 3
    python simulate.py --server http://192.168.1.100:5000
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE_FILE = os.path.join(HERE, "routes_cache.json")
OSRM_URL = ("https://router.project-osrm.org/route/v1/driving/"
            "{lon1},{lat1};{lon2},{lat2}?overview=full&geometries=geojson")

# Capete de linie (simulate) în cartiere diferite (aproximativ; OSRM le lipește de cel mai apropiat drum)
PLACES = {
    "Bartolomeu": (45.6600, 25.5750), "Răcădău": (45.6290, 25.6110),
    "Tractorul": (45.6690, 25.6200), "Schei": (45.6370, 25.5810),
    "Gara": (45.6610, 25.6150), "Noua": (45.6440, 25.6560),
    "Triaj": (45.6760, 25.6010), "Centrul Civic": (45.6530, 25.6070),
    "Astra": (45.6560, 25.6400), "Craiter": (45.6460, 25.6330),
}
ROUTE_PAIRS = [
    ("Bartolomeu", "Răcădău"), ("Tractorul", "Schei"), ("Gara", "Noua"),
    ("Triaj", "Centrul Civic"), ("Astra", "Bartolomeu"), ("Craiter", "Gara"),
]

# Folosit doar dacă nu există internet și nici cache (aproximativ, poate tăia colțuri)
FALLBACK_ROUTE = [(45.6552, 25.6012), (45.6519, 25.5968), (45.6481, 25.5921),
                  (45.6427, 25.5887), (45.6399, 25.5843)]

THRESHOLD_G = 0.45        # același prag de șoc ca în firmware
ON_ROAD_M = 8             # o groapă e "pe traseu" dacă e la sub 8 m de linia drumului
MIN_SPACING_M = 250       # gropile simulate sunt rare: minimum 250 m între ele
METERS_PER_POTHOLE = 900  # aproximativ o groapă la 900 m de drum

SENSOR_OFFSETS_CM = [-40, -20, 0, 20, 40]   # bara de senzori, față de axul autobuzului
WHEEL_OFFSETS_CM = [-105, 105]              # roțile din spate, față de ax
WHEEL_WIDTH_CM = 30
RAIN_PROBABILITY = 0.2
LAT0 = 45.65              # originea proiecției locale


# ------------------------------------------------------------------ geometrie

def to_xy(lat: float, lon: float) -> tuple[float, float]:
    return ((lon - 25.6) * 111_320 * math.cos(math.radians(LAT0)), (lat - LAT0) * 111_320)


def to_latlon(x: float, y: float) -> tuple[float, float]:
    return (LAT0 + y / 111_320, 25.6 + x / (111_320 * math.cos(math.radians(LAT0))))


class Route:
    """O polilinie de drum în metri, cu poziție după distanța parcursă."""

    def __init__(self, name: str, latlon: list[tuple[float, float]]):
        self.name = name
        self.pts = [to_xy(*p) for p in latlon]
        self.cum = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.length = self.cum[-1]

    def point_at(self, d: float) -> tuple[float, float]:
        d = min(max(d, 0), self.length)
        for i in range(1, len(self.pts)):
            if d <= self.cum[i]:
                seg = self.cum[i] - self.cum[i - 1] or 1
                f = (d - self.cum[i - 1]) / seg
                (ax, ay), (bx, by) = self.pts[i - 1], self.pts[i]
                return ax + (bx - ax) * f, ay + (by - ay) * f
        return self.pts[-1]

    def project(self, p: tuple[float, float]) -> tuple[float, float]:
        """(distanța de-a lungul drumului, distanța perpendiculară) până la punctul p."""
        best = (0.0, float("inf"))
        for i in range(1, len(self.pts)):
            (ax, ay), (bx, by) = self.pts[i - 1], self.pts[i]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0 if seg2 == 0 else max(0, min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2))
            qx, qy = ax + t * dx, ay + t * dy
            dist = math.hypot(p[0] - qx, p[1] - qy)
            if dist < best[1]:
                best = (self.cum[i - 1] + t * math.sqrt(seg2), dist)
        return best


# ------------------------------------------------------------------ trasee reale

def fetch_osrm(a: tuple[float, float], b: tuple[float, float]) -> list[tuple[float, float]]:
    url = OSRM_URL.format(lat1=a[0], lon1=a[1], lat2=b[0], lon2=b[1])
    req = urllib.request.Request(url, headers={"User-Agent": "harta-gropilor-simulator/1.0"})
    with urllib.request.urlopen(req, timeout=15) as r:
        data = json.load(r)
    if data.get("code") != "Ok" or not data.get("routes"):
        raise ValueError(f"OSRM: {data.get('code')}")
    return [(lat, lon) for lon, lat in data["routes"][0]["geometry"]["coordinates"]]


def load_routes(fetch=fetch_osrm, cache_file: str = CACHE_FILE, verbose: bool = True) -> list[Route]:
    """Trasee din cache; ce lipsește se descarcă o singură dată și se salvează."""
    cache = {}
    if os.path.exists(cache_file):
        with open(cache_file, encoding="utf-8") as f:
            cache = json.load(f)

    routes, changed, failed = [], False, False
    for a, b in ROUTE_PAIRS:
        key = f"{a} -> {b}"
        if key not in cache:
            try:
                if verbose:
                    print(f"  descarc traseul pe străzi: {key}")
                cache[key] = fetch(PLACES[a], PLACES[b])
                changed = True
                time.sleep(0.5)              # politicos cu serverul public
            except Exception as exc:          # fără internet, server indisponibil etc.
                if verbose:
                    print(f"  [atenție] nu pot descărca {key}: {exc}")
                failed = True
                continue
        routes.append(Route(key, [tuple(p) for p in cache[key]]))

    if changed:
        with open(cache_file, "w", encoding="utf-8") as f:
            json.dump(cache, f)
    if not routes:
        print("  [atenție] fără trasee reale (e nevoie de internet la prima rulare);"
              " folosesc un traseu aproximativ care poate tăia prin parcuri.")
        routes = [Route("aproximativ", FALLBACK_ROUTE)]
    elif failed and verbose:
        print(f"  continui cu {len(routes)} trasee din {len(ROUTE_PAIRS)}")
    return routes


# ------------------------------------------------------------------ generare

def jittered(xy: tuple[float, float], meters: float) -> tuple[float, float]:
    """Poziție cu eroare GPS, întoarsă ca (lat, lon)."""
    lat, lon = to_latlon(xy[0] + random.gauss(0, meters), xy[1] + random.gauss(0, meters))
    return round(lat, 6), round(lon, 6)


def place_potholes(routes: list[Route]) -> list[dict]:
    """Gropi „reale”: poziție, adâncime, lungime, lățime și poziția laterală în bandă."""
    total = sum(r.length for r in routes)
    target = max(8, min(40, int(total / METERS_PER_POTHOLE)))
    potholes, tries = [], 0
    while len(potholes) < target and tries < 5000:
        tries += 1
        r = random.choices(routes, weights=[x.length for x in routes])[0]
        if r.length < 300:
            continue
        xy = r.point_at(random.uniform(100, r.length - 100))
        if any(math.dist(xy, p["xy"]) < MIN_SPACING_M for p in potholes):
            continue
        potholes.append({
            "xy": xy,
            "depth": random.choice([random.uniform(2.2, 2.9), random.uniform(3, 5.9), random.uniform(6, 12)]),
            "length": random.uniform(25, 120),
            "width": random.uniform(25, 100),
            "lateral": random.uniform(-120, 120),      # cm față de mijlocul benzii
        })
    return potholes


def pass_over(p: dict, bus_offset: float, rainy: bool) -> dict | None:
    """Ce înregistrează autobuzul trecând pe lângă/peste groapa p (sau None)."""
    half = p["width"] / 2
    under = [s for s in SENSOR_OFFSETS_CM if abs(bus_offset + s - p["lateral"]) < half]
    wheel = any(abs(bus_offset + w - p["lateral"]) < half + WHEEL_WIDTH_CM / 2 for w in WHEEL_OFFSETS_CM)
    flooded = rainy and random.random() < 0.8

    if under and not flooded and random.random() < 0.95:
        # senzorii nu prind mereu punctul cel mai adânc; plus zgomot de măsurare
        depth = max(2.0, p["depth"] * random.uniform(0.85, 1.0) + random.gauss(0, 0.6))
        return {"k": "lidar", "d": round(depth, 1), "l": round(p["length"] * random.uniform(0.85, 1.1)),
                "w": len(under) * 20}
    if wheel and p["depth"] >= 2:
        peak = (0.25 + p["depth"] * 0.12) * random.uniform(0.8, 1.2)
        if peak >= THRESHOLD_G:
            return {"k": "soc", "g": round(peak, 3)}
    return None


def generate(routes: list[Route], passes: int = 6) -> tuple[list[dict], list[dict]]:
    """(evenimente pe autobuz, gropile reale). Fiecare linie are autobuzul ei: bus-001, bus-002…"""
    potholes = place_potholes(routes)
    now = int(time.time())
    events = []

    for li, route in enumerate(routes):
        bus = f"bus-{li + 1:03d}"
        for _ in range(passes):
            reverse = random.random() < 0.5
            start = now - random.randint(0, 13) * 86400 - random.randint(0, 50000)
            speed = random.uniform(20, 45)
            mps = speed / 3.6
            rainy = random.random() < RAIN_PROBABILITY
            bus_offset = random.gauss(0, 30)          # autobuzul nu merge exact pe același loc

            def at(d):
                return start + int((route.length - d if reverse else d) / mps)

            def add(d, rec):
                lat, lon = jittered(route.point_at(d), 2.5)
                events.append({"device": bus, "t": at(d), "lat": lat, "lon": lon, "v": round(speed, 1), **rec})

            for p in potholes:
                d, off = route.project(p["xy"])
                if off <= ON_ROAD_M:
                    rec = pass_over(p, bus_offset + random.gauss(0, 10), rainy)
                    if rec:
                        add(d, rec)

            # alarme false: capace de canal / rosturi (LiDAR puțin adânc), șine (șoc)
            for _ in range(int(route.length / 4000 * random.uniform(0.5, 1.5))):
                add(random.uniform(0, route.length),
                    {"k": "lidar", "d": round(random.uniform(2.0, 2.6), 1),
                     "l": round(random.uniform(15, 60)), "w": random.choice([20, 40])})
            for _ in range(int(route.length / 8000 * random.uniform(0.5, 1.5))):
                add(random.uniform(0, route.length), {"k": "soc", "g": round(random.uniform(0.46, 0.8), 3)})
    return events, potholes


def evaluate_against_truth(events: list[dict], potholes: list[dict], routes: list[Route]) -> dict:
    """Câte gropi reale apar confirmate pe hartă și câte confirmări sunt false."""
    from clustering import cluster_events
    evs = [{"device_id": e["device"], "ts": e["t"], "lat": e["lat"], "lon": e["lon"], "speed": e["v"],
            "kind": e["k"], "depth_cm": e.get("d"), "length_cm": e.get("l"), "width_cm": e.get("w"),
            "peak_g": e.get("g")} for e in events]
    confirmed = [c for c in cluster_events(evs) if c["confirmed"]]
    real = [p for p in potholes if any(r.project(p["xy"])[1] <= ON_ROAD_M for r in routes)]
    found, depth_err = 0, []
    for p in real:

        near = [c for c in confirmed if math.dist(to_xy(c["lat"], c["lon"]), p["xy"]) <= 15]
        if near:
            found += 1
            if near[0]["depth_cm"] is not None:
                depth_err.append(abs(near[0]["depth_cm"] - p["depth"]))
    false = sum(1 for c in confirmed
                if not any(math.dist(to_xy(c["lat"], c["lon"]), p["xy"]) <= 15 for p in real))
    return {"real": len(real), "found": found, "false_confirmed": false,
            "depth_error_cm": round(sum(depth_err) / len(depth_err), 1) if depth_err else None}


def post(server: str, events: list[dict]) -> dict:
    """Trimite evenimentele grupate pe autobuz, ca firmware-ul (loturi de 50)."""
    totals = {"inserted": 0, "duplicates": 0, "rejected": 0}
    by_bus: dict[str, list] = {}
    for e in events:
        by_bus.setdefault(e["device"], []).append({k: v for k, v in e.items() if k != "device"})
    for bus, evs in by_bus.items():
        for i in range(0, len(evs), 50):
            res = _post_json(server, "/api/events", {"device_id": bus, "events": evs[i:i + 50]})
            for k in totals:
                totals[k] += res[k]
    return totals


def post_heartbeats(server: str, buses: list[str]):
    """Semne de viață; un autobuz are un senzor murdar, ca să se vadă în panou."""
    for i, bus in enumerate(buses):
        sensors = [{"valid": round(random.uniform(0.93, 0.99), 2)} for _ in SENSOR_OFFSETS_CM]
        if i == 2:
            sensors[1]["valid"] = 0.31
        _post_json(server, "/api/heartbeat", {"device_id": bus, "sensors": sensors, "gps": True,
                                               "pending": 0, "fw": "2.0-sim"})


def _post_json(server: str, path: str, payload: dict) -> dict:
    req = urllib.request.Request(server.rstrip("/") + path, data=json.dumps(payload).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


def run(server: str, passes: int = 6) -> dict:
    routes = load_routes()
    events, potholes = generate(routes, passes)
    buses = sorted({e["device"] for e in events} | {f"bus-{i + 1:03d}" for i in range(len(routes))})
    km = sum(r.length for r in routes) / 1000
    print(f"  {len(routes)} linii ({km:.1f} km), {len(buses)} autobuze, {len(events)} detecții")
    res = post(server, events)
    post_heartbeats(server, buses)
    ev = evaluate_against_truth(events, potholes, routes)
    print(f"  gropi reale pe linii: {ev['real']}, găsite și confirmate: {ev['found']}, "
          f"confirmări false: {ev['false_confirmed']}, eroare medie adâncime: {ev['depth_error_cm']} cm")
    return {**res, **ev}


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Simulator pentru Harta gropilor (autobuze cu LiDAR)")
    ap.add_argument("--server", default="http://localhost:5000")
    ap.add_argument("--passes", type=int, default=6, help="treceri pe fiecare linie")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    random.seed(args.seed)
    print(run(args.server, args.passes))
