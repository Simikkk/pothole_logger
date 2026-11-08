"""Logica de analiză: gruparea evenimentelor în gropi, severitate după adâncime, stare, străzi.

Funcții pure, fără bază de date, ca să poată fi testate ușor.

Un eveniment vine de la bara de senzori de pe autobuz și are:
  device_id, ts, lat, lon, speed
  kind      "lidar" = groapă măsurată de LiDAR (adâncime, lungime, lățime în cm)
            "soc"   = doar accelerometrul a simțit-o (de obicei groapă plină cu apă
                      sau în afara zonei văzute de senzori) – fără dimensiuni
  depth_cm, length_cm, width_cm   (doar pentru lidar)
  peak_g                          (doar pentru soc)
"""
from __future__ import annotations

import math
from statistics import median

EARTH_RADIUS_M = 6_371_000

CLUSTER_RADIUS_M = 12      # detecții mai apropiate de atât = aceeași groapă
PASS_GAP_S = 120           # detecții la peste 2 minute distanță = treceri diferite

# Praguri de adâncime (cm) – de stabilit împreună cu primăria
DEPTH_LIMITS_CM = (3.0, 6.0)    # sub 3 = mică, 3–6 = medie, peste 6 = mare

ASPHALT_KG_PER_LITER = 2.4      # mixtură asfaltică compactată ≈ 2,4 t/m³

SEVERITIES = ("mica", "medie", "mare", "necunoscuta")


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distanța în metri între două puncte GPS."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def severity_from_depth(depth_cm: float | None) -> str:
    if depth_cm is None:
        return "necunoscuta"
    low, high = DEPTH_LIMITS_CM
    if depth_cm < low:
        return "mica"
    if depth_cm < high:
        return "medie"
    return "mare"


def count_passes(timestamps: list[int], gap_s: int = PASS_GAP_S) -> int:
    """Câte treceri distincte prin același loc (grupuri separate de pauze mari)."""
    if not timestamps:
        return 0
    ts = sorted(timestamps)
    return 1 + sum(1 for prev, cur in zip(ts, ts[1:]) if cur - prev > gap_s)


def cluster_events(events: list[dict], radius_m: float = CLUSTER_RADIUS_M) -> list[dict]:
    """Grupează detecțiile apropiate în gropi (greedy, O(n·k))."""
    clusters: list[dict] = []
    for ev in sorted(events, key=lambda e: e["ts"]):
        best, best_d = None, radius_m
        for c in clusters:
            d = haversine_m(ev["lat"], ev["lon"], c["lat"], c["lon"])
            if d <= best_d:
                best, best_d = c, d
        if best is None:
            clusters.append({"lat": ev["lat"], "lon": ev["lon"], "events": [ev]})
        else:
            best["events"].append(ev)
            n = len(best["events"])
            best["lat"] += (ev["lat"] - best["lat"]) / n
            best["lon"] += (ev["lon"] - best["lon"]) / n
    return [_summarize(i + 1, c) for i, c in enumerate(clusters)]


def _summarize(cid: int, c: dict) -> dict:
    evs = c["events"]
    lidar = [e for e in evs if e["kind"] == "lidar"]
    shocks = [e for e in evs if e["kind"] == "soc"]

    if lidar:
        depth = median(e["depth_cm"] for e in lidar)
        length = median(e["length_cm"] for e in lidar)
        # lățimea văzută la o trecere depinde de cât din groapă a prins bara; luăm maximul
        width = max(e["width_cm"] for e in lidar)
        volume_l = depth * length * width / 1000      # cm³ -> litri
        dims = {"depth_cm": round(depth, 1), "max_depth_cm": round(max(e["depth_cm"] for e in lidar), 1),
                "length_cm": round(length), "width_cm": round(width),
                "volume_l": round(volume_l, 1), "asphalt_kg": round(volume_l * ASPHALT_KG_PER_LITER)}
    else:
        depth = None
        dims = {"depth_cm": None, "max_depth_cm": None, "length_cm": None, "width_cm": None,
                "volume_l": None, "asphalt_kg": None}

    passes = count_passes([e["ts"] for e in evs])
    return {
        "id": cid,
        "lat": round(c["lat"], 6),
        "lon": round(c["lon"], 6),
        "severity": severity_from_depth(depth),
        **dims,
        "lidar_detections": len(lidar),
        "shock_detections": len(shocks),
        "passes": passes,
        "buses": sorted({e["device_id"] for e in evs}),
        "first_seen": min(e["ts"] for e in evs),
        "last_seen": max(e["ts"] for e in evs),
        # confirmată = văzută la cel puțin 2 treceri separate
        "confirmed": passes >= 2,
    }


# ------------------------------------------------------------------ stare și străzi

STATUSES = ("semnalata", "raportata", "reparata")
STREET_MATCH_M = 20


def _nearest(lat: float, lon: float, records: list[dict], radius_m: float) -> dict | None:
    best, best_d = None, radius_m
    for r in records:
        d = haversine_m(lat, lon, r["lat"], r["lon"])
        if d <= best_d:
            best, best_d = r, d
    return best


def apply_status(clusters: list[dict], statuses: list[dict],
                 radius_m: float = CLUSTER_RADIUS_M) -> list[dict]:
    """Atașează fiecărei gropi starea salvată (după locație).

    O groapă marcată „reparată” care e detectată din nou DUPĂ data reparației
    revine automat la „semnalată” și primește reappeared=True.
    """
    for c in clusters:
        st = _nearest(c["lat"], c["lon"], statuses, radius_m)
        c["status"], c["status_changed_at"], c["reappeared"] = "semnalata", None, False
        if st:
            c["status"], c["status_changed_at"] = st["status"], st["changed_at"]
            if st["status"] == "reparata" and c["last_seen"] > st["changed_at"]:
                c["status"], c["reappeared"] = "semnalata", True
    return clusters


def apply_streets(clusters: list[dict], streets: list[dict],
                  radius_m: float = STREET_MATCH_M) -> list[dict]:
    """street = numele străzii, "" dacă nu are nume, None dacă încă nu e căutat."""
    for c in clusters:
        st = _nearest(c["lat"], c["lon"], streets, radius_m)
        c["street"] = st["street"] if st else None
    return clusters


def street_summary(clusters: list[dict]) -> dict:
    """Gropile confirmate și nereparate, grupate pe străzi (cele mai grave primele)."""
    by_street: dict[str, dict] = {}
    pending = 0
    for c in clusters:
        if not c["confirmed"] or c["status"] == "reparata":
            continue
        if c["street"] is None:
            pending += 1
            continue
        name = c["street"] or "Stradă fără nume"
        s = by_street.setdefault(name, {"street": name, "total": 0, "asphalt_kg": 0, "ids": [],
                                        **{k: 0 for k in SEVERITIES}})
        s["total"] += 1
        s[c["severity"]] += 1
        s["asphalt_kg"] += c["asphalt_kg"] or 0
        s["ids"].append(c["id"])
    streets = sorted(by_street.values(), key=lambda s: (-s["mare"], -s["total"], s["street"]))
    return {"streets": streets, "pending": pending}


# ------------------------------------------------------------------ starea autobuzelor

OFFLINE_AFTER_S = 15 * 60
DIRTY_BELOW = 0.5          # sub 50% citiri valide = senzor probabil murdar


def device_health(device: dict, now: int) -> dict:
    """Starea unui autobuz din ultimul heartbeat: ok / senzor murdar / fără GPS / offline."""
    sensors = device.get("sensors") or []
    dirty = [i + 1 for i, s in enumerate(sensors) if s.get("valid", 1) < DIRTY_BELOW]
    if now - device["last_seen"] > OFFLINE_AFTER_S:
        state = "offline"
    elif dirty:
        state = "senzor_murdar"
    elif not device.get("gps", True):
        state = "fara_gps"
    else:
        state = "ok"
    return {"device_id": device["device_id"], "state": state, "last_seen": device["last_seen"],
            "dirty_sensors": dirty, "pending": device.get("pending", 0)}
