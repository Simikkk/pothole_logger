"""Server pentru Harta gropilor (Flask + SQLite) – senzori LiDAR pe autobuzele orașului.

Pornire:   python app.py            -> http://localhost:5000
Variabile de mediu opționale:
  POTHOLE_DB       calea bazei de date (implicit: potholes.db lângă acest fișier)
  POTHOLE_API_KEY  dacă e setată, autobuzele trebuie să trimită header-ul X-API-Key
  POTHOLE_STREETS  pune 0 ca să NU caute numele străzilor pe internet

Confidențialitate: datele vin doar de la autobuzele flotei, identificate prin codul
autobuzului (ex. bus-001). Serverul nu salvează adrese IP și nu primește trasee,
doar locurile unde bara de senzori a măsurat o groapă.
"""
from __future__ import annotations

import csv
import io
import json
import os
import sqlite3
import threading
import time
import urllib.parse
import urllib.request
from datetime import datetime

from flask import Flask, Response, g, jsonify, request, send_from_directory

from clustering import (SEVERITIES, STATUSES, apply_status, apply_streets, cluster_events,
                        device_health, haversine_m, street_summary)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAX_EVENTS_PER_REQUEST = 1000

EVENTS_TABLE = """
CREATE TABLE IF NOT EXISTS events (
    id          INTEGER PRIMARY KEY,
    device_id   TEXT    NOT NULL,
    ts          INTEGER NOT NULL,
    lat         REAL    NOT NULL,
    lon         REAL    NOT NULL,
    speed       REAL    NOT NULL,
    kind        TEXT    NOT NULL,          -- 'lidar' sau 'soc'
    depth_cm    REAL,
    length_cm   REAL,
    width_cm    REAL,
    peak_g      REAL,
    received_at INTEGER NOT NULL,
    UNIQUE (device_id, ts, lat, lon, kind)
);
"""

SCHEMA = EVENTS_TABLE + """
CREATE INDEX IF NOT EXISTS idx_events_ts ON events (ts);

-- starea unei gropi, legată de locație (gropile se recalculează din evenimente)
CREATE TABLE IF NOT EXISTS pothole_status (
    id         INTEGER PRIMARY KEY,
    lat        REAL    NOT NULL,
    lon        REAL    NOT NULL,
    status     TEXT    NOT NULL,
    changed_at INTEGER NOT NULL
);

-- cache pentru numele străzilor ("" = drum fără nume)
CREATE TABLE IF NOT EXISTS streets (
    id     INTEGER PRIMARY KEY,
    lat    REAL NOT NULL,
    lon    REAL NOT NULL,
    street TEXT NOT NULL
);

-- ultimul semn de viață de la fiecare autobuz
CREATE TABLE IF NOT EXISTS devices (
    device_id TEXT PRIMARY KEY,
    last_seen INTEGER NOT NULL,
    payload   TEXT    NOT NULL
);
"""

STATUS_LABELS = {"semnalata": "semnalată", "raportata": "raportată", "reparata": "reparată"}
SEVERITY_LABELS = {"mica": "mică", "medie": "medie", "mare": "mare", "necunoscuta": "nemăsurată (posibil apă)"}


def init_db(path: str):
    """Creează tabelele; o bază din versiunea cu accelerometru e migrată automat."""
    con = sqlite3.connect(path)
    try:
        cols = {r[1] for r in con.execute("PRAGMA table_info(events)")}
        if cols and "kind" not in cols:
            # versiunea veche: doar șocuri; le păstrăm ca evenimente de tip 'soc'
            con.executescript("ALTER TABLE events RENAME TO events_v1;" + EVENTS_TABLE + """
                INSERT OR IGNORE INTO events (device_id, ts, lat, lon, speed, kind, peak_g, received_at)
                    SELECT device_id, ts, lat, lon, speed, 'soc', peak_g, received_at
                    FROM events_v1 WHERE manual = 0;
                DROP TABLE events_v1;""")
        con.executescript(SCHEMA)
        con.commit()
    finally:
        con.close()


# ------------------------------------------------------------------ nume de străzi

def nominatim_street(lat: float, lon: float) -> str:
    """Numele străzii de la OpenStreetMap Nominatim. Ridică excepție dacă nu e internet."""
    q = urllib.parse.urlencode({"format": "jsonv2", "lat": lat, "lon": lon, "zoom": 17,
                                "accept-language": "ro"})
    req = urllib.request.Request("https://nominatim.openstreetmap.org/reverse?" + q,
                                 headers={"User-Agent": "harta-gropilor/2.0 (proiect universitar)"})
    with urllib.request.urlopen(req, timeout=10) as r:
        addr = json.load(r).get("address", {})
    return addr.get("road") or addr.get("pedestrian") or ""


def load_all(con: sqlite3.Connection, since: int = 0) -> tuple[list, list, list]:
    con.row_factory = sqlite3.Row
    events = [dict(r) for r in con.execute(
        "SELECT device_id, ts, lat, lon, speed, kind, depth_cm, length_cm, width_cm, peak_g"
        " FROM events WHERE ts >= ?", (since,))]
    statuses = [dict(r) for r in con.execute("SELECT lat, lon, status, changed_at FROM pothole_status")]
    streets = [dict(r) for r in con.execute("SELECT lat, lon, street FROM streets")]
    return events, statuses, streets


def resolve_streets(db_path: str, geocode, max_lookups: int = 20, pause: float = 1.1) -> int:
    """Caută numele străzii pentru gropile care nu îl au încă. Întoarce câte a găsit.

    Gropile confirmate au prioritate. Pauza respectă regula Nominatim de max. 1 cerere/s.
    """
    con = sqlite3.connect(db_path)
    try:
        events, _, streets = load_all(con)
        clusters = apply_streets(cluster_events(events), streets)
        todo = sorted((c for c in clusters if c["street"] is None), key=lambda c: not c["confirmed"])
        done = 0
        for c in todo[:max_lookups]:
            if any(haversine_m(c["lat"], c["lon"], s["lat"], s["lon"]) <= 20 for s in streets):
                continue
            name = geocode(c["lat"], c["lon"])
            con.execute("INSERT INTO streets (lat, lon, street) VALUES (?, ?, ?)", (c["lat"], c["lon"], name))
            con.commit()
            streets.append({"lat": c["lat"], "lon": c["lon"], "street": name})
            done += 1
            if pause:
                time.sleep(pause)
        return done
    finally:
        con.close()


def start_street_resolver(db_path: str, geocode, interval: float = 10):
    def loop():
        while True:
            try:
                resolve_streets(db_path, geocode)
            except Exception:
                pass                       # fără internet: încercăm din nou mai târziu
            time.sleep(interval)
    threading.Thread(target=loop, daemon=True, name="street-resolver").start()


# ------------------------------------------------------------------ aplicația

def create_app(db_path: str | None = None, geocode=nominatim_street,
               resolve_in_background: bool | None = None) -> Flask:
    app = Flask(__name__, static_folder="static")
    app.config["DB_PATH"] = db_path or os.environ.get(
        "POTHOLE_DB", os.path.join(BASE_DIR, "potholes.db"))
    app.config["API_KEY"] = os.environ.get("POTHOLE_API_KEY", "")
    init_db(app.config["DB_PATH"])

    if resolve_in_background is None:
        resolve_in_background = os.environ.get("POTHOLE_STREETS", "1") != "0"
    if resolve_in_background:
        start_street_resolver(app.config["DB_PATH"], geocode)

    def db() -> sqlite3.Connection:
        if "db" not in g:
            g.db = sqlite3.connect(app.config["DB_PATH"])
            g.db.row_factory = sqlite3.Row
        return g.db

    @app.teardown_appcontext
    def close_db(_exc):
        c = g.pop("db", None)
        if c is not None:
            c.close()

    def days_param() -> int:
        try:
            return max(0, int(request.args.get("days", 0)))
        except ValueError:
            return 0

    def potholes_now() -> tuple[list[dict], list[dict]]:
        days = days_param()
        since = int(time.time()) - days * 86400 if days > 0 else 0
        events, statuses, streets = load_all(db(), since)
        clusters = apply_streets(apply_status(cluster_events(events), statuses), streets)
        return clusters, events

    def key_ok() -> bool:
        return not app.config["API_KEY"] or request.headers.get("X-API-Key") == app.config["API_KEY"]

    def device_from(body) -> str:
        return str(body.get("device_id", "")).strip()[:64] if isinstance(body, dict) else ""

    # ------------------------------------------------------------ pagini

    @app.get("/")
    def index():
        return send_from_directory(app.static_folder, "index.html")

    # ------------------------------------------------------------ API autobuze

    @app.post("/api/events")
    def ingest():
        if not key_ok():
            return jsonify(error="cheie API invalidă"), 401
        body = request.get_json(silent=True)
        device_id = device_from(body)
        events = body.get("events") if isinstance(body, dict) else None
        if not device_id or not isinstance(events, list):
            return jsonify(error="lipsește device_id sau events"), 400
        if len(events) > MAX_EVENTS_PER_REQUEST:
            return jsonify(error=f"maxim {MAX_EVENTS_PER_REQUEST} evenimente pe cerere"), 413

        now = int(time.time())
        rows = [r for r in (_validate(e, device_id, now) for e in events) if r is not None]
        c = db()
        before = c.total_changes
        c.executemany(
            "INSERT OR IGNORE INTO events (device_id, ts, lat, lon, speed, kind, depth_cm, length_cm,"
            " width_cm, peak_g, received_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", rows)
        c.commit()
        inserted = c.total_changes - before
        return jsonify(inserted=inserted, duplicates=len(rows) - inserted, rejected=len(events) - len(rows))

    @app.post("/api/heartbeat")
    def heartbeat():
        """Semn de viață de la autobuz: starea senzorilor, GPS, câte date așteaptă pe card."""
        if not key_ok():
            return jsonify(error="cheie API invalidă"), 401
        body = request.get_json(silent=True)
        device_id = device_from(body)
        if not device_id:
            return jsonify(error="lipsește device_id"), 400
        sensors = body.get("sensors") if isinstance(body.get("sensors"), list) else []
        try:
            payload = {
                "sensors": [{"valid": max(0.0, min(1.0, float(s.get("valid", 1))))}
                            for s in sensors[:16] if isinstance(s, dict)],
                "gps": bool(body.get("gps", True)),
                "pending": max(0, int(body.get("pending", 0) or 0)),
                "fw": str(body.get("fw", ""))[:16],
            }
        except (TypeError, ValueError):
            return jsonify(error="heartbeat invalid"), 400
        c = db()
        c.execute("INSERT INTO devices (device_id, last_seen, payload) VALUES (?, ?, ?)"
                  " ON CONFLICT(device_id) DO UPDATE SET last_seen = excluded.last_seen, payload = excluded.payload",
                  (device_id, int(time.time()), json.dumps(payload)))
        c.commit()
        return jsonify(ok=True)

    @app.get("/api/devices")
    def devices():
        now = int(time.time())
        out = [device_health({"device_id": r["device_id"], "last_seen": r["last_seen"],
                              **json.loads(r["payload"])}, now)
               for r in db().execute("SELECT device_id, last_seen, payload FROM devices ORDER BY device_id")]
        return jsonify(devices=out)

    @app.delete("/api/devices/<device_id>")
    def delete_device(device_id):
        """Șterge tot ce a trimis un dispozitiv (de ex. la scoaterea din flotă)."""
        if not key_ok():
            return jsonify(error="cheie API invalidă"), 401
        c = db()
        n = c.execute("DELETE FROM events WHERE device_id = ?", (device_id,)).rowcount
        c.execute("DELETE FROM devices WHERE device_id = ?", (device_id,))
        c.commit()
        return jsonify(deleted=n)

    # ------------------------------------------------------------ API hartă

    @app.get("/api/potholes")
    def potholes():
        clusters, _ = potholes_now()
        return jsonify(potholes=clusters, generated_at=int(time.time()))

    @app.post("/api/potholes/status")
    def set_status():
        body = request.get_json(silent=True) or {}
        try:
            lat, lon = float(body["lat"]), float(body["lon"])
        except (KeyError, TypeError, ValueError):
            return jsonify(error="lipsește lat/lon"), 400
        status = body.get("status")
        if status not in STATUSES:
            return jsonify(error="stare necunoscută"), 400

        c = db()
        existing = [dict(r) for r in c.execute("SELECT id, lat, lon FROM pothole_status")]
        near = [r for r in existing if haversine_m(lat, lon, r["lat"], r["lon"]) <= 15]
        now = int(time.time())
        if near:
            c.execute("UPDATE pothole_status SET status = ?, changed_at = ? WHERE id = ?",
                      (status, now, near[0]["id"]))
        else:
            c.execute("INSERT INTO pothole_status (lat, lon, status, changed_at) VALUES (?, ?, ?, ?)",
                      (lat, lon, status, now))
        c.commit()
        return jsonify(ok=True, status=status, changed_at=now)

    @app.get("/api/streets")
    def streets():
        clusters, _ = potholes_now()
        return jsonify(street_summary(clusters))

    @app.get("/api/stats")
    def stats():
        clusters, events = potholes_now()
        open_ = [cl for cl in clusters if cl["status"] != "reparata"]
        by_class = {k: sum(cl["severity"] == k for cl in open_) for k in SEVERITIES}
        by_status = {k: sum(cl["status"] == k for cl in clusters) for k in STATUSES}
        return jsonify(
            events=len(events),
            buses=len({e["device_id"] for e in events}),
            potholes=len(open_),
            confirmed=sum(cl["confirmed"] for cl in open_),
            by_severity=by_class,
            by_status=by_status,
            asphalt_kg=sum(cl["asphalt_kg"] or 0 for cl in open_ if cl["confirmed"]),
        )

    @app.get("/api/report.csv")
    def report():
        clusters, _ = potholes_now()
        only_confirmed = request.args.get("confirmed") == "1"
        include_fixed = request.args.get("fixed") == "1"
        order = {"mare": 0, "medie": 1, "mica": 2, "necunoscuta": 3}
        clusters.sort(key=lambda cl: (order[cl["severity"]], -(cl["depth_cm"] or 0)))

        buf = io.StringIO()
        w = csv.writer(buf, delimiter=";")
        w.writerow(["nr", "strada", "severitate", "adancime_cm", "lungime_cm", "latime_cm",
                    "asfalt_estimat_kg", "stare", "treceri", "autobuze", "prima_detectare",
                    "ultima_detectare", "latitudine", "longitudine", "link_harta"])
        nr = 0
        for cl in clusters:
            if only_confirmed and not cl["confirmed"]:
                continue
            if cl["status"] == "reparata" and not include_fixed:
                continue
            nr += 1
            num = lambda v: "" if v is None else str(v).replace(".", ",")   # Excel în română
            w.writerow([nr, cl["street"] or "", SEVERITY_LABELS[cl["severity"]], num(cl["depth_cm"]),
                        num(cl["length_cm"]), num(cl["width_cm"]), num(cl["asphalt_kg"]),
                        STATUS_LABELS[cl["status"]] + (" (reapărută)" if cl["reappeared"] else ""),
                        cl["passes"], len(cl["buses"]), _fmt(cl["first_seen"]), _fmt(cl["last_seen"]),
                        cl["lat"], cl["lon"], f"https://www.google.com/maps?q={cl['lat']},{cl['lon']}"])
        return Response("﻿" + buf.getvalue(), mimetype="text/csv",
                        headers={"Content-Disposition": "attachment; filename=raport_gropi.csv"})

    return app


def _validate(e, device_id: str, now: int) -> tuple | None:
    """Verifică un eveniment primit; întoarce rândul pentru DB sau None."""
    if not isinstance(e, dict):
        return None
    try:
        ts, lat, lon = int(e["t"]), float(e["lat"]), float(e["lon"])
        speed = float(e.get("v", 0))
        kind = e.get("k")
        if kind == "lidar":
            depth, length, width = float(e["d"]), float(e["l"]), float(e["w"])
            peak = None
        elif kind == "soc":
            depth = length = width = None
            peak = float(e["g"])
        else:
            return None
    except (KeyError, TypeError, ValueError):
        return None
    if not (1_577_836_800 <= ts <= now + 3600):          # după 2020, nu din viitor
        return None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
        return None
    if not 0 <= speed <= 120:
        return None
    if kind == "lidar" and not (0.5 <= depth <= 40 and 0 < length <= 2000 and 0 < width <= 400):
        return None
    if kind == "soc" and not 0 < peak <= 16:
        return None
    r = lambda v, n: None if v is None else round(v, n)
    return (device_id, ts, round(lat, 6), round(lon, 6), round(speed, 1), kind,
            r(depth, 1), r(length, 0), r(width, 0), r(peak, 3), now)


def _fmt(ts: int) -> str:
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")


if __name__ == "__main__":
    create_app().run(host="0.0.0.0", port=5000, debug=False)
