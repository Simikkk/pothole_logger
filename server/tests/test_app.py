"""Teste pentru logica de grupare și pentru API.  Rulare:  python -m unittest discover -s tests"""
import json
import os
import sqlite3
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import create_app, init_db, resolve_streets            # noqa: E402
from clustering import (apply_status, apply_streets, cluster_events,  # noqa: E402
                        count_passes, device_health, haversine_m, severity_from_depth,
                        street_summary)

BRASOV = (45.6427, 25.5887)
M = 1 / 111_320          # aproximativ un metru în grade de latitudine


def lidar(lat, lon, ts, depth=5.0, length=60, width=40, device="bus-001"):
    return {"device_id": device, "ts": ts, "lat": lat, "lon": lon, "speed": 30, "kind": "lidar",
            "depth_cm": depth, "length_cm": length, "width_cm": width, "peak_g": None}


def shock(lat, lon, ts, g=0.8, device="bus-001"):
    return {"device_id": device, "ts": ts, "lat": lat, "lon": lon, "speed": 30, "kind": "soc",
            "depth_cm": None, "length_cm": None, "width_cm": None, "peak_g": g}


class ClusteringTests(unittest.TestCase):
    def test_haversine_known_distance(self):
        self.assertAlmostEqual(haversine_m(45.0, 25.0, 45.001, 25.0), 111.2, delta=0.5)

    def test_close_detections_form_one_pothole(self):
        lat, lon = BRASOV
        res = cluster_events([lidar(lat, lon, 1000), lidar(lat + 5 * M, lon, 5000)])
        self.assertEqual(len(res), 1)
        self.assertEqual(res[0]["lidar_detections"], 2)

    def test_distant_detections_form_two_potholes(self):
        lat, lon = BRASOV
        self.assertEqual(len(cluster_events([lidar(lat, lon, 1000), lidar(lat + 30 * M, lon, 1002)])), 2)

    def test_passes_split_by_time_gap(self):
        self.assertEqual(count_passes([0, 5, 30]), 1)
        self.assertEqual(count_passes([0, 500, 1000]), 3)
        self.assertEqual(count_passes([]), 0)

    def test_confirmed_needs_two_passes(self):
        lat, lon = BRASOV
        self.assertFalse(cluster_events([lidar(lat, lon, 1000)])[0]["confirmed"])
        self.assertTrue(cluster_events([lidar(lat, lon, 1000), lidar(lat, lon, 9000)])[0]["confirmed"])

    def test_severity_by_depth(self):
        self.assertEqual(severity_from_depth(2.5), "mica")
        self.assertEqual(severity_from_depth(4.0), "medie")
        self.assertEqual(severity_from_depth(7.5), "mare")
        self.assertEqual(severity_from_depth(None), "necunoscuta")

    def test_dimensions_median_depth_max_width_and_asphalt(self):
        lat, lon = BRASOV
        c = cluster_events([lidar(lat, lon, 1000, depth=4, length=50, width=20),
                            lidar(lat, lon, 9000, depth=6, length=70, width=60),
                            lidar(lat, lon, 20000, depth=5, length=60, width=40)])[0]
        self.assertEqual((c["depth_cm"], c["max_depth_cm"], c["length_cm"], c["width_cm"]), (5, 6, 60, 60))
        self.assertEqual(c["volume_l"], 18.0)                # 5 × 60 × 60 cm³ = 18 L
        self.assertEqual(c["asphalt_kg"], 43)                # 18 L × 2,4 kg/L

    def test_shock_only_pothole_has_unknown_severity(self):
        lat, lon = BRASOV
        c = cluster_events([shock(lat, lon, 1000), shock(lat, lon, 9000)])[0]
        self.assertEqual((c["severity"], c["depth_cm"], c["confirmed"]), ("necunoscuta", None, True))

    def test_lidar_measurement_wins_over_shock(self):
        lat, lon = BRASOV
        c = cluster_events([shock(lat, lon, 1000), lidar(lat, lon, 9000, depth=8)])[0]
        self.assertEqual((c["severity"], c["shock_detections"], c["lidar_detections"]), ("mare", 1, 1))

    def test_buses_listed(self):
        lat, lon = BRASOV
        c = cluster_events([lidar(lat, lon, 1000, device="bus-002"), lidar(lat, lon, 9000, device="bus-001")])[0]
        self.assertEqual(c["buses"], ["bus-001", "bus-002"])


class StatusStreetHealthTests(unittest.TestCase):
    def setUp(self):
        lat, lon = BRASOV
        self.clusters = cluster_events([lidar(lat, lon, 1000, depth=8), lidar(lat, lon, 9000, depth=8)])

    def test_default_status(self):
        c = apply_status(self.clusters, [])[0]
        self.assertEqual((c["status"], c["reappeared"]), ("semnalata", False))

    def test_repaired_pothole_reappears_after_new_detection(self):
        lat, lon = BRASOV
        c = apply_status(self.clusters, [{"lat": lat, "lon": lon, "status": "reparata", "changed_at": 5000}])[0]
        self.assertEqual((c["status"], c["reappeared"]), ("semnalata", True))
        c = apply_status(self.clusters, [{"lat": lat, "lon": lon, "status": "reparata", "changed_at": 9999}])[0]
        self.assertEqual((c["status"], c["reappeared"]), ("reparata", False))

    def test_street_summary(self):
        lat, lon = BRASOV
        cl = cluster_events([lidar(lat, lon, 1000, 8), lidar(lat, lon, 9000, 8),
                             lidar(lat + 300 * M, lon, 1000, 2.5), lidar(lat + 300 * M, lon, 9000, 2.5),
                             lidar(lat + 600 * M, lon, 1000, 8), lidar(lat + 600 * M, lon, 9000, 8),
                             lidar(lat + 900 * M, lon, 1000, 8), lidar(lat + 900 * M, lon, 9000, 8)])
        apply_status(cl, [{"lat": lat + 600 * M, "lon": lon, "status": "reparata", "changed_at": 99999}])
        apply_streets(cl, [{"lat": lat, "lon": lon, "street": "Strada Lungă"},
                           {"lat": lat + 300 * M, "lon": lon, "street": "Strada Lungă"},
                           {"lat": lat + 600 * M, "lon": lon, "street": "Strada Mureșenilor"}])
        r = street_summary(cl)
        self.assertEqual(r["pending"], 1)
        s = r["streets"][0]
        self.assertEqual((len(r["streets"]), s["street"], s["total"], s["mare"], s["mica"]),
                         (1, "Strada Lungă", 2, 1, 1))
        self.assertGreater(s["asphalt_kg"], 0)

    def test_device_health(self):
        now = 100_000
        ok = device_health({"device_id": "a", "last_seen": now - 60, "sensors": [{"valid": .97}] * 5}, now)
        dirty = device_health({"device_id": "b", "last_seen": now - 60,
                               "sensors": [{"valid": .97}, {"valid": .2}, {"valid": .95}]}, now)
        off = device_health({"device_id": "c", "last_seen": now - 3600, "sensors": []}, now)
        nogps = device_health({"device_id": "d", "last_seen": now, "sensors": [], "gps": False}, now)
        self.assertEqual(ok["state"], "ok")
        self.assertEqual((dirty["state"], dirty["dirty_sensors"]), ("senzor_murdar", [2]))
        self.assertEqual(off["state"], "offline")
        self.assertEqual(nogps["state"], "fara_gps")


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
        self.tmp.close()
        self.client = create_app(self.tmp.name, resolve_in_background=False).test_client()
        self.now = int(time.time()) - 60

    def tearDown(self):
        os.unlink(self.tmp.name)

    def post(self, events, device="bus-001"):
        return self.client.post("/api/events", json={"device_id": device, "events": events})

    def L(self, t, d=7.5, **kw):
        return {"t": t, "lat": 45.6427, "lon": 25.5887, "v": 32, "k": "lidar", "d": d, "l": 60, "w": 40, **kw}

    def test_ingest_and_deduplicate(self):
        self.assertEqual(self.post([self.L(self.now)]).get_json()["inserted"], 1)
        r = self.post([self.L(self.now)]).get_json()
        self.assertEqual((r["inserted"], r["duplicates"]), (0, 1))

    def test_lidar_and_shock_same_moment_both_kept(self):
        r = self.post([self.L(self.now), {"t": self.now, "lat": 45.6427, "lon": 25.5887, "v": 32, "k": "soc", "g": 0.9}])
        self.assertEqual(r.get_json()["inserted"], 2)

    def test_invalid_events_rejected(self):
        bad = [
            self.L(self.now, lat=0, lon=0),                       # fără fix GPS
            self.L(100),                                          # ceas nesetat
            self.L(self.now, d=80),                               # adâncime imposibilă
            {"t": self.now, "lat": 45.6, "lon": 25.5, "k": "lidar"},   # lipsesc dimensiunile
            {"t": self.now, "lat": 45.6, "lon": 25.5, "k": "soc"},     # lipsește vârful
            {"t": self.now, "lat": 45.6, "lon": 25.5, "k": "manual"},  # tip necunoscut
            "nu sunt dict",
        ]
        r = self.post(bad).get_json()
        self.assertEqual((r["inserted"], r["rejected"]), (0, 7))

    def test_missing_device_is_400(self):
        self.assertEqual(self.client.post("/api/events", json={"events": []}).status_code, 400)

    def test_potholes_stats_and_report(self):
        self.post([self.L(self.now - 3600), self.L(self.now)])
        ph = self.client.get("/api/potholes").get_json()["potholes"]
        self.assertEqual(len(ph), 1)
        self.assertEqual((ph[0]["severity"], ph[0]["depth_cm"], ph[0]["confirmed"]), ("mare", 7.5, True))
        st = self.client.get("/api/stats").get_json()
        self.assertEqual((st["events"], st["potholes"], st["by_severity"]["mare"], st["buses"]), (2, 1, 1, 1))
        self.assertGreater(st["asphalt_kg"], 0)
        csv_text = self.client.get("/api/report.csv").get_data(as_text=True)
        self.assertIn("adancime_cm", csv_text)
        self.assertIn(";7,5;", csv_text)                      # zecimală cu virgulă, pentru Excel
        self.assertIn("google.com/maps?q=45.6427,25.5887", csv_text)

    def test_days_filter(self):
        self.post([self.L(self.now - 40 * 86400)])
        self.assertEqual(len(self.client.get("/api/potholes?days=30").get_json()["potholes"]), 0)
        self.assertEqual(len(self.client.get("/api/potholes?days=0").get_json()["potholes"]), 1)

    def test_status_flow(self):
        self.post([self.L(self.now - 3600), self.L(self.now)])
        p = self.client.get("/api/potholes").get_json()["potholes"][0]
        for st in ("raportata", "reparata"):
            r = self.client.post("/api/potholes/status", json={"lat": p["lat"], "lon": p["lon"], "status": st})
            self.assertEqual(r.status_code, 200)
        self.assertEqual(self.client.get("/api/potholes").get_json()["potholes"][0]["status"], "reparata")
        self.assertEqual(self.client.get("/api/stats").get_json()["potholes"], 0)
        self.assertEqual(self.client.get("/api/report.csv").get_data(as_text=True).count("\n"), 1)
        bad = self.client.post("/api/potholes/status", json={"lat": 1, "lon": 1, "status": "pierduta"})
        self.assertEqual(bad.status_code, 400)

    def test_streets_resolved(self):
        self.post([self.L(self.now - 3600), self.L(self.now)])
        calls = []
        geocode = lambda lat, lon: calls.append(1) or "Strada Republicii"
        self.assertEqual(resolve_streets(self.tmp.name, geocode, pause=0), 1)
        self.assertEqual(resolve_streets(self.tmp.name, geocode, pause=0), 0)      # din cache
        self.assertEqual(len(calls), 1)
        self.assertEqual(self.client.get("/api/streets").get_json()["streets"][0]["street"], "Strada Republicii")

    def test_heartbeat_and_devices(self):
        r = self.client.post("/api/heartbeat", json={"device_id": "bus-003",
                             "sensors": [{"valid": 0.98}, {"valid": 0.2}], "gps": True, "pending": 4})
        self.assertEqual(r.status_code, 200)
        self.client.post("/api/heartbeat", json={"device_id": "bus-001", "sensors": [{"valid": 0.97}]})
        d = {x["device_id"]: x for x in self.client.get("/api/devices").get_json()["devices"]}
        self.assertEqual((d["bus-003"]["state"], d["bus-003"]["dirty_sensors"], d["bus-003"]["pending"]),
                         ("senzor_murdar", [2], 4))
        self.assertEqual(d["bus-001"]["state"], "ok")
        self.assertEqual(self.client.post("/api/heartbeat", json={"device_id": "x", "pending": "abc"}).status_code, 400)

    def test_delete_device(self):
        self.post([self.L(self.now)])
        self.client.post("/api/heartbeat", json={"device_id": "bus-001"})
        self.assertEqual(self.client.delete("/api/devices/bus-001").get_json()["deleted"], 1)
        self.assertEqual(self.client.get("/api/devices").get_json()["devices"], [])

    def test_api_key_enforced_when_set(self):
        os.environ["POTHOLE_API_KEY"] = "secret"
        try:
            client = create_app(self.tmp.name, resolve_in_background=False).test_client()
            body = {"device_id": "x", "events": []}
            self.assertEqual(client.post("/api/events", json=body).status_code, 401)
            self.assertEqual(client.post("/api/heartbeat", json=body).status_code, 401)
            self.assertEqual(client.post("/api/events", json=body, headers={"X-API-Key": "secret"}).status_code, 200)
        finally:
            del os.environ["POTHOLE_API_KEY"]

    def test_no_ip_or_personal_data_stored(self):
        con = sqlite3.connect(self.tmp.name)
        cols = {r[1] for t in ("events", "pothole_status", "streets", "devices")
                for r in con.execute(f"PRAGMA table_info({t})")}
        con.close()
        self.assertFalse(cols & {"ip", "remote_addr", "user", "name", "email"})


class MigrationTests(unittest.TestCase):
    def test_old_accelerometer_database_is_migrated(self):
        fd, path = tempfile.mkstemp(suffix=".db")
        os.close(fd)
        try:
            con = sqlite3.connect(path)
            con.executescript("""CREATE TABLE events (id INTEGER PRIMARY KEY, device_id TEXT NOT NULL, ts INTEGER NOT NULL,
                lat REAL NOT NULL, lon REAL NOT NULL, speed REAL NOT NULL, peak_g REAL NOT NULL,
                manual INTEGER NOT NULL DEFAULT 0, received_at INTEGER NOT NULL,
                UNIQUE (device_id, ts, lat, lon, manual));
                INSERT INTO events VALUES (1, 'esp32-01', 1700000000, 45.6, 25.5, 30, 1.2, 0, 1700000000);
                INSERT INTO events VALUES (2, 'esp32-01', 1700000001, 45.6, 25.5, 30, 0, 1, 1700000001);""")
            con.commit()
            con.close()
            init_db(path)
            init_db(path)                                   # a doua oară nu strică nimic
            con = sqlite3.connect(path)
            rows = con.execute("SELECT kind, peak_g, depth_cm FROM events").fetchall()
            con.close()
            self.assertEqual(rows, [("soc", 1.2, None)])     # marcajul manual nu se mai păstrează
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
