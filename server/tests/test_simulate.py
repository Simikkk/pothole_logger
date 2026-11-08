"""Teste pentru simulator, cu un serviciu de rutare fals (fără internet)."""
import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import simulate  # noqa: E402


def fake_osrm(a, b):
    """Drum în L: întâi pe est-vest, apoi pe nord-sud (ca pe o rețea de străzi)."""
    pts, n = [], 40
    for i in range(n + 1):
        pts.append((a[0], a[1] + (b[1] - a[1]) * i / n))
    for i in range(1, n + 1):
        pts.append((a[0] + (b[0] - a[0]) * i / n, b[1]))
    return pts


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        fd, self.cache = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        os.unlink(self.cache)
        self.calls = 0

        def counting_fetch(a, b):
            self.calls += 1
            return fake_osrm(a, b)
        self.fetch = counting_fetch
        simulate.time.sleep = lambda s: None   # fără pauze în teste

    def tearDown(self):
        if os.path.exists(self.cache):
            os.unlink(self.cache)

    def test_all_events_lie_on_roads(self):
        random.seed(1)
        routes = simulate.load_routes(self.fetch, self.cache, verbose=False)
        events, _ = simulate.generate(routes)
        self.assertGreater(len(events), 30)
        for e in events:
            xy = simulate.to_xy(e["lat"], e["lon"])
            off = min(r.project(xy)[1] for r in routes)
            self.assertLess(off, 25, f"eveniment la {off:.0f} m de orice drum")

    def test_events_look_like_firmware_output(self):
        random.seed(4)
        routes = simulate.load_routes(self.fetch, self.cache, verbose=False)
        events, _ = simulate.generate(routes)
        kinds = {e["k"] for e in events}
        self.assertEqual(kinds, {"lidar", "soc"})
        for e in events:
            self.assertTrue(e["device"].startswith("bus-"))
            if e["k"] == "lidar":
                self.assertGreaterEqual(e["d"], 2.0)             # pragul din firmware
                self.assertIn(e["w"], (20, 40, 60, 80, 100))       # multiplu de distanța dintre senzori

    def test_rain_hides_flooded_pothole_from_lidar(self):
        random.seed(5)
        # groapa e sub marginea barei ȘI sub roata dreaptă
        p = {"depth": 6, "length": 60, "width": 100, "lateral": 60}
        dry = [simulate.pass_over(p, 0, rainy=False) for _ in range(200)]
        wet = [simulate.pass_over(p, 0, rainy=True) for _ in range(200)]
        frac = lambda rs: sum(1 for r in rs if r and r["k"] == "lidar") / len(rs)
        self.assertGreater(frac(dry), 0.9)
        self.assertLess(frac(wet), 0.35)
        self.assertTrue(any(r and r["k"] == "soc" for r in wet))   # roata o simte totuși

    def test_bar_misses_pothole_outside_its_reach(self):
        p = {"depth": 6, "length": 60, "width": 30, "lateral": 75}   # între bară și roată
        self.assertIsNone(simulate.pass_over(p, 0, rainy=False))

    def test_most_real_potholes_found_with_few_false_confirmations(self):
        random.seed(6)
        routes = simulate.load_routes(self.fetch, self.cache, verbose=False)
        events, potholes = simulate.generate(routes, passes=8)
        ev = simulate.evaluate_against_truth(events, potholes, routes)
        self.assertGreaterEqual(ev["found"] / ev["real"], 0.85)
        # alarmele false aleatorii pot coincide rar; acceptăm sub 10% din confirmări
        self.assertLessEqual(ev["false_confirmed"], max(1, 0.1 * ev["found"]))
        self.assertLess(ev["depth_error_cm"], 1.5)

    def test_potholes_are_sparse(self):
        random.seed(2)
        routes = simulate.load_routes(self.fetch, self.cache, verbose=False)
        ph = simulate.place_potholes(routes)
        self.assertGreaterEqual(len(ph), 8)
        for i, p in enumerate(ph):
            for q in ph[i + 1:]:
                self.assertGreaterEqual(math.dist(p["xy"], q["xy"]), simulate.MIN_SPACING_M)

    def test_cache_is_reused(self):
        simulate.load_routes(self.fetch, self.cache, verbose=False)
        first = self.calls
        self.assertEqual(first, len(simulate.ROUTE_PAIRS))
        simulate.load_routes(self.fetch, self.cache, verbose=False)
        self.assertEqual(self.calls, first)       # a doua oară nu mai descarcă nimic

    def test_offline_falls_back(self):
        def offline(a, b):
            raise OSError("fără internet")
        routes = simulate.load_routes(offline, self.cache, verbose=False)
        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0].name, "aproximativ")


if __name__ == "__main__":
    unittest.main()
