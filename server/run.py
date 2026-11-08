"""Pornire cu un click: server + browser (+ date demo opțional).

Apelat de PORNESTE.bat / DEMO.bat (Windows) sau porneste.sh / demo.sh (Mac/Linux).
    python run.py           # date reale (potholes.db)
    python run.py --demo    # bază de date separată, umplută cu date simulate
"""
from __future__ import annotations

import argparse
import os
import random
import socket
import sys
import threading
import time
import urllib.request
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def lan_ip() -> str:
    """IP-ul calculatorului în rețeaua locală (de pus în config.h pe ESP32)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))     # nu trimite nimic, doar alege interfața
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def port_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("127.0.0.1", port)) != 0


def wait_until_up(url: str, timeout: float = 15) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        try:
            urllib.request.urlopen(url, timeout=1).close()
            return True
        except OSError:
            time.sleep(0.2)
    return False


def after_start(base: str, demo: bool, open_browser: bool):
    if not wait_until_up(base + "/api/stats"):
        print("[eroare] serverul nu a pornit")
        return
    if demo:
        import simulate
        random.seed(7)
        print("[demo] simulez autobuzele cu bară LiDAR...")
        res = simulate.run(base)
        print(f"[demo] gata: {res['inserted']} detecții încărcate")
    if open_browser:
        webbrowser.open(base)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="pornește cu date simulate")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    if not port_free(args.port):
        print(f"[eroare] portul {args.port} e ocupat. Poate serverul rulează deja într-o altă fereastră?")
        print(f"         Deschide http://localhost:{args.port} sau închide fereastra veche.")
        return 1

    from app import create_app

    db = os.path.join(HERE, "demo.db" if args.demo else "potholes.db")
    if args.demo and os.path.exists(db):
        os.remove(db)                       # demo-ul pornește mereu curat

    base = f"http://localhost:{args.port}"
    ip = lan_ip()
    print("=" * 60)
    print("  HARTA GROPILOR" + ("  (mod DEMO, date simulate)" if args.demo else ""))
    print("=" * 60)
    print(f"  Harta:            {base}")
    print(f"  De pe telefon:    http://{ip}:{args.port}  (aceeași rețea WiFi)")
    print(f"  Pentru config.h:  SERVER_URL \"http://{ip}:{args.port}/api/events\"")
    print("  Oprire: închide fereastra sau apasă Ctrl+C")
    print("=" * 60)

    threading.Thread(target=after_start, args=(base, args.demo, not args.no_browser),
                     daemon=True).start()

    import logging
    logging.getLogger("werkzeug").setLevel(logging.WARNING)   # fără un rând pe fiecare cerere
    create_app(db).run(host="0.0.0.0", port=args.port, debug=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
