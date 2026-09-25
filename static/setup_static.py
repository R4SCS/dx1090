#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# setup_static.py - download Chart.js and Leaflet for local serving

import os, sys

static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
os.makedirs(static_dir, exist_ok=True)

files = {
    "chart.umd.min.js": "https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js",
    "leaflet.js": "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js",
    "leaflet.css": "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css",
}

try:
    import urllib.request
except ImportError:
    print("urllib not available")
    sys.exit(1)

ok = 0
for name, url in files.items():
    path = os.path.join(static_dir, name)
    if os.path.exists(path) and os.path.getsize(path) > 1000:
        print("  SKIP (exists): " + name)
        ok += 1
        continue
    print("  Downloading: " + name + " ...")
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "dx1090/1.0"})
        resp = urllib.request.urlopen(req, timeout=30)
        data = resp.read()
        with open(path, "wb") as f:
            f.write(data)
        print("    OK: {} ({} bytes)".format(name, len(data)))
        ok += 1
    except Exception as e:
        print("    FAIL: " + str(e))
        print("    Manual: wget -O static/" + name + " " + url)

print()
if ok == 3:
    print("All files ready in: " + static_dir)
else:
    print("{}/3 files downloaded. Put missing files in: {}".format(ok, static_dir))
