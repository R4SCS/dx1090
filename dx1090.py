#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==================================================================
# ADS-B + Mode S + Mode A/C + UAT decoder
# Receiver support: RTL-SDR, Airspy, HackRF, BladeRF
# Decoding: DF 0-25, all TC, BDS 1,0-6,0, UAT 978
# Network protocols: Beast, SBS1, JSON
# 26 sound notifications, auto-PPM, sessions, dual quality metrics
# Polar plots, analytics, doppler detector, conflict detector
# Export: CSV, GZ, GPX, KML, JSON, AeroXML, FlightAware
# dx1090 — ADS-B / Mode S / Mode A/C / UAT Decoder
# ==================================================================
# Copyright (c) 2026-present, R4SCS https://github.com/r4scs
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# ==================================================================

import numpy as np
import subprocess, sys, math, time, os, wave, struct, datetime, threading, queue, atexit
import shutil, select, csv, socket
import json as json_module
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections import deque
_json = json_module  # alias for HTTP handler
import gzip as _gz
import logging
from logging.handlers import RotatingFileHandler

_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(_log_dir, exist_ok=True)
log = logging.getLogger("adsb9")
log.setLevel(logging.INFO)
if not log.handlers:
    _h = RotatingFileHandler(os.path.join(_log_dir, "dx1090.log"), maxBytes=5*1024*1024, backupCount=3, encoding="utf-8")
    _h.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
    log.addHandler(_h)
log.info("ADS-B decoder started")

DEBUG = False  # Enable debug output to stderr

# ==================================================================
# SDR RECEIVER
# ==================================================================

# Receiver type.
#   "rtl_sdr" — RTL-SDR (RTL2832U), most common
#   "airspy"  — Airspy R2 / Mini
#   "hackrf"  — HackRF One
#   "bladerf" — bladeRF 2.0 micro
SDR_TYPE = "rtl_sdr"

# Receive frequency, Hz. ADS-B and Mode S — 1090 MHz (ICAO Annex 10).
FREQ = 1090000000

# Sample rate, Hz.
#   2000000  — optimal: 2 samples per bit, no interpolation
#   2400000  — acceptable for rtl_sdr, fractional SAMPLES_PER_BIT
#   2500000  — acceptable, but requires phase correction
SAMPLE_RATE = 2000000

# Receiver gain.
#   RTL-SDR:  in tenths of dB (0 = auto/AGC, 421 = 42.1 dB, 496 = maximum)
#   Airspy:   in dB (0-21)
#   HackRF:   in dB (0-14, steps of 8)
#   bladeRF:  in dB (0-60)
GAIN = 0

# PPM correction — crystal frequency deviation, ppm.
#   Range: integer from -100 to +100.
#   RTL-SDR with cheap crystals: 20-80 ppm.
#   Inaccurate value -> position offset on the map.
#   For auto-calibration enable ENABLE_PPM_AUTO.
PPM_CORRECTION = 0

# SDR read block size, bytes.
#   131072  — 64 KB, minimal latency
#   262144  — 128 KB, balanced (recommended)
#   524288  — 256 KB, fewer read() calls
CHUNK_BYTES = 262144

# Minimum signal level for preamble detection.
#   0 — auto-detection based on noise_floor
#   > 0 — fixed magnitude threshold
MIN_SIGNAL_LEVEL = 0

# ==================================================================
# UAT 978 MHz
# ==================================================================

# UAT — second ADS-B channel (USA, Australia). Requires a separate SDR at 978 MHz.
# Not used in Europe and Russia.
ENABLE_UAT = False

# UAT frequency, Hz. Standard — 978 MHz.
UAT_FREQ = 978000000

# UAT sample rate, Hz. Standard — 2083333 Hz.
UAT_SAMPLE_RATE = 2083333

# UAT receiver gain (same as GAIN).
UAT_GAIN = 0

# PPM correction for UAT receiver.
UAT_PPM = 0

# ==================================================================
# RECEPTION MODES
# ==================================================================

# Mode S — primary mode (ADS-B, DF 0-25, 1090 MHz).
ENABLE_MODE_S = True

# Mode A/C — classic secondary radar (squawk + altitude).
# Same 1090 MHz frequency, different format. Useful for aircraft without ADS-B.
ENABLE_MODE_AC = False

# ==================================================================
# ADAPTIVE GAIN
# ==================================================================

# Auto-adjust gain based on message rate.
#   True  — if messages < GAIN_MIN_MSG_RATE, gain increases;
#           if > GAIN_GOOD_MSG_RATE — decreases.
#   False — fixed gain.
ADAPTIVE_GAIN = False

# Supported gain steps for RTL-SDR, tenths of dB.
# Do not change — these are values supported by rtl_sdr.
GAIN_STEPS = [0, 90, 140, 270, 370, 770, 870, 1250, 1440, 1570,
              1660, 1970, 2070, 2290, 2540, 2800, 2970, 3280, 3380,
              3640, 3720, 3860, 4020, 4210, 4340, 4390, 4450, 4800, 4960]

# Message rate check interval, sec (10-120).
GAIN_CHECK_INTERVAL = 30

# Minimum rate, msg/sec. Below this -> gain increases (1-50).
GAIN_MIN_MSG_RATE = 3

# Good rate, msg/sec. Above this -> gain decreases (5-100).
GAIN_GOOD_MSG_RATE = 15

# ==================================================================
# PHASE CORRECTION
# ==================================================================

# Trial bit position offsets for CRC pass.
#   True  — increases valid message percentage, slows down decoding.
#   False — no correction.
ENABLE_PHASE_CORRECTION = True

# Phase offsets in fractions of a sample.
#   0.0 — no offset, 0.5 — half sample forward, etc.
PHASE_OFFSETS = [0.0, 0.5, -0.5]  # ±1.0 removed: whole-bit shift causes false CRC passes

# ==================================================================
# RSSI AND POSITION RELIABILITY
# ==================================================================

# RSSI calculation — ratio of message amplitude to noise_floor, dB.
ENABLE_RSSI = True

# Minimum number of successful position decodings for "reliable" status.
# Below this threshold, position is not used for dead-reckoning and smoothing.
#   Range: 1-20.
POSITION_RELIABLE_MIN = 1

# Maximum reliability counter (cap, stops growing).
#   Range: 5-50.
POSITION_RELIABLE_MAX = 20

# ==================================================================
# RECEIVER COORDINATES (WGS84, decimal degrees)
# ==================================================================

# Observation point latitude. Used for distance calculation,
# CPR Local, dead-reckoning and geofilter.
#   Range: -90.0 ... +90.0
HOME_LAT = 46.856614

# Observation point longitude.
#   Range: -180.0 ... +180.0
HOME_LON = 2.213749

# ==================================================================
# RADIUS GEOFILTER
# ==================================================================

# Filter radius — discard aircraft beyond this distance.
#   0 — filter disabled.
#   > 0 — discard aircraft outside the radius.
FILTER_RADIUS = 0

# Radius unit:
#   "km" — kilometers
#   "mi" — miles
#   "nm" — nautical miles
FILTER_RADIUS_UNIT = "mi"

# Action for aircraft without position:
#   "show" — show
#   "hide" — hide
FILTER_NO_POSITION = "show"

# Remove aircraft that go beyond radius.
#   True  — remove from database when out of range
#   False — keep in database
FILTER_REMOVE_OUT_OF_RANGE = True

# ==================================================================
# DISPLAY UNITS
# ==================================================================

# Altitude: "ft" — feet, "m" — meters.
UNIT_ALT = "ft"

# Speed: "kt" — knots, "kmh" — km/h, "mph" — mph, "ms" — m/s.
UNIT_SPEED = "kt"

# Distance: "km" — kilometers, "mi" — miles, "nm" — nautical miles.
UNIT_DISTANCE = "mi"

# Vertical rate: "fpm" — feet/min, "ms" — m/s.
UNIT_VRATE = "fpm"

# Pressure: "hPa" — hectopascals, "mmHg" — mmHg, "inHg" — inHg.
UNIT_PRESSURE = "hPa"

# Temperature: "C" — Celsius, "F" — Fahrenheit.
UNIT_TEMP = "F"

# Map display mode:
#   "leaflet" — online tiles via OSM proxy (requires internet)
#   "canvas"  — offline radar (no internet needed, draws on canvas)
#   "auto"   — leaflet with canvas fallback when tiles unavailable
MAP_MODE = "auto"

# Tile cache directory for OSM tiles (relative to script path).
# Used by the tile proxy to cache downloaded tiles on disk.
#   "." — current directory
#   "tile_cache" — subdirectory (default)
TILE_CACHE_DIR = "tile_cache"

# ==================================================================
# DATA SOURCES FOR DISPLAY
# ==================================================================

# Horizontal speed source.
#   "GS"  — ADS-B (TC 19), ground speed
#   "IAS" — BDS 6,0, indicated airspeed
#   "TAS" — BDS 5,0 / 6,0, true airspeed
# If selected source unavailable -> fallback to GS.
SPEED_SOURCE = "GS"

# Vertical speed source.
#   "TC19"  — ADS-B (TC 19)
#   "BDS60" — BDS 6,0
# If BDS60 unavailable -> fallback to TC19.
VRATE_SOURCE = "TC19"

# ==================================================================
# SIGNAL PROCESSING
# ==================================================================

# DC removal (ADC zero offset).
ENABLE_DC_REMOVAL = True

# Adaptive noise floor estimation.
# 20th percentile of amplitudes, EMA-smoothed.
ENABLE_NOISE_FLOOR = True

# Noise floor smoothing coefficient (EMA alpha).
#   0.001 — very smooth
#   0.1   — fast
NOISE_FLOOR_ALPHA = 0.005

# Pulse blanking (impulse noise suppression).
# Samples above median x RATIO are clipped.
ENABLE_PULSE_BLANKING = True

# Pulse blanking coefficient.
#   10 — aggressive
#   50 — gentle
PULSE_BLANKING_RATIO = 50

# ==================================================================
# MLAT TIMESTAMPS
# ==================================================================

# Add MLAT timestamps to Beast (12 MHz timer, like dump1090 / piaware).
ENABLE_MLAT_TIMESTAMPS = True

# MLAT timer frequency, MHz (12 = 12 MHz, standard for Beast protocol / dump1090 / piaware).
MLAT_TIMESTAMP_PRECISION_US = 12

# ==================================================================
# DOPPLER / PPM ESTIMATION
# ==================================================================

# Estimate crystal PPM error from IQ phase slope of received messages.
#   True  — compute Doppler/PPM from IQ samples (needs IQ data from SDR)
#   False — disable
ENABLE_DOPPLER = True

# Minimum IQ samples for Doppler estimation (50-240).
DOPPLER_MIN_SAMPLES = 100

# Throttle: minimum seconds between measurements per aircraft (1-30).
DOPPLER_MIN_INTERVAL = 3

# ==================================================================
# FILTERS AND SMOOTHING
# ==================================================================

# Maximum position jump, km, between two decodings.
# Exceeded -> position rejected, pos_reliable reset.
#   Range: 10-200.
MAX_POSITION_JUMP_KM = 50.0

# Maximum range from receiver, km.
# Positions beyond this limit are rejected.
#   Range: 100-1000. Recommended 400-600.
MAX_RANGE_KM = 400.0

# Position smoothing (exponential smoothing).
#   lat_new = lat_old + alpha * (lat_raw - lat_old)
ENABLE_SMOOTHING = True

# Smoothing coefficient.
#   0.1 — very smooth
#   1.0 — no smoothing
SMOOTHING_ALPHA = 0.6

# Dead-reckoning — position extrapolation by speed and heading.
ENABLE_DEAD_RECKONING = True

# Maximum position age, sec, for extrapolation.
#   Range: 10-60.
DR_MAX_AGE = 30

# Maximum vertical rate, fpm. Exceeded -> altitude rejected.
# 10000 fpm ~ 50 m/s — unrealistic for civil aviation.
#   Range: 5000-20000.
MAX_ALT_RATE_FPM = 10000

# Maximum number of track points per aircraft (30-500).
TRAIL_MAX_POINTS = 100

# Minimum interval between track points, sec (0.1-10.0).
TRAIL_MIN_INTERVAL = 1.0

# ==================================================================
# DECODING — BDS REGISTERS
# ==================================================================

# Each flag: True (enable decoding), False (disable).
ENABLE_BDS10 = True   # BDS 1,0 — Data Link Capability
ENABLE_BDS17 = True   # BDS 1,7 — Common Usage GICB Capability
ENABLE_BDS20 = True   # BDS 2,0 — Aircraft Identification (callsign)
ENABLE_BDS30 = True   # BDS 3,0 — ACAS Resolution Advisory Report (TCAS RA)
ENABLE_BDS40 = True   # BDS 4,0 — Selected Altitude (selected altitude, QNH)
ENABLE_BDS44 = True   # BDS 4,4 — Meteorological Information (wind, temperature)
ENABLE_BDS45 = True   # BDS 4,5 — Weather hazards (turbulence, icing)
ENABLE_BDS50 = True   # BDS 5,0 — Track and Turn Report (roll, heading, GS, TAS)
ENABLE_BDS60 = True   # BDS 6,0
ENABLE_BDS51 = True   # BDS 5,1
ENABLE_BDS52 = True   # BDS 5,2
ENABLE_BDS53 = True   # BDS 5,3
ENABLE_BDS61 = True   # BDS 6,1 — Heading and Speed Report (mag. heading, IAS, Mach)

# ==================================================================
# DECODING — ADS-B INTEGRITY AND VERSION
# ==================================================================

# NIC — Navigation Integrity Category (position accuracy).
ENABLE_NIC = True
# SIL — Source Integrity Level (position error probability).
ENABLE_SIL = True
# NACv — Navigation Accuracy Category Velocity (speed accuracy).
ENABLE_NACV = True
# ADS-B version: DO-260 / DO-260A / DO-260B.
ENABLE_ADSB_VERSION = True

# ==================================================================
# DECODING — CRC
# ==================================================================

# Single-bit CRC correction (syndrome method).
#   True — recovers ~5-10% of corrupted messages.
ENABLE_CRC_CORRECTION = True

# Two-bit CRC correction (full brute-force of bit pairs).
#   True  — additionally recovers messages with 2 errors.
#   False — disabled (faster).
ENABLE_CRC_TWO_BIT = True

# Search range for 2-bit correction, bits.
#   16  — first 16 bits only (fast)
#   32  — first 32 bits (moderate)
#   112 — entire 112-bit frame (very slow)
CRC_TWO_BIT_RANGE = 56

# Adaptive 2-bit correction: disable at high message rates.
#   True  — 2-bit correction disabled if msg_rate > CRC_TWO_BIT_MAX_RATE
#   False — always enabled (if ENABLE_CRC_TWO_BIT = True)
CRC_TWO_BIT_ADAPTIVE = True

# Rate threshold, msg/sec, for disabling 2-bit correction.
#   Range: 50-1000.
CRC_TWO_BIT_MAX_RATE = 200
CRC_TWO_BIT_DYNAMIC = True
CRC_TWO_BIT_TIME_RATIO = 0.5
CRC_TWO_BIT_MAX_DECODE_TIME = 0.05
CRC_TWO_BIT_TIME_ALPHA = 0.1

# ==================================================================
# DECODING — DF TYPES
# ==================================================================

# DF 18 — TIS-B / ADS-R (non-ICAO addresses).
ENABLE_DF18 = True
# DF 19 — Military Extended Squitter (military).
ENABLE_DF19 = True

# Doppler detector (see DOPPLER / PPM ESTIMATION section above)

# ==================================================================
# LOGGING AND UI
# ==================================================================

# Log directory. Structure: LOG_DIR/DD.MM.YYYY/FLIGHT_HH.MM.txt
LOG_DIR = "logs"

# Event log (text below the table).
#   True  — show events on screen
#   False — only in log files
ENABLE_EVENT_LOG = False

# Logging mode.
#   "full"   — all events to files
#   "screen" — only screen-visible events
LOG_MODE = "full"

# WAV file cache directory.
SOUND_DIR = "sounds"

# === CROSSPLATFORM SUPPORT ===
import signal

IS_WINDOWS = sys.platform == "win32"
IS_MACOS   = sys.platform == "darwin"
IS_LINUX   = sys.platform.startswith("linux")

# UTF-8 encoding on Windows
if IS_WINDOWS:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ANSI colors on Windows 10+
if IS_WINDOWS:
    os.system("")

# Base directories by platform
if IS_WINDOWS:
    _BASE = os.environ.get("LOCALAPPDATA", os.path.expanduser("~"))
    if not os.path.isdir(SOUND_DIR):
        SOUND_DIR = os.path.join(_BASE, "sqwk1090", "sounds")
    if not os.path.isdir(LOG_DIR):
        LOG_DIR = os.path.join(_BASE, "sqwk1090", "logs")
else:
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    if not os.path.isdir(SOUND_DIR):
        SOUND_DIR = os.path.join(_SCRIPT_DIR, "sounds")
    if not os.path.isdir(LOG_DIR):
        LOG_DIR = os.path.join(_SCRIPT_DIR, "adsb_logs")



# ==================================================================
# HISTORICAL STATISTICS
# ==================================================================

# Save statistics snapshots (aircraft types, histograms, max values) to disk.
#   True  — a snapshot is saved every STATS_HISTORY_INTERVAL sec
#   False — disabled
ENABLE_STATS_HISTORY = True

# Historical statistics directory.
# Structure: STATS_HISTORY_DIR/YYYY-MM-DD/YYYY-MM-DD_HH-MM.json
STATS_HISTORY_DIR = "stats_history"

# Snapshot save interval, sec (60-3600). Recommended 300 (5 min).
STATS_HISTORY_INTERVAL = 300

# How many days to keep history (1-365). Older is deleted.
STATS_HISTORY_RETENTION_DAYS = 30

# ==================================================================
# AIRCRAFT POSITION HISTORY (for per-chart history mode)
# ==================================================================
ENABLE_POS_HISTORY = True
POS_HISTORY_DIR = "pos_history"
POS_HISTORY_INTERVAL = 30
POS_HISTORY_RETENTION_DAYS = 7


# === Browser sound takeover ===
_browser_sound_mode = False
_browser_heartbeat_time = 0.0
_sound_event_queue = deque(maxlen=200)

SOUND_ATTR_TO_TYPE = {
    "sound_file": "new", "emergency_sound": "emergency", "loss_sound": "loss",
    "tcas_sound": "tcas", "low_alt_sound": "low_alt", "mil_sound": "mil",
    "tracked_sound": "tracked", "tracked_loss_sound": "tracked_loss",
    "record_sound": "record", "silence_sound": "silence", "turbulence_sound": "turbulence",
    "long_range_sound": "long_range", "overhead_sound": "overhead",
    "speed_record_sound": "speed_record", "alt_record_sound": "alt_record",
    "helicopter_sound": "helicopter", "rare_type_sound": "rare_type",
    "rapid_descent_sound": "rapid_descent", "return_sound": "return",
    "level_off_sound": "level_off", "ground_vehicle_sound": "ground_vehicle",
    "proximity_sound": "proximity", "headon_sound": "headon",
    "breakthrough_sound": "breakthrough", "squawk_change_sound": "squawk_change",
}

SOUND_TYPE_TO_FILE = {
    "new": "beep.wav", "emergency": "emergency.wav", "loss": "loss.wav",
    "tcas": "tcas.wav", "low_alt": "low_alt.wav", "mil": "mil.wav",
    "tracked": "tracked.wav", "tracked_loss": "tracked_loss.wav",
    "record": "record.wav", "silence": "silence.wav", "turbulence": "turbulence.wav",
    "squawk_7500": "squawk_7500.wav", "squawk_7600": "squawk_7600.wav", "squawk_7700": "squawk_7700.wav",
    "long_range": "long_range.wav", "overhead": "overhead.wav",
    "speed_record": "speed_record.wav", "alt_record": "alt_record.wav",
    "helicopter": "helicopter.wav", "rare_type": "rare_type.wav",
    "rapid_descent": "rapid_descent.wav", "return": "return.wav",
    "level_off": "level_off.wav", "ground_vehicle": "ground_vehicle.wav",
    "proximity": "proximity.wav", "headon": "headon.wav",
    "breakthrough": "breakthrough.wav", "squawk_change": "squawk_change.wav",
}

def _heartbeat_checker():
    global _browser_sound_mode, _browser_heartbeat_time
    while True:
        time.sleep(5)
        if _browser_sound_mode and (time.time() - _browser_heartbeat_time) > 15:
            _browser_sound_mode = False
            log.info("Browser heartbeat lost, reverting to server sound")

# Quiet mode (minimal output)
QUIET_MODE = False

# Version
__version__ = "0.1.0"

# Create directories if they don't exist
for _d in (SOUND_DIR, LOG_DIR, STATS_HISTORY_DIR, POS_HISTORY_DIR):
    os.makedirs(_d, exist_ok=True)

# Aircraft photo cache directory
PHOTO_CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos_cache")
os.makedirs(PHOTO_CACHE_DIR, exist_ok=True)
# Photo cache TTL, sec (7 days by default)
PHOTO_CACHE_TTL = 7 * 24 * 3600

# Signals by platform
if IS_WINDOWS:
    SIGNAL_RELOAD = signal.SIGTERM
else:
    SIGNAL_RELOAD = getattr(signal, "SIGUSR1", signal.SIGTERM)
# === END CROSSPLATFORM SUPPORT ===


# Metadata embedded into each WAV file.
#   Displayed in Audacity, ffprobe, Mediainfo and file properties.
WAV_AUTHOR_TAG    = "R4SCS"
WAV_SOFTWARE_TAG  = "DX1090"
WAV_COMMENT_TAG   = "github.com/r4scs"


# Stale, sec — aircraft shown in gray.
#   Range: 15-120.
STALE_TIMEOUT = 60

# Loss of contact, sec — aircraft removed + sound.
#   Range: 60-600.
LOSS_TIMEOUT = 180

# ==================================================================
# SOUND NOTIFICATIONS — 26 TYPES
# ==================================================================

# Master sound switch. False — fully disabled.
ENABLE_SOUND = True

# --- Basic events (11 types) ---

# New aircraft — 1000 Hz, 1 beep.
ENABLE_SOUND_NEW = True
# Emergency squawk 7700 — 1500 Hz, 5 beeps.
ENABLE_SOUND_EMERGENCY = True
# Loss of contact — 1000 Hz, 3 long beeps.
ENABLE_SOUND_LOSS = True
# TCAS RA — 2000 Hz, double.
ENABLE_SOUND_TCAS = True
# Low aircraft — 400->200 Hz hum.
ENABLE_SOUND_LOW_ALT = True
# Military aircraft — 600 Hz, double.
ENABLE_SOUND_MIL = True
# Tracked flight — 880->1320 Hz.
ENABLE_SOUND_TRACKED = True
# Range record — chime.
ENABLE_SOUND_RECORD = True
# Silence — 300 Hz, 1 beep.
ENABLE_SOUND_SILENCE = True
# Turbulence — 500 Hz, 3 beeps.
ENABLE_SOUND_TURB = True
# Emergency squawk 7500/7600/7700 — different tones.
ENABLE_SOUND_SQUAWK = True

# --- Extended events (14 types) ---

# Long-range aircraft — bass + ascending tone.
ENABLE_SOUND_LONG_RANGE = True
# Overhead pass — whistle 600->1200->600 Hz.
ENABLE_SOUND_OVERHEAD = True
# Speed record — ascending glissando.
ENABLE_SOUND_SPEED_RECORD = True
# Altitude record — high descending tone.
ENABLE_SOUND_ALT_RECORD = True
# Helicopter — rotor chopper.
ENABLE_SOUND_HELICOPTER = True
# Rare type — triad chord.
ENABLE_SOUND_RARE_TYPE = True
# Rapid descent — descending alarming tone.
ENABLE_SOUND_RAPID_DESCENT = True
# Aircraft return — boomerang.
ENABLE_SOUND_RETURN = True
# Level-off at cruise — ding.
ENABLE_SOUND_LEVEL_OFF = True
# Ground vehicle — roar.
ENABLE_SOUND_GROUND_VEHICLE = True
# Aircraft proximity — knock-knock.
ENABLE_SOUND_PROXIMITY = True
# Head-on courses — opposing whistle.
ENABLE_SOUND_HEADON = True
# End of silence — ping.
ENABLE_SOUND_BREAKTHROUGH = True
# Squawk change — short beep.
ENABLE_SOUND_SQUAWK_CHANGE = True


# ==================================================================
# SOUND TYPES LIST FOR DASHBOARD
# ==================================================================
SOUND_TYPES = {
    "new":            ("New aircraft",        "ENABLE_SOUND_NEW"),
    "emergency":      ("Emergency (7700)",     "ENABLE_SOUND_EMERGENCY"),
    "loss":           ("Loss of contact",     "ENABLE_SOUND_LOSS"),
    "tcas":           ("TCAS RA",          "ENABLE_SOUND_TCAS"),
    "low_alt":        ("Low aircraft",      "ENABLE_SOUND_LOW_ALT"),
    "mil":            ("Military",          "ENABLE_SOUND_MIL"),
    "tracked":        ("Tracked",    "ENABLE_SOUND_TRACKED"),
    "tracked_loss":   ("Tracked loss",   "ENABLE_SOUND_TRACKED_LOSS"),
    "record":         ("Range record", "ENABLE_SOUND_RECORD"),
    "silence":        ("Silence",           "ENABLE_SOUND_SILENCE"),
    "turbulence":     ("Turbulence",   "ENABLE_SOUND_TURB"),
    "squawk":         ("Squawk emerg.",    "ENABLE_SOUND_SQUAWK"),
    "long_range":     ("Long range",    "ENABLE_SOUND_LONG_RANGE"),
    "overhead":       ("Overhead",     "ENABLE_SOUND_OVERHEAD"),
    "speed_record":   ("Speed record", "ENABLE_SOUND_SPEED_RECORD"),
    "alt_record":     ("Altitude record",   "ENABLE_SOUND_ALT_RECORD"),
    "helicopter":     ("Helicopter",        "ENABLE_SOUND_HELICOPTER"),
    "rare_type":      ("Rare type",      "ENABLE_SOUND_RARE_TYPE"),
    "rapid_descent":  ("Rapid descent",  "ENABLE_SOUND_RAPID_DESCENT"),
    "return":         ("Return",     "ENABLE_SOUND_RETURN"),
    "level_off":      ("Level-off",    "ENABLE_SOUND_LEVEL_OFF"),
    "ground_vehicle": ("Ground vehicle", "ENABLE_SOUND_GROUND_VEHICLE"),
    "proximity":      ("Proximity",       "ENABLE_SOUND_PROXIMITY"),
    "headon":         ("Head-on", "ENABLE_SOUND_HEADON"),
    "breakthrough":   ("End of silence",    "ENABLE_SOUND_BREAKTHROUGH"),
    "squawk_change":  ("Squawk change",    "ENABLE_SOUND_SQUAWK_CHANGE"),
}

# --- Thresholds for extended sounds ---

# Long-range aircraft threshold, km (50-500).
LONG_RANGE_THRESHOLD_KM = 300
# "Overhead" threshold, km (1-20).
OVERHEAD_THRESHOLD_KM = 5
# Speed record threshold, knots (300-700).
SPEED_RECORD_THRESHOLD = 500
# Altitude record threshold, feet (30000-51000).
ALT_RECORD_THRESHOLD = 40000
# Rapid descent threshold, fpm (2000-6000).
RAPID_DESCENT_FPM = 3000
# Level-off threshold, fpm (100-500). VR below this after climb/descent.
LEVEL_OFF_FPM = 200
# Minimum vertical rate before level-off, fpm (300-2000).
LEVEL_OFF_MIN_VRATE = 500
# Horizontal proximity, km (3-20).
PROXIMITY_HORIZ_KM = 10
# Vertical proximity, feet (500-3000).
PROXIMITY_VERT_FT = 1000
# Course difference for head-on, deg (90-180).
HEADON_COURSE_DIFF = 120
# Distance for head-on, km (10-50).
HEADON_DIST_KM = 30
# Remember aircraft after loss, sec (60-1800).
RETURN_MEMORY_TIME = 600
# Rare aircraft types for separate notification.
RARE_TYPES = {"A388", "B744", "B748", "A359", "A35K", "AN124", "A225", "CONC"}

# --- Thresholds for basic sounds ---

# Low aircraft threshold, meters (500-3000).
LOW_ALT_THRESHOLD = 1000
# Silence timeout, sec (60-600).
SILENCE_TIMEOUT = 300
# ADS-B categories considered military.
#   {"A6","A7"} — heavy military + helicopters
MIL_CATEGORIES = {"A6", "A7"}

# Tracked flights (callsigns). Example: ["AFL123", "ABC456"]
TRACKED_CALLSIGNS = []
# Tracked ICAO (hex, 6 characters). Example: ["4A3B2C"]
TRACKED_ICAO = []
# Sound when tracked flight disappears.
ENABLE_SOUND_TRACKED_LOSS = True

# ==================================================================
# AUTO PPM DETECTION
# ==================================================================

# Auto-calibrate PPM using aircraft dead-reckon positions.
ENABLE_PPM_AUTO = True

# Check interval, sec (10-300).
PPM_AUTO_INTERVAL = 60

# Minimum aircraft with positions for calculation (3-30).
PPM_AUTO_MIN_SAMPLES = 10

# Minimum valid aircraft positions (1-10). Uses max(PPM_AUTO_MIN_VALID, POSITION_RELIABLE_MIN).
PPM_AUTO_MIN_VALID = 3

# Maximum correction per step, ppm (1-20).
PPM_AUTO_MAX_ADJUST = 5

# Tolerance offset, km (0.1-5.0).
PPM_AUTO_TOLERANCE_KM = 0.5

# Smoothing (0 = instant, 1 = no change).
PPM_AUTO_SMOOTHING = 0.3

# Maximum position age for auto-PPM, sec (5-30).
PPM_AUTO_MAX_POS_AGE = 10

# Restart SDR on PPM change.
PPM_AUTO_RESTART_SDR = True

# ==================================================================
# SESSION SAVE AND RESTORE
# ==================================================================

# Save aircraft database, statistics and PPM to JSON on exit.
ENABLE_SESSION_SAVE = True
# Restore on next launch.
ENABLE_SESSION_RESTORE = True
# Session file name.
SESSION_FILE = "session.json"
# Do not restore aircraft older than N sec (300-7200).
SESSION_MAX_AGE = 3600
# Save statistics.
SESSION_SAVE_STATS = True
# Save PPM.
SESSION_SAVE_PPM = True

# ==================================================================
# EXPORT AND FILTERING
# ==================================================================

# CSV export on 'e' key.
ENABLE_CSV_EXPORT = True
# AeroXML export on 'x' key.
ENABLE_AEROXML_EXPORT = True
# FlightAware export on 'a' key.
ENABLE_FLIGHTAWARE_EXPORT = True

# Default table sort.
#   "icao", "callsign", "altitude", "speed", "distance", "age"
SORT_BY = "distance"

# Emergency aircraft only (7500/7600/7700).
FILTER_EMERGENCY_ONLY = False

# Sort cycle on 's' key.
SORT_CYCLE = ["icao", "callsign", "altitude", "speed", "distance", "age"]

# ==================================================================
# AIRCRAFT DATABASE (aircraft.csv — OpenSky format)
# ==================================================================

# Database file. Download: wget https://s3.opensky-network.org/data-samples/metadata/aircraft-database-complete-2025-02.csv -O aircraft.csv

AIRCRAFT_DB_FILE = "aircraft.csv"

# Enable database lookup.
ENABLE_AIRCRAFT_LOOKUP = True

# Fields to extract.
LOOKUP_REGISTRATION  = True
LOOKUP_TYPECODE      = True
LOOKUP_OPERATOR      = True
LOOKUP_MODEL         = True
LOOKUP_MANUFACTURER  = True
LOOKUP_OWNER         = True
LOOKUP_COUNTRY       = True
LOOKUP_ENGINES       = True

# ==================================================================
# DECODE THREAD
# ==================================================================

# Separate thread for decoding (parallel with reception).
#   True  — DecodeWorker in background, reception not blocked
#   False — decoding in main thread
ENABLE_DECODE_THREAD = True

# Decode queue size. On overflow, messages are lost.
#   Range: 500-10000.
DECODE_QUEUE_SIZE = 2000

# ==================================================================
# NETWORK PROTOCOLS — OUTPUT
# ==================================================================

# Beast — binary, for dump1090 / piaware.
ENABLE_BEAST_OUT = False
BEAST_OUT_HOST = "127.0.0.1"
BEAST_OUT_PORT = 30005

# JSON API — HTTP, dump1090 /data.json format.
ENABLE_JSON_OUT = True
JSON_OUT_HOST = "127.0.0.1"
JSON_OUT_PORT = 8080

# SBS1 — text, for BaseStation / VRS.
ENABLE_SBS1_OUT = False
SBS1_OUT_HOST = "127.0.0.1"
SBS1_OUT_PORT = 30003

# ==================================================================
# SDR WATCHDOG
# ==================================================================

# Automatic SDR restart on process crash.
#   True  — SDR process is restarted on termination
#   False — program exits on SDR error
ENABLE_SDR_WATCHDOG = True

# Data wait timeout, sec. No data -> restart.
#   Range: 30-300.
SDR_WATCHDOG_TIMEOUT = 60

# Maximum restart attempts. After -> exit.
#   Range: 1-20.
SDR_WATCHDOG_MAX_RETRIES = 10

# ==================================================================
# TABLE COLUMNS
# ==================================================================

# List of columns to display.
# Available keys: icao, callsign, reg, type, model, manufacturer,
# operator, owner, country, category, engines, altitude, speed,
# speed_ias, speed_tas, heading, vrate, vr_src, lat, lon, ground,
# distance, phase, autopilot, gnss, sel_alt, sel_hdg, qnh, nac,
# sil, nacv, version, nic, emergency, age, squawk, weather, hazard,
# tis_b, rssi, reliable
COL_SHOW = [
    "icao", "callsign", "reg", "type",
    "altitude", "speed", "heading",
    "lat", "lon", "emergency", "squawk",
    "distance", "weather", "nic",
]

LOG_COL_MAP = {
    "[CALL]":  ["callsign", "category"],
    "[POS]":   ["lat", "lon", "altitude", "distance"],
    "[GND]":   ["lat", "lon", "ground", "speed", "distance"],
    "[VEL]":   ["speed", "speed_ias", "speed_tas", "heading", "vrate", "vr_src", "nacv"],
    "[GNSS]":  ["gnss"],
    "[EMER]":  ["emergency"],
    "[TC29]":  ["sel_alt", "sel_hdg", "qnh", "autopilot"],
    "[TC31]":  ["nac", "sil", "version"],
    "[NIC]":   ["nic"],
    "[DF0]":   ["altitude"],
    "[DF5]":   ["squawk"],
    "[DF18]":  ["tis_b"],
    "[DF19]":  [], "[DF24]": [], "[DF25]": [],
    "[BDS10]": [], "[BDS17]": [],
    "[BDS20]": ["callsign"],
    "[BDS30]": [],
    "[BDS44]": ["weather"],
    "[BDS45]": ["hazard"],
    "[BDS40]": ["sel_alt", "sel_hdg", "qnh"],
    "[BDS50]": ["speed_tas", "heading"],
    "[BDS60]": ["speed_ias", "speed_tas", "vrate", "heading"],
    "[MAC]":   ["squawk", "altitude"],
    "[NEW]":      ["icao", "callsign"],
    "[TRACKING]":    ["callsign"],
    "[MIL]":        ["category"],
    "[LOW]":      ["altitude"],
    "[TCAS RA]":    [],
    "[RECORD]":     ["distance"],
    "[TURB]":       ["hazard"],
    "[SILENCE]":     [],
    "[LOSS]":     ["icao"],
    "[SQUAWK 7500]": ["squawk"],
    "[SQUAWK 7600]": ["squawk"],
    "[SQUAWK 7700]": ["squawk"],
}

# ==================================================================
# PROTOCOL CONSTANTS
# ==================================================================

SAMPLES_PER_BIT = SAMPLE_RATE / 1000000.0

def _build_preamble_positions(spb):
    high = [int(round(p * spb)) for p in [0.0, 1.0, 3.5, 4.5]]
    low = [int(round(p * spb)) for p in [0.5, 1.5, 2.0, 2.5, 3.0, 4.0]]
    return high, low

PREAMBLE_HIGH, PREAMBLE_LOW = _build_preamble_positions(SAMPLES_PER_BIT)

MODE_AC_DATA_OFFSET = int(round(11 * SAMPLES_PER_BIT))
MODE_AC_BIT_COUNT = 20
MODE_AC_MSG_SAMPLES = MODE_AC_DATA_OFFSET + int(round(MODE_AC_BIT_COUNT * SAMPLES_PER_BIT))

NZ = 15
DLAT_EVEN = 360.0 / (4 * NZ)
DLAT_ODD = 360.0 / (4 * NZ - 1)
CPR_TIMEOUT = 10.0
AIRBORN_CPR_MAX = 131072.0
GROUND_CPR_MAX = 131072.0

SQUAWK_SPECIAL = {"7500": "HIJACK!", "7600": "RADIO FAIL", "7700": "EMERGENCY!"}
CHARS = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######"

AC_CATEGORY = {i: f"{'ABCD'[i//8]}{i%8}" for i in range(32)}
AC_CAT_DESC = {
    "A0": "No category info",
    "A1": "Light aircraft (<5.7t)", "A2": "Medium aircraft (5.7-27t)",
    "A3": "Medium aircraft (27-272t)", "A4": "High vortex (e.g. B757)",
    "A5": "Heavy aircraft (>136t)", "A6": "High performance / high speed",
    "A7": "Rotorcraft / helicopter",
    "B0": "No category info", "B1": "Glider", "B2": "Balloon / airship",
    "B3": "Ultralight / paraglider", "B4": "UAV / drone",
    "B5": "UAV / drone", "B6": "Space vehicle", "B7": "Surface vehicle",
    "C0": "No category info", "C1": "Surface emergency vehicle",
    "C2": "Surface service vehicle", "C3": "Fixed ground obstacle",
    "D0": "No category info (TC=4)", "D1": "Reserved", "D2": "Reserved",
    "D3": "Reserved", "D4": "Reserved", "D5": "Reserved",
    "D6": "Reserved", "D7": "Reserved",
}

# --- Decoded field validation ---
MIN_CRUISE_ALT_FT = 10000  # above this — cruise flight
MIN_CRUISE_SPD_KT = 100    # min speed at cruise level
MAX_SUBSONIC_KT = 700      # max subsonic speed
MAX_VRATE_FPM = 12000      # max vertical rate
MAX_HEADING_JUMP_DEG = 90   # max heading jump
MAX_ALT_FT = 70000          # max altitude (military ~70000)
MIN_ALT_FT = -1200          # min altitude
MAX_IAS_TAS_KT = 700       # max IAS/TAS
MAX_TURN_RATE_DEG = 10.0    # max turn rate
MIN_QNH_HPA = 800           # min barometric pressure
MAX_QNH_HPA = 1100          # max barometric pressure
MAX_SEL_ALT_FT = 60000      # max selected altitude
MIN_SEL_ALT_FT = -1200      # min selected altitude
MAX_GNSS_ALT_FT = 70000    # max GNSS altitude

EMERGENCY_TIMEOUT = 60
EMERGENCY = {0: "No emergency", 1: "General emergency", 2: "Lifeguard",
             3: "Minimum fuel", 4: "Unlawful interference",
             5: "Hijack", 6: "Downed", 7: "(reserved)"}
AUTOPILOT_MODE = {0: "Unknown", 1: "VNAV", 2: "ALT HOLD", 3: "VNAV/ALT HOLD"}

NAC_P = {0: ">10 NM", 1: "<10 NM", 2: "<4 NM", 3: "<2 NM",
         4: "<1 NM", 5: "<0.5 NM", 6: "<0.3 NM", 7: "<0.1 NM",
         8: "<0.05 NM", 9: "<30 m", 10: "<10 m", 11: "<3 m"}

NIC_TABLE = {
    (0, 0): {0: ">20 NM", 1: "<20 NM", 2: "<8 NM", 3: "<4 NM",
             4: "<2 NM", 5: "<1 NM", 6: "<0.6 NM", 7: "<0.2 NM",
             8: "<0.1 NM", 9: "<0.05 NM", 10: "<30 m", 11: "<10 m"},
    (1, 0): {0: ">20 NM", 1: "<20 NM", 2: "<8 NM", 3: "<4 NM",
             4: "<2 NM", 5: "<1 NM", 6: "<0.6 NM", 7: "<0.2 NM",
             8: "<0.1 NM", 9: "<0.05 NM", 10: "<30 m", 11: "<10 m"},
    (1, 1): {9: "<3 m"},
    (2, 0): {0: ">20 NM", 1: "<20 NM", 2: "<8 NM", 3: "<4 NM",
             4: "<2 NM", 5: "<1 NM", 6: "<0.6 NM", 7: "<0.2 NM",
             8: "<0.1 NM", 9: "<0.05 NM", 10: "<30 m", 11: "<10 m"},
    (2, 1): {6: "<0.3 NM", 7: "<0.1 NM", 9: "<3 m", 10: "<1 m"},
}

SIL_TABLE = {0: ">1e-3", 1: "<1e-5", 2: "<1e-7", 3: "(reserved)"}
NAC_V = {0: ">10 m/s", 1: "<10 m/s", 2: "<3 m/s", 3: "<1 m/s"}
ADSB_VERSION = {0: "DO-260 (v0)", 1: "DO-260A (v1)", 2: "DO-260B (v2)"}

def _build_ground_speed_table():
    """Ground speed table for surface position.
    Specification: DO-260B / mode-s.org Table 6.2.
    MOV 0 = no data, 1 = stopped, 124 = >=175 kt."""
    table = [0.0] * 125
    for i in range(2, 9): table[i] = 0.125 * (i - 1)
    for i in range(9, 13): table[i] = 1.0 + 0.25 * (i - 9)
    for i in range(13, 39): table[i] = 2.0 + 0.5 * (i - 13)
    for i in range(39, 94): table[i] = 15.0 + 1.0 * (i - 39)
    for i in range(94, 109): table[i] = 70.0 + 2.0 * (i - 94)
    for i in range(109, 124): table[i] = 100.0 + 5.0 * (i - 109)
    table[124] = 175.0
    return table

GROUND_SPEED_TABLE = _build_ground_speed_table()

def _find_gain_index(gain):
    if gain in GAIN_STEPS: return GAIN_STEPS.index(gain)
    best_idx, best_diff = 0, abs(GAIN_STEPS[0] - gain)
    for i, g in enumerate(GAIN_STEPS[1:], 1):
        d = abs(g - gain)
        if d < best_diff: best_diff, best_idx = d, i
    return best_idx

PREAMBLE_SAMPLES = int(round(8 * SAMPLES_PER_BIT))
LONG_MSG_SAMPLES = PREAMBLE_SAMPLES + int(round(112 * SAMPLES_PER_BIT))
SHORT_MSG_SAMPLES = PREAMBLE_SAMPLES + int(round(56 * SAMPLES_PER_BIT))
MSG_SAMPLES = LONG_MSG_SAMPLES
OVERLAP = MSG_SAMPLES
MIN_PREAMBLE_SPACING = int(round(60 * SAMPLES_PER_BIT))

TURBULENCE_LEVEL = {0: "None", 1: "Light", 2: "Moderate", 3: "Severe",
                     4: "Very severe", 5: "Extreme", 6: "(reserved)", 7: "(reserved)"}
ICING_LEVEL = {0: "None", 1: "Light", 2: "Moderate", 3: "Severe"}
WINDSHEAR_LEVEL = {0: "None", 1: "Light", 2: "Moderate", 3: "Severe",
                    4: "Very severe", 5: "Extreme", 6: "(reserved)", 7: "(reserved)"}

NIC_AIRBORNE_BASE = {9: 11, 10: 10, 11: 9, 12: 8, 13: 7, 14: 6, 15: 5, 16: 4, 17: 3, 18: 2}
NIC_SURFACE_BASE = {5: 11, 6: 10, 7: 9, 8: 8}
NIC_RADIUS = {0: ">20 NM", 1: "<20 NM", 2: "<8 NM", 3: "<4 NM",
              4: "<2 NM", 5: "<1 NM", 6: "<0.6 NM", 7: "<0.2 NM",
              8: "<0.1 NM", 9: "<0.05 NM", 10: "<30 m", 11: "<10 m"}

# ==================================================================
# INTERNAL PARAMETERS COMPUTATION
# ==================================================================

# ==================================================================
# THRESHOLD INPUT UNITS
# ==================================================================

# Distance threshold input units in settings:
#   "km" — kilometers, "mi" — miles, "nm" — nautical miles
UNIT_INPUT_DIST = "km"

# Altitude threshold input units:
#   "ft" — feet, "m" — meters
UNIT_INPUT_ALT = "ft"

# Speed threshold input units:
#   "kt" — knots, "kmh" — km/h, "mph" — mph
UNIT_INPUT_SPEED = "kt"

# Vertical rate threshold input units:
#   "fpm" — feet/min, "ms" — m/s
UNIT_INPUT_VRATE = "fpm"

# --- Input conversion functions to internal units ---
def _conv_dist_input(v):
    """Convert from UNIT_INPUT_DIST to km."""
    if UNIT_INPUT_DIST == "mi": return v / 0.621371
    if UNIT_INPUT_DIST == "nm": return v / 0.539957
    return v

def _conv_alt_input(v):
    """Convert from UNIT_INPUT_ALT to feet."""
    if UNIT_INPUT_ALT == "m": return v * 3.28084
    return v

def _conv_speed_input(v):
    """Convert from UNIT_INPUT_SPEED to knots."""
    if UNIT_INPUT_SPEED == "kmh": return v / 1.852
    if UNIT_INPUT_SPEED == "mph": return v / 1.15078
    return v

def _conv_vrate_input(v):
    """Convert from UNIT_INPUT_VRATE to fpm."""
    if UNIT_INPUT_VRATE == "ms": return v * 196.85
    return v

# --- Convert all thresholds to internal units ---
MAX_RANGE_KM              = _conv_dist_input(MAX_RANGE_KM)
MAX_POSITION_JUMP_KM      = _conv_dist_input(MAX_POSITION_JUMP_KM)
LONG_RANGE_THRESHOLD_KM   = _conv_dist_input(LONG_RANGE_THRESHOLD_KM)
OVERHEAD_THRESHOLD_KM     = _conv_dist_input(OVERHEAD_THRESHOLD_KM)
PROXIMITY_HORIZ_KM        = _conv_dist_input(PROXIMITY_HORIZ_KM)
HEADON_DIST_KM            = _conv_dist_input(HEADON_DIST_KM)
PPM_AUTO_TOLERANCE_KM     = _conv_dist_input(PPM_AUTO_TOLERANCE_KM)

LOW_ALT_THRESHOLD_FT      = _conv_alt_input(LOW_ALT_THRESHOLD)
PROXIMITY_VERT_FT         = _conv_alt_input(PROXIMITY_VERT_FT)

SPEED_RECORD_THRESHOLD_KT = _conv_speed_input(SPEED_RECORD_THRESHOLD)
RAPID_DESCENT_FPM         = _conv_vrate_input(RAPID_DESCENT_FPM)
MAX_ALT_RATE_FPM          = _conv_vrate_input(MAX_ALT_RATE_FPM)

# Convert filter radius to kilometers for internal use.
if FILTER_RADIUS_UNIT == "mi":
    FILTER_RADIUS_KM = FILTER_RADIUS / 0.621371 if FILTER_RADIUS > 0 else 0
elif FILTER_RADIUS_UNIT == "nm":
    FILTER_RADIUS_KM = FILTER_RADIUS / 0.539957 if FILTER_RADIUS > 0 else 0
else:
    FILTER_RADIUS_KM = FILTER_RADIUS

# ==================================================================
# AIRCRAFT DATABASE LOADING
# ==================================================================

def load_aircraft_db():
    db = {}
    if not ENABLE_AIRCRAFT_LOOKUP or not os.path.exists(AIRCRAFT_DB_FILE):
        return db
    try:
        # Count total lines for progress bar
        total_lines = 0
        with open(AIRCRAFT_DB_FILE, 'r', encoding='utf-8', errors='replace') as f:
            for _ in f:
                total_lines += 1
        print("Loading aircraft database...")
        bar_width = 40
        loaded = 0
        with open(AIRCRAFT_DB_FILE, 'r', encoding='utf-8', errors='replace') as f:
            reader = csv.DictReader(f, quotechar="'")
            for row in reader:
                loaded += 1
                if loaded % 5000 == 0 or loaded == total_lines:
                    pct = min(loaded / total_lines * 100, 100.0)
                    filled = int(bar_width * pct / 100)
                    bar = "\u2588" * filled + "\u2591" * (bar_width - filled)
                    sys.stderr.write(f"\r  {bar} {pct:5.1f}%  ({loaded}/{total_lines})")
                    sys.stderr.flush()
                icao = (row.get("icao24") or "").strip().upper()
                if len(icao) != 6 or icao == "000000": continue
                entry = {}
                if LOOKUP_REGISTRATION: entry["reg"] = (row.get("registration") or "").strip()
                if LOOKUP_TYPECODE: entry["type"] = (row.get("typecode") or "").strip()
                if LOOKUP_OPERATOR: entry["operator"] = (row.get("operator") or "").strip()
                if LOOKUP_MODEL: entry["model"] = (row.get("model") or "").strip()
                if LOOKUP_MANUFACTURER:
                    entry["manufacturer"] = (row.get("manufacturerName") or "").strip()
                    if not entry["manufacturer"]:
                        entry["manufacturer"] = (row.get("manufacturerIcao") or "").strip()
                if LOOKUP_OWNER: entry["owner"] = (row.get("owner") or "").strip()
                if LOOKUP_COUNTRY:
                    entry["country"] = (row.get("country") or "").strip()
                    entry["flag"] = country_to_flag(entry["country"])
                if LOOKUP_ENGINES: entry["engines"] = (row.get("engines") or "").strip()
                if any(v for v in entry.values()): db[icao] = entry
        sys.stderr.write(f"\r  {bar_width * chr(9608)} 100.0%  ({total_lines}/{total_lines}) \u2014 {len(db)} entries loaded\n")
        sys.stderr.flush()
    except Exception as e:
        sys.stderr.write(f"\n[aircraft-db] {e}\n")
    return db

# Country name → ISO 2-letter code → flag emoji
_COUNTRY_CODE = {
    "Russia":"RU","China":"CN","United States":"US","United Kingdom":"UK",
    "Germany":"DE","France":"FR","Japan":"JP","Turkey":"TR",
    "United Arab Emirates":"AE","South Korea":"KR","India":"IN",
    "Brazil":"BR","Australia":"AU","Canada":"CA","Netherlands":"NL",
    "Spain":"ES","Italy":"IT","Switzerland":"CH","Sweden":"SE",
    "Norway":"NO","Finland":"FI","Denmark":"DK","Poland":"PL",
    "Belgium":"BE","Austria":"AT","Portugal":"PT","Greece":"GR",
    "Ireland":"IE","Luxembourg":"LU","Malta":"MT","Cyprus":"CY",
    "Czech Republic":"CZ","Hungary":"HU","Romania":"RO","Bulgaria":"BG",
    "Serbia":"RS","Croatia":"HR","Slovenia":"SI","Slovakia":"SK",
    "Ukraine":"UA","Belarus":"BY","Kazakhstan":"KZ","Uzbekistan":"UZ",
    "Azerbaijan":"AZ","Armenia":"AM","Georgia":"GE","Moldova":"MD",
    "Lithuania":"LT","Latvia":"LV","Estonia":"EE","Iceland":"IS",
    "Saudi Arabia":"SA","Qatar":"QA","Oman":"OM","Kuwait":"KW",
    "Bahrain":"BH","Israel":"IL","Egypt":"EG","Morocco":"MA",
    "South Africa":"ZA","Nigeria":"NG","Kenya":"KE","Ethiopia":"ET",
    "Singapore":"SG","Malaysia":"MY","Thailand":"TH","Indonesia":"ID",
    "Philippines":"PH","Vietnam":"VN","Pakistan":"PK","Bangladesh":"BD",
    "Sri Lanka":"LK","Nepal":"NP","Mongolia":"MN","Taiwan":"TW",
    "Hong Kong":"HK","Macau":"MO","New Zealand":"NZ","Mexico":"MX",
    "Argentina":"AR","Chile":"CL","Colombia":"CO","Peru":"PE",
    "Venezuela":"VE","Cuba":"CU","Dominican Republic":"DO","Panama":"PA",
    "Lebanon":"LB","Jordan":"JO","Iraq":"IQ","Iran":"IR","Afghanistan":"AF",
    "Algeria":"DZ","Tunisia":"TN","Libya":"LY","Sudan":"SD","Ghana":"GH",
    "Ivory Coast":"CI","Senegal":"SN","Cameroon":"CM","Uganda":"UG",
    "Tanzania":"TZ","Mauritius":"MU","Reunion":"RE","Fiji":"FJ",
    "Papua New Guinea":"PG","Bolivia":"BO","Ecuador":"EC","Uruguay":"UY",
    "Paraguay":"PY","Costa Rica":"CR","Guatemala":"GT","Honduras":"HN",
    "El Salvador":"SV","Nicaragua":"NI","Jamaica":"JM","Bahamas":"BS",
    "Trinidad and Tobago":"TT","Barbados":"BB","Bermuda":"BM",
    "Cayman Islands":"KY","Isle of Man":"IM","Guernsey":"GG",
    "Jersey":"JE","Gibraltar":"GI","Andorra":"AD","Monaco":"MC",
    "Liechtenstein":"LI","San Marino":"SM","Vatican":"VA",
    "Faroe Islands":"FO","Greenland":"GL","Antarctica":"AQ",
    "Aruba":"AW","Curacao":"CW","Suriname":"SR","Guyana":"GY",
    "Macedonia":"MK","Albania":"AL","Bosnia and Herzegovina":"BA",
    "Montenegro":"ME","Kosovo":"XK","Tajikistan":"TJ","Kyrgyzstan":"KG",
    "Turkmenistan":"TM","Syria":"SY","Yemen":"YE","Somalia":"SO",
    "Djibouti":"DJ","Eritrea":"ER","Rwanda":"RW","Burundi":"BI",
    "Mozambique":"MZ","Angola":"AO","Zambia":"ZM","Zimbabwe":"ZW",
    "Namibia":"NA","Botswana":"BW","Lesotho":"LS","Eswatini":"SZ",
    "Madagascar":"MG","Seychelles":"SC","Comoros":"KM","Cape Verde":"CV",
    "Guinea":"GN","Mali":"ML","Burkina Faso":"BF","Niger":"NE",
    "Chad":"TD","Central African Republic":"CF","Equatorial Guinea":"GQ",
    "Gabon":"GA","Congo":"CG","DR Congo":"CD","South Sudan":"SS",
    "Bhutan":"BT","Maldives":"MV","Brunei":"BN","Cambodia":"KH",
    "Laos":"LA","Myanmar":"MM","Timor-Leste":"TL","Solomon Islands":"SB",
    "Vanuatu":"VU","Samoa":"WS","Tonga":"TO","Kiribati":"KI",
    "Palau":"PW","Marshall Islands":"MH","Micronesia":"FM","Nauru":"NR",
    "Tuvalu":"TV","Cook Islands":"CK","Niue":"NU","Tokelau":"TK",
    "Russian Federation":"RU","Russian Fed.":"RU",
    "United States of America":"US","USA":"US",
    "Great Britain":"UK","Britain":"UK","England":"UK",
    "Czechia":"CZ","Czech Rep.":"CZ",
    "Republic of Korea":"KR",
    "Republic of Ireland":"IE",
    "Slovak Republic":"SK",
    "North Macedonia":"MK","Republic of North Macedonia":"MK",
    "Republic of Kosovo":"XK",
    "Republic of Moldova":"MD",
    "Republic of Belarus":"BY",
    "People's Republic of China":"CN","PRC":"CN",
    "Republic of China":"TW","Chinese Taipei":"TW",
    "Viet Nam":"VN","Burma":"MM",
    "Federal Republic of Germany":"DE","Deutschland":"DE",
    "French Republic":"FR","Italian Republic":"IT","Italia":"IT",
    "Kingdom of Spain":"ES","Espana":"ES",
    "Kingdom of the Netherlands":"NL","The Netherlands":"NL",
    "Republic of Austria":"AT","Swiss Confederation":"CH","Suisse":"CH",
    "Kingdom of Sweden":"SE","Sverige":"SE",
    "Kingdom of Norway":"NO","Norge":"NO",
    "Republic of Finland":"FI","Suomi":"FI",
    "Kingdom of Denmark":"DK","Danmark":"DK",
    "Republic of Poland":"PL","Polska":"PL",
    "Republic of Hungary":"HU",
    "Republic of Bulgaria":"BG","Hellenic Republic":"GR",
    "Republic of Portugal":"PT",
    "Republic of Turkey":"TR","Turkiye":"TR",
    "State of Israel":"IL","Kingdom of Saudi Arabia":"SA",
    "State of Qatar":"QA","Sultanate of Oman":"OM",
    "State of Kuwait":"KW","Kingdom of Bahrain":"BH",
    "UAE":"AE","Islamic Republic of Iran":"IR",
    "Republic of Iraq":"IQ","Hashemite Kingdom of Jordan":"JO",
    "Republic of Lebanon":"LB","Syrian Arab Republic":"SY",
    "Republic of Yemen":"YE","Islamic Republic of Pakistan":"PK",
    "Republic of India":"IN","Bharat":"IN",
    "Republic of Singapore":"SG","Kingdom of Thailand":"TH",
    "Republic of Kazakhstan":"KZ","Republic of Uzbekistan":"UZ",
    "Republic of Azerbaijan":"AZ","Republic of Armenia":"AM",
    "Cabo Verde":"CV","Republic of Cabo Verde":"CV",
    "Federative Republic of Brazil":"BR","Argentine Republic":"AR",
    "United Mexican States":"MX","Republic of the Philippines":"PH",
    "Republic of Indonesia":"ID",
}
def country_to_flag(name):
    """Convert country name to flag emoji, or return name if unknown."""
    if not name: return ""
    name = name.strip()
    code = _COUNTRY_CODE.get(name) or _COUNTRY_CODE.get(name.title())
    if not code or len(code) != 2:
        nl = name.lower()
        for k, v in _COUNTRY_CODE.items():
            if k.lower() == nl:
                code = v
                break
    if not code or len(code) != 2: return name
    return chr(0x1F1E6 + ord(code[0]) - ord('A')) + chr(0x1F1E6 + ord(code[1]) - ord('A'))

AIRCRAFT_INFO = load_aircraft_db()

# ==================================================================
# UNIT CONVERSION
# ==================================================================

def conv_alt(ft):
    if ft is None: return None
    return ft * 0.3048 if UNIT_ALT == "m" else ft

def fmt_alt(ft):
    v = conv_alt(ft)
    return "—" if v is None else (f"{v:.0f} m" if UNIT_ALT == "m" else f"{v:.0f} ft")

def conv_speed(kt):
    if kt is None: return None
    if UNIT_SPEED == "kmh": return kt * 1.852
    if UNIT_SPEED == "mph": return kt * 1.15078
    if UNIT_SPEED == "ms": return kt * 0.514444
    return kt

def fmt_speed(kt):
    v = conv_speed(kt)
    if v is None: return "—"
    if UNIT_SPEED == "kmh": return f"{v:.0f} km/h"
    if UNIT_SPEED == "mph": return f"{v:.0f} mph"
    if UNIT_SPEED == "ms": return f"{v:.0f} m/s"
    return f"{v:.0f} kt"

def conv_distance(km):
    if km is None: return None
    if UNIT_DISTANCE == "mi": return km * 0.621371
    if UNIT_DISTANCE == "nm": return km * 0.539957
    return km

def fmt_distance(km):
    v = conv_distance(km)
    if v is None: return "—"
    if UNIT_DISTANCE == "mi": return f"{v:.0f} mi"
    if UNIT_DISTANCE == "nm": return f"{v:.0f} nm"
    return f"{v:.0f} km"

def conv_radius(km):
    if km is None: return None
    if FILTER_RADIUS_UNIT == "mi": return km * 0.621371
    if FILTER_RADIUS_UNIT == "nm": return km * 0.539957
    return km

def fmt_radius(km):
    v = conv_radius(km)
    if v is None: return "—"
    if FILTER_RADIUS_UNIT == "mi": return f"{v:.0f} mi"
    if FILTER_RADIUS_UNIT == "nm": return f"{v:.0f} nm"
    return f"{v:.0f} km"

def radius_to_km(value):
    if value is None: return None
    if FILTER_RADIUS_UNIT == "mi": return value / 0.621371
    if FILTER_RADIUS_UNIT == "nm": return value / 0.539957
    return value

def conv_vrate(fpm):
    if fpm is None: return None
    return fpm * 0.00508 if UNIT_VRATE == "ms" else fpm

def fmt_vrate(fpm):
    v = conv_vrate(fpm)
    if v is None: return "—"
    return f"{v:+.1f} m/s" if UNIT_VRATE == "ms" else f"{v:+.0f} fpm"

def get_display_speed(ac_data):
    if SPEED_SOURCE == "IAS":
        v = ac_data.get("speed_ias")
        if v is not None: return v
        return ac_data.get("speed_gs") or ac_data.get("speed_gs_bds50") or ac_data.get("speed_tas")
    if SPEED_SOURCE == "TAS":
        v = ac_data.get("speed_tas")
        if v is not None: return v
        return ac_data.get("speed_gs") or ac_data.get("speed_gs_bds50") or ac_data.get("speed_ias")
    v = ac_data.get("speed_gs")
    if v is not None: return v
    v = ac_data.get("speed_gs_bds50")
    if v is not None: return v
    return ac_data.get("speed_ias") or ac_data.get("speed_tas")

def get_display_vrate(ac_data):
    if VRATE_SOURCE == "BDS60":
        v = ac_data.get("vrate_bds60")
        if v is not None: return v
    return ac_data.get("vrate_tc19")

def conv_pressure(hpa):
    if hpa is None: return None
    if UNIT_PRESSURE == "mmHg": return hpa * 0.750062
    if UNIT_PRESSURE == "inHg": return hpa * 0.02953
    return hpa

def fmt_pressure(hpa):
    v = conv_pressure(hpa)
    if v is None: return "—"
    if UNIT_PRESSURE == "mmHg": return f"{v:.1f} mmHg"
    if UNIT_PRESSURE == "inHg": return f"{v:.2f} in"
    return f"{v:.1f} hPa"

def conv_temp(celsius):
    if celsius is None: return None
    return celsius * 9.0 / 5.0 + 32.0 if UNIT_TEMP == "F" else celsius

def fmt_temp(celsius):
    v = conv_temp(celsius)
    if v is None: return "—"
    return f"{v:.0f}°F" if UNIT_TEMP == "F" else f"{v:.0f}°C"

# ==================================================================
# COLOR CONSOLE OUTPUT
# ==================================================================

class Color:
    RESET = "\033[0m"; BOLD = "\033[1m"
    RED = "\033[91m"; GREEN = "\033[92m"; YELLOW = "\033[93m"
    BLUE = "\033[94m"; MAGENTA = "\033[95m"; CYAN = "\033[96m"; GRAY = "\033[90m"

    @staticmethod
    def enabled(): return sys.stdout.isatty()

def c(text, color):
    return f"{color}{text}{Color.RESET}" if Color.enabled() else text

def clear_screen():
    if not sys.stdout.isatty(): return
    sys.stdout.write("\033[H\033[J")
    sys.stdout.flush()

def quality_bar(pct):
    filled = round(pct / 100 * 5)
    return '★' * filled + '☆' * (5 - filled) + f" {pct:.0f}%"

# ==================================================================
# TABLE COLUMN DEFINITIONS
# ==================================================================

COLUMNS_DEF = {
    "icao":         {"label": "ICAO",       "width": 8,  "color": Color.CYAN},
    "callsign":     {"label": "Flight",       "width": 9,  "color": Color.GREEN},
    "reg":          {"label": "Reg.",       "width": 8,  "color": None},
    "type":         {"label": "Type",        "width": 5,  "color": None},
    "model":        {"label": "Model",     "width": 15, "color": None},
    "manufacturer": {"label": "Mfr.",    "width": 10, "color": None},
    "operator":     {"label": "Operator",   "width": 10, "color": None},
    "owner":        {"label": "Owner",   "width": 12, "color": None},
    "country":      {"label": "Country",     "width": 10, "color": None},
    "category":     {"label": "Cat",        "width": 4,  "color": None},
    "engines":      {"label": "Eng.",      "width": 10, "color": None},
    "altitude":     {"label": "Altitude",     "width": 9,  "color": Color.YELLOW},
    "speed":        {"label": "Speed",      "width": 8,  "color": None},
    "speed_ias":    {"label": "IAS",        "width": 8,  "color": None},
    "speed_tas":    {"label": "TAS",        "width": 8,  "color": None},
    "heading":      {"label": "Hdg",       "width": 6,  "color": None},
    "vrate":        {"label": "V-rate",       "width": 9,  "color": None},
    "vr_src":       {"label": "VRS",        "width": 5,  "color": None},
    "lat":          {"label": "Latitude",     "width": 10, "color": Color.BLUE},
    "lon":          {"label": "Longitude",    "width": 11, "color": Color.BLUE},
    "ground":       {"label": "Gnd",      "width": 5,  "color": None},
    "distance":     {"label": "Dist.",      "width": 8,  "color": None},
    "phase":        {"label": "Phase",       "width": 9,  "color": None},
    "autopilot":    {"label": "AP",        "width": 14, "color": None},
    "gnss":         {"label": "GNSS",       "width": 8,  "color": None},
    "sel_alt":      {"label": "Sel.alt",      "width": 8,  "color": None},
    "sel_hdg":      {"label": "Sel.hdg",      "width": 6,  "color": None},
    "qnh":          {"label": "QNH",      "width": 9,  "color": None},
    "nac":          {"label": "NACp",       "width": 8,  "color": None},
    "sil":          {"label": "SIL",        "width": 9,  "color": None},
    "nacv":         {"label": "NACv",       "width": 8,  "color": None},
    "version":      {"label": "ADS-B ver", "width": 12, "color": None},
    "nic":          {"label": "NIC",        "width": 8,  "color": None},
    "emergency":    {"label": "Emergency",     "width": 8,  "color": Color.RED},
    "age":          {"label": "Age",      "width": 7,  "color": Color.GRAY},
    "squawk":       {"label": "Sqwk",       "width": 5,  "color": Color.MAGENTA},
    "weather":      {"label": "Weather/wind","width": 18, "color": None},
    "hazard":       {"label": "Hazard",  "width": 14, "color": None},
    "tis_b":        {"label": "TIS-B",      "width": 5,  "color": Color.MAGENTA},
    "rssi":         {"label": "RSSI",       "width": 7,  "color": None},
    "reliable":     {"label": "Rel.",     "width": 7,  "color": None},
}

def get_active_columns():
    return [(k, COLUMNS_DEF[k]) for k in COL_SHOW if k in COLUMNS_DEF]

# ==================================================================
# SOUND GENERATION (WAV)
# ==================================================================

def _ensure_sound_dir(): os.makedirs(SOUND_DIR, exist_ok=True)
def _sound_path(name): return os.path.join(SOUND_DIR, name)

def _write_wav(filename, samples, sr=22050):
    """Write WAV file with embedded metadata (RIFF INFO chunk)."""
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(struct.pack('<' + 'h' * len(samples), *samples))
    _inject_wav_info(filename)

def _inject_wav_info(filename):
    """Add RIFF LIST/INFO chunk with metadata to WAV file."""
    try:
        with open(filename, 'rb') as f:
            data = f.read()

        def _info_field(field_id, text):
            encoded = text.encode('ascii', errors='replace')
            if len(encoded) % 2:
                encoded += b'\x00'
            return field_id.encode('ascii') + struct.pack('<I', len(encoded)) + encoded

        creation_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        info_data = (
            _info_field('ISFT', WAV_SOFTWARE_TAG) +
            _info_field('IART', WAV_AUTHOR_TAG) +
            _info_field('ICMT', WAV_COMMENT_TAG) +
            _info_field('ICRD', creation_date)
        )
        list_chunk = b'LIST' + struct.pack('<I', len(info_data) + 4) + b'INFO' + info_data

        data_pos = data.find(b'data')
        if data_pos < 12:
            return

        new_data = data[:data_pos] + list_chunk + data[data_pos:]
        new_riff_size = len(new_data) - 8
        new_data = new_data[:4] + struct.pack('<I', new_riff_size) + new_data[8:]

        with open(filename, 'wb') as f:
            f.write(new_data)
    except Exception as e:
        sys.stderr.write("[wav-info] %s\n" % e)

def _envelope(t, dur, attack=0.005):
    return min(1.0, t / attack) * min(1.0, (dur - t) / attack)

# --- Basic sounds (11 types) ---

def generate_beep_wav(filename=None, count=3, long=False):
    if filename is None: filename = _sound_path("loss.wav" if long else "beep.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; beep_dur, pause_dur, freq = 0.08, 0.10, 1000.0
    samples = []
    if long:
        total_dur = count * (beep_dur + pause_dur) - pause_dur
        fade_len = int(sr * 0.01)
        for i in range(int(sr * total_dur)):
            t = i / sr; env = 1.0
            if i < fade_len: env = t / (fade_len / sr)
            elif i > int(sr * total_dur) - fade_len: env = (total_dur - t) / (fade_len / sr)
            samples.append(int(32767 * 0.6 * env * math.sin(2 * math.pi * freq * t)))
    else:
        for _ in range(count):
            for i in range(int(sr * beep_dur)):
                t = i / sr
                env = min(1.0, t / 0.005) * min(1.0, (beep_dur - t) / 0.0005)
                samples.append(int(32767 * 0.6 * env * math.sin(2 * math.pi * freq * t)))
            samples.extend([0] * int(sr * pause_dur))
    _write_wav(filename, samples, sr); return filename

def generate_emergency_wav(filename=None):
    if filename is None: filename = _sound_path("emergency.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; beep_dur, pause_dur, freq = 0.06, 0.06, 1500.0
    samples = []
    for _ in range(5):
        for i in range(int(sr * beep_dur)):
            t = i / sr
            env = min(1.0, t / 0.003) * min(1.0, (beep_dur - t) / 0.003)
            samples.append(int(32767 * 0.8 * env * math.sin(2 * math.pi * freq * t)))
        samples.extend([0] * int(sr * pause_dur))
    _write_wav(filename, samples, sr); return filename

def generate_tcas_wav(filename=None):
    if filename is None: filename = _sound_path("tcas.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for _ in range(2):
        for i in range(int(sr * 0.10)):
            t = i / sr; env = _envelope(t, 0.10)
            samples.append(int(32767 * 0.7 * env * math.sin(2 * math.pi * 2000.0 * t)))
        samples.extend([0] * int(sr * 0.08))
    _write_wav(filename, samples, sr); return filename

def generate_tone_wav(f1, f2, count=1, dur=0.10, gap=0.06, vol=0.7, filename=None):
    if filename is None: filename = _sound_path(f"tone_{f1}_{f2}_{count}_{dur}.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for _ in range(count):
        for i in range(int(sr * dur)):
            t = i / sr; freq = f1 + (f2 - f1) * t / dur; env = _envelope(t, dur)
            samples.append(int(32767 * vol * env * math.sin(2 * math.pi * freq * t)))
        samples.extend([0] * int(sr * gap))
    _write_wav(filename, samples, sr); return filename

def generate_melody(freqs, dur=0.12, gap=0.04, vol=0.6, filename=None):
    if filename is None: filename = _sound_path(f"melody_{'_'.join(str(f) for f in freqs)}.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for freq in freqs:
        for i in range(int(sr * dur)):
            t = i / sr; env = _envelope(t, dur)
            samples.append(int(32767 * vol * env * math.sin(2 * math.pi * freq * t)))
        samples.extend([0] * int(sr * gap))
    _write_wav(filename, samples, sr); return filename

def generate_squawk_wav(squawk, filename=None):
    if filename is None: filename = _sound_path(f"squawk_{squawk}.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050
    freqs = {"7500": 800.0, "7600": 1200.0, "7700": 1600.0}
    counts = {"7500": 3, "7600": 2, "7700": 3}
    freq = freqs.get(squawk, 1000.0); count = counts.get(squawk, 3); samples = []
    for _ in range(count):
        for i in range(int(sr * 0.10)):
            t = i / sr; env = _envelope(t, 0.10)
            samples.append(int(32767 * 0.8 * env * math.sin(2 * math.pi * freq * t)))
        samples.extend([0] * int(sr * 0.12))
    _write_wav(filename, samples, sr); return filename

def generate_low_alt_wav(filename=None):
    if filename is None: filename = _sound_path("low_alt.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr, dur = 22050, 0.4; samples = []
    for i in range(int(sr * dur)):
        t = i / sr; freq = 400.0 - 200.0 * (t / dur); env = _envelope(t, dur, 0.02)
        samples.append(int(32767 * 0.6 * env * math.sin(2 * math.pi * freq * t)))
    _write_wav(filename, samples, sr); return filename

def generate_mil_wav(filename=None):
    if filename is None: filename = _sound_path("mil.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for _ in range(2):
        for i in range(int(sr * 0.06)):
            t = i / sr; env = _envelope(t, 0.06)
            samples.append(int(32767 * 0.6 * env * math.sin(2 * math.pi * 600.0 * t)))
        samples.extend([0] * int(sr * 0.04))
    _write_wav(filename, samples, sr); return filename

def generate_tracked_wav(filename=None):
    if filename is None: filename = _sound_path("tracked.wav")
    if os.path.exists(filename): return filename
    return generate_melody([880, 1320], dur=0.10, gap=0.05, vol=0.6, filename=filename)

def generate_tracked_loss_wav(filename=None):
    if filename is None: filename = _sound_path("tracked_loss.wav")
    if os.path.exists(filename): return filename
    return generate_tone_wav(880, 440, count=1, dur=0.3, gap=0, vol=0.6, filename=filename)

def generate_record_wav(filename=None):
    if filename is None: filename = _sound_path("record.wav")
    if os.path.exists(filename): return filename
    return generate_melody([523, 659, 784, 1047], dur=0.12, gap=0.04, vol=0.6, filename=filename)

def generate_silence_wav(filename=None):
    if filename is None: filename = _sound_path("silence.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr, dur = 22050, 0.05; samples = []
    for i in range(int(sr * dur)):
        t = i / sr; env = _envelope(t, dur)
        samples.append(int(32767 * 0.3 * env * math.sin(2 * math.pi * 300.0 * t)))
    _write_wav(filename, samples, sr); return filename

def generate_turbulence_wav(filename=None):
    if filename is None: filename = _sound_path("turbulence.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for _ in range(3):
        for i in range(int(sr * 0.08)):
            t = i / sr; env = _envelope(t, 0.08)
            samples.append(int(32767 * 0.5 * env * math.sin(2 * math.pi * 500.0 * t)))
        samples.extend([0] * int(sr * 0.06))
    _write_wav(filename, samples, sr); return filename

# --- Extended sounds (14 types) ---

def generate_long_range_wav(filename=None):
    if filename is None: filename = _sound_path("long_range.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for i in range(int(sr * 0.4)):
        t = i / sr; env = _envelope(t, 0.4, 0.02)
        bass = int(32767 * 0.4 * env * math.sin(2 * math.pi * 200.0 * t))
        rising = int(32767 * 0.3 * env * math.sin(2 * math.pi * (400 + 400 * t / 0.4) * t))
        samples.append(bass + rising)
    _write_wav(filename, samples, sr); return filename

def generate_overhead_wav(filename=None):
    if filename is None: filename = _sound_path("overhead.wav")
    if os.path.exists(filename): return filename
    return generate_melody([600, 1200, 600], dur=0.12, gap=0.02, vol=0.7, filename=filename)

def generate_speed_record_wav(filename=None):
    if filename is None: filename = _sound_path("speed_record.wav")
    if os.path.exists(filename): return filename
    return generate_melody([440, 880, 1320, 1760], dur=0.08, gap=0.02, vol=0.7, filename=filename)

def generate_alt_record_wav(filename=None):
    if filename is None: filename = _sound_path("alt_record.wav")
    if os.path.exists(filename): return filename
    return generate_melody([1760, 1568, 1319], dur=0.15, gap=0.03, vol=0.5, filename=filename)

def generate_helicopter_wav(filename=None):
    if filename is None: filename = _sound_path("helicopter.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for _ in range(8):
        for i in range(int(sr * 0.04)):
            t = i / sr; env = _envelope(t, 0.04)
            f = 200 + 50 * math.sin(2 * math.pi * 30 * t)
            samples.append(int(32767 * 0.6 * env * math.sin(2 * math.pi * f * t)))
        samples.extend([0] * int(sr * 0.02))
    _write_wav(filename, samples, sr); return filename

def generate_rare_type_wav(filename=None):
    if filename is None: filename = _sound_path("rare_type.wav")
    if os.path.exists(filename): return filename
    return generate_melody([523, 659, 784], dur=0.15, gap=0.03, vol=0.6, filename=filename)

def generate_rapid_descent_wav(filename=None):
    if filename is None: filename = _sound_path("rapid_descent.wav")
    if os.path.exists(filename): return filename
    return generate_tone_wav(1000, 300, count=1, dur=0.5, gap=0, vol=0.8, filename=filename)

def generate_return_wav(filename=None):
    if filename is None: filename = _sound_path("return.wav")
    if os.path.exists(filename): return filename
    return generate_melody([523, 784, 523], dur=0.10, gap=0.03, vol=0.5, filename=filename)

def generate_level_off_wav(filename=None):
    if filename is None: filename = _sound_path("level_off.wav")
    if os.path.exists(filename): return filename
    return generate_melody([659, 880], dur=0.08, gap=0.02, vol=0.4, filename=filename)

def generate_ground_vehicle_wav(filename=None):
    if filename is None: filename = _sound_path("ground_vehicle.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for i in range(int(sr * 0.5)):
        t = i / sr; env = _envelope(t, 0.5, 0.03)
        f = 80 + 20 * math.sin(2 * math.pi * 8 * t)
        samples.append(int(32767 * 0.5 * env * math.sin(2 * math.pi * f * t)))
    _write_wav(filename, samples, sr); return filename

def generate_proximity_wav(filename=None):
    if filename is None: filename = _sound_path("proximity.wav")
    if os.path.exists(filename): return filename
    _ensure_sound_dir(); sr = 22050; samples = []
    for _ in range(2):
        for i in range(int(sr * 0.05)):
            t = i / sr; env = _envelope(t, 0.05)
            samples.append(int(32767 * 0.5 * env * math.sin(2 * math.pi * 300.0 * t)))
        samples.extend([0] * int(sr * 0.03))
    _write_wav(filename, samples, sr); return filename

def generate_headon_wav(filename=None):
    if filename is None: filename = _sound_path("headon.wav")
    if os.path.exists(filename): return filename
    return generate_melody([1320, 880, 660, 440], dur=0.06, gap=0.01, vol=0.6, filename=filename)

def generate_breakthrough_wav(filename=None):
    if filename is None: filename = _sound_path("breakthrough.wav")
    if os.path.exists(filename): return filename
    return generate_tone_wav(880, 880, count=1, dur=0.15, gap=0, vol=0.4, filename=filename)

def generate_squawk_change_wav(filename=None):
    if filename is None: filename = _sound_path("squawk_change.wav")
    if os.path.exists(filename): return filename
    return generate_tone_wav(1200, 1200, count=1, dur=0.06, gap=0, vol=0.4, filename=filename)

# --- Playback ---


# === RTL-SDR CROSSPLATFORM ===
try:
    from rtlsdr import RtlSdr
    HAVE_RTLSDR = True
except (ImportError, AttributeError, OSError):
    HAVE_RTLSDR = False

# Fallback: search for rtl_sdr / rtl_tcp in PATH
def _find_rtl_tool():
    for cmd in ("rtl_sdr", "rtl_tcp"):
        path = shutil.which(cmd)
        if path:
            return cmd
    return None

RTL_TOOL = _find_rtl_tool() if not HAVE_RTLSDR else None
# === END RTL-SDR CROSSPLATFORM ===

def find_player():
    for cmd in ("paplay", "aplay", "ffplay"):
        if shutil.which(cmd): return cmd
    return None

def play_sound(player, sound_file):
    if not player or not sound_file:
        return
    if _browser_sound_mode:
        return
    try:
        if player == "ffplay":
            subprocess.Popen(
                [player, "-nodisp", "-autoexit", "-loglevel", "quiet", sound_file],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        else:
            # aplay, paplay, afplay — just the filename
            subprocess.Popen(
                [player, sound_file],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
    except Exception:
        pass
_NL_TABLE = [
    (10.47047130, 59), (14.82817437, 58), (18.18626357, 57),
    (21.02939493, 56), (23.54504487, 55), (25.82924707, 54),
    (28.00325269, 53), (29.93535453, 52), (31.77209749, 51),
    (33.53993613, 50), (35.22899688, 49), (36.85025108, 48),
    (38.41241892, 47), (39.92256684, 46), (41.38651832, 45),
    (42.80614052, 44), (44.18563703, 43), (45.52834229, 42),
    (46.83706280, 41), (48.11416636, 40), (49.36215573, 39),
    (50.58341110, 38), (51.77930251, 37), (52.95004562, 36),
    (54.09191782, 35), (55.20662206, 34), (56.29550600, 33),
    (57.35908096, 32), (58.39870198, 31), (59.41582176, 30),
    (60.41148230, 29), (61.38649565, 28), (62.34146904, 27),
    (63.26778250, 26), (64.16622144, 25), (65.03744531, 24),
    (65.88231442, 23), (66.70159982, 22), (67.49583472, 21),
    (68.26557646, 20), (69.01108549, 19), (69.73299073, 18),
    (70.43140106, 17), (71.10671471, 16), (71.75932898, 15),
    (72.38941951, 14), (72.99701324, 13), (73.58236298, 12),
    (74.14532445, 11), (74.68616937, 10), (75.20529906, 9),
    (75.70299567, 8), (76.17952719, 7), (76.63495079, 6),
    (77.07015633, 5), (77.48531820, 4), (77.88077879, 3),
    (78.25678085, 2), (78.61360346, 1), (90.0, 1),
]

def nl(lat):
    """Number of longitude zones (NL) for given latitude."""
    lat = abs(lat)
    for boundary, nl_val in _NL_TABLE:
        if lat < boundary: return nl_val
    return 1

def cpr_decode(lat_e, lon_e, lat_o, lon_o, even_newer, ground=False):
    """Global CPR decoding from even/odd message pair."""
    if ground: dlat_e, dlat_o, max_j = 90.0/60.0, 90.0/59.0, 60
    else: dlat_e, dlat_o, max_j = DLAT_EVEN, DLAT_ODD, 60
    j = math.floor(59.0 * lat_e - 60.0 * lat_o + 0.5)
    lat_even = dlat_e * (j % max_j + lat_e)
    lat_odd = dlat_o * (j % (max_j - 1) + lat_o)
    if lat_even > 270: lat_even -= 360
    if lat_odd > 270: lat_odd -= 360
    if not (-90 <= lat_even <= 90 and -90 <= lat_odd <= 90): return None, None
    if nl(lat_even) != nl(lat_odd): return None, None
    lat = lat_even if even_newer else lat_odd
    n = nl(lat)
    if n == 0: return None, None
    m = math.floor(lon_e * (n - 1) - lon_o * n + 0.5)
    ni = max(n, 1) if even_newer else max(n - 1, 1)
    lon = (360.0 / ni) * (m % ni + (lon_e if even_newer else lon_o))
    if lon >= 180: lon -= 360
    return lat, lon

def cpr_decode_local(lat_cpr, lon_cpr, fmt, ref_lat, ref_lon, ground=False):
    """Local CPR decoding from single message and reference position."""
    if ground: dlat = 90.0/60.0 if fmt == 0 else 90.0/59.0
    else: dlat = DLAT_EVEN if fmt == 0 else DLAT_ODD
    j = math.floor(ref_lat / dlat + 0.5)
    lat = dlat * (j + lat_cpr)
    if not (-90 <= lat <= 90): return None, None
    if nl(lat) != nl(ref_lat): return None, None
    n = nl(lat); ni = max(n, 1) if fmt == 0 else max(n - 1, 1)
    dlon = 360.0 / ni; m = math.floor(ref_lon / dlon + 0.5)
    lon = dlon * (m + lon_cpr)
    if lon >= 180: lon -= 360
    if lon < -180: lon += 360
    return lat, lon

# ==================================================================
# BIT DECODERS
# ==================================================================

def bits_to_int(bits):
    """Convert bit array to integer."""
    n = 0
    for b in bits: n = (n << 1) | int(b)
    return n

def gray_to_bin(gray, n):
    """Convert Gray code to binary."""
    result = gray; shift = 1
    while shift < n: result ^= (result >> shift); shift <<= 1
    return result & ((1 << n) - 1)

def decode_altitude(bits, offset=40):
    """Decode altitude from ADS-B or Mode S message."""
    raw = bits_to_int(bits[offset:offset + 12])
    m_bit = (raw >> 11) & 1
    q_bit = (raw >> 10) & 1
    if q_bit:
        n = raw & 0x3FF
        alt = n * 25 - 1000
    elif not m_bit:
        g = raw & 0x7FF
        # Gillham 10-bit Gray code (without M-bit):
        # Order: D1 D2 A1 A2 C1 C2 B1 B2 D4 B4
        alt = gray_to_bin(g, 10) * 100 - 1200
    else:
        return None
    return alt if MIN_ALT_FT <= alt <= MAX_ALT_FT else None

def decode_squawk(bits, offset=19):
    """Decode squawk code from Mode S DF5/DF21 message.
    Interleaved: C1 A1 C2 A2 C4 A4 B1 D1 X B2 D2 B4 D4
    X = SPI bit (position 8, skipped). Weights: X4=4, X2=2, X1=1."""
    ac = bits[offset:offset + 13]
    a = ac[5]*4 + ac[3]*2 + ac[1]     # A4 A2 A1
    b = ac[11]*4 + ac[9]*2 + ac[6]    # B4 B2 B1
    c = ac[4]*4 + ac[2]*2 + ac[0]     # C4 C2 C1
    d = ac[12]*4 + ac[10]*2 + ac[7]   # D4 D2 D1
    return f"{a}{b}{c}{d}"

# ==================================================================
# CRC-24
# ==================================================================

_CRC24_TABLE = [0] * 256
_poly24 = 0x1FFF409
for _ci in range(256):
    _cc = _ci << 16
    for _ in range(8):
        _cc = ((_cc << 1) ^ _poly24) & 0xFFFFFF if _cc & 0x800000 else (_cc << 1) & 0xFFFFFF
    _CRC24_TABLE[_ci] = _cc
del _ci, _cc, _

def crc24(bits, msglen=None):
    """Compute CRC-24 for bit array."""
    if msglen is None: msglen = len(bits)
    crc = 0; i = 0; n_full = msglen - (msglen % 8)
    while i < n_full:
        byte_val = (bits[i]<<7)|(bits[i+1]<<6)|(bits[i+2]<<5)|(bits[i+3]<<4)|(bits[i+4]<<3)|(bits[i+5]<<2)|(bits[i+6]<<1)|bits[i+7]
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24_TABLE[((crc >> 16) ^ byte_val) & 0xFF]
        i += 8
    while i < msglen:
        crc = ((crc << 1) ^ _poly24) & 0xFFFFFF if ((crc >> 23) & 1) ^ bits[i] else (crc << 1) & 0xFFFFFF
        i += 1
    return crc

_FULL_SYNDROMES = {}
def _init_crc_syndromes():
    """Precompute syndromes for 1-bit and 2-bit correction."""
    for msglen in (56, 112):
        syndromes = []
        for i in range(msglen):
            bits = [0]*msglen; bits[i] = 1; syndromes.append(crc24(bits, msglen))
        _FULL_SYNDROMES[msglen] = syndromes
_init_crc_syndromes()

# ==================================================================
# SIGNAL PROCESSING
# ==================================================================

def find_preambles(mag, noise_floor=0.0):
    """Search for ADS-B preambles in amplitude array."""
    n = len(mag) - MSG_SAMPLES
    if n <= 0: return []
    high_sum = np.zeros(n)
    for p in PREAMBLE_HIGH: high_sum += mag[p:p + n]
    low_sum = np.zeros(n)
    for p in PREAMBLE_LOW: low_sum += mag[p:p + n]
    mask = high_sum > low_sum * 2
    min_high = max(noise_floor * 2.5, 0.001) * len(PREAMBLE_HIGH) if noise_floor > 0 else 0.001 * len(PREAMBLE_HIGH)
    mask &= high_sum > min_high
    idx = np.where(mask)[0]
    if len(idx) == 0: return []
    trailing_start = PREAMBLE_SAMPLES
    trailing_len = max(1, int(3 * SAMPLES_PER_BIT))
    filtered = []
    for s in idx:
        if s + trailing_start + trailing_len > len(mag): filtered.append(s); continue
        trailing_avg = float(np.mean(mag[s + trailing_start:s + trailing_start + trailing_len]))
        if trailing_avg < float(high_sum[s]) / len(PREAMBLE_HIGH) * 2.0: filtered.append(s)
    idx = np.array(filtered, dtype=int)
    if len(idx) > 1:
        keep = np.concatenate(([True], np.diff(idx) > MIN_PREAMBLE_SPACING))
        idx = idx[keep]
    return idx

def decode_bits(mag, start, count=112, phase_offset=0.0):
    """Demodulate bits from PPM signal amplitudes."""
    if SAMPLES_PER_BIT == 2.0 and phase_offset == 0.0:
        d = start + PREAMBLE_SAMPLES
        even = mag[d:d + count * 2:2]; odd = mag[d + 1:d + count * 2:2]
        return (even > odd).astype(int).tolist()
    d = start + PREAMBLE_SAMPLES + phase_offset; bits = []
    for i in range(count):
        t1 = d + i * SAMPLES_PER_BIT; t2 = t1 + 0.5 * SAMPLES_PER_BIT
        i1, i2 = int(round(t1)), int(round(t2))
        bits.append(1 if (0 <= i1 < len(mag) and 0 <= i2 < len(mag) and mag[i1] > mag[i2]) else 0)
    return bits

def decode_bits_best(mag, start, count=112):
    """Demodulate with automatic phase adjustment for CRC pass."""
    bits0 = decode_bits(mag, start, count, 0.0)
    df = bits_to_int(bits0[:5]); msglen = 56 if df in (0, 1, 4, 5, 6, 11, 12) else 112
    if crc24(bits0, msglen) == 0: return bits0, True
    if ENABLE_PHASE_CORRECTION:
        for phase in PHASE_OFFSETS[1:]:
            bits = decode_bits(mag, start, count, phase)
            if crc24(bits, msglen) == 0: return bits, True
    return bits0, False

def compute_rssi(mag, start, noise_floor):
    """Calculate RSSI — ratio of message amplitude to noise_floor, dB."""
    if not ENABLE_RSSI: return None
    end = min(start + MSG_SAMPLES, len(mag))
    if start >= end or noise_floor <= 0: return -100.0
    avg = float(np.mean(mag[start:end]))
    return round(10.0 * math.log10(avg / noise_floor), 1) if avg > 0 else -100.0

def haversine_km(lat1, lon1, lat2, lon2):
    """Distance between two points using haversine formula, km."""
    R = 6371.0; p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def in_range(ac_data):
    """Check if aircraft is within geofilter."""
    if FILTER_RADIUS_KM <= 0: return True
    dist = ac_data.get("distance") or ac_data.get("distance_dr")
    if dist is None: return FILTER_NO_POSITION == "show"
    return dist <= FILTER_RADIUS_KM

def heading_to_direction(h):
    """Convert heading to direction (N, NE, E, ...)."""
    return ["N","NE","E","SE","S","SW","W","NW"][int((h+22.5)/45)%8]

def determine_phase(alt, vrate, on_ground, ground_speed):
    """Determine flight phase from altitude and vertical rate."""
    if on_ground: return "Takeoff" if ground_speed and ground_speed > 5 else "Taxiing"
    if vrate is None: return "In air"
    if vrate > 50: return "Climb"
    if vrate < -50: return "Descent"
    if alt is not None and alt >= 10000: return "Cruise"
    return "In air"

# ==================================================================
# MODE A/C
# ==================================================================

def find_mode_ac_preambles(mag, noise_floor=0.0):
    """Search for Mode A/C preambles in amplitude array."""
    f2_dist = int(round(20.3 * SAMPLES_PER_BIT))
    n = len(mag) - f2_dist - MODE_AC_MSG_SAMPLES
    if n <= 0: return []
    threshold = max(noise_floor * 2.5, 0.001) if noise_floor > 0 else 0.6
    mask = (mag[0:n] > threshold) & (mag[f2_dist:f2_dist+n] > threshold)
    for p in range(int(3*SAMPLES_PER_BIT), int(17*SAMPLES_PER_BIT), max(1, int(SAMPLES_PER_BIT))):
        if p + n <= len(mag): mask &= mag[p:p+n] < threshold * 1.5
    idx = np.where(mask)[0]
    if len(idx) > 1:
        keep = np.concatenate(([True], np.diff(idx) > MIN_PREAMBLE_SPACING))
        idx = idx[keep]
    return idx

def gillham_to_altitude(c1,a1,c2,a2,c4,a4,b1,b2,b4,d1,d2,d4):
    """Decode Mode A/C altitude from Gillham code to feet."""
    gray = [d2,d4,a1,a2,a4,b1,b2,b4,c1,c2,c4]
    binary_bits = [gray[0]]
    for i in range(1, len(gray)): binary_bits.append(binary_bits[-1] ^ gray[i])
    n = bits_to_int(binary_bits); alt_ft = n * 100 - 1200
    return alt_ft if MIN_ALT_FT <= alt_ft <= MAX_ALT_FT else None

def decode_mode_ac(mag, start):
    """Decode Mode A/C message: squawk and altitude."""
    bit_spacing = 1.45 * SAMPLES_PER_BIT; levels = []
    for i in range(13):
        pos = start + int(round((i+1) * bit_spacing))
        if pos >= len(mag) or pos < 0: return None, None, False, False
        levels.append(float(mag[pos]))
    threshold = float(np.median(levels))
    if threshold <= 0: return None, None, False, False
    bits = [1 if l > threshold else 0 for l in levels]
    spi = bool(bits[8])
    if not any(bits) or all(bits): return None, None, False, False
    if float(mag[start]) < threshold * 1.5: return None, None, False, False
    if start + int(round(20.3*SAMPLES_PER_BIT)) >= len(mag): return None, None, False, False
    if float(mag[start + int(round(20.3*SAMPLES_PER_BIT))]) < threshold * 0.8: return None, None, False, False
    if sum(bits) < 3: return None, None, False, False
    c1,a1,c2,a2,c4,a4,b1,d1,b2,d2,b4,d4 = bits[0],bits[1],bits[2],bits[3],bits[4],bits[5],bits[6],bits[7],bits[9],bits[10],bits[11],bits[12]
    a,b,c,d = a4*4+a2*2+a1, b4*4+b2*2+b1, c4*4+c2*2+c1, d4*4+d2*2+d1
    if any(x > 7 for x in [a,b,c,d]): return None, None, False, False
    squawk = f"{a}{b}{c}{d}"
    alt_ft = gillham_to_altitude(c1,a1,c2,a2,c4,a4,b1,b2,b4,d1,d2,d4)
    return squawk, alt_ft, True, spi

# ==================================================================
# BDS DECODERS
# ==================================================================

def _looks_like_callsign(bits):
    """Check if bits look like a callsign (BDS 2,0)."""
    cs = ''
    for i in range(8):
        v = bits_to_int(bits[40+i*6:40+i*6+6])
        if v >= len(CHARS) or CHARS[v] == '#': return False, None
        cs += CHARS[v]
    return True, cs.strip()

def try_bds10(bits, cs_info=None):
    """BDS 1,0 — Data Link Capability."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    cont = bits_to_int(bits[33:38])
    return None if cont > 2 else {"dlc_capability": cont}

def try_bds17(bits, cs_info=None):
    """BDS 1,7 — Common Usage GICB Capability."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    cont = bits_to_int(bits[33:38])
    if cont == 0 and not any(bits[33:88]): return None
    return {"gicb_capability": True}

def try_bds30(bits, cs_info=None):
    """BDS 3,0 — ACAS Resolution Advisory Report (TCAS RA)."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    if not any(bits[32:88]): return None
    result = {"acas_arv": bool(bits[32]), "acas_rat": bool(bits[38]),
              "acas_mte": bool(bits[39]), "acas_tti": bits_to_int(bits[40:42])}
    _AV = {0:"",1:"descend",2:"climb",3:"descend crossing",4:"climb crossing",
           5:"increase descent",6:"increase climb",7:"descent not below",
           8:"climb not above",9:"maintain descent",10:"maintain climb",
           11:"do not descend",12:"do not climb",13:"increase VR",14:"reduce VR",15:""}
    _AH = {0:"",1:"turn left",2:"turn right",3:"turn left crossing",
           4:"turn right crossing",5:"increase left",6:"increase right",
           7:"maintain left",8:"maintain right",9:"do not turn left",
           10:"do not turn right",11:"",12:"",13:"",14:"",15:""}
    _RC = {0:"",1:"do not descend",2:"do not climb",3:"do not turn left",
           4:"do not turn right",5:"do not turn",6:"",7:"",8:"",9:"",10:"",
           11:"",12:"",13:"",14:"",15:""}
    result["ara_vertical"] = _AV.get(bits_to_int(bits[42:46]), "?")
    result["ara_horizontal"] = _AH.get(bits_to_int(bits[46:50]), "?")
    result["rac"] = _RC.get(bits_to_int(bits[50:54]), "?")
    result["vrc"] = bool(bits[54])
    result["stc"] = bits_to_int(bits[55:56])
    return result

def try_bds20(bits, cs_info=None):
    """BDS 2,0 — Aircraft Identification (callsign)."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    return {"callsign_bds20": cs_info[1]} if cs_info[0] and cs_info[1] else None

def try_bds44(bits):
    """BDS 4,4 — Meteorological Information (wind, temperature, pressure)."""
    source = bits_to_int(bits[32:36])
    if source == 0 or source > 4 or not bits[36]: return None
    ws = bits_to_int(bits[37:46])
    if ws > 300: return None
    wd = bits_to_int(bits[46:55]) * 180.0 / 256.0
    result = {"wind_dir": wd, "wind_spd": ws}
    if bits[55]:
        temp_raw = bits_to_int(bits[56:66])
        temp = (temp_raw - 1024) * 0.25 if temp_raw & 0x200 else temp_raw * 0.25
        if not (-80 <= temp <= 60): return None
        result["temp"] = round(temp, 1)
    if bits[66]:
        pres = bits_to_int(bits[67:79]) * 0.1 + 800
        if not (MIN_QNH_HPA <= pres <= MAX_QNH_HPA): return None
        result["pressure_hpa"] = round(pres, 1)
    return result

def try_bds45(bits):
    """BDS 4,5 — Weather hazards (turbulence, windshear, microburst, icing, wake vortex,
    temperature, pressure).
    Specification: ICAO Doc 9871 Table A-2-21, mode-s.org.
    BDS 4,5 does NOT contain wind fields — wind is only in BDS 4,4.
    BDS 4,5 does NOT have a source/FOM field (unlike BDS 4,4 which has MB 1-4 source).

    Layout (0-indexed MSG bits, starting at bit 32):
      MB 1      bit 32   — turbulence status
      MB 2-3    bits 33-34 — turbulence level (0: NIL, 1: LIGHT, 2: MODERATE, 3: SEVERE)
      MB 4      bit 35   — windshear status
      MB 5-6    bits 36-37 — windshear level
      MB 7      bit 38   — microburst status (not decoded)
      MB 8-9    bits 39-40 — microburst level (not decoded)
      MB 10     bit 41   — icing status
      MB 11-12  bits 42-43 — icing level
      MB 13     bit 44   — wake vortex status (not decoded)
      MB 14-15  bits 45-46 — wake vortex level (not decoded)
      MB 16     bit 47   — temperature status
      MB 17-26  bits 48-57 — temperature (10-bit two's complement, LSB = 0.25 C)
      MB 27     bit 58   — pressure status
      MB 28-38  bits 59-69 — pressure (11-bit, 800 + value * 0.1 hPa)

    Source: https://mode-s.org/1090mhz/content/mode-s/8-meteo.html"""
    result = {}; has_any = False

    # Turbulence: MB 1 (status) + MB 2-3 (level)
    if bits[32]:
        result["turbulence"] = TURBULENCE_LEVEL.get(bits_to_int(bits[33:35]), "?")
        has_any = True

    # Windshear: MB 4 (status) + MB 5-6 (level)
    if bits[35]:
        result["windshear"] = WINDSHEAR_LEVEL.get(bits_to_int(bits[36:38]), "?")
        has_any = True

    # Icing: MB 10 (status) + MB 11-12 (level)
    if bits[41]:
        result["icing"] = ICING_LEVEL.get(bits_to_int(bits[42:44]), "?")
        has_any = True
    if bits[38]:
        result["microburst"] = WINDSHEAR_LEVEL.get(bits_to_int(bits[39:41]), "?")
        has_any = True
    if bits[44]:
        result["wake_vortex"] = WINDSHEAR_LEVEL.get(bits_to_int(bits[45:47]), "?")
        has_any = True

    # Temperature: MB 16 (status) + MB 17-26 (10-bit two's complement, LSB 0.25 C)
    if bits[47]:
        temp_raw = bits_to_int(bits[48:58])  # 10-bit two's complement
        temp = (temp_raw - 1024) * 0.25 if temp_raw & 0x200 else temp_raw * 0.25
        if not (-80 <= temp <= 60):
            return None
        result["temp"] = round(temp, 1)
        has_any = True

    # Pressure: MB 27 (status) + MB 28-38 (11 bit, 800 + value * 0.1 hPa)
    if bits[58]:
        pres = 800 + bits_to_int(bits[59:70]) * 0.1
        if not (MIN_QNH_HPA <= pres <= MAX_QNH_HPA):
            return None
        result["pressure_hpa"] = round(pres, 1)
        has_any = True

    if not has_any:
        return None
    return result

def try_bds40(bits):
    """BDS 4,0 — Selected Altitude (selected altitude, QNH, autopilot)."""
    result = {}
    sel_alt = None
    if bits[32]: sel_alt = bits_to_int(bits[33:45]) * 16
    elif bits[45]: sel_alt = bits_to_int(bits[46:58]) * 16
    else: return None
    if not (MIN_SEL_ALT_FT <= sel_alt <= MAX_SEL_ALT_FT): return None
    result["selected_altitude"] = sel_alt
    if bits[58]:
        baro = 800 + bits_to_int(bits[59:71]) * 0.1
        if not (MIN_QNH_HPA <= baro <= MAX_QNH_HPA): return None
        result["baro_setting"] = baro
    return result

def try_bds50(bits):
    """BDS 5,0 — Track and Turn Report (roll, heading, GS, TAS, turn)."""
    if not bits[32]: return None
    result = {}; roll = bits_to_int(bits[34:44]) * 45.0 / 512.0
    if bits[33]: roll = -roll
    if abs(roll) > 60: return None
    result["roll"] = round(roll, 1)
    if bits[44]:
        trk = bits_to_int(bits[46:56]) * 90.0 / 512.0
        if bits[45]: trk = -trk
        if trk < 0: trk += 360
        result["true_hdg"] = round(trk, 1)
    if bits[56]:
        gs = bits_to_int(bits[57:67]) * 2
        if gs > 600: return None
        result["ground_speed"] = gs
    if bits[67]:
        tr = bits_to_int(bits[69:79]) * 8.0 / 256.0
        if bits[68]: tr = -tr
        if abs(tr) > MAX_TURN_RATE_DEG: tr = None
        if tr is not None: result["turn_rate"] = round(tr, 2)
    if bits[79]:
        tas_val = bits_to_int(bits[80:88]) * 2
        if tas_val > MAX_IAS_TAS_KT: return None
        result["tas"] = tas_val
    return result

def try_bds60(bits):
    """BDS 6,0 — Heading and Speed Report (mag. heading, IAS, vertical rate)."""
    if not bits[32]: return None
    result = {}; hdg = bits_to_int(bits[33:44]) * 90.0 / 512.0
    result["mag_hdg"] = round(hdg, 1)
    if bits[44]:
        ias_val = bits_to_int(bits[45:55])
        if ias_val > MAX_IAS_TAS_KT: return None
        result["ias"] = ias_val
    if bits[63]:
        vr_val = bits_to_int(bits[65:74])
        if vr_val not in (0, 511):
            if bits[64]: vr_val = -vr_val
            result["vrate_bds60"] = vr_val * 32
            if abs(result["vrate_bds60"]) > 6000: del result["vrate_bds60"]
    return result


def try_bds51(bits, cs_info=None):
    """BDS 5,1 — Selected Heading."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    r = {}
    if bits[32]: r["sel_heading_bds51"] = round(bits_to_int(bits[33:44]) * 90.0 / 512.0, 1)
    if bits[44]:
        b = 800 + bits_to_int(bits[45:57]) * 0.1
        if MIN_QNH_HPA <= b <= MAX_QNH_HPA: r["baro_bds51"] = b
    if bits[57]:
        a = bits_to_int(bits[58:70]) * 16
        if MIN_SEL_ALT_FT <= a <= MAX_SEL_ALT_FT: r["sel_alt_bds51"] = a
    m = []
    if bits[70]: m.append("VNAV")
    if bits[71]: m.append("ALT HOLD")
    if bits[72]: m.append("APPR")
    if bits[73]: m.append("LNAV")
    if bits[74]: m.append("FMS")
    if bits[75]: m.append("TCAS")
    if m: r["autopilot_modes"] = m
    return r or None

def try_bds52(bits, cs_info=None):
    """BDS 5,2 — Trajectory Change Point."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    r = {}
    t = bits_to_int(bits[33:35])
    if t == 0 and not any(bits[35:88]): return None
    r["tcp_type"] = {0:"",1:"turn",2:"alt constraint",3:"time constraint"}.get(t, f"?{t}")
    if bits[35]: r["turn_dir"] = {0:"left",1:"right",2:"straight"}.get(bits_to_int(bits[36:37]), "?")
    if bits[37]:
        tr = bits_to_int(bits[38:48]) * 0.25
        if tr <= 15: r["turn_rate_tcp"] = round(tr, 2)
    if bits[48]: r["tcp_lat"] = round(bits_to_int(bits[49:62]) * 180.0 / (1<<13) - 90.0, 5)
    if bits[62]: r["tcp_lon"] = round(bits_to_int(bits[63:76]) * 360.0 / (1<<13) - 180.0, 5)
    if bits[76]:
        a = bits_to_int(bits[77:87]) * 10
        if MIN_ALT_FT <= a <= MAX_ALT_FT: r["tcp_alt"] = a
    return r if len(r) > 1 else None

def try_bds53(bits, cs_info=None):
    """BDS 5,3 — Trajectory Change Intent."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    r = {}
    t = bits_to_int(bits[33:35])
    if t == 0 and not any(bits[35:88]): return None
    r["tc_type"] = {0:"",1:"climb",2:"descend",3:"level"}.get(t, f"?{t}")
    if bits[35]: r["tc_turn_dir"] = {0:"left",1:"right",2:"straight"}.get(bits_to_int(bits[36:37]), "?")
    if bits[37]:
        a = bits_to_int(bits[38:48]) * 0.01
        if a <= 30: r["climb_angle"] = round(a, 2)
    if bits[48]:
        a = bits_to_int(bits[49:61]) * 16
        if MIN_SEL_ALT_FT <= a <= MAX_SEL_ALT_FT: r["tc_target_alt"] = a
    return r if len(r) > 1 else None

def try_bds61(bits, cs_info=None):
    """BDS 6,1 — Steering Command."""
    if cs_info is None: cs_info = _looks_like_callsign(bits)
    if cs_info[0] and cs_info[1]: return None
    r = {}
    if bits[32]: r["steer_hdg"] = round(bits_to_int(bits[33:44]) * 90.0 / 512.0, 1)
    if bits[44]:
        a = bits_to_int(bits[45:55]) * 0.5
        if a <= 30: r["steer_angle_rate"] = round(a, 1)
    if bits[55]:
        a = bits_to_int(bits[56:68]) * 16
        if MIN_SEL_ALT_FT <= a <= MAX_SEL_ALT_FT: r["fms_alt"] = a
    if bits[68]:
        s = bits_to_int(bits[69:79])
        if s <= 600: r["fms_spd"] = s
    m = []
    if bits[79]: m.append("VNAV")
    if bits[80]: m.append("ALT HOLD")
    if bits[81]: m.append("LNAV")
    if bits[82]: m.append("APPR")
    if bits[83]: m.append("TCAS")
    if m: r["fms_modes"] = m
    return r or None

def _score_bds51(r, ac):
    if not r: return -1
    s = 0
    if r.get("sel_heading_bds51") is not None and ac.get("heading") is not None:
        d = abs(r["sel_heading_bds51"] - ac["heading"])
        if d > 180: d = 360 - d
        s += (3 if d < 15 else (1 if d < 30 else 0)) + 1
    if r.get("sel_alt_bds51") is not None:
        s += 1
        if ac.get("selected_altitude") is not None and abs(r["sel_alt_bds51"] - ac["selected_altitude"]) < 500: s += 2
    return s

def _score_bds61(r, ac):
    if not r: return -1
    s = 0
    if r.get("steer_hdg") is not None and ac.get("heading") is not None:
        d = abs(r["steer_hdg"] - ac["heading"])
        if d > 180: d = 360 - d
        s += (2 if d < 20 else 0) + 1
    if r.get("fms_spd") is not None:
        s += 1
        sp = ac.get("speed_gs") or ac.get("speed_ias")
        if sp is not None and abs(r["fms_spd"] - sp) < 50: s += 2
    return s

def identify_bds(bds):
    """Identify BDS type from dictionary contents."""
    if bds is None: return None
    if "callsign_bds20" in bds: return "BDS20"
    if "dlc_capability" in bds: return "BDS10"
    if "gicb_capability" in bds: return "BDS17"
    if "acas_arv" in bds: return "BDS30"
    if "sel_heading_bds51" in bds: return "BDS51"
    if "tcp_type" in bds: return "BDS52"
    if "tc_type" in bds: return "BDS53"
    if "steer_hdg" in bds: return "BDS61"
    if "turbulence" in bds or "icing" in bds or "windshear" in bds: return "BDS45"
    if "wind_dir" in bds: return "BDS44"
    if "selected_altitude" in bds: return "BDS40"
    if "roll" in bds: return "BDS50"
    if "mag_hdg" in bds: return "BDS60"
    return "Unknown"

def _score_bds50(r, ac):
    """Score BDS 5,0 candidate by matching known aircraft data."""
    if not r: return -1; score = 0
    if r.get("tas") is not None:
        score += 1
        if ac.get("speed_tas") is not None and abs(r["tas"] - ac["speed_tas"]) < 50: score += 3
        elif ac.get("speed_gs") is not None and abs(r["tas"] - ac["speed_gs"]) < 100: score += 2
    if r.get("ground_speed") is not None and ac.get("speed_gs") is not None and abs(r["ground_speed"] - ac["speed_gs"]) < 50: score += 3
    if r.get("roll") is not None and abs(r["roll"]) < 30: score += 1
    return score

def _score_bds60(r, ac):
    """Score BDS 6,0 candidate by matching known aircraft data."""
    if not r: return -1; score = 0
    if r.get("ias") is not None:
        score += 1
        if ac.get("speed_ias") is not None and abs(r["ias"] - ac["speed_ias"]) < 50: score += 3
        elif ac.get("speed_gs") is not None and abs(r["ias"] - ac["speed_gs"]) < 150: score += 1
    if r.get("mag_hdg") is not None and ac.get("heading") is not None:
        diff = abs(r["mag_hdg"] - ac["heading"])
        if diff > 180: diff = 360 - diff
        score += (3 if diff < 15 else (1 if diff < 30 else 0)) + 1
    if r.get("vrate_bds60") is not None and abs(r["vrate_bds60"]) < 6000:
        score += 1
        if ac.get("vrate_tc19") is not None and abs(r["vrate_bds60"] - ac["vrate_tc19"]) < 1000: score += 2
    return score

def guess_bds(bits, ac_data=None):
    """Guess BDS type for DF 20/21 (Comm-B) with numeric scoring."""
    cs_info = _looks_like_callsign(bits)
    for enabled, fn in [(ENABLE_BDS10, try_bds10), (ENABLE_BDS17, try_bds17),
                        (ENABLE_BDS20, try_bds20), (ENABLE_BDS30, try_bds30)]:
        if enabled:
            res = fn(bits, cs_info)
            if res: return res
    candidates = []
    if ENABLE_BDS45:
        r = try_bds45(bits)
        if r: candidates.append(("BDS45", r, 5))
    if ENABLE_BDS44:
        r = try_bds44(bits)
        if r: candidates.append(("BDS44", r, 5))
    if ENABLE_BDS40:
        r = try_bds40(bits)
        if r: candidates.append(("BDS40", r, 4))
    if ENABLE_BDS50:
        r = try_bds50(bits)
        if r: candidates.append(("BDS50", r, _score_bds50(r, ac_data or {})))
    if ENABLE_BDS60:
        r = try_bds60(bits)
        if r: candidates.append(("BDS60", r, _score_bds60(r, ac_data or {})))
    if ENABLE_BDS51:
        r = try_bds51(bits)
        if r: candidates.append(("BDS51", r, _score_bds51(r, ac_data or {})))
    if ENABLE_BDS52:
        r = try_bds52(bits)
        if r: candidates.append(("BDS52", r, 3))
    if ENABLE_BDS53:
        r = try_bds53(bits)
        if r: candidates.append(("BDS53", r, 3))
    if ENABLE_BDS61:
        r = try_bds61(bits)
        if r: candidates.append(("BDS61", r, _score_bds61(r, ac_data or {})))
    if not candidates: return None
    return max(candidates, key=lambda x: x[2])[1]



def bearing_to(lat1, lon1, lat2, lon2):
    """Bearing from point 1 to point 2, degrees (0=north, clockwise)."""
    import math as _m
    rl1, rl2 = _m.radians(lat1), _m.radians(lat2)
    dlon = _m.radians(lon2 - lon1)
    x = _m.sin(dlon) * _m.cos(rl2)
    y = _m.cos(rl1) * _m.sin(rl2) - _m.sin(rl1) * _m.cos(rl2) * _m.cos(dlon)
    return (_m.degrees(_m.atan2(x, y)) + 360) % 360

# ==================================================================
# AIRCRAFTDB — AIRCRAFT DATABASE
# ==================================================================



class FlightLogger:
    """Per-aircraft event logger with date-based folders."""
    def __init__(self):
        self.files = {}
        self.first_seen = {}
        self.start_str = {}

    def _day_dir(self, ts=None):
        if ts is None: ts = time.time()
        return os.path.join(LOG_DIR, time.strftime("%d.%m.%Y", time.localtime(ts)))

    def _path(self, icao, ts=None):
        return os.path.join(self._day_dir(ts), icao + ".log")

    def _get(self, icao, cs=""):
        if icao not in self.files:
            now = time.time()
            day_dir = self._day_dir(now)
            os.makedirs(day_dir, exist_ok=True)
            self.first_seen[icao] = now
            self.start_str[icao] = time.strftime("%H:%M:%S", time.localtime(now))
            date_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
            f = open(self._path(icao, now), "a", encoding="utf-8")
            bort_name = cs if cs else "(no callsign)"
            w = f.write
            w("════════════════════════════════════════════════════════════════════════\n")
            w("│  Aircraft: %-60s │\n" % bort_name)
            w("│  ICAO: %-60s │\n" % icao)
            w("│  First detected: %-47s │\n" % date_str)
            w("════════════════════════════════════════════════════════════════════════\n")
            w("\n")
            self.files[icao] = f
        return self.files[icao]

    def log_event(self, icao, cs, text):
        try:
            f = self._get(icao, cs)
            f.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), text))
            f.flush()
        except Exception:
            log.exception("log_event: error")

    def close_old(self, active):
        for icao in list(self.files):
            if icao not in active:
                self._write_footer(icao)
                try: self.files[icao].close()
                except Exception: pass
                del self.files[icao]
                self.first_seen.pop(icao, None)
                self.start_str.pop(icao, None)

    def close_all(self):
        for icao in list(self.files):
            self._write_footer(icao)
            try: self.files[icao].close()
            except Exception: pass
        self.files.clear()
        self.first_seen.clear()
        self.start_str.clear()

    def _write_footer(self, icao):
        try:
            f = self.files.get(icao)
            if f and icao in self.first_seen:
                dur = int(time.time() - self.first_seen[icao])
                end_str = time.strftime("%H:%M:%S")
                mm, ss = divmod(dur, 60)
                hh, mm = divmod(mm, 60)
                if hh > 0: dur_str = "%d h %d min %d sec" % (hh, mm, ss)
                elif mm > 0: dur_str = "%d min %d sec" % (mm, ss)
                else: dur_str = "%d sec" % ss
                w = f.write
                w("\n")
                w("────────────────────────────────────────────────────────────────────────\n")
                w("  Observation: %s — %s\n" % (self.start_str.get(icao, "?"), end_str))
                w("  Duration: %s\n" % dur_str)
                w("────────────────────────────────────────────────────────────────────────\n")
                f.flush()
        except Exception:
            pass


class AircraftDB:
    """Central aircraft database, tracking and event processing."""

    def __init__(self, **sounds):
        self.ac = {}; self.cpr = {}; self.cpr_ground = {}; self.last_print = 0
        self.new_events = []; self.noise_floor = MIN_SIGNAL_LEVEL
        self.ppm_adjusted = PPM_CORRECTION; self.mac_pending = {}
        self.beast_encoder = None; self.sbs1_encoder = None; self.json_server = None
        self.last_msg_time = time.time(); self.silence_played = False
        self.recently_lost = {}; self._proximity_checked = 0
        self.elm_buffers = {}
        self._decode_q_window = deque(); self._recv_q_window = deque()
        self._decode_ok_count = 0; self._recv_ok_count = 0
        self.decode_quality_ema = 0.0; self.recv_quality_ema = 0.0; self._q_alpha = 0.05
        self._two_bit_disabled = False
        self._crc2_time_ema = 0.0
        self._lock = threading.RLock()  # reentrant: process() calls check_alert() which also locks
        self.coverage_bins = [0.0] * 72

        # === Analytics fields ===
        self.rssi_bins = [0.0] * 72
        self.rssi_counts = [0] * 72
        self.df_history = deque(maxlen=60)
        self.cat_history = deque(maxlen=180)
        self.quality_history = deque(maxlen=180)
        self.silence_periods = []
        self._last_msg_time_an = time.time()
        self.silence_active = False
        self._silence_start = None
        self.last_msg_check = time.time()
        self.doppler_data = {"measurements": [], "ppm_estimate": 0.0}
        self.type_counts = {}
        self.flag_counts = {}
        self.alt_histogram = [0] * 35  # bins of 2000 ft up to 70000
        self.speed_histogram = [0] * 30  # bins of 50 kt up to 1500
        self._analytics_last = 0
        self._analytics_cache = None
        self._stats_hist_last = 0
        self._pos_hist_last = 0
        # Session accumulators
        self._session_icao_type = {}
        self._seen_cats = set()
        self._session_icao_alt = {}
        self.msg_rate_history = deque(maxlen=360)
        self.rssi_history = deque(maxlen=360)
        self.unique_aircraft_history = deque(maxlen=360)
        self.squawk_counts = {}
        self.adsb_version_counts = {}
        self.heatmap_grid = {}
        self._heatmap_max_cells = 10000

        self._session_icao_spd = {}
        self.type_counts_session = {}
        self.flag_counts_session = {}
        self._session_icao_flag = {}
        self.alt_histogram_session = [0] * 35
        self.speed_histogram_session = [0] * 30


        for k in ("sound_file","emergency_sound","loss_sound","tcas_sound",
                  "low_alt_sound","mil_sound","tracked_sound","tracked_loss_sound",
                  "record_sound","silence_sound","turbulence_sound",
                  "squawk_7500_sound","squawk_7600_sound","squawk_7700_sound","startup_sound","player",
                  "long_range_sound","overhead_sound","speed_record_sound",
                  "alt_record_sound","helicopter_sound","rare_type_sound",
                  "rapid_descent_sound","return_sound","level_off_sound",
                  "ground_vehicle_sound","proximity_sound","headon_sound",
                  "breakthrough_sound","squawk_change_sound"):
            setattr(self, k, sounds.get(k))
        self.stats = {
            "total_aircraft": 0, "msg_by_type": {}, "max_range_km": 0,
            "max_speed_kt": 0, "max_alt_ft": 0,
            "bds_attempts": 0, "bds_success": 0, "bds_fail": 0, "bds_by_type": {},
            "pos_jumps_rejected": 0, "alt_rejected": 0, "crc_corrected": 0,
            "vel_rejected": 0, "hdg_rejected": 0, "bds_rejected": 0,
            "crc_two_bit_corrected": 0, "mode_ac_count": 0, "mode_ac_emergency": 0,
            "nic_decoded": 0, "sil_decoded": 0, "nacv_decoded": 0, "version_decoded": 0,
            "bds45_count": 0, "uat_count": 0, "queue_drops": 0,
            "preambles_total": 0, "preambles_strong": 0,
            "long_range_count": 0, "overhead_count": 0,
            "rapid_descent_count": 0, "squawk_change_count": 0,
            "return_count": 0, "level_off_count": 0,
            "proximity_count": 0, "headon_count": 0,
        }
        self.logger = FlightLogger()

# --- Dual quality metrics ---
    def record_quality(self, decode_ok, recv_strong=None):
        now = time.time()
        self._decode_q_window.append((now, decode_ok))
        if decode_ok: self._decode_ok_count += 1
        cutoff = now - 10.0
        while self._decode_q_window and self._decode_q_window[0][0] < cutoff:
            _, old_ok = self._decode_q_window.popleft()
            if old_ok: self._decode_ok_count -= 1
        total_d = len(self._decode_q_window)
        decode_pct = self._decode_ok_count / total_d * 100 if total_d > 0 else 0
        self.decode_quality_ema = self.decode_quality_ema*(1-self._q_alpha) + decode_pct*self._q_alpha
        if recv_strong is not None:
            self._recv_q_window.append((now, recv_strong))
            if recv_strong: self._recv_ok_count += 1
            while self._recv_q_window and self._recv_q_window[0][0] < cutoff:
                _, old_strong = self._recv_q_window.popleft()
                if old_strong: self._recv_ok_count -= 1
            total_r = len(self._recv_q_window)
            recv_pct = self._recv_ok_count / total_r * 100 if total_r > 0 else 0
            self.recv_quality_ema = self.recv_quality_ema*(1-self._q_alpha) + recv_pct*self._q_alpha

    def get_quality(self):
        return min(self.decode_quality_ema, self.recv_quality_ema) if self.recv_quality_ema > 0 else self.decode_quality_ema

    def get_msg_rate(self):
        if not self._decode_q_window:
            return 0.0
        first_t = self._decode_q_window[0][0]
        window = max(time.time() - first_t, 1.0)
        return self._decode_ok_count / window

# --- Session ---
    def save_session(self):
        if not ENABLE_SESSION_SAVE: return
        try:
            now = time.time()
            session = {"timestamp": now, "ppm": self.ppm_adjusted if SESSION_SAVE_PPM else PPM_CORRECTION}
            if SESSION_SAVE_STATS: session["stats"] = dict(self.stats)
            ac_list = []
            with self._lock:
                _session_ac_snapshot = list(self.ac.items())
            for icao, a in _session_ac_snapshot:
                if now - a.get("last_seen", 0) > SESSION_MAX_AGE: continue
                ac_list.append({"icao": icao, "callsign": a.get("callsign",""),
                    "altitude": a.get("altitude"), "speed_gs": a.get("speed_gs"),
                    "heading": a.get("heading"), "lat": a.get("lat"), "lon": a.get("lon"),
                    "squawk": a.get("squawk",""), "category": a.get("category",""),
                    "first_seen": a.get("first_seen",now), "last_seen": a.get("last_seen",now),
                    "pos_reliable": a.get("pos_reliable",0), "rssi": a.get("rssi")})
            session["aircraft"] = ac_list
            # Session accumulators (preserve across restarts)
            session["coverage_bins"] = list(self.coverage_bins)
            session["heatmap_grid"] = dict(self.heatmap_grid)
            session["squawk_counts"] = dict(self.squawk_counts)
            session["adsb_version_counts"] = dict(self.adsb_version_counts)
            session["_session_icao_type"] = dict(self._session_icao_type)
            session["_session_icao_flag"] = dict(self._session_icao_flag)
            session["_session_icao_alt"] = dict(self._session_icao_alt)
            session["_session_icao_spd"] = dict(self._session_icao_spd)
            with open(SESSION_FILE, 'w', encoding='utf-8') as f:
                json_module.dump(session, f, ensure_ascii=False, indent=2)
        except Exception as e: sys.stderr.write(f"[session] save: {e}\n")

    def restore_session(self):
        if not ENABLE_SESSION_RESTORE or not os.path.exists(SESSION_FILE): return
        try:
            with open(SESSION_FILE, 'r', encoding='utf-8') as f:
                session = json_module.load(f)
            now = time.time()
            if now - session.get("timestamp", 0) > SESSION_MAX_AGE: return
            if SESSION_SAVE_PPM and "ppm" in session: self.ppm_adjusted = session["ppm"]
            # Restore session accumulators
            if "coverage_bins" in session:
                cb = session["coverage_bins"]
                for i in range(min(len(cb), 72)):
                    self.coverage_bins[i] = cb[i]
            if "heatmap_grid" in session:
                self.heatmap_grid = dict(session["heatmap_grid"])
            if "squawk_counts" in session:
                self.squawk_counts = dict(session["squawk_counts"])
            if "adsb_version_counts" in session:
                self.adsb_version_counts = dict(session["adsb_version_counts"])
            if "_session_icao_type" in session:
                self._session_icao_type = dict(session["_session_icao_type"])
            if "_session_icao_flag" in session:
                self._session_icao_flag = dict(session["_session_icao_flag"])
            if "_session_icao_alt" in session:
                self._session_icao_alt = dict(session["_session_icao_alt"])
            if "_session_icao_spd" in session:
                self._session_icao_spd = dict(session["_session_icao_spd"])
            restored = 0
            for entry in session.get("aircraft", []):
                icao = entry.get("icao", "")
                if not icao or icao.startswith("~"): continue
                if now - entry.get("last_seen", 0) > SESSION_MAX_AGE: continue
                ac = {"first_seen": entry.get("first_seen", now), "last_seen": now,
                      "alerted": True, "callsign": entry.get("callsign",""),
                      "squawk": entry.get("squawk",""), "category": entry.get("category",""),
                      "pos_reliable": 0, "rssi": entry.get("rssi")}
                for k in ("altitude","speed_gs","heading"):
                    if entry.get(k) is not None: ac[k] = entry[k]
                if entry.get("altitude") is not None: ac["alt_time"] = now
                if entry.get("lat") is not None and entry.get("lon") is not None:
                    ac["lat"] = entry["lat"]; ac["lon"] = entry["lon"]
                    ac["pos_time"] = entry.get("last_seen", now)
                    ac["distance"] = haversine_km(HOME_LAT, HOME_LON, entry["lat"], entry["lon"])
                self.ac[icao] = ac; restored += 1
            if restored: self.stats["total_aircraft"] = restored; sys.stderr.write(f"[session] restored: {restored}\n")
            if SESSION_SAVE_STATS and "stats" in session:
                for k, v in session["stats"].items():
                    if k != "total_aircraft": self.stats[k] = v
        except Exception as e: sys.stderr.write(f"[session] restore: {e}\n")

# --- Auto-PPM ---
    def auto_ppm(self, proc, current_ppm, current_gain):
        if not ENABLE_PPM_AUTO: return current_ppm, proc
        min_valid = max(PPM_AUTO_MIN_VALID, POSITION_RELIABLE_MIN)
        valid = [a for a in self.ac.values()
                 if a.get("pos_reliable",0) >= min_valid and a.get("lat") and a.get("lon")]
        if len(valid) < PPM_AUTO_MIN_SAMPLES: return current_ppm, proc
        now = time.time(); offset_sum = 0; offset_count = 0
        for a in valid:
            spd = a.get("speed_gs")
            hdg = a.get("heading")
            dt = now - a.get("pos_time", now)
            if spd is None or hdg is None or dt < 2 or dt > PPM_AUTO_MAX_POS_AGE: continue
            spd_kms = spd * 1.852 / 3600.0; dist = spd_kms * dt
            dr_lat = a["lat"] + dist*math.cos(math.radians(hdg))/111.32
            dr_lon = a["lon"] + dist*math.sin(math.radians(hdg))/(111.32*max(math.cos(math.radians(a["lat"])),0.01))
            diff = haversine_km(a["lat"], a["lon"], dr_lat, dr_lon)
            if diff > 0.1: offset_sum += diff; offset_count += 1
        if offset_count < PPM_AUTO_MIN_SAMPLES: return current_ppm, proc
        avg_offset = offset_sum / offset_count
        if avg_offset < PPM_AUTO_TOLERANCE_KM: return current_ppm, proc
        adjust = int(min(PPM_AUTO_MAX_ADJUST, int(avg_offset / PPM_AUTO_TOLERANCE_KM)) * (1.0 - PPM_AUTO_SMOOTHING))
        if adjust == 0: return current_ppm, proc
        new_ppm = current_ppm + adjust; self.ppm_adjusted = new_ppm
        self.new_events.append(f"  {c('[PPM-AUTO]', Color.YELLOW)} offset {avg_offset:.1f}km -> PPM {current_ppm:+d} -> {new_ppm:+d}")
        if PPM_AUTO_RESTART_SDR: proc = restart_sdr(proc, current_gain, new_ppm)
        return new_ppm, proc

    # --- Sounds ---
    # Per-type sound cooldown (seconds). Prevents audio flooding when many
    # aircraft trigger the same sound simultaneously (e.g. 50 new aircraft).
    # Critical sounds (emergency, squawk, loss, rapid_descent) have NO cooldown.
    _SOUND_COOLDOWN = {
        "new": 1.0, "mil": 5.0, "low_alt": 10.0,
        "tracked": 5.0, "tracked_loss": 5.0,
        "long_range": 30.0, "overhead": 30.0,
        "return": 5.0, "level_off": 10.0,
        "squawk_change": 5.0, "proximity": 10.0,
        "headon": 10.0, "breakthrough": 30.0,
        "ground_vehicle": 30.0, "rare_type": 30.0,
        "helicopter": 30.0, "silence": 30.0,
        "turbulence": 15.0,
        # No cooldown (always play): emergency, tcas, loss,
        # rapid_descent, record, speed_record, alt_record
    }
    _sound_last_play = {}

    def _play(self, flag, attr):
        if not ENABLE_SOUND or not flag:
            return
        st = SOUND_ATTR_TO_TYPE.get(attr)
        if not st:
            return
        cd = self._SOUND_COOLDOWN.get(st)
        if cd:
            now = time.time()
            last = self._sound_last_play.get(st, 0)
            if now - last < cd:
                return
            self._sound_last_play[st] = now
        if _browser_sound_mode:
            _sound_event_queue.append({"type": st, "t": time.time()})
            return
        play_sound(self.player, getattr(self, attr, None))
    def play_beep(self): self._play(ENABLE_SOUND_NEW, "sound_file")
    def play_emergency(self): self._play(ENABLE_SOUND_EMERGENCY, "emergency_sound")
    def play_loss_of_contact(self): self._play(ENABLE_SOUND_LOSS, "loss_sound")
    def play_tcas(self): self._play(ENABLE_SOUND_TCAS, "tcas_sound")
    def play_low_alt(self): self._play(ENABLE_SOUND_LOW_ALT, "low_alt_sound")
    def play_mil(self): self._play(ENABLE_SOUND_MIL, "mil_sound")
    def play_tracked(self): self._play(ENABLE_SOUND_TRACKED, "tracked_sound")
    def play_tracked_loss(self): self._play(ENABLE_SOUND_TRACKED_LOSS, "tracked_loss_sound")
    def play_record(self): self._play(ENABLE_SOUND_RECORD, "record_sound")
    def play_silence(self): self._play(ENABLE_SOUND_SILENCE, "silence_sound")
    def play_turbulence(self): self._play(ENABLE_SOUND_TURB, "turbulence_sound")
    def play_long_range(self): self._play(ENABLE_SOUND_LONG_RANGE, "long_range_sound")
    def play_overhead(self): self._play(ENABLE_SOUND_OVERHEAD, "overhead_sound")
    def play_speed_record(self): self._play(ENABLE_SOUND_SPEED_RECORD, "speed_record_sound")
    def play_alt_record(self): self._play(ENABLE_SOUND_ALT_RECORD, "alt_record_sound")
    def play_helicopter(self): self._play(ENABLE_SOUND_HELICOPTER, "helicopter_sound")
    def play_rare_type(self): self._play(ENABLE_SOUND_RARE_TYPE, "rare_type_sound")
    def play_rapid_descent(self): self._play(ENABLE_SOUND_RAPID_DESCENT, "rapid_descent_sound")
    def play_return(self): self._play(ENABLE_SOUND_RETURN, "return_sound")
    def play_level_off(self): self._play(ENABLE_SOUND_LEVEL_OFF, "level_off_sound")
    def play_ground_vehicle(self): self._play(ENABLE_SOUND_GROUND_VEHICLE, "ground_vehicle_sound")
    def play_proximity(self): self._play(ENABLE_SOUND_PROXIMITY, "proximity_sound")
    def play_headon(self): self._play(ENABLE_SOUND_HEADON, "headon_sound")
    def play_breakthrough(self): self._play(ENABLE_SOUND_BREAKTHROUGH, "breakthrough_sound")
    def play_squawk_change(self): self._play(ENABLE_SOUND_SQUAWK_CHANGE, "squawk_change_sound")

    def play_squawk(self, sq):
        if not ENABLE_SOUND or not ENABLE_SOUND_SQUAWK: return
        st = f"squawk_{sq}"
        # Squawk emergencies are critical — no cooldown
        if _browser_sound_mode:
            _sound_event_queue.append({"type": st, "t": time.time()})
            return
        play_sound(self.player, getattr(self, f"squawk_{sq}_sound", None))

    def check_silence(self):
        """Unified silence detection point — for both sound and dashboard."""
        now = time.time()
        gap = now - self.last_msg_time

        if gap > SILENCE_TIMEOUT:
            if not self.silence_played:
                self.play_silence()
                self.silence_played = True
                self.silence_active = True
                self._silence_start = self.last_msg_time
                self.new_events.append(
                    f"  {c('[SILENCE]', Color.GRAY)} no signal for {SILENCE_TIMEOUT}s"
                )
        else:
            if self.silence_played:
                if self._silence_start is not None:
                    with self._lock:
                        self.silence_periods.append({
                            "start": self._silence_start,
                            "end": now,
                            "duration": round(now - self._silence_start, 1),
                            "status": "ended"
                        })
                        if len(self.silence_periods) > 50:
                            self.silence_periods.pop(0)
                if ENABLE_SOUND_BREAKTHROUGH:
                    self.play_breakthrough()
                    self.new_events.append(
                        f"  {c('[SIGNAL]', Color.GREEN)} silence ended"
                    )
            self.silence_played = False
            self.silence_active = False

# --- Main methods ---
    def update(self, icao, rssi=None, **kw):
        e = self.ac.setdefault(icao, {"first_seen": time.time()})
        e["msg_count"] = e.get("msg_count", 0) + 1
        if rssi is not None: e["rssi"] = rssi; e["max_rssi"] = max(rssi, e.get("max_rssi", -100))
        if "squawk" in kw and kw["squawk"]:
            sq = kw["squawk"]
            if sq in self.squawk_counts or len(self.squawk_counts) < 50:
                self.squawk_counts[sq] = self.squawk_counts.get(sq, 0) + 1
        if "altitude" in kw and kw["altitude"] is not None:
            old_alt, old_t = e.get("altitude"), e.get("alt_time")
            if old_alt is not None and old_t is not None:
                dt = max(time.time() - old_t, 0.5)
                if dt > 0 and abs(kw["altitude"] - old_alt) / dt * 60 > MAX_ALT_RATE_FPM:
                    self.stats["alt_rejected"] += 1; del kw["altitude"]
            if "altitude" in kw:
                kw["alt_time"] = time.time()
                alt_hist = e.get("alt_history", [])
                alt_hist.append((time.time(), kw["altitude"]))
                if len(alt_hist) > 120:
                    alt_hist.pop(0)
                e["alt_history"] = alt_hist
                # Check rapid descent
                if old_alt is not None and old_t is not None:
                    dt = time.time() - old_t
                    if dt > 0:
                        alt_rate = (kw["altitude"] - old_alt) / dt * 60
                        if alt_rate < -RAPID_DESCENT_FPM and ENABLE_SOUND_RAPID_DESCENT:
                            if not e.get("rapid_descent_played"):
                                e["rapid_descent_played"] = True
                                self.play_rapid_descent()
                                self.stats["rapid_descent_count"] += 1
                                self.new_events.append(
                                    f"  {c('[DESCENT]', Color.RED)} {fmt_vrate(alt_rate)} — {icao}")
                        elif alt_rate > 0:
                            e["rapid_descent_played"] = False
                # Check level-off at cruise
                if old_alt is not None and old_t is not None:
                    dt = time.time() - old_t
                    if dt > 0:
                        alt_rate = (kw["altitude"] - old_alt) / dt * 60
                        old_vr = e.get("prev_alt_rate", 0)
                        if abs(alt_rate) < LEVEL_OFF_FPM and abs(old_vr) > LEVEL_OFF_MIN_VRATE and ENABLE_SOUND_LEVEL_OFF:
                            if not e.get("level_off_played"):
                                e["level_off_played"] = True
                                self.play_level_off()
                                self.stats["level_off_count"] += 1
                                self.new_events.append(
                                    f"  {c('[LEVEL-OFF]', Color.BLUE)} level-off — {icao}")
                        elif abs(alt_rate) > LEVEL_OFF_MIN_VRATE:
                            e["level_off_played"] = False
                        e["prev_alt_rate"] = alt_rate
        # Check squawk change
        if "squawk" in kw:
            old_sq = e.get("squawk")
            if old_sq and old_sq != kw["squawk"] and ENABLE_SOUND_SQUAWK_CHANGE:
                self.play_squawk_change()
                self.stats["squawk_change_count"] += 1
                self.new_events.append(
                    f"  {c('[SQUAWK]', Color.MAGENTA)} {old_sq} → {kw['squawk']} — {icao}")
                if kw["squawk"] in SQUAWK_SPECIAL:
                    self.play_squawk(kw["squawk"])
        e.update(kw); e["last_seen"] = time.time(); e["stale"] = False
        self.last_msg_time = time.time()

        # Speed history (for Chart)
        if "speed_gs" in kw and kw["speed_gs"] is not None:
            spd_hist = e.get("spd_history", [])
            spd_hist.append((time.time(), kw["speed_gs"]))
            if len(spd_hist) > 120:
                spd_hist.pop(0)
            e["spd_history"] = spd_hist

        # Heading history (for Chart)
        if "heading" in kw and kw["heading"] is not None:
            hdg_hist = e.get("hdg_history", [])
            hdg_hist.append((time.time(), kw["heading"]))
            if len(hdg_hist) > 120:
                hdg_hist.pop(0)
            e["hdg_history"] = hdg_hist

        # Vertical rate history (for Chart)
        _vr = kw.get("vrate_tc19")
        if _vr is None:
            _vr = kw.get("vrate_bds60")
        if _vr is not None:
            vr_hist = e.get("vrate_history", [])
            vr_hist.append((time.time(), _vr))
            if len(vr_hist) > 120:
                vr_hist.pop(0)
            e["vrate_history"] = vr_hist

        # Check speed record
        if "speed_gs" in kw and kw["speed_gs"] is not None:
            if kw["speed_gs"] > self.stats.get("max_speed_kt", 0):
                self.stats["max_speed_kt"] = kw["speed_gs"]
                if kw["speed_gs"] > SPEED_RECORD_THRESHOLD and ENABLE_SOUND_SPEED_RECORD:
                    self.play_speed_record()
                    self.new_events.append(
                        f"  {c('[SPEED]', Color.YELLOW)} {fmt_speed(kw['speed_gs'])} — {icao}")
            if kw["speed_gs"] > 0:
                cur_min = self.stats.get("min_speed_kt")
                if cur_min is None or kw["speed_gs"] < cur_min:
                    self.stats["min_speed_kt"] = kw["speed_gs"]

        # Check altitude record
        if "altitude" in kw and kw["altitude"] is not None:
            if kw["altitude"] > self.stats.get("max_alt_ft", 0):
                self.stats["max_alt_ft"] = kw["altitude"]
                if kw["altitude"] > ALT_RECORD_THRESHOLD and ENABLE_SOUND_ALT_RECORD:
                    self.play_alt_record()
                    self.new_events.append(
                        f"  {c('[ALTITUDE]', Color.BLUE)} {fmt_alt(kw['altitude'])} — {icao}")
            if kw["altitude"] > 0:
                cur_min_a = self.stats.get("min_alt_ft")
                if cur_min_a is None or kw["altitude"] < cur_min_a:
                    self.stats["min_alt_ft"] = kw["altitude"]

    def event(self, icao, text):
        if FILTER_RADIUS_KM > 0 and not in_range(self.ac.get(icao, {})): return
        cs = self.ac.get(icao, {}).get("callsign", "")
        self.new_events.append(f"  {icao}  {cs:<8} {text}")
        if LOG_MODE == "screen":
            logged = False
            for tag, cols in LOG_COL_MAP.items():
                if text.startswith(tag):
                    if any(col in COL_SHOW for col in cols): logged = True
                    break
            if not logged: return
        self.logger.log_event(icao, cs, text)

    def check_alert(self, icao, callsign=""):
        with self._lock:
            ac = self.ac.get(icao)
            if ac is None: return
            sq = ac.get("squawk", "")
            if not ac.get("typecode"):
                info = AIRCRAFT_INFO.get(icao, {})
                if info.get("type"):
                    ac["typecode"] = info["type"]
                    ac["model"] = info.get("model", "")
                    ac["manufacturer"] = info.get("manufacturer", "")
                    ac["operator"] = info.get("operator", "")
                    ac["reg"] = info.get("reg", "")
                    ac["country"] = info.get("country", "")
            if FILTER_RADIUS_KM > 0:
                dist = ac.get("distance")
                if dist is not None and dist > FILTER_RADIUS_KM: return
            is_return = icao in self.recently_lost and ENABLE_SOUND_RETURN
            if ac.get("alerted") and not is_return:
                return
            cs = callsign or ac.get("callsign", "")
            if not ac.get("alerted"):
                ac["alerted"] = True; self.stats["total_aircraft"] += 1
            if sq in ("7500","7600","7700"):
                self.play_squawk(sq)
                sq_info = f"  {c('SQUAWK '+sq+': '+SQUAWK_SPECIAL[sq], Color.RED)}"
            elif is_return:
                self.play_return()
                self.stats["return_count"] += 1
                sq_info = ""
            else: self.play_beep(); sq_info = ""
            if is_return:
                self.new_events.append(f"  {c('[RETURN]', Color.CYAN)} {icao} ({cs}) returned")
                self.event(icao, f"[RETURN] {cs} returned after loss")
                del self.recently_lost[icao]
            else:
                self.new_events.append(f"  {c('[NEW]', Color.GREEN)}: {c(icao, Color.CYAN)} {c(cs, Color.GREEN)}{sq_info}")
                self.event(icao, f"[NEW] {cs}{sq_info}")
            cat = ac.get("category", "")
            if cat in MIL_CATEGORIES:
                self.play_mil()
                self.new_events.append(f"  {c('[MIL]', Color.YELLOW)} {cat} {AC_CAT_DESC.get(cat,'')}")
            if cat == "A7" and ENABLE_SOUND_HELICOPTER:
                self.play_helicopter()
                self.new_events.append(f"  {c('[HELICOPTER]', Color.GREEN)} {icao}")
            if cat and cat[0] == "C" and ENABLE_SOUND_GROUND_VEHICLE:
                self.play_ground_vehicle()
                self.new_events.append(f"  {c('[GROUND VEHICLE]', Color.YELLOW)} {cat} {AC_CAT_DESC.get(cat,'')}")
            if cs and cs in TRACKED_CALLSIGNS:
                self.play_tracked(); self.new_events.append(f"  {c('[TRACKING]', Color.MAGENTA)} {cs}")
            elif icao in TRACKED_ICAO:
                self.play_tracked(); self.new_events.append(f"  {c('[TRACKING]', Color.MAGENTA)} {icao}")
            alt = ac.get("altitude")
            if alt is not None:
                alt_m = conv_alt(alt)
                if alt_m is not None and alt_m < LOW_ALT_THRESHOLD:
                    self.play_low_alt()
                    self.new_events.append(f"  {c('[LOW]', Color.YELLOW)} {fmt_alt(alt)}")
            # Check rare type
            info = AIRCRAFT_INFO.get(icao, {})
            type_code = info.get("type", "")
            if type_code in RARE_TYPES and ENABLE_SOUND_RARE_TYPE:
                self.play_rare_type()
                self.new_events.append(
                    f"  {c('[RARE TYPE]', Color.MAGENTA)} {type_code} — {icao}")

    def check_tcas_ra(self, icao, bds):
        if not (bds and bds.get("acas_rat")): return
        with self._lock:
            ac = self.ac.get(icao)
            if ac is None or ac.get("tcas_alerted"): return
            ac["tcas_alerted"] = True
        self.play_tcas()
        self.new_events.append(f"  {c('[TCAS RA]', Color.RED)} {icao} — resolution advisory!")

    def _process_elm(self, icao, bits, is_reply=False):
        is_elm = bool(bits[32])
        if not is_elm:
            self.event(icao, f"[DF{'25' if is_reply else '24'}] Comm-D (non-ELM)")
            return
        sp = bits_to_int(bits[33:37])
        sn = bits_to_int(bits[37:40])
        seg_data = bits[40:88]
        if sp == 0 or sp > 16:
            self.event(icao, f"[ELM] invalid SP={sp}")
            return
        buf = self.elm_buffers.setdefault(icao, {"sp": sp, "segs": {}, "t": time.time()})
        if buf["sp"] != sp or time.time() - buf["t"] > 10:
            buf = {"sp": sp, "segs": {}, "t": time.time()}
            self.elm_buffers[icao] = buf
        buf["segs"][sn] = seg_data
        buf["t"] = time.time()
        if len(buf["segs"]) >= sp:
            assembled = []
            for i in range(sp):
                if i not in buf["segs"]:
                    self.event(icao, f"[ELM] Missing segment {i}")
                    del self.elm_buffers[icao]
                    return
                assembled.extend(buf["segs"][i])
            del self.elm_buffers[icao]
            if len(assembled) >= 56:
                pb = [0]*32 + assembled[:56] + [0]*24
                bds = guess_bds(pb, self.ac.get(icao, {}))
                self.event(icao, f"[ELM] Assembled: {identify_bds(bds) if bds else f'{sp} segs (unknown)'}")
        else:
            self.event(icao, f"[DF{'25' if is_reply else '24'}] ELM seg {sn+1}/{sp}")

    def update_cpr(self, icao, lat_cpr, lon_cpr, fmt, t, ground=False):
        store = self.cpr_ground if ground else self.cpr
        key = "even" if fmt == 0 else "odd"
        store.setdefault(icao, {})[key] = {"lat": lat_cpr, "lon": lon_cpr, "t": t}
        ev = store.get(icao, {}).get("even"); od = store.get(icao, {}).get("odd")
        if ev and od and abs(ev["t"] - od["t"]) > CPR_TIMEOUT:
            if ev["t"] >= od["t"]:
                store[icao].pop("odd", None); od = None
            else:
                store[icao].pop("even", None); ev = None
        if ev and od:
            even_newer = ev["t"] >= od["t"]
            lat, lon = cpr_decode(ev["lat"], ev["lon"], od["lat"], od["lon"], even_newer, ground=ground)
            if lat is not None and self._validate_and_update(icao, lat, lon, ground):
                return (lat, lon)
        ac = self.ac.get(icao, {})
        ref_lat, ref_lon = ac.get("lat"), ac.get("lon")
        if ref_lat is not None and ref_lon is not None and ac.get("pos_reliable", 0) >= POSITION_RELIABLE_MIN:
            lat, lon = cpr_decode_local(lat_cpr, lon_cpr, fmt, ref_lat, ref_lon, ground=ground)
            if lat is not None and self._validate_and_update(icao, lat, lon, ground):
                return (lat, lon)
        return None

    def _validate_and_update(self, icao, lat, lon, ground):
        ac = self.ac.get(icao, {})
        old_lat, old_lon = ac.get("lat"), ac.get("lon")
        pos_rel = ac.get("pos_reliable", 0)
        dist_home = haversine_km(HOME_LAT, HOME_LON, lat, lon)
        if dist_home > MAX_RANGE_KM:
            self.stats["pos_jumps_rejected"] += 1; ac["pos_reliable"] = 0
            ac["lat"] = ac["lon"] = ac["lat_dr"] = ac["lon_dr"] = ac["distance_dr"] = None
            return False
        if old_lat is not None and old_lon is not None:
            jump = haversine_km(old_lat, old_lon, lat, lon)
            if jump > (MAX_POSITION_JUMP_KM if pos_rel >= POSITION_RELIABLE_MIN else 500.0):
                self.stats["pos_jumps_rejected"] += 1; ac["pos_reliable"] = 0
                ac["lat"] = ac["lon"] = ac["lat_dr"] = ac["lon_dr"] = ac["distance_dr"] = None
                return False
        if ENABLE_SMOOTHING and old_lat is not None and old_lon is not None and pos_rel >= POSITION_RELIABLE_MIN:
            lat = old_lat + SMOOTHING_ALPHA * (lat - old_lat)
            lon = old_lon + SMOOTHING_ALPHA * (lon - old_lon)
        dist = haversine_km(HOME_LAT, HOME_LON, lat, lon)
        self.update(icao, lat=lat, lon=lon, on_ground=ground, distance=dist, pos_time=time.time())
        ac = self.ac.get(icao, {})
        ac["pos_reliable"] = min(ac.get("pos_reliable", 0) + 1, POSITION_RELIABLE_MAX)

        # Trail history
        trail = ac.get("trail", [])
        last_trail_t = ac.get("trail_time", 0)
        if time.time() - last_trail_t >= TRAIL_MIN_INTERVAL:
            trail.append((lat, lon))
            if len(trail) > TRAIL_MAX_POINTS:
                trail.pop(0)
            ac["trail"] = trail
            ac["trail_time"] = time.time()

        # Coverage
        bear = bearing_to(HOME_LAT, HOME_LON, lat, lon)
        bin_idx = int(bear / 5.0) % 72
        if dist > self.coverage_bins[bin_idx]:
            self.coverage_bins[bin_idx] = dist
        hm_key = f"{round(lat, 2)},{round(lon, 2)}"
        if hm_key in self.heatmap_grid or len(self.heatmap_grid) < self._heatmap_max_cells:
            self.heatmap_grid[hm_key] = self.heatmap_grid.get(hm_key, 0) + 1
        if rssi is not None:
            self.update_rssi_bin(bear, rssi)

        # Range record
        if dist > self.stats["max_range_km"]:
            self.stats["max_range_km"] = dist
            if ac.get("alerted") and ENABLE_SOUND_RECORD:
                self.play_record()
                self.new_events.append(f"  {c('[RECORD]', Color.CYAN)} {fmt_distance(dist)} — {icao}")

        # Long-range aircraft
        if dist > LONG_RANGE_THRESHOLD_KM and not ac.get("long_range_played") and ENABLE_SOUND_LONG_RANGE:
            ac["long_range_played"] = True
            self.play_long_range()
            self.stats["long_range_count"] += 1
            self.new_events.append(f"  {c('[LONG RANGE]', Color.CYAN)} {fmt_distance(dist)} — {icao}")

        # Overhead pass
        prev_dist = ac.get("prev_distance")
        ac["prev_distance"] = dist
        if prev_dist is not None and dist > prev_dist and prev_dist < OVERHEAD_THRESHOLD_KM:
            if not ac.get("overhead_played") and ENABLE_SOUND_OVERHEAD:
                ac["overhead_played"] = True
                self.play_overhead()
                self.stats["overhead_count"] += 1
                self.new_events.append(
                    f"  {c('[OVERHEAD]', Color.MAGENTA)} {icao} ({ac.get('callsign','')}) — {fmt_distance(prev_dist)}")

        if self.sbs1_encoder:
            try: self.sbs1_encoder.send(icao, callsign=ac.get("callsign",""), lat=lat, lon=lon,
                                         alt=ac.get("altitude"), speed=get_display_speed(ac),
                                         heading=ac.get("heading"), squawk=ac.get("squawk",""))
            except Exception: pass
        return True

    def check_proximity(self):
        """Check proximity and head-on courses between active aircraft."""
        if not (ENABLE_SOUND_PROXIMITY or ENABLE_SOUND_HEADON): return
        now = time.time()
        if now - self._proximity_checked < 10: return
        self._proximity_checked = now
        with self._lock:
            icaos = [ic for ic, a in self.ac.items()
                     if a.get("lat") is not None and a.get("lon") is not None
                     and a.get("pos_reliable", 0) >= POSITION_RELIABLE_MIN]
            ac_snapshot = {ic: self.ac[ic] for ic in icaos}
        for i in range(len(icaos)):
            for j in range(i + 1, len(icaos)):
                a1 = ac_snapshot[icaos[i]]; a2 = ac_snapshot[icaos[j]]
                lat1, lon1 = a1.get("lat_dr", a1.get("lat")), a1.get("lon_dr", a1.get("lon"))
                lat2, lon2 = a2.get("lat_dr", a2.get("lat")), a2.get("lon_dr", a2.get("lon"))
                if lat1 is None or lat2 is None: continue
                dist = haversine_km(lat1, lon1, lat2, lon2)
                alt_diff = abs((a1.get("altitude") or 0) - (a2.get("altitude") or 0))
                # Proximity
                if dist < PROXIMITY_HORIZ_KM and alt_diff < PROXIMITY_VERT_FT:
                    pair_key = tuple(sorted([icaos[i], icaos[j]]))
                    if not a1.get("proximity_played") and ENABLE_SOUND_PROXIMITY:
                        a1["proximity_played"] = True; a2["proximity_played"] = True
                        self.play_proximity()
                        self.stats["proximity_count"] += 1
                        self.new_events.append(
                            f"  {c('[PROXIMITY]', Color.YELLOW)} {icaos[i]} ↔ {icaos[j]} — {fmt_distance(dist)}")
                # Head-on courses
                h1, h2 = a1.get("heading"), a2.get("heading")
                if h1 is not None and h2 is not None and dist < HEADON_DIST_KM:
                    diff = abs(h1 - h2)
                    if diff > 180: diff = 360 - diff
                    if diff > HEADON_COURSE_DIFF:
                        if not a1.get("headon_played") and ENABLE_SOUND_HEADON:
                            a1["headon_played"] = True; a2["headon_played"] = True
                            self.play_headon()
                            self.stats["headon_count"] += 1
                            self.new_events.append(
                                f"  {c('[HEAD-ON]', Color.MAGENTA)} {icaos[i]} ({h1:.0f}°) ↔ {icaos[j]} ({h2:.0f}°)")

    def dead_reckon(self):
        if not ENABLE_DEAD_RECKONING: return
        now = time.time()
        with self._lock:
            for icao, a in self.ac.items():
                if a.get("lat") is None or a.get("lon") is None: continue
                if a.get("pos_reliable", 0) < POSITION_RELIABLE_MIN: continue
                dt = now - a.get("pos_time", now)
                if dt < 1 or dt > DR_MAX_AGE: continue
                spd = a.get("speed_gs")
                hdg = a.get("heading")
                if spd is None or hdg is None or spd < 1: continue
                spd_kms = spd * 1.852 / 3600.0; dist = spd_kms * dt
                a["lat_dr"] = a["lat"] + dist*math.cos(math.radians(hdg))/111.32
                a["lon_dr"] = a["lon"] + dist*math.sin(math.radians(hdg))/(111.32*max(math.cos(math.radians(a["lat"])),0.01))
                a["distance_dr"] = haversine_km(HOME_LAT, HOME_LON, a["lat_dr"], a["lon_dr"])

    def compute_doppler(self, iq_complex, freq_msg, aircraft_speed_kt, aircraft_bearing_deg, icao=None):
        """Estimate PPM from IQ samples using phase slope."""
        if iq_complex is None or len(iq_complex) < 16:
            return
        if aircraft_speed_kt is None:
            return
        try:
            c = 299792458.0
            freq_carrier = 1090000000.0
            v_kt = aircraft_speed_kt * 0.514444
            if aircraft_bearing_deg is not None:
                bearing_rad = math.radians(aircraft_bearing_deg)
                doppler_expected = 2.0 * v_kt * math.cos(bearing_rad) / c * freq_carrier
                has_coords = True
            else:
                doppler_expected = 0.0
                has_coords = False

            mag = np.abs(iq_complex)
            if np.max(mag) < 1e-6:
                return

            noise_level = float(np.percentile(mag, 25))
            signal_level = float(np.percentile(mag, 75))
            threshold = (noise_level + signal_level) / 2.0
            if threshold < 1e-8:
                threshold = float(np.median(mag))

            on_mask = mag > threshold
            if int(np.sum(on_mask)) < 30:
                return

            on_samples = iq_complex[on_mask]
            on_indices = np.where(on_mask)[0].astype(np.float64)
            weights = mag[on_mask]
            phases = np.unwrap(np.angle(on_samples))
            t = on_indices / SAMPLE_RATE
            if len(t) < 5:
                return

            W = np.sum(weights)
            if W < 1e-10:
                return
            wt = np.sum(weights * t)
            wy = np.sum(weights * phases)
            wtt = np.sum(weights * t * t)
            wty = np.sum(weights * t * phases)
            det = W * wtt - wt * wt
            if abs(det) < 1e-20:
                return
            slope = (W * wty - wt * wy) / det
            freq_offset = slope / (2 * math.pi)
            if abs(freq_offset) > 500000:
                return

            ppm_corr = (freq_offset - doppler_expected) / freq_carrier * 1e6
            if abs(ppm_corr) > 200:
                return

            self.doppler_data["measurements"].append({
                "t": time.time(), "icao": icao or "",
                "measured_hz": round(freq_offset, 1),
                "expected_hz": round(doppler_expected, 1),
                "ppm": round(ppm_corr, 2),
                "no_coords": not has_coords
            })
            if len(self.doppler_data["measurements"]) > 100:
                self.doppler_data["measurements"].pop(0)

            recent = self.doppler_data["measurements"][-20:]
            ppms = sorted(m["ppm"] for m in recent)
            self.doppler_data["ppm_estimate"] = round(ppms[len(ppms) // 2], 2)
        except Exception:
            log.exception("compute_doppler: error")



    def cleanup(self):
        now = time.time()
        with self._lock:
            for icao in list(self.mac_pending):
                if now - self.mac_pending[icao]["last_seen"] > 10: del self.mac_pending[icao]
            active = set()
            for icao in list(self.ac):
                age = now - self.ac[icao].get("last_seen", 0)
                is_mac = icao.startswith("~")
                timeout = LOSS_TIMEOUT * 2 if is_mac else LOSS_TIMEOUT
                if FILTER_REMOVE_OUT_OF_RANGE and FILTER_RADIUS_KM > 0 and not in_range(self.ac[icao]):
                    cs = self.ac[icao].get('callsign', '')
                    if not is_mac:
                        self.new_events.append(f"  {c('[OUT OF RADIUS]', Color.GRAY)} {icao} ({cs})")
                        self.logger.log_event(icao, cs, "[OUT OF RADIUS] removed")
                    del self.ac[icao]; self.cpr.pop(icao, None); self.cpr_ground.pop(icao, None); continue
                if age > timeout:
                    cs = self.ac[icao].get("callsign", "")
                    if is_mac:
                        self.new_events.append(f"  {c('[MAC LOSS]', Color.GRAY)} {icao} ({cs})")
                        self.logger.log_event(icao, cs, "[MAC LOSS] contact lost")
                    else:
                        self.new_events.append(f"  {c('[LOSS]', Color.YELLOW)} {icao} ({cs})")
                        self.logger.log_event(icao, cs, "[LOSS] contact lost")
                        self.play_loss_of_contact()
                    del self.ac[icao]; self.cpr.pop(icao, None); self.cpr_ground.pop(icao, None)
                    self.recently_lost[icao] = now
                    continue
                if icao in self.recently_lost: del self.recently_lost[icao]
                active.add(icao)
            for icao in list(self.recently_lost):
                if now - self.recently_lost[icao] > RETURN_MEMORY_TIME: del self.recently_lost[icao]
        self.logger.close_old(active)

    def print_events(self):
        if not ENABLE_EVENT_LOG:
            self.new_events.clear(); return
        if not self.new_events: return
        print(c("── Event log ──", Color.GRAY))
        for e in self.new_events[-30:]: print(e)
        self.new_events.clear(); print()

    def _sort_key_safe(self, icao, ac_ref):
        a = ac_ref.get(icao)
        if a is None: return (chr(127), icao)
        if SORT_BY == "callsign": return (a.get("callsign") or "ZZZZ", icao)
        elif SORT_BY == "altitude": return (-a["altitude"] if a.get("altitude") is not None else 99999, icao)
        elif SORT_BY == "speed": return (-(get_display_speed(a) or -1), icao)
        elif SORT_BY == "distance": return (a.get("distance") or 99999, icao)
        elif SORT_BY == "age": return (time.time() - a.get("last_seen", 0), icao)
        return (icao,)
    def _sort_key(self, icao):
        return self._sort_key_safe(icao, self.ac)

    def print_table(self, quality=None):
        with self._lock:
            ac_snapshot = dict(self.ac)
        if not ac_snapshot: print(c("No aircraft detected...", Color.GRAY)); return
        active = get_active_columns()
        if not active: return
        self.dead_reckon()
        sep = sum(col["width"] + 1 for _, col in active) + 1
        print(f"{'─'*sep}")
        print(f" {c('ACTIVE AIRCRAFT', Color.BOLD)} — {c('Quality:', Color.GRAY)} {c(quality_bar(quality), Color.GRAY)}" if quality is not None else f" {c('ACTIVE AIRCRAFT', Color.BOLD)}")
        header = " "
        for _, col in active:
            lbl = col["label"].rjust(col["width"])
            header += c(lbl, col["color"]) + " " if col["color"] else lbl + " "
        print(header); print(f"{'─'*sep}")
        stale_count = 0
        for icao in sorted(ac_snapshot, key=lambda ic: self._sort_key_safe(ic, ac_snapshot)):
            a = ac_snapshot[icao]
            if FILTER_RADIUS_KM > 0 and not in_range(a): continue
            if FILTER_EMERGENCY_ONLY:
                if a.get("squawk","") not in SQUAWK_SPECIAL and a.get("emergency","No emergency") == "No emergency": continue
            age = int(time.time() - a.get("last_seen", time.time()))
            is_stale = a.get("stale", False)
            if is_stale: stale_count += 1
            disp_lat = a.get("lat_dr", a.get("lat")); disp_lon = a.get("lon_dr", a.get("lon"))
            disp_dist = a.get("distance_dr", a.get("distance")); pos_rel = a.get("pos_reliable", 0)
            info = AIRCRAFT_INFO.get(icao, {})
            hazard_parts = []
            if a.get("turbulence") and a.get("turbulence") != "None": hazard_parts.append(f"turb:{a['turbulence']}")
            if a.get("icing") and a.get("icing") != "None": hazard_parts.append(f"icing:{a['icing']}")
            if a.get("windshear") and a.get("windshear") != "None": hazard_parts.append(f"wshear:{a['windshear']}")
            vals = {
                "icao": icao, "callsign": (a.get("callsign") or "")[:9],
                "reg": info.get("reg","") or "—", "type": info.get("type","") or "—",
                "model": info.get("model","") or "—", "manufacturer": info.get("manufacturer","") or "—",
                "operator": info.get("operator","") or "—", "owner": info.get("owner","") or "—",
                "country": info.get("country","") or "—", "engines": info.get("engines","") or "—",
                "category": a.get("category","")[:4], "altitude": fmt_alt(a.get("altitude")),
                "speed": fmt_speed(get_display_speed(a)), "speed_ias": fmt_speed(a.get("speed_ias")),
                "speed_tas": fmt_speed(a.get("speed_tas")),
                "heading": f"{a['heading']:.0f}°" if a.get("heading") is not None else "—",
                "vrate": fmt_vrate(get_display_vrate(a)), "vr_src": a.get("vr_source_tc19","—"),
                "lat": f"{disp_lat:.5f}" if disp_lat is not None else "—",
                "lon": f"{disp_lon:.5f}" if disp_lon is not None else "—",
                "ground": "YES" if a.get("on_ground") else "—",
                "autopilot": a.get("autopilot","—"), "emergency": a.get("emergency","—")[:8],
                "age": f"{age}s", "squawk": a.get("squawk","—"),
                "distance": fmt_distance(disp_dist),
                "phase": determine_phase(a.get("altitude"), get_display_vrate(a), a.get("on_ground",False), a.get("ground_speed")) if (a.get("altitude") is not None or a.get("on_ground")) else "—",
                "gnss": fmt_alt(a.get("gnss_altitude")), "sel_alt": fmt_alt(a.get("selected_altitude")),
                "sel_hdg": f"{a['selected_heading']:.0f}°" if a.get("selected_heading") is not None else "—",
                "qnh": fmt_pressure(a.get("baro_setting")), "nac": a.get("nac_p","—"),
                "sil": a.get("sil","—"), "nacv": a.get("nacv","—"),
                "version": a.get("adsb_version","—"), "nic": a.get("nic","—"),
                "weather": (f"{a['wind_dir']:.0f}°/{a['wind_spd']:.0f}kt {fmt_temp(a.get('temp'))}" if a.get("wind_dir") is not None else "—"),
                "hazard": " ".join(hazard_parts) if hazard_parts else "—",
                "tis_b": "T" if a.get("tis_b") else "—",
                "rssi": f"{a['rssi']:.0f}" if a.get("rssi") is not None else "—",
                "reliable": str(pos_rel) if pos_rel > 0 else "—",
            }
            row = " "
            for k, col in active:
                v = str(vals.get(k, "—")).rjust(col["width"])
                if is_stale: v = c(v, Color.GRAY)
                elif k == "emergency" and a.get("emergency","No emergency") != "No emergency": v = c(v, Color.RED)
                elif k == "squawk" and a.get("squawk") in SQUAWK_SPECIAL: v = c(v, Color.RED)
                elif k == "hazard" and hazard_parts: v = c(v, Color.YELLOW)
                elif col["color"] and k != "emergency": v = c(v, col["color"])
                row += v + " "
            print(row)
        print(f"{'─'*sep}")
        stale_info = f" | {c('Stale', Color.GRAY)}: {stale_count}" if stale_count else ""
        sort_info = f" | {c('Sort:', Color.GRAY)} {SORT_BY}" if SORT_BY != "icao" else ""
        filter_info = f" | {c('Filter:', Color.RED)} EMERGENCY" if FILTER_EMERGENCY_ONLY else ""
        radius_info = f" | {c('Radius:', Color.CYAN)} {fmt_radius(FILTER_RADIUS_KM)}" if FILTER_RADIUS_KM > 0 else ""
        print(f"Aircraft: {len(self.ac)} | Session total: {self.stats['total_aircraft']}{stale_info}{sort_info}{filter_info}{radius_info}")

    def print_stats(self, msg_total, msg_valid, t_start):
        rate = msg_valid / max(time.time() - t_start, 1)
        print(f"Preambles: {msg_total} | Valid: {msg_valid} | {rate:.1f} msg/s | Aircraft: {self.stats['total_aircraft']}")
        print(f"Quality: decode={self.decode_quality_ema:.0f}% | receive={self.recv_quality_ema:.0f}%")
        if self.ppm_adjusted != PPM_CORRECTION: print(f"PPM: {PPM_CORRECTION:+d} -> {self.ppm_adjusted:+d} (auto)")
        if self.stats.get("crc_corrected",0) > 0:
            print(f"CRC corrected: {self.stats['crc_corrected']} (1-bit: {self.stats['crc_corrected']-self.stats.get('crc_two_bit_corrected',0)}, 2-bit: {self.stats.get('crc_two_bit_corrected',0)})")
        if self.stats["bds_attempts"] > 0:
            print(f"BDS: attempts={self.stats['bds_attempts']} success={self.stats['bds_success']} unrecognized={self.stats['bds_fail']} ({self.stats['bds_success']/self.stats['bds_attempts']*100:.0f}%)")
        if self.stats.get("mode_ac_count",0) > 0: print(f"Mode A/C: {self.stats['mode_ac_count']} | Emerg.sqwk: {self.stats.get('mode_ac_emergency',0)}")
        if self.stats.get("queue_drops",0) > 0: print(f"Queue: lost {self.stats['queue_drops']} messages")
        rej = self.stats
        if rej["pos_jumps_rejected"] or rej["alt_rejected"] or rej.get("vel_rejected",0) or rej.get("hdg_rejected",0) or rej.get("bds_rejected",0):
            print(f"Filters: pos={rej['pos_jumps_rejected']} alt={rej['alt_rejected']} spd={rej.get('vel_rejected',0)} hdg={rej.get('hdg_rejected',0)} BDS={rej.get('bds_rejected',0)}")
        if self.stats.get("max_range_km",0) > 0: print(f"Max range: {fmt_distance(self.stats['max_range_km'])}")
        if self.stats.get("max_speed_kt",0) > 0: print(f"Max speed: {fmt_speed(self.stats['max_speed_kt'])}")
        if self.stats.get("max_alt_ft",0) > 0: print(f"Max altitude: {fmt_alt(self.stats['max_alt_ft'])}")
        parts = []
        for k, label in [("nic_decoded","NIC"),("sil_decoded","SIL"),("nacv_decoded","NACv"),("version_decoded","Ver")]:
            if self.stats.get(k,0) > 0: parts.append(f"{label}={self.stats[k]}")
        if parts: print(f"Integrity: {' | '.join(parts)}")
        if self.stats.get("bds45_count",0) > 0: print(f"BDS 4,5 (weather): {self.stats['bds45_count']}")
        if self.stats.get("uat_count",0) > 0: print(f"UAT 978: {self.stats['uat_count']} messages")
        # Extended sound statistics
        ext_parts = []
        for k, label in [("long_range_count","long range"),("overhead_count","overhead"),
                         ("rapid_descent_count","descents"),("squawk_change_count","squawk changes"),
                         ("return_count","returns"),("level_off_count","level-offs"),
                         ("proximity_count","proximity"),("headon_count","head-on")]:
            if self.stats.get(k,0) > 0: ext_parts.append(f"{label}={self.stats[k]}")
        if ext_parts: print(f"Events: {' | '.join(ext_parts)}")
        if self.stats.get("preambles_total",0) > 0:
            print(f"Preambles: total={self.stats['preambles_total']} strong={self.stats.get('preambles_strong',0)/self.stats['preambles_total']*100:.0f}%")

    def process_mode_ac(self, squawk, alt_ft=None, rssi=None, spi=False):
        if not squawk: return
        icao = f"~{squawk}"; self.stats["mode_ac_count"] += 1
        if icao not in self.ac:
            if icao not in self.mac_pending:
                self.mac_pending[icao] = {"squawk": squawk, "last_seen": time.time(), "rssi": rssi, "alt_ft": alt_ft}
                return
            del self.mac_pending[icao]
            self.ac[icao] = {"first_seen": time.time(), "last_seen": time.time(), "squawk": squawk, "rssi": rssi, "alerted": True}
            if alt_ft is not None: self.ac[icao]["altitude"] = alt_ft; self.ac[icao]["alt_time"] = time.time()
            self.stats["total_aircraft"] += 1
            if spi: self.ac[icao]["ident"] = True; self.event(icao, "[MODE-A/C] IDENT (SPI)")
            if squawk in SQUAWK_SPECIAL:
                self.stats["mode_ac_emergency"] += 1; self.play_squawk(squawk)
                self.ac[icao]["emergency"] = SQUAWK_SPECIAL[squawk]
                self.new_events.append(f"  {c('[MAC EMERGENCY]', Color.RED)} squawk {squawk}: {SQUAWK_SPECIAL[squawk]}")
            else: self.play_beep()
            self.new_events.append(f"  {c('[MAC]', Color.MAGENTA)} squawk {squawk}")
            return
        e = self.ac[icao]; e["last_seen"] = time.time()
        e["msg_count"] = e.get("msg_count", 0) + 1
        if rssi is not None: e["rssi"] = rssi
        if alt_ft is not None: e["altitude"] = alt_ft; e["alt_time"] = time.time()
        if spi: e["ident"] = True; self.event(icao, "[MODE-A/C] IDENT (SPI)")

    def process(self, bits, t, rssi=None):
        with self._lock:
            df = bits_to_int(bits[:5])
            self.on_message_received(df, rssi)
            self.last_msg_time = time.time()
            self.stats["msg_by_type"][f"DF{df}"] = self.stats["msg_by_type"].get(f"DF{df}", 0) + 1
            if self.beast_encoder:
                try: self.beast_encoder.send(bits, rssi=rssi, mlat_ts=t if ENABLE_MLAT_TIMESTAMPS else None)
                except Exception as e:
                    if DEBUG: sys.stderr.write(f"[beast] process send error: {e}\n")
            if df == 17:
                icao = format(bits_to_int(bits[8:32]), '06X')
                self._process_adsb(icao, bits_to_int(bits[32:37]), bits, t, rssi)
            elif df == 18 and ENABLE_DF18:
                cf = bits_to_int(bits[5:8])
                self.stats["msg_by_type"][f"DF18-CF{cf}"] = self.stats["msg_by_type"].get(f"DF18-CF{cf}", 0) + 1
                if cf in (0, 2, 6):
                    icao = format(bits_to_int(bits[8:32]), '06X')
                    self._process_adsb(icao, bits_to_int(bits[32:37]), bits, t, rssi)
                    self.update(icao, rssi=rssi, tis_b=True)
                    self.event(icao, f"[DF18] TIS-B/ADS-R CF={cf}")
                elif cf == 1:
                    icao = "~" + format(bits_to_int(bits[8:32]), '06X')
                    self._process_adsb(icao, bits_to_int(bits[32:37]), bits, t, rssi)
                    self.update(icao, rssi=rssi, tis_b=True)
                    self.event(icao, f"[DF18] TIS-B non-ICAO CF={cf}")
                elif cf in (3, 4, 5):
                    icao = format(bits_to_int(bits[8:32]), '06X')
                    self._process_adsb(icao, bits_to_int(bits[32:37]), bits, t, rssi)
                    self.update(icao, rssi=rssi, tis_b=True, military=True)
                    self.event(icao, f"[DF18] Mil TIS-B/ADS-R CF={cf}")
            elif df == 19 and ENABLE_DF19:
                icao = format(bits_to_int(bits[8:32]), '06X')
                self._process_adsb(icao, bits_to_int(bits[32:37]), bits, t, rssi)
                self.event(icao, f"[DF19] Mil Ext Squitter TC={bits_to_int(bits[32:37])}")
            elif df == 0:
                icao = format(self.crc24_icao(bits, 56), '06X')
                alt = decode_altitude(bits, 19)
                if alt is not None: self.update(icao, rssi=rssi, altitude=alt); self.check_alert(icao); self.event(icao, f"[DF0] altitude {fmt_alt(alt)}")
            elif df == 1:
                icao = format(self.crc24_icao(bits, 56), '06X')
                sp1 = bool(bits[19])
                self.update(icao, rssi=rssi, spi=sp1); self.check_alert(icao)
                self.event(icao, f"[DF1] Short ACAS reply" + (" SPI" if sp1 else ""))
            elif df == 2:
                icao = format(self.crc24_icao(bits, 112), '06X')
                alt = decode_altitude(bits, 19)
                if alt is not None: self.update(icao, rssi=rssi, altitude=alt)
                mb = bits[40:88]
                if any(mb):
                    pb = bits[:32] + mb + bits[88:]
                    bds = try_bds30(pb)
                    if bds and bds.get("acas_rat"): self.check_tcas_ra(icao, bds)
                self.check_alert(icao)
                self.event(icao, f"[DF2] Long ACAS alt={fmt_alt(alt) if alt is not None else '?'}")
            elif df == 6:
                icao = format(self.crc24_icao(bits, 56), '06X')
                self.update(icao, rssi=rssi); self.check_alert(icao)
                self.event(icao, "[DF6] ACAS (reserved)")
            elif df == 8:
                icao = format(self.crc24_icao(bits, 112), '06X')
                self.update(icao, rssi=rssi); self.check_alert(icao)
                self.event(icao, "[DF8] ACAS (reserved)")
            elif df == 12:
                icao = format(self.crc24_icao(bits, 56), '06X')
                self.update(icao, rssi=rssi); self.check_alert(icao)
                self.event(icao, "[DF12] ACAS (reserved)")
            elif df == 15:
                icao = format(self.crc24_icao(bits, 112), '06X')
                self.update(icao, rssi=rssi); self.check_alert(icao)
                self.event(icao, "[DF15] (reserved)")
            elif df == 22:
                icao = format(self.crc24_icao(bits, 112), '06X')
                self.update(icao, rssi=rssi, military=True); self.check_alert(icao)
                self.event(icao, "[DF22] Military Ext Squitter")
            elif df == 4:
                icao = format(self.crc24_icao(bits, 56), '06X')
                alt = decode_altitude(bits, 19)
                if alt is not None: self.update(icao, rssi=rssi, altitude=alt); self.check_alert(icao)
            elif df == 5:
                icao = format(self.crc24_icao(bits, 56), '06X')
                sq = decode_squawk(bits, 19); self.update(icao, rssi=rssi, squawk=sq)
                if sq in SQUAWK_SPECIAL: self.event(icao, f"[DF5] SQUAWK {sq}: {SQUAWK_SPECIAL[sq]}")
                self.check_alert(icao)
            elif df == 11:
                icao = format(bits_to_int(bits[8:32]), '06X')
                ca = bits_to_int(bits[5:8])
                _CA = {0:"no Level2",1:"Level2 (comm-B)",2:"comm-B+C",3:"comm-B+C+uplink",
                       4:"comm-A",5:"comm-A+B",6:"comm-A+B+uplink",7:"unspecified"}
                self.update(icao, rssi=rssi, transponder_ca=_CA.get(ca, f"?{ca}"))
                self.check_alert(icao)
            elif df == 16:
                icao = format(self.crc24_icao(bits, 112), '06X')
                alt = decode_altitude(bits, 40)
                if alt is not None: self.update(icao, rssi=rssi, altitude=alt)
                mb = bits[40:88]
                if any(mb):
                    pb = bits[:32] + mb + bits[88:]
                    bds = try_bds30(pb)
                    if bds and bds.get("acas_rat"):
                        self.check_tcas_ra(icao, bds)
                        av, ah = bds.get("ara_vertical",""), bds.get("ara_horizontal","")
                        parts = [f"[DF16] ACAS RA alt={fmt_alt(alt) if alt is not None else '?'}"]
                        if av: parts.append(f"V:{av}")
                        if ah: parts.append(f"H:{ah}")
                        self.event(icao, " ".join(parts))
                self.check_alert(icao)
            elif df in (20, 21):
                icao = format(self.crc24_icao(bits, 112), '06X')
                self.update(icao, rssi=rssi); self.stats["bds_attempts"] += 1
                bds = guess_bds(bits, self.ac.get(icao, {}))
                if bds:
                    self.stats["bds_success"] += 1; bds_name = identify_bds(bds)
                    self.stats["bds_by_type"][bds_name] = self.stats["bds_by_type"].get(bds_name, 0) + 1
                    if "callsign_bds20" in bds:
                        cs = bds["callsign_bds20"]; self.update(icao, callsign=cs)
                        self.event(icao, f"[BDS20] callsign: {cs}")
                    elif "dlc_capability" in bds: self.event(icao, f"[BDS10] DLC: {bds['dlc_capability']}")
                    elif "gicb_capability" in bds: self.event(icao, f"[BDS17] GICB capability")
                    elif "acas_arv" in bds:
                        arv = "ACTIVE" if bds["acas_arv"] else "inactive"
                        rat = " (RA terminated)" if bds.get("acas_rat") else ""
                        mte = " (MTE)" if bds.get("acas_mte") else ""
                        parts = [f"[BDS30] ACAS RA: {arv}{rat}{mte}"]
                        av, ah, rc = bds.get("ara_vertical",""), bds.get("ara_horizontal",""), bds.get("rac","")
                        if av: parts.append(f"VRA:{av}")
                        if ah: parts.append(f"HRA:{ah}")
                        if rc: parts.append(f"RAC:{rc}")
                        self.update(icao, ara_vertical=av, ara_horizontal=ah, rac=rc)
                        self.event(icao, " ".join(parts))
                        self.check_tcas_ra(icao, bds)
                    elif "turbulence" in bds or "icing" in bds or "windshear" in bds:
                        self.stats["bds45_count"] += 1
                        parts = ["[BDS45]"]
                        if "turbulence" in bds:
                            parts.append(f"turb={bds['turbulence']}")
                            if bds["turbulence"] != "None": self.play_turbulence()
                        if "icing" in bds: parts.append(f"icing={bds['icing']}")
                        if "windshear" in bds: parts.append(f"wshear={bds['windshear']}")
                        if "microburst" in bds: parts.append(f"mburst={bds['microburst']}")
                        if "wake_vortex" in bds: parts.append(f"wvortex={bds['wake_vortex']}")
                        if "temp" in bds: parts.append(f"t={fmt_temp(bds.get('temp'))}")
                        if "pressure_hpa" in bds: parts.append(f"QNH={fmt_pressure(bds.get('pressure_hpa'))}")
                        self.update(icao, turbulence=bds.get("turbulence"),
                                     icing=bds.get("icing"), windshear=bds.get("windshear"),
                                     microburst=bds.get("microburst"), wake_vortex=bds.get("wake_vortex"))
                        if "temp" in bds: self.update(icao, temp=bds["temp"])
                        if "pressure_hpa" in bds: self.update(icao, pressure_hpa=bds["pressure_hpa"])
                        self.event(icao, " ".join(parts))
                    elif "wind_dir" in bds:
                        self.update(icao, wind_dir=bds.get("wind_dir"), wind_spd=bds.get("wind_spd"), temp=bds.get("temp"))
                        self.event(icao, f"[BDS44] wind {bds.get('wind_dir',0):.0f}° {fmt_speed(bds.get('wind_spd'))} t={fmt_temp(bds.get('temp'))} QNH={fmt_pressure(bds.get('pressure_hpa'))}")
                    elif "selected_altitude" in bds:
                        self.update(icao, selected_altitude=bds["selected_altitude"])
                        if "baro_setting" in bds: self.update(icao, baro_setting=bds["baro_setting"])
                        parts = [f"[BDS40] sel.alt={fmt_alt(bds['selected_altitude'])}"]
                        if "baro_setting" in bds: parts.append(f"QNH={fmt_pressure(bds['baro_setting'])}")
                        self.event(icao, " ".join(parts))
                    elif "roll" in bds:
                        parts = [f"[BDS50] roll={bds['roll']}°"]
                        if "true_hdg" in bds: parts.append(f"HDG={bds['true_hdg']}°")
                        if "ground_speed" in bds: parts.append(f"GS={fmt_speed(bds['ground_speed'])}")
                        if "turn_rate" in bds: parts.append(f"turn={bds['turn_rate']}°/s")
                        if "tas" in bds: parts.append(f"TAS={fmt_speed(bds['tas'])}")
                        self.event(icao, " ".join(parts))
                        if "tas" in bds: self.update(icao, speed_tas=bds["tas"])
                        if "true_hdg" in bds: self.update(icao, true_hdg_bds50=bds["true_hdg"])
                        if "ground_speed" in bds: self.update(icao, speed_gs_bds50=bds["ground_speed"])
                    elif "mag_hdg" in bds:
                        if bds.get("vrate_bds60") is not None: self.update(icao, vrate_bds60=bds["vrate_bds60"])
                        if "ias" in bds: self.update(icao, speed_ias=bds["ias"])
                        spd_str = f" IAS={fmt_speed(bds['ias'])}" if "ias" in bds else ""
                        self.event(icao, f"[BDS60] HDG={bds['mag_hdg']}°{spd_str} VR={fmt_vrate(bds.get('vrate_bds60'))}")
                    elif "sel_heading_bds51" in bds:
                        p = [f"[BDS51] sel.hdg={bds['sel_heading_bds51']}"]
                        if "sel_alt_bds51" in bds: p.append(f"sel.alt={fmt_alt(bds['sel_alt_bds51'])}")
                        if "baro_bds51" in bds: p.append(f"QNH={fmt_pressure(bds['baro_bds51'])}")
                        if "autopilot_modes" in bds: p.append(f"modes={','.join(bds['autopilot_modes'])}")
                        self.update(icao, selected_heading=bds["sel_heading_bds51"])
                        if "sel_alt_bds51" in bds: self.update(icao, selected_altitude=bds["sel_alt_bds51"])
                        if "baro_bds51" in bds: self.update(icao, baro_setting=bds["baro_bds51"])
                        self.event(icao, " ".join(p))
                    elif "tcp_type" in bds:
                        p = [f"[BDS52] TCP={bds['tcp_type']}"]
                        if "turn_dir" in bds: p.append(f"turn={bds['turn_dir']}")
                        if "turn_rate_tcp" in bds: p.append(f"rate={bds['turn_rate_tcp']}")
                        if "tcp_lat" in bds: p.append(f"pos={bds['tcp_lat']:.4f},{bds['tcp_lon']:.4f}")
                        if "tcp_alt" in bds: p.append(f"alt={fmt_alt(bds['tcp_alt'])}")
                        self.event(icao, " ".join(p))
                    elif "tc_type" in bds:
                        p = [f"[BDS53] TC={bds['tc_type']}"]
                        if "tc_turn_dir" in bds: p.append(f"dir={bds['tc_turn_dir']}")
                        if "climb_angle" in bds: p.append(f"angle={bds['climb_angle']}")
                        if "tc_target_alt" in bds: p.append(f"tgt={fmt_alt(bds['tc_target_alt'])}")
                        self.event(icao, " ".join(p))
                    elif "steer_hdg" in bds:
                        p = [f"[BDS61] steer.hdg={bds['steer_hdg']}"]
                        if "steer_angle_rate" in bds: p.append(f"rate={bds['steer_angle_rate']}")
                        if "fms_alt" in bds: p.append(f"alt={fmt_alt(bds['fms_alt'])}")
                        if "fms_spd" in bds: p.append(f"spd={fmt_speed(bds['fms_spd'])}")
                        if "fms_modes" in bds: p.append(f"modes={','.join(bds['fms_modes'])}")
                        self.event(icao, " ".join(p))
                else:
                    self.stats["bds_fail"] += 1
                self.check_alert(icao)
            elif df == 24:
                icao = format(self.crc24_icao(bits, 112), '06X')
                self.update(icao, rssi=rssi)
                self._process_elm(icao, bits, is_reply=False)
                self.check_alert(icao)
            elif df == 25:
                icao = format(self.crc24_icao(bits, 112), '06X')
                self.update(icao, rssi=rssi)
                self._process_elm(icao, bits, is_reply=True)
                self.check_alert(icao)

    def _process_adsb(self, icao, tc, bits, t, rssi=None):
        if 1 <= tc <= 4:
            cs = ''
            for i in range(8):
                v = bits_to_int(bits[40+i*6:40+i*6+6])
                cs += CHARS[v] if v < len(CHARS) else '?'
            cs = cs.strip()
            cat_code = (tc - 1) * 8 + bits_to_int(bits[37:40])
            cat_str = AC_CATEGORY.get(cat_code, f"?{cat_code}")
            self.update(icao, rssi=rssi, callsign=cs, category=cat_str, cat_desc=AC_CAT_DESC.get(cat_str, ""))
            self.check_alert(icao, cs)
            self.event(icao, f"[CALL] {cs} ({cat_str} {AC_CAT_DESC.get(cat_str, '')})")
        elif 5 <= tc <= 8:
            gs = bits_to_int(bits[37:44]); ground_speed = None
            if gs == 0: ground_speed = None
            elif gs == 1: ground_speed = 0.0
            elif gs < len(GROUND_SPEED_TABLE): ground_speed = GROUND_SPEED_TABLE[gs]
            fmt = bits[53]
            lat_cpr = bits_to_int(bits[54:71]) / GROUND_CPR_MAX
            lon_cpr = bits_to_int(bits[71:88]) / GROUND_CPR_MAX
            self.update(icao, rssi=rssi, on_ground=True, ground_speed=ground_speed)
            res = self.update_cpr(icao, lat_cpr, lon_cpr, fmt, t, ground=True)
            if ENABLE_NIC:
                nic = NIC_SURFACE_BASE.get(tc, 0)
                ac = self.ac.get(icao, {})
                if bits[44] or ac.get("nic_supplement", False): nic = max(nic - 1, 0)
                self.update(icao, nic=NIC_RADIUS.get(nic, f"?{nic}")); self.stats["nic_decoded"] += 1
            if res:
                self.check_alert(icao)
            move_str = "moving" if ground_speed and ground_speed > 0 else ("stopped" if ground_speed is not None else "n/a")
            self.event(icao, f"[GND] {res[0]:.5f},{res[1]:.5f} gs={fmt_speed(ground_speed)} {move_str}")
        elif 9 <= tc <= 18:
            alt = decode_altitude(bits)
            self.update(icao, rssi=rssi, altitude=alt, on_ground=False)
            fmt = bits[53]
            lat_cpr = bits_to_int(bits[54:71]) / AIRBORN_CPR_MAX
            lon_cpr = bits_to_int(bits[71:88]) / AIRBORN_CPR_MAX
            res = self.update_cpr(icao, lat_cpr, lon_cpr, fmt, t, ground=False)
            if ENABLE_NIC:
                nic = NIC_AIRBORNE_BASE.get(tc, 0)
                ac = self.ac.get(icao, {})
                if (bool(bits[39]) and tc >= 12) or ac.get("nic_supplement", False): nic = max(nic - 1, 0)
                self.update(icao, nic=NIC_RADIUS.get(nic, f"?{nic}")); self.stats["nic_decoded"] += 1
            if res:
                self.check_alert(icao)
                _ac_dist = self.ac.get(icao, {}).get('distance', 0)
                self.event(icao, f"[POS] {res[0]:.5f},{res[1]:.5f} {fmt_alt(alt)} ({fmt_distance(_ac_dist)})")
        elif tc == 19:
            st = bits_to_int(bits[37:40])
            if st in (1, 2): self._decode_velocity(bits, icao, subtype=st, supersonic=(st == 2), rssi=rssi)
            elif st in (3, 4): self._decode_velocity(bits, icao, subtype=st, supersonic=(st == 4), rssi=rssi)
            if ENABLE_NACV:
                nacv_val = bits_to_int(bits[42:45])
                self.update(icao, nacv=NAC_V.get(nacv_val, f"?{nacv_val}")); self.stats["nacv_decoded"] += 1
            self.check_alert(icao)
        elif 20 <= tc <= 22:
            alt = decode_altitude(bits)
            fmt = bits[53]
            lat_cpr = bits_to_int(bits[54:71]) / AIRBORN_CPR_MAX
            lon_cpr = bits_to_int(bits[71:88]) / AIRBORN_CPR_MAX
            self.update(icao, rssi=rssi, gnss_altitude=alt, on_ground=False)
            res = self.update_cpr(icao, lat_cpr, lon_cpr, fmt, t, ground=False)
            if ENABLE_NIC:
                nic = NIC_AIRBORNE_BASE.get(tc, 0)
                self.update(icao, nic=NIC_RADIUS.get(nic, f"?{nic}")); self.stats["nic_decoded"] += 1
            if res:
                self.check_alert(icao)
                _ac_dist = self.ac.get(icao, {}).get('distance', 0)
                self.event(icao, f"[GNSS] {fmt_alt(alt)} {res[0]:.5f},{res[1]:.5f} ({fmt_distance(_ac_dist)})")
            elif alt is not None:
                self.check_alert(icao)
                self.event(icao, f"[GNSS] {fmt_alt(alt)}")
        elif tc == 28:
            sub = bits_to_int(bits[37:40])
            if sub != 1: return
            emer = bits_to_int(bits[40:44])
            emer_str = EMERGENCY.get(emer, f"Code {emer}")
            self.update(icao, rssi=rssi)
            ac = self.ac.get(icao)
            if ac is None: return
            if emer != 0:
                # Confirmation: need 2 identical TC=28 with emergency != 0
                if ac.get("emer_pending") == emer:
                    ac.pop("emer_pending", None)
                    ac["emergency"] = emer_str
                    ac["emergency_time"] = time.time()
                    self.event(icao, f"[EMER] {emer_str}")
                    self.play_emergency()
                else:
                    ac["emer_pending"] = emer
            else:
                ac.pop("emer_pending", None)
                ac["emergency"] = emer_str
                ac["emergency_time"] = time.time()
                self.event(icao, f"[EMER] {emer_str}")
        elif tc == 23:
            self.update(icao, rssi=rssi)
            self.stats["tc23_count"] = self.stats.get("tc23_count", 0) + 1
            self.event(icao, "[TC23] Test/Reserved type")
        elif tc == 24:
            self.update(icao, rssi=rssi)
            self.stats["tc24_count"] = self.stats.get("tc24_count", 0) + 1
            self.event(icao, "[TC24] Reserved type")
        elif tc == 29:
            sub = bits_to_int(bits[37:40])
            if sub == 2:
                self.update(icao, rssi=rssi)
                vt = bits_to_int(bits[40:43])
                VT = {0:"",1:"climb",2:"descend",3:"level",4:"accel",5:"decel",6:"turn",7:""}
                tr = bits_to_int(bits[43:50]) * 0.25 if bits[43] else None
                td = {0:"left",1:"right",2:"straight"}.get(bits_to_int(bits[50:52]), "?")
                parts = [f"[TC29/2] VTC: {VT.get(vt, '?')}"]
                if tr is not None and tr <= 15: parts.append(f"turn={tr}")
                parts.append(f"dir={td}")
                self.event(icao, " ".join(parts))
                self.check_alert(icao)
                return
            vert_avail = bits[40]
            sel_alt_type = bits[41]; sel_alt = None
            if vert_avail:
                if sel_alt_type: sel_alt = bits_to_int(bits[42:54]) * 32 - 8192
                else: sel_alt = bits_to_int(bits[42:54]) * 25 - 1000
                if not (MIN_SEL_ALT_FT <= sel_alt <= MAX_SEL_ALT_FT):
                    self.stats["alt_rejected"] += 1; sel_alt = None
            if sel_alt is not None:
                self.update(icao, selected_altitude=sel_alt)
            baro_status = bits[54]; baro_hpa = None
            if baro_status:
                baro_hpa = bits_to_int(bits[55:63]) * 0.25 + 796.875
                if MIN_QNH_HPA <= baro_hpa <= MAX_QNH_HPA:
                    self.update(icao, baro_setting=baro_hpa)
                else: baro_hpa = None
            ap_mode = bits_to_int(bits[63:65])
            ap_str = AUTOPILOT_MODE.get(ap_mode, "?")
            self.update(icao, rssi=rssi, autopilot=ap_str)
            parts = []
            if sel_alt is not None: parts.append(f"sel.alt={fmt_alt(sel_alt)}")
            if baro_status and baro_hpa is not None: parts.append(f"QNH={fmt_pressure(baro_hpa)}")
            parts.append(f"AP={ap_str}")
            if sub == 1:
                if bits[65]: parts.append("TCAS-RA!"); self.play_tcas()
                if bits[66]:
                    sel_hdg = bits_to_int(bits[67:75]) * 90.0 / 256.0
                    self.update(icao, selected_heading=round(sel_hdg, 1))
                    parts.append(f"sel.hdg={sel_hdg:.0f}°")
            self.event(icao, f"[TC29] {', '.join(parts)}")
        elif tc == 31:
            nac_val = bits_to_int(bits[40:44])
            nac_str = NAC_P.get(nac_val, f"?{nac_val}")
            self.update(icao, rssi=rssi, nac_p=nac_str)
            if ENABLE_SIL:
                sil_val = bits_to_int(bits[44:46])
                self.update(icao, sil=SIL_TABLE.get(sil_val, f"?{sil_val}")); self.stats["sil_decoded"] += 1
            if ENABLE_ADSB_VERSION:
                version = bits_to_int(bits[72:74])
                ver_name = ADSB_VERSION.get(version, "Unknown")
                self.adsb_version_counts[ver_name] = self.adsb_version_counts.get(ver_name, 0) + 1
                self.update(icao, adsb_version=ADSB_VERSION.get(version, f"?{version}")); self.stats["version_decoded"] += 1
            self.update(icao, nic_supplement=bool(bits[39]))
            self.update(icao, sil_supplement={0:"per-hour",1:"per-sample",2:"(res)",3:"(res)"}.get(bits_to_int(bits[46:48]), "?"))
            self.update(icao, gva={0:"<150 m",1:"<45 m",2:"<15 m",3:"<4 m"}.get(bits_to_int(bits[74:76]), "?"))
            self.update(icao, sda={0:"unavailable",1:"low",2:"medium",3:"high"}.get(bits_to_int(bits[76:78]), "?"))
            self.event(icao, f"[TC31] accuracy: {nac_str}" + (f" SIL={self.ac.get(icao,{}).get('sil','')}" if ENABLE_SIL else "") + (f" Ver={self.ac.get(icao,{}).get('adsb_version','')}" if ENABLE_ADSB_VERSION else ""))

    def _decode_velocity(self, bits, icao, subtype, supersonic=False, rssi=None):
        mult = 4 if supersonic else 1
        vr_source_bit = bits[67]; vr_sign = bits[68]; vr_raw = bits_to_int(bits[69:78]); vr = None
        vr_source_str = "GNSS" if vr_source_bit else "baro"
        if vr_raw > 0:
            vr = (vr_raw - 1) * 64
            if vr_sign: vr = -vr
            if abs(vr) > MAX_VRATE_FPM:
                self.stats["vel_rejected"] += 1; vr = None
            else:
                self.update(icao, vrate_tc19=vr, vr_source_tc19=vr_source_str)
        vr_str = f" VR={fmt_vrate(vr)} ({vr_source_str})" if vr is not None else ""
        if subtype in (1, 2):
            ew_sign = bits[45]; ew_vel = bits_to_int(bits[46:56]) - 1
            ns_sign = bits[56]; ns_vel = bits_to_int(bits[57:67]) - 1
            if ew_vel >= 0 and ns_vel >= 0:
                if ew_sign: ew_vel = -ew_vel
                if ns_sign: ns_vel = -ns_vel
                spd = math.sqrt(ew_vel**2 + ns_vel**2) * mult
                hdg = (math.degrees(math.atan2(ew_vel, ns_vel)) + 360) % 360
                self.update(icao, rssi=rssi, speed_gs=round(spd), heading=hdg)
                self.event(icao, f"[VEL] {fmt_speed(round(spd))} GS hdg {hdg:.0f}° ({heading_to_direction(hdg)}){vr_str}")
        elif subtype in (3, 4):
            hdg_status = bits[45]
            hdg = bits_to_int(bits[46:56]) / 1024.0 * 360.0 if hdg_status else None
            ias_tas_flag = bits[56]; spd_raw = bits_to_int(bits[57:67])
            if spd_raw > 0:
                spd = round((spd_raw - 1) * mult)
                if ias_tas_flag: self.update(icao, rssi=rssi, speed_tas=spd)
                else: self.update(icao, rssi=rssi, speed_ias=spd)
                if hdg is not None: self.update(icao, heading=hdg)
                spd_type = "TAS" if ias_tas_flag else "IAS"
                hdg_str = f" hdg {hdg:.0f}°" if hdg is not None else ""
                self.event(icao, f"[VEL] {fmt_speed(spd)} {spd_type}{hdg_str}{vr_str}")

    @staticmethod
    def crc24_icao(bits, msglen):
        return crc24(bits, msglen - 24) ^ bits_to_int(bits[msglen - 24:msglen])

    def __del__(self):
        try:
            if hasattr(self, 'logger') and self.logger: self.logger.close_all()
        except Exception: pass

    # === Analytics methods ===

    def on_message_received(self, df, rssi=None):
        """Track message for analytics: DF types only."""
        now = time.time()
        self.last_msg_check = now
        self.df_history.append({"t": now, "df": df})

    def update_rssi_bin(self, bearing, rssi):
        """Update RSSI average per 5-degree sector."""
        if rssi is None or bearing is None:
            return
        idx = int(bearing / 5.0) % 72
        n = self.rssi_counts[idx]
        if n == 0:
            self.rssi_bins[idx] = rssi
        else:
            self.rssi_bins[idx] = (self.rssi_bins[idx] * n + rssi) / (n + 1)
        self.rssi_counts[idx] = n + 1

    def collect_analytics(self, force=False):
        """Collect analytics snapshot every 5 seconds."""
        now = time.time()
        if not force and now - self._analytics_last < 5:
            return
        self._analytics_last = now
        
        # Time-series
        try:
            self.msg_rate_history.append({"t": now, "rate": round(self.get_msg_rate(), 1)})
        except Exception:
            pass
        # Thread-safe snapshot of aircraft dict
        with self._lock:
            rssi_vals = [a.get("rssi") for _, a in self.ac.items() if a.get("rssi") is not None]
            avg_rssi = round(sum(rssi_vals) / len(rssi_vals), 1) if rssi_vals else None
            self.rssi_history.append({"t": now, "rssi": avg_rssi})
            unique_icaos = set(icao for icao, a in self.ac.items()
                               if now - a.get("last_seen", now) < 120)
            self.unique_aircraft_history.append({"t": now, "count": len(unique_icaos)})
            ac_snapshot = list(self.ac.items())

        # Category history
        cats = {}
        for icao, ac in ac_snapshot:
            cat = ac.get("category", "")
            if not cat:
                cat = "Unknown"
            cats[cat] = cats.get(cat, 0) + 1
            self._seen_cats.add(cat)
        for c in self._seen_cats:
            if c not in cats:
                cats[c] = 0
        self.cat_history.append({"t": now, "cats": cats})
        
        # Quality history
        self.quality_history.append({
            "t": now,
            "decode": round(self.decode_quality_ema, 1),
            "receive": round(self.recv_quality_ema, 1),
            "msg_rate": round(self.get_msg_rate(), 1) if hasattr(self, "get_msg_rate") else 0
        })
        
        # Type counts and histograms (reset each cycle)
        self.type_counts = {}
        self.flag_counts = {}
        self.alt_histogram = [0] * 35
        self.speed_histogram = [0] * 30
        
        for icao, ac in ac_snapshot:
            if ac.get("last_seen", 0) and now - ac.get("last_seen", 0) > 60:
                continue
            tc = ac.get("typecode", "") or AIRCRAFT_INFO.get(icao, {}).get("type", "")
            if tc:
                self.type_counts[tc] = self.type_counts.get(tc, 0) + 1
                self._session_icao_type[icao] = tc
            flag = ac.get("flag", "") or AIRCRAFT_INFO.get(icao, {}).get("flag", "")
            if flag:
                self.flag_counts[flag] = self.flag_counts.get(flag, 0) + 1
                self._session_icao_flag[icao] = flag
            
            alt = ac.get("altitude")
            if alt is not None and -1200 <= alt <= 70000:
                bin_idx = min(int((alt + 1200) / 2000), 34)
                if bin_idx >= 0:
                    self.alt_histogram[bin_idx] += 1
            
            spd = ac.get("speed_gs") or ac.get("speed_ias") or ac.get("speed_tas")
            if spd is not None and 0 <= spd <= 1500:
                bin_idx = min(int(spd / 50), 29)
                if bin_idx >= 0:
                    self.speed_histogram[bin_idx] += 1
        
        # Flag session accumulation
        for icao, ac in ac_snapshot:
            flag = ac.get("flag", "") or AIRCRAFT_INFO.get(icao, {}).get("flag", "")
            if flag and icao not in self._session_icao_flag:
                self.flag_counts_session[flag] = self.flag_counts_session.get(flag, 0) + 1
                self._session_icao_flag[icao] = flag
        # Session accumulation
        for icao, ac in ac_snapshot:
            tc = ac.get("typecode", "") or AIRCRAFT_INFO.get(icao, {}).get("type", "")
            if tc and icao not in self._session_icao_type:
                self._session_icao_type[icao] = tc
            alt = ac.get("altitude")
            if alt is not None and -1200 <= alt <= 70000:
                if icao not in self._session_icao_alt or alt > self._session_icao_alt[icao]:
                    self._session_icao_alt[icao] = alt
            spd = ac.get("speed_gs") or ac.get("speed_ias") or ac.get("speed_tas")
            if spd is not None and 0 <= spd <= 1500:
                if icao not in self._session_icao_spd or spd > self._session_icao_spd[icao]:
                    self._session_icao_spd[icao] = spd
        
        self.type_counts_session = {}
        self.flag_counts_session = {}
        self.alt_histogram_session = [0] * 35
        self.speed_histogram_session = [0] * 30
        for tc in self._session_icao_type.values():
            self.type_counts_session[tc] = self.type_counts_session.get(tc, 0) + 1
        for flag in self._session_icao_flag.values():
            self.flag_counts_session[flag] = self.flag_counts_session.get(flag, 0) + 1
        for alt in self._session_icao_alt.values():
            bin_idx = min(int((alt + 1200) / 2000), 34)
            if 0 <= bin_idx < 35:
                self.alt_histogram_session[bin_idx] += 1
        for spd in self._session_icao_spd.values():
            bin_idx = min(int(spd / 50), 29)
            if 0 <= bin_idx < 30:
                self.speed_histogram_session[bin_idx] += 1

    def get_analytics_json(self):
        """Build analytics JSON for /analytics.json endpoint."""
        now = time.time()
        
        # Thread-safe snapshots (single lock)
        with self._lock:
            _ac_items = list(self.ac.items())
            _df_hist = list(self.df_history)
            _type_s = dict(self.type_counts_session)
            _flag_s = dict(self.flag_counts_session)
            _squawk_s = dict(self.squawk_counts)
            _version_s = dict(self.adsb_version_counts)
            _heatmap_s = dict(self.heatmap_grid)
            _msg_rate_s = list(self.msg_rate_history)
            _rssi_h_s = list(self.rssi_history)
            _unique_h_s = list(self.unique_aircraft_history)
            _coverage_s = list(self.coverage_bins)
            _rssi_bins_s = list(self.rssi_bins)
            _rssi_counts_s = list(self.rssi_counts)

            ac_items = _ac_items
            df_hist = _df_hist
            heatmap_items = list(self.heatmap_grid.items())
            squawk_items = list(self.squawk_counts.items())
            version_items = list(self.adsb_version_counts.items())
            coverage = _coverage_s
            rssi_b = _rssi_bins_s
            rssi_c = _rssi_counts_s
            msg_rate = list(_msg_rate_s)
            rssi_h = list(_rssi_h_s)
            unique_h = list(_unique_h_s)
            cat_h = list(self.cat_history)[-10:]
            quality_h = list(self.quality_history)
            types_s = self.type_counts_session
            flags_s = self.flag_counts_session
            alt_hist_s = list(self.alt_histogram_session)
            spd_hist_s = list(self.speed_histogram_session)
            doppler_d = dict(self.doppler_data)
            if "measurements" in doppler_d:
                doppler_d["measurements"] = list(doppler_d["measurements"])
        
        # RSSI bins
        rssi_data = []
        for i in range(72):
            rssi_data.append({
                "angle": i * 5.0,
                "rssi": round(_rssi_bins_s[i], 1) if _rssi_counts_s[i] > 0 else None,
                "count": _rssi_counts_s[i]
            })
        
        # DF history (last 60 entries)
        df_counts = {}
        for entry in df_hist:
            df = entry["df"]
            df_counts[f"DF{df}"] = df_counts.get(f"DF{df}", 0) + 1
        
        # Category history (last 10 snapshots)
        cat_snapshots = cat_h
        
        # Quality history
        quality_snapshots = quality_h
        
        # Top aircraft types
        sorted_types = sorted(types_s.items(), key=lambda x: -x[1])[:15]
        
        # Extended analytics
        msg_rate_ts = msg_rate
        rssi_ts = rssi_h
        unique_ts = unique_h
        squawk_top = sorted(squawk_items, key=lambda x: -x[1])[:20]
        version_data = [{"version": k, "count": v} for k, v in sorted(version_items, key=lambda x: -x[1])]
        sector_ranges = []
        for i in range(12):
            s = i * 6; e = s + 6
            bins = coverage[s:e]; rssi_bins = rssi_b[s:e]; counts = rssi_c[s:e]
            max_r = max(bins) if bins else 0
            avg_rssi = sum(rb * c for rb, c in zip(rssi_bins, counts)) / max(sum(counts), 1)
            sector_ranges.append({"sector": i, "azimuth": f"{s*5}-{e*5}", "max_range_km": round(max_r, 1), "avg_rssi": round(avg_rssi, 1) if sum(counts) > 0 else None})
        crc_total = self.stats.get("msg_total", 0) or 1
        crc_corrected = self.stats.get("crc_corrected", 0)
        crc_two = self.stats.get("crc_two_bit_corrected", 0)
        crc_error_pct = round(crc_corrected / crc_total * 100, 2) if crc_total > 0 else 0
        heatmap_list = [{"lat": float(k.split(",")[0]), "lon": float(k.split(",")[1]), "count": v}
                        for k, v in sorted(heatmap_items, key=lambda x: -x[1])[:500]]
        alt_dist = []
        for icao, a in _ac_items:
            alt = a.get("altitude"); dist = a.get("distance")
            if alt is not None and dist is not None and dist > 0:
                alt_dist.append({"icao": icao, "alt": alt, "dist": round(dist, 1), "rssi": a.get("rssi")})
        result = {}
        result["msg_rate_timeline"] = msg_rate_ts
        result["rssi_timeline"] = rssi_ts
        result["unique_aircraft_timeline"] = unique_ts
        result["squawk_counts"] = [{"squawk": s, "count": c} for s, c in squawk_top]
        result["adsb_versions"] = version_data
        result["sector_ranges"] = sector_ranges
        result["crc_stats"] = {"total": crc_total, "corrected": crc_corrected, "two_bit": crc_two, "error_pct": crc_error_pct}
        result["heatmap"] = heatmap_list
        result["alt_dist_scatter"] = alt_dist[:200]
        result["crc_corrected"] = crc_corrected
        result["crc_two_bit_corrected"] = crc_two
        result["rssi_bins"] = rssi_data
        
        # Altitude histogram
        alt_bins = []
        for i in range(35):
            alt_bins.append({
                "label": f"{'FL' if i >= 1 else ''}{int((i * 2000 - 1200) / 100)}",
                "range": f"{i*2000 - 1200}-{(i+1)*2000 - 1200}",
                "count": alt_hist_s[i]
            })
        
        # Speed histogram
        spd_bins = []
        for i in range(30):
            spd_bins.append({
                "range": f"{i*50}-{(i+1)*50}",
                "count": spd_hist_s[i]
            })
        
        # Meteo data (BDS 4,4/4,5)
        # meteo (lock already held)
        _meteo_snapshot = list(self.ac.items())
        meteo = []
        for icao, ac in _meteo_snapshot:
            if ac.get("wind_dir") is not None or ac.get("temp") is not None:
                meteo.append({
                    "icao": icao,
                    "altitude": ac.get("altitude"),
                    "wind_dir": ac.get("wind_dir"),
                    "wind_spd": ac.get("wind_spd"),
                    "temp": ac.get("temp"),
                    "turbulence": ac.get("turbulence"),
                    "icing": ac.get("icing")
                })
        
        # Conflict detection
        conflicts = []
        with self._lock:
            _conflict_ac_snapshot = list(self.ac.items())
        aircraft_list = []
        for icao, ac in _conflict_ac_snapshot:
            if ac.get("lat") is None or ac.get("lon") is None:
                continue
            if now - ac.get("last_seen", 0) > 30:
                continue
            aircraft_list.append((icao, ac))
        
        for i in range(len(aircraft_list)):
            for j in range(i + 1, len(aircraft_list)):
                icao1, ac1 = aircraft_list[i]
                icao2, ac2 = aircraft_list[j]
                dist = haversine_km(ac1["lat"], ac1["lon"], ac2["lat"], ac2["lon"])
                if dist < 15:  # 15 km
                    alt1 = ac1.get("altitude", 0) or 0
                    alt2 = ac2.get("altitude", 0) or 0
                    alt_diff = abs(alt1 - alt2)
                    hdg1 = ac1.get("heading")
                    hdg2 = ac2.get("heading")
                    converging = False
                    if hdg1 is not None and hdg2 is not None:
                        diff = abs(((hdg1 - hdg2 + 180) % 360) - 180)
                        converging = diff > 120
                    conflicts.append({
                        "icao1": icao1,
                        "icao2": icao2,
                        "callsign1": ac1.get("callsign", ""),
                        "callsign2": ac2.get("callsign", ""),
                        "distance_km": round(dist, 2),
                        "alt_diff_ft": round(alt_diff, 0),
                        "converging": converging,
                        "lat1": ac1["lat"],
                        "lon1": ac1["lon"],
                        "lat2": ac2["lat"],
                        "lon2": ac2["lon"]
                    })
        
        # Silence periods
        with self._lock:
            silence = list(self.silence_periods[-19:])
            if self.silence_active and self._silence_start is not None:
                silence.append({
                    "start": self._silence_start,
                    "end": None,
                    "duration": round(time.time() - self._silence_start, 1),
                    "status": "active"
                })
        
        # Doppler
        doppler = doppler_d if doppler_d.get("measurements") else {"measurements": [], "ppm_estimate": 0.0}
        
        result.update({
            "timestamp": now,
            "rssi_bins": rssi_data,
            "df_counts": df_counts,
            "cat_history": cat_snapshots,
            "quality_history": quality_snapshots,
            "type_counts": [{"type": t, "count": c} for t, c in sorted_types],
            "flag_counts": [{"flag": f, "count": c} for f, c in sorted(flags_s.items(), key=lambda x: -x[1])[:15]],
            "alt_histogram": alt_bins,
            "speed_histogram": spd_bins,
            "meteo": meteo,
            "conflicts": conflicts,
            "silence_periods": silence,
            "doppler": doppler,
            "home_lat": HOME_LAT,
            "home_lon": HOME_LON
        })
        
        return _json.dumps(result, ensure_ascii=False).encode("utf-8")

    def save_stats_snapshot(self):
        """Save analytics snapshot to disk for historical viewing."""
        if not ENABLE_STATS_HISTORY:
            return
        now_dt = datetime.datetime.now()
        date_str = now_dt.strftime("%Y-%m-%d")
        time_str = now_dt.strftime("%Y-%m-%d_%H-%M")

        hist_dir = os.path.join(STATS_HISTORY_DIR, date_str)
        os.makedirs(hist_dir, exist_ok=True)

        self.collect_analytics(force=True)

        sorted_types = sorted(self.type_counts_session.items(), key=lambda x: -x[1])[:15]
        snapshot = {
            "timestamp": time.time(),
            "datetime": now_dt.strftime("%Y-%m-%d %H:%M"),
            "type_counts": [{"type": t, "count": c} for t, c in sorted_types],
            "flag_counts": [{"flag": f, "count": c} for f, c in sorted(self.flag_counts_session.items(), key=lambda x: -x[1])[:15]],
            "alt_histogram": [
                {"range": f"{i*2000 - 1200}-{(i+1)*2000 - 1200}", "count": self.alt_histogram_session[i]}
                for i in range(35)
            ],
            "speed_histogram": [
                {"range": f"{i*50}-{(i+1)*50}", "count": self.speed_histogram_session[i]}
                for i in range(30)
            ],
            "total_aircraft": self.stats.get("total_aircraft", 0),
            "max_speed": self.stats.get("max_speed_kt", 0),
            "min_speed": self.stats.get("min_speed_kt", 0),
            "max_alt": self.stats.get("max_alt_ft", 0),
            "min_alt": self.stats.get("min_alt_ft", 0),
            "max_range": round(self.stats.get("max_range_km", 0), 1),
            "msg_rate": round(self.get_msg_rate(), 1) if hasattr(self, "get_msg_rate") else 0,
            "active_aircraft": sum(1 for ac in self.ac.values() if time.time() - ac.get("last_seen", 0) < 60),
            "stats": dict(self.stats),
            "df_counts": {f"DF{e['df']}": sum(1 for x in self.df_history if x['df'] == e['df']) for e in self.df_history},
            "cat_history": list(self.cat_history)[-10:],
            "coverage_bins": list(self.coverage_bins),
            "rssi_bins": list(self.rssi_bins),
            "rssi_counts": list(self.rssi_counts),
            "heatmap_grid": {k: v for k, v in list(self.heatmap_grid.items())[:500]},
            "doppler": self.doppler_data if self.doppler_data.get("measurements") else {"measurements": [], "ppm_estimate": 0.0},
            "squawk_counts": dict(self.squawk_counts),
            "adsb_version_counts": dict(self.adsb_version_counts),
        }

        filepath = os.path.join(hist_dir, f"{time_str}.json")
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json_module.dump(snapshot, f, ensure_ascii=False, indent=2)
        except Exception as e:
            sys.stderr.write(f"[stats-history] save: {e}\n")

    def cleanup_stats_history(self):
        """Delete snapshots older than STATS_HISTORY_RETENTION_DAYS."""
        if not ENABLE_STATS_HISTORY or not os.path.isdir(STATS_HISTORY_DIR):
            return
        cutoff = datetime.datetime.now() - datetime.timedelta(days=STATS_HISTORY_RETENTION_DAYS)
        cutoff_str = cutoff.strftime("%Y-%m-%d")
        try:
            for d in os.listdir(STATS_HISTORY_DIR):
                if d < cutoff_str:
                    shutil.rmtree(os.path.join(STATS_HISTORY_DIR, d), ignore_errors=True)
        except Exception:
            pass

    def save_pos_snapshot(self):
        """Save positions of all aircraft to disk."""
        if not ENABLE_POS_HISTORY:
            return
        now_dt = datetime.datetime.now()
        date_str = now_dt.strftime("%Y-%m-%d")
        time_str = now_dt.strftime("%Y-%m-%d_%H-%M-%S")
        hist_dir = os.path.join(POS_HISTORY_DIR, date_str)
        os.makedirs(hist_dir, exist_ok=True)
        positions = []
        now = time.time()
        with self._lock:
            ac_snapshot = list(self.ac.items())
        for icao, ac in ac_snapshot:
            lat = ac.get("lat") or ac.get("lat_dr")
            lon = ac.get("lon") or ac.get("lon_dr")
            if lat is None or lon is None:
                continue
            positions.append({
                "icao": icao,
                "callsign": ac.get("callsign", ""),
                "lat": lat,
                "lon": lon,
                "alt": ac.get("altitude"),
                "speed": ac.get("speed_gs") or ac.get("speed_ias"),
                "heading": ac.get("heading"),
                "rssi": ac.get("rssi"),
                "distance": ac.get("distance"),
                        "msg_count": ac.get("msg_count", 0),
                "squawk": ac.get("squawk", ""),
                "emergency": ac.get("emergency", ""),
                "stale": now - ac.get("last_seen", 0) > STALE_TIMEOUT,
                "category": ac.get("category", ""),
                "t": now,
            })
        snapshot = {"timestamp": now, "datetime": now_dt.strftime("%Y-%m-%d %H:%M:%S"),
                     "count": len(positions), "positions": positions}
        filepath = os.path.join(hist_dir, f"{time_str}.json")
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json_module.dump(snapshot, f, ensure_ascii=False)
        except Exception as e:
            sys.stderr.write(f"[pos-history] save: {e}")

    def cleanup_pos_history(self):
        """Delete position snapshots older than POS_HISTORY_RETENTION_DAYS."""
        if not ENABLE_POS_HISTORY or not os.path.isdir(POS_HISTORY_DIR):
            return
        cutoff = datetime.datetime.now() - datetime.timedelta(days=POS_HISTORY_RETENTION_DAYS)
        cutoff_str = cutoff.strftime("%Y-%m-%d")
        try:
            for d in os.listdir(POS_HISTORY_DIR):
                if d < cutoff_str:
                    shutil.rmtree(os.path.join(POS_HISTORY_DIR, d), ignore_errors=True)
        except Exception:
            pass

    def _collect_live_as_snapshot(self):
        """Collect current live statistics into snapshot format."""
        self.collect_analytics(force=True)
        sorted_types = sorted(self.type_counts_session.items(), key=lambda x: -x[1])[:15]
        return {
            "timestamp": time.time(),
            "datetime": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
            "type_counts": [{"type": t, "count": c} for t, c in sorted_types],
            "flag_counts": [{"flag": f, "count": c} for f, c in sorted(self.flag_counts_session.items(), key=lambda x: -x[1])[:15]],
            "alt_histogram": [
                {"range": f"{i*2000 - 1200}-{(i+1)*2000 - 1200}", "count": self.alt_histogram_session[i]}
                for i in range(35)
            ],
            "speed_histogram": [
                {"range": f"{i*50}-{(i+1)*50}", "count": self.speed_histogram_session[i]}
                for i in range(30)
            ],
            "total_aircraft": self.stats.get("total_aircraft", 0),
            "active_aircraft": len(self.ac),
            "max_speed": self.stats.get("max_speed_kt", 0),
            "min_speed": self.stats.get("min_speed_kt", 0),
            "max_alt": self.stats.get("max_alt_ft", 0),
            "min_alt": self.stats.get("min_alt_ft", 0),
            "doppler": self.doppler_data if self.doppler_data["measurements"] else {"measurements": [], "ppm_estimate": 0.0},
            "max_range": round(self.stats.get("max_range_km", 0), 1),
            "coverage_bins": list(self.coverage_bins),
            "rssi_bins": list(self.rssi_bins),
            "rssi_counts": list(self.rssi_counts),
            "heatmap_grid": {k: v for k, v in list(self.heatmap_grid.items())[:500]},
            "squawk_counts": dict(self.squawk_counts),
            "adsb_version_counts": dict(self.adsb_version_counts),
            "df_counts": {f"DF{e['df']}": sum(1 for x in self.df_history if x['df'] == e['df']) for e in self.df_history},
            "cat_history": list(self.cat_history)[-10:],
            "sector_ranges": [
                {"sector": i, "azimuth": f"{i*30}-{(i+1)*30}",
                 "max_range_km": round(max(self.coverage_bins[i*6:(i+1)*6]), 1),
                 "avg_rssi": round(
                     sum(self.rssi_bins[i*6:(i+1)*6][j] * self.rssi_counts[i*6:(i+1)*6][j]
                         for j in range(6)) /
                     max(sum(self.rssi_counts[i*6:(i+1)*6]), 1), 1)
                 if sum(self.rssi_counts[i*6:(i+1)*6]) > 0 else None}
                for i in range(12)
            ],
            "stats": {
                "total_aircraft": self.stats.get("total_aircraft", 0),
                "max_speed": self.stats.get("max_speed_kt", 0),
                "max_alt": self.stats.get("max_alt_ft", 0),
                "max_range": round(self.stats.get("max_range_km", 0), 1),
                "returns": self.stats.get("return_count", 0),
                "unit_speed": UNIT_SPEED,
                "unit_alt": UNIT_ALT,
                "unit_distance": UNIT_DISTANCE,
                "unit_vrate": UNIT_VRATE,
            },
        }


def get_history_range(db, from_date, to_date):
    """Aggregate snapshots for date range + live session.

    Snapshots contain CUMULATIVE session counters (unique aircraft since
    server start). Summing all snapshots would give N-fold inflation
    (e.g. 50 aircraft x 20 snapshots = 1000). Instead, we take the LAST
    snapshot for each day (maximum session coverage) and sum across days.
    """
    from datetime import datetime as _dt_inner, timedelta as _td_inner

    result = {
        "type_counts": [],
        "flag_counts": [],
        "alt_histogram": [],
        "speed_histogram": [],
        "max_speed": 0,
        "max_alt": 0,
        "max_range": 0,
        "active_aircraft": 0,
        "total_aircraft": 0,
        "stats": {},
        "doppler": {"measurements": [], "ppm_estimate": 0.0},
        "coverage_bins": [0.0] * 72,
        "rssi_bins": [0.0] * 72,
        "rssi_counts": [0] * 72,
        "heatmap_grid": {},
        "squawk_counts": {},
        "adsb_version_counts": {},
        "df_counts": {},
        "cat_history": [],
        "sector_ranges": [],
    }

    try:
        d_from = _dt_inner.strptime(from_date, "%Y-%m-%d").date()
        d_to = _dt_inner.strptime(to_date, "%Y-%m-%d").date()
    except ValueError:
        return result

    if d_from > d_to:
        d_from, d_to = d_to, d_from

    type_agg = {}
    flag_agg = {}
    alt_agg = [0] * 35
    spd_agg = [0] * 30
    max_spd = 0
    max_alt = 0
    max_rng = 0
    total_ac = 0
    coverage_agg = [0.0] * 72
    rssi_agg = [0.0] * 72
    rssi_count_agg = [0] * 72
    heatmap_agg = {}
    squawk_agg = {}
    version_agg = {}
    df_agg = {}
    cat_agg = []

    # --- Disk snapshots: take LAST snapshot per day (cumulative counter) ---
    cur = d_from
    while cur <= d_to:
        date_str = cur.strftime("%Y-%m-%d")
        date_dir = os.path.join(STATS_HISTORY_DIR, date_str)
        if os.path.isdir(date_dir):
            last_snap = None
            for fn in sorted(os.listdir(date_dir)):
                if not fn.endswith(".json"):
                    continue
                try:
                    with open(os.path.join(date_dir, fn), 'r', encoding='utf-8') as sf:
                        last_snap = json_module.load(sf)
                except Exception:
                    continue
            if last_snap:
                for t in last_snap.get("type_counts", []):
                    tname = t.get("type", "Unknown")
                    type_agg[tname] = type_agg.get(tname, 0) + t.get("count", 0)
                for f in last_snap.get("flag_counts", []):
                    fname = f.get("flag", "Unknown")
                    flag_agg[fname] = flag_agg.get(fname, 0) + f.get("count", 0)
                for i, b in enumerate(last_snap.get("alt_histogram", [])):
                    if i < 35:
                        alt_agg[i] += b.get("count", 0)
                for i, b in enumerate(last_snap.get("speed_histogram", [])):
                    if i < 30:
                        spd_agg[i] += b.get("count", 0)
                if last_snap.get("max_speed", 0) > max_spd:
                    max_spd = last_snap["max_speed"]
                if last_snap.get("max_alt", 0) > max_alt:
                    max_alt = last_snap["max_alt"]
                if last_snap.get("max_range", 0) > max_rng:
                    max_rng = last_snap["max_range"]
                if last_snap.get("total_aircraft", 0) > total_ac:
                    total_ac = last_snap["total_aircraft"]
                if last_snap.get("doppler"):
                    result["doppler"] = last_snap["doppler"]
                # Coverage bins (max per sector)
                for i, val in enumerate(last_snap.get("coverage_bins", [])):
                    if i < 72 and val > coverage_agg[i]:
                        coverage_agg[i] = val
                # RSSI bins
                for i, val in enumerate(last_snap.get("rssi_bins", [])):
                    if i < 72 and val > rssi_agg[i]:
                        rssi_agg[i] = val
                for i, val in enumerate(last_snap.get("rssi_counts", [])):
                    if i < 72 and val > rssi_count_agg[i]:
                        rssi_count_agg[i] = val
                # Heatmap (merge — max per cell)
                for k, v in last_snap.get("heatmap_grid", {}).items():
                    heatmap_agg[k] = max(heatmap_agg.get(k, 0), v)
                # Squawk counts (max — cumulative session counter)
                for k, v in last_snap.get("squawk_counts", {}).items():
                    squawk_agg[k] = max(squawk_agg.get(k, 0), v)
                # ADS-B version counts (max — cumulative)
                for k, v in last_snap.get("adsb_version_counts", {}).items():
                    version_agg[k] = max(version_agg.get(k, 0), v)
                # DF counts (max — cumulative)
                for k, v in last_snap.get("df_counts", {}).items():
                    df_agg[k] = max(df_agg.get(k, 0), v)
                # Category history (keep last)
                if last_snap.get("cat_history"):
                    cat_agg = last_snap["cat_history"]
        cur += _td_inner(days=1)

    # --- Live session: add if today is in range ---
    today_str = datetime.datetime.now().strftime("%Y-%m-%d")
    try:
        today = _dt_inner.strptime(today_str, "%Y-%m-%d").date()
        if d_from <= today <= d_to:
            live = db._collect_live_as_snapshot()
            for t in live.get("type_counts", []):
                tname = t.get("type", "Unknown")
                type_agg[tname] = type_agg.get(tname, 0) + t.get("count", 0)
            for f in live.get("flag_counts", []):
                fname = f.get("flag", "Unknown")
                flag_agg[fname] = flag_agg.get(fname, 0) + f.get("count", 0)
            for i, b in enumerate(live.get("alt_histogram", [])):
                if i < 35:
                    alt_agg[i] += b.get("count", 0)
            for i, b in enumerate(live.get("speed_histogram", [])):
                if i < 30:
                    spd_agg[i] += b.get("count", 0)
            if live.get("max_speed", 0) > max_spd:
                max_spd = live["max_speed"]
            if live.get("max_alt", 0) > max_alt:
                max_alt = live["max_alt"]
            if live.get("max_range", 0) > max_rng:
                max_rng = live["max_range"]
            if live.get("total_aircraft", 0) > total_ac:
                total_ac = live["total_aircraft"]
            # Coverage bins (max per sector)
            for i, val in enumerate(live.get("coverage_bins", [])):
                if i < 72 and val > coverage_agg[i]:
                    coverage_agg[i] = val
            for i, val in enumerate(live.get("rssi_bins", [])):
                if i < 72 and val > rssi_agg[i]:
                    rssi_agg[i] = val
            for i, val in enumerate(live.get("rssi_counts", [])):
                if i < 72 and val > rssi_count_agg[i]:
                    rssi_count_agg[i] = val
            for k, v in live.get("heatmap_grid", {}).items():
                heatmap_agg[k] = max(heatmap_agg.get(k, 0), v)
            for k, v in live.get("squawk_counts", {}).items():
                squawk_agg[k] = max(squawk_agg.get(k, 0), v)
            for k, v in live.get("adsb_version_counts", {}).items():
                version_agg[k] = max(version_agg.get(k, 0), v)
            for k, v in live.get("df_counts", {}).items():
                df_agg[k] = max(df_agg.get(k, 0), v)
            if live.get("cat_history"):
                cat_agg = live["cat_history"]
    except Exception:
        pass

    sorted_types = sorted(type_agg.items(), key=lambda x: -x[1])[:15]
    result["type_counts"] = [{"type": t, "count": c} for t, c in sorted_types]
    sorted_flags = sorted(flag_agg.items(), key=lambda x: -x[1])[:15]
    result["flag_counts"] = [{"flag": f, "count": c} for f, c in sorted_flags]
    result["alt_histogram"] = [
        {"range": f"{i*2000 - 1200}-{(i+1)*2000 - 1200}", "count": alt_agg[i]}
        for i in range(35)
    ]
    result["speed_histogram"] = [
        {"range": f"{i*50}-{(i+1)*50}", "count": spd_agg[i]}
        for i in range(30)
    ]
    result["max_speed"] = max_spd
    result["max_alt"] = max_alt
    result["max_range"] = max_rng
    result["total_aircraft"] = total_ac
    result["active_aircraft"] = len(db.ac) if hasattr(db, "ac") else 0
    result["coverage_bins"] = coverage_agg
    result["rssi_bins"] = rssi_agg
    result["rssi_counts"] = rssi_count_agg
    result["heatmap_grid"] = heatmap_agg
    result["squawk_counts"] = squawk_agg
    result["adsb_version_counts"] = version_agg
    result["df_counts"] = df_agg
    result["cat_history"] = cat_agg
    # Compute sector_ranges from coverage_agg (12 sectors x 30 degrees)
    sector_ranges = []
    for i in range(12):
        s = i * 6; e = s + 6
        bins = coverage_agg[s:e]
        rssi_bins_s = rssi_agg[s:e]
        counts_s = rssi_count_agg[s:e]
        max_r = max(bins) if bins else 0
        total_counts = sum(counts_s)
        avg_rssi = sum(rb * c for rb, c in zip(rssi_bins_s, counts_s)) / max(total_counts, 1) if total_counts > 0 else None
        sector_ranges.append({
            "sector": i,
            "azimuth": f"{s*5}-{e*5}",
            "max_range_km": round(max_r, 1),
            "avg_rssi": round(avg_rssi, 1) if avg_rssi is not None else None,
        })
    result["sector_ranges"] = sector_ranges
    result["stats"] = {
        "total_aircraft": total_ac,
        "max_speed": max_spd,
        "max_alt": max_alt,
        "max_range": max_rng,
        "unit_speed": UNIT_SPEED,
        "unit_alt": UNIT_ALT,
        "unit_distance": UNIT_DISTANCE,
        "unit_vrate": UNIT_VRATE,
    }

    return result


def get_pos_history_range(from_date, to_date):
    """Load position snapshots for date range."""
    from datetime import datetime as _dt, timedelta as _td
    positions = []
    dates = []
    try:
        d_from = _dt.strptime(from_date, "%Y-%m-%d").date()
        d_to = _dt.strptime(to_date, "%Y-%m-%d").date()
    except ValueError:
        return {"positions": positions, "dates": dates}
    if d_from > d_to:
        d_from, d_to = d_to, d_from
    cur = d_from
    while cur <= d_to:
        ds = cur.strftime("%Y-%m-%d")
        dd = os.path.join(POS_HISTORY_DIR, ds)
        if os.path.isdir(dd):
            dates.append(ds)
            for fn in sorted(os.listdir(dd)):
                if not fn.endswith(".json"):
                    continue
                try:
                    with open(os.path.join(dd, fn), 'r', encoding='utf-8') as sf:
                        snap = json_module.load(sf)
                    for p in snap.get("positions", []):
                        positions.append(p)
                except Exception:
                    pass
        cur += _td(days=1)
    return {"positions": positions, "dates": dates, "count": len(positions)}


# ==================================================================
# AIRCRAFTDB HELPER FUNCTIONS
# ==================================================================


def generate_startup_wav():
    """Startup sound: two ascending tones 660->990 Hz + final beep 1320 Hz."""
    sr = 22050
    samples = []
    dur1 = int(sr * 0.15)
    for i in range(dur1):
        env = min(1.0, i / (sr * 0.005)) * min(1.0, (dur1 - i) / (sr * 0.01))
        samples.append(int(12000 * env * math.sin(2 * math.pi * 660 * i / sr)))
    for i in range(int(sr * 0.03)):
        samples.append(0)
    dur2 = int(sr * 0.20)
    for i in range(dur2):
        env = min(1.0, i / (sr * 0.005)) * min(1.0, (dur2 - i) / (sr * 0.015))
        samples.append(int(14000 * env * math.sin(2 * math.pi * 990 * i / sr)))
    for i in range(int(sr * 0.05)):
        samples.append(0)
    dur3 = int(sr * 0.08)
    for i in range(dur3):
        env = min(1.0, i / (sr * 0.003)) * min(1.0, (dur3 - i) / (sr * 0.005))
        samples.append(int(10000 * env * math.sin(2 * math.pi * 1320 * i / sr)))
    filename = os.path.join(SOUND_DIR, "startup.wav")
    _write_wav(filename, samples, sr)
    return filename

def create_db(player=None):
    """Create AircraftDB with pre-generated sounds."""
    sounds = {
        "sound_file": generate_beep_wav() if player else None,
        "emergency_sound": generate_emergency_wav() if player else None,
        "loss_sound": generate_beep_wav(count=3, long=True) if player else None,
        "tcas_sound": generate_tcas_wav() if player else None,
        "low_alt_sound": generate_low_alt_wav() if player else None,
        "mil_sound": generate_mil_wav() if player else None,
        "tracked_sound": generate_tracked_wav() if player else None,
        "tracked_loss_sound": generate_tracked_loss_wav() if player else None,
        "record_sound": generate_record_wav() if player else None,
        "silence_sound": generate_silence_wav() if player else None,
        "turbulence_sound": generate_turbulence_wav() if player else None,
        "squawk_7500_sound": generate_squawk_wav("7500") if player else None,
        "squawk_7600_sound": generate_squawk_wav("7600") if player else None,
        "squawk_7700_sound": generate_squawk_wav("7700") if player else None,
        "long_range_sound": generate_long_range_wav() if player else None,
        "overhead_sound": generate_overhead_wav() if player else None,
        "speed_record_sound": generate_speed_record_wav() if player else None,
        "alt_record_sound": generate_alt_record_wav() if player else None,
        "helicopter_sound": generate_helicopter_wav() if player else None,
        "rare_type_sound": generate_rare_type_wav() if player else None,
        "rapid_descent_sound": generate_rapid_descent_wav() if player else None,
        "return_sound": generate_return_wav() if player else None,
        "level_off_sound": generate_level_off_wav() if player else None,
        "ground_vehicle_sound": generate_ground_vehicle_wav() if player else None,
        "proximity_sound": generate_proximity_wav() if player else None,
        "headon_sound": generate_headon_wav() if player else None,
        "breakthrough_sound": generate_breakthrough_wav() if player else None,
        "squawk_change_sound": generate_squawk_change_wav() if player else None,
        "startup_sound": generate_startup_wav() if player else None,
        "player": player,
    }
    return AircraftDB(**sounds)



# ==================================================================
# NETWORK OUTPUTS (missing classes)
# ==================================================================


class BeastEncoder:
    """Beast protocol (Beast binary format) output of raw ADS-B/Mode S messages over TCP.

    Beast frame format (compatible with dump1090-fa / piaware / readsb):
      0x1a  — escape byte (start marker)
      type  — 1 byte: 0x32=Mode AC, 0x33=Mode S short(56bit), 0x34=Mode S long(112bit)
      MLAT  — 7 bytes big-endian timestamp (12 MHz timer)
      SIG   — 1 byte signal level (0x00 — no data)
      MSG   — raw message bytes (7 bytes for 56-bit, 14 bytes for 112-bit)

    Escaping: any 0x1a byte inside MLAT/SIG/MSG is doubled.
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self._retry_t = 0
        self._connect()

    def _connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print("Beast OUT -> %s:%d" % (self.host, self.port))
        except Exception as e:
            if DEBUG: sys.stderr.write(f"[beast] connect: {e}\n")
            self.sock = None
            self._retry_t = time.time() + 10

    @staticmethod
    def _bits_to_bytes(bits):
        """Convert bit list [0,1,1,0,...] to bytes."""
        n = len(bits) // 8
        return bytes(
            (bits[i*8] << 7) | (bits[i*8+1] << 6) | (bits[i*8+2] << 5) |
            (bits[i*8+3] << 4) | (bits[i*8+4] << 3) | (bits[i*8+5] << 2) |
            (bits[i*8+6] << 1) | bits[i*8+7]
            for i in range(n)
        )

    @staticmethod
    def _escape(data):
        """Escape 0x1a byte in Beast frame data."""
        out = bytearray()
        for b in data:
            if b == 0x1a:
                out.append(0x1a)
                out.append(0x1a)
            else:
                out.append(b)
        return bytes(out)

    def send(self, bits, rssi=None, mlat_ts=None):
        """Send message in Beast format.

    Parameters:
      bits    — bit list [0,1,...] of full Mode S message (56 or 112 bits)
      rssi    — RSSI in dB (float) or None; converted to 1 byte (0-255)
      mlat_ts — receive time (float, seconds from time.time()) or None
        """
        if not self.sock:
            if time.time() >= self._retry_t:
                self._connect()
            if not self.sock:
                return

        # Determine DF for frame type selection
        df = (bits[0] << 4) | (bits[1] << 3) | (bits[2] << 2) | (bits[3] << 1) | bits[4]

        if df in (0, 1, 4, 5, 6, 11, 12):
            # Short Mode S message — 56 bits = 7 bytes
            msg_type = 0x33
            msg_bytes = self._bits_to_bytes(bits[:56])
        else:
            # Long Mode S message — 112 bits = 14 bytes
            msg_type = 0x34
            msg_bytes = self._bits_to_bytes(bits[:112])

        # MLAT timestamp: 7 bytes big-endian
        if mlat_ts is not None:
            if MLAT_TIMESTAMP_PRECISION_US > 0:
                ticks = int(mlat_ts * MLAT_TIMESTAMP_PRECISION_US * 1e6)
            else:
                ticks = int(mlat_ts * 1e6)
            mlat_bytes = ticks.to_bytes(7, byteorder='big')
        else:
            mlat_bytes = b'\x00\x00\x00\x00\x00\x00\x00'

        # Signal: convert RSSI from dB to arbitrary 0-255 level
        if rssi is not None:
            # RSSI in dB, typically from -50 (strong) to -100 (weak)
            # Map -100..-30 to 0..255
            sig_raw = max(0, min(255, int((rssi + 100) * 255 / 70)))
            sig_bytes = bytes([sig_raw])
        else:
            sig_bytes = b'\x00'

        # Build frame: 0x1a + type + escaped(MLAT + SIG + MSG)
        payload = mlat_bytes + sig_bytes + msg_bytes
        escaped_payload = self._escape(payload)

        try:
            frame = bytes([0x1a, msg_type]) + escaped_payload
            self.sock.sendall(frame)
        except Exception as e:
            if DEBUG: sys.stderr.write(f"[beast] send: {e}\n")
            self.sock = None
            self._retry_t = time.time() + 10

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
class SBS1Encoder:
    """SBS1/BaseStation format output over TCP."""
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self._retry_t = 0
        self._connect()

    def _connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print("SBS1 OUT -> %s:%d" % (self.host, self.port))
        except Exception as e:
            if DEBUG: sys.stderr.write(f"[sbs1] connect: {e}\n")
            self.sock = None
            self._retry_t = time.time() + 10

    def send(self, icao, callsign="", lat=None, lon=None, alt=None,
             speed=None, heading=None, squawk=""):
        if not self.sock:
            if time.time() >= self._retry_t:
                self._connect()
            if not self.sock:
                return
        try:
            fields = ["MSG", "3", "", "", icao, "", squawk, callsign, "",
                      "", "", str(alt) if alt is not None else "", "", "",
                      str(lat) if lat is not None else "", str(lon) if lon is not None else "",
                      "", "", "", str(speed) if speed is not None else "",
                      str(heading) if heading is not None else ""]
            line = ",".join(fields) + "\n"
            self.sock.sendall(line.encode("ascii"))
        except Exception as e:
            if DEBUG: sys.stderr.write(f"[sbs1] send: {e}\n")
            self.sock = None
            self._retry_t = time.time() + 10

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


# ==================================================================
# AIRCRAFT PHOTOS (Planespotters.net — free, no API key)
# ==================================================================

import urllib.request as _url_req
import urllib.error as _url_err

def _photo_cache_path(icao):
    """Path to photo cache file for given ICAO."""
    return os.path.join(PHOTO_CACHE_DIR, icao.upper() + ".json")

def _photo_neg_cache_path(icao):
    """Path to negative cache file (no photo found)."""
    return os.path.join(PHOTO_CACHE_DIR, icao.upper() + ".neg")

# Negative cache TTL, sec (1 hour — shorter than positive cache).
PHOTO_NEG_CACHE_TTL = 3600

def _photo_cache_valid(path):
    """Check cache freshness (by TTL)."""
    if not os.path.exists(path):
        return False
    age = time.time() - os.path.getmtime(path)
    return age < PHOTO_CACHE_TTL

def _fetch_planespotters(icao_hex, registration=None):
    """
    Query Planespotters API by ICAO hex, with fallback to registration.
    Returns dict: {"url": ..., "thumbnail": ..., "photographer": ..., "link": ...}
    or None if no photos found.
    """
    # Planespotters API requires a descriptive User-Agent with contact info.
    # See: https://www.planespotters.net/photo/api
    PS_USER_AGENT = "DX1090-ADS-B-Dashboard/9.0 (https://github.com/r4scs/dx1090)"
    PS_HEADERS = {
        "User-Agent": PS_USER_AGENT,
        "Accept": "application/json",
        "Referer": "https://github.com/r4scs/dx1090",
    }

    def _extract_photo_url(val):
        """Extract URL from Planespotters API field (string or {"src": "..."} object)."""
        if isinstance(val, str):
            return val
        if isinstance(val, dict):
            return val.get("src") or ""
        return ""

    urls_to_try = []
    if icao_hex and len(icao_hex) == 6 and icao_hex != "000000":
        urls_to_try.append(
            f"https://api.planespotters.net/pub/photos/hex/{icao_hex}"
        )
    if registration:
        urls_to_try.append(
            f"https://api.planespotters.net/pub/photos/reg/{registration}"
        )

    for api_url in urls_to_try:
        try:
            req = _url_req.Request(api_url, headers=PS_HEADERS)
            with _url_req.urlopen(req, timeout=10) as resp:
                data = json_module.loads(resp.read().decode("utf-8"))
            # Check for API error response (e.g. blocked User-Agent)
            if "error" in data:
                log.warning("Planespotters API error for %s: %s", icao_hex, data["error"])
                continue
            photos = data.get("photos", [])
            if photos:
                p = photos[0]
                url = _extract_photo_url(p.get("thumbnail_large")) or _extract_photo_url(p.get("thumbnail"))
                thumb = _extract_photo_url(p.get("thumbnail"))
                if not url:
                    continue  # skip if no valid URL
                return {
                    "url": url,
                    "thumbnail": thumb,
                    "photographer": p.get("photographer") or "",
                    "link": p.get("link") or "",
                    "registration": p.get("registration") or "",
                    "aircraft_type": p.get("aircraft_type") or "",
                    "airline": p.get("airline") or "",
                    "source": "planespotters.net",
                }
        except _url_err.HTTPError as e:
            log.warning("Planespotters HTTP %d for %s (%s): %s",
                        e.code, icao_hex, api_url, e.reason)
            continue
        except _url_err.URLError as e:
            log.warning("Planespotters URL error for %s: %s", icao_hex, e)
            continue
        except Exception as e:
            log.warning("Planespotters error for %s: %s", icao_hex, e)
            continue
    return None


def fetch_aircraft_photo(icao_hex, registration=None):
    """
    Get aircraft photo (with disk caching).
    Returns dict or None.
    """
    cache_file = _photo_cache_path(icao_hex)

    # Cache check
    if _photo_cache_valid(cache_file):
        try:
            with open(cache_file, "r", encoding="utf-8") as f:
                return json_module.load(f)
        except Exception:
            pass
    # Check negative cache (1 hour TTL)
    neg_file = _photo_neg_cache_path(icao_hex)
    if os.path.exists(neg_file):
        if time.time() - os.path.getmtime(neg_file) < PHOTO_NEG_CACHE_TTL:
            return None

    # API request
    result = _fetch_planespotters(icao_hex, registration)

    # Save to cache
    if result is not None:
        try:
            with open(cache_file, "w", encoding="utf-8") as f:
                json_module.dump(result, f, ensure_ascii=False)
        except Exception:
            pass
    else:
        # Save negative cache marker (1 hour TTL)
        try:
            with open(_photo_neg_cache_path(icao_hex), "w", encoding="utf-8") as f:
                f.write("")
        except Exception:
            pass

    return result


class JsonServer:
    """HTTP server: serves JSON with aircraft list at /data.json."""
    def __init__(self, db, host, port):
        self.db = db
        self.host = host
        self.port = port
        self._thread = None
        self._server = None

    def _make_handler(self):
        db = self.db
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def end_headers(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                super().end_headers()
            def do_OPTIONS(self):
                self.send_response(204)
                self.end_headers()
            def do_GET(self):
                                # === Static file serving (Chart.js, Leaflet, etc.) ===
                if self.path.startswith("/static/"):
                    static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
                    rel_path = self.path[8:]
                    if ".." in rel_path or rel_path.startswith("/"):
                        self.send_error(403); return
                    file_path = os.path.join(static_dir, rel_path)
                    if os.path.exists(file_path) and os.path.isfile(file_path):
                        ext = os.path.splitext(rel_path)[1].lower()
                        ctypes = {".js":"application/javascript; charset=utf-8",".css":"text/css; charset=utf-8",".png":"image/png",".jpg":"image/jpeg",".svg":"image/svg+xml",".woff2":"font/woff2",".json":"application/json; charset=utf-8"}
                        ct = ctypes.get(ext, "application/octet-stream")
                        with open(file_path, "rb") as sf: data = sf.read()
                        self.send_response(200)
                        self.send_header("Content-Type", ct)
                        self.send_header("Content-Length", str(len(data)))
                        self.send_header("Cache-Control", "public, max-age=86400")
                        self.end_headers()
                        self.wfile.write(data)
                    else:
                        self.send_error(404)
                    return

                if self.path == "/" or self.path == "/dashboard.html":
                    dash = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")
                    if os.path.exists(dash):
                        with open(dash, "r", encoding="utf-8") as df:
                            html = df.read()
                        payload = html.encode("utf-8")
                        self.send_response(200)
                        self.send_header("Content-Type", "text/html; charset=utf-8")
                        self.send_header("Content-Length", str(len(payload)))
                        self.end_headers()
                        self.wfile.write(payload)
                        return
                    self.send_response(404)
                    self.end_headers()
                    return
                if self.path.startswith("/tiles/"):
                    # Tile proxy with disk cache
                    import urllib.request as _ureq
                    parts = self.path.replace("/tiles/", "").split("/")
                    if len(parts) >= 3:
                        z, x, y = parts[0], parts[1], parts[2].split(".")[0]
                        cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), TILE_CACHE_DIR, z, x)
                        cache_file = os.path.join(cache_dir, y + ".png")
                        if os.path.exists(cache_file):
                            with open(cache_file, "rb") as tf:
                                payload = tf.read()
                            self.send_response(200)
                            self.send_header("Content-Type", "image/png")
                            self.send_header("Cache-Control", "public, max-age=86400")
                            self.send_header("Content-Length", str(len(payload)))
                            self.end_headers()
                            self.wfile.write(payload)
                            return
                        # Fetch from OSM
                        try:
                            url = f"https://tile.openstreetmap.org/{z}/{x}/{y}.png"
                            req = _ureq.Request(url, headers={"User-Agent": "dx1090/1.0"})
                            resp = _ureq.urlopen(req, timeout=10)
                            payload = resp.read()
                            os.makedirs(cache_dir, exist_ok=True)
                            with open(cache_file, "wb") as tf:
                                tf.write(payload)
                            self.send_response(200)
                            self.send_header("Content-Type", "image/png")
                            self.send_header("Cache-Control", "public, max-age=86400")
                            self.send_header("Content-Length", str(len(payload)))
                            self.end_headers()
                            self.wfile.write(payload)
                            return
                        except Exception:
                            self.send_response(404)
                            self.end_headers()
                            return
                    self.send_response(404)
                    self.end_headers()
                    return
                if self.path == "/stats.json":
                    _s = db.stats
                    _pt = _s.get("preambles_total", 0)
                    _ps = _s.get("preambles_strong", 0)
                    _mt = _s.get("msg_total", _pt)
                    _mv = _s.get("msg_valid", 0)
                    stats = {
                        "preambles": _mt,
                        "valid": _mv,
                        "msg_rate": round(db.get_msg_rate(), 1) if hasattr(db, "get_msg_rate") else 0,
                        "total_preambles": _pt,
                        "strong_percent": round(_ps / _pt * 100, 1) if _pt > 0 else 0,
                        "decode_percent": round(db.decode_quality_ema, 1),
                        "receive_percent": round(db.recv_quality_ema, 1),
                        "sil": _s.get("sil_decoded", 0),
                        "nacv": _s.get("nacv_decoded", 0),
                        "ver": _s.get("version_decoded", 0),
                        "returns": _s.get("return_count", 0),
                        "max_speed": _s.get("max_speed_kt", 0),
                        "min_speed": _s.get("min_speed_kt", 0),
                        "max_alt": _s.get("max_alt_ft", 0),
                        "min_alt": _s.get("min_alt_ft", 0),
                        "max_range": round(_s.get("max_range_km", 0), 1),
                        "total_aircraft": _s.get("total_aircraft", 0),
                        "home_lat": HOME_LAT,
                        "home_lon": HOME_LON,
                        "unit_speed": UNIT_SPEED,
                        "unit_alt": UNIT_ALT,
                        "unit_distance": UNIT_DISTANCE,
                        "unit_vrate": UNIT_VRATE,
                        "unit_pressure": UNIT_PRESSURE,
                        "unit_temp": UNIT_TEMP,
                        "map_mode": MAP_MODE,
                    }
                    payload = _json.dumps(stats, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path == "/coverage.json":
                    bins = []
                    for i in range(72):
                        angle = i * 5.0
                        r = db.coverage_bins[i] if hasattr(db, "coverage_bins") else 0
                        bins.append({"angle": angle, "range": round(r, 1), "count": db.rssi_counts[i]})
                    payload = _json.dumps({"bins": bins, "home_lat": HOME_LAT, "home_lon": HOME_LON}, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path == "/sysinfo.json":
                    try:
                        si = get_sysinfo()
                        body = _json.dumps(si).encode()
                        self.send_response(200)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                    except Exception:
                        self.send_error(500)
                    return
                if self.path == "/extended_analytics.json":
                    try:
                        db.collect_analytics(force=True)
                        _ea = db.get_analytics_json()
                        payload = _ea if isinstance(_ea, bytes) else _json.dumps(_ea, ensure_ascii=False).encode("utf-8")
                    except Exception as e:
                        payload = _json.dumps({"error": str(e)}, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path == "/analytics.json":
                    try:
                        db.collect_analytics()
                        _aj = db.get_analytics_json()
                        payload = _aj if isinstance(_aj, bytes) else _json.dumps(_aj, ensure_ascii=False).encode("utf-8")
                    except Exception as e:
                        payload = _json.dumps({"error": str(e)}, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path.startswith("/stats_history.json"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    date = qs.get("date", [None])[0]

                    if date is None:
                        dates = []
                        if os.path.isdir(STATS_HISTORY_DIR):
                            for d in sorted(os.listdir(STATS_HISTORY_DIR), reverse=True):
                                if os.path.isdir(os.path.join(STATS_HISTORY_DIR, d)):
                                    dates.append(d)
                        payload = _json.dumps({"dates": dates}, ensure_ascii=False).encode("utf-8")
                    else:
                        date_dir = os.path.join(STATS_HISTORY_DIR, date)
                        snapshots = []
                        if os.path.isdir(date_dir):
                            for fn in sorted(os.listdir(date_dir)):
                                if fn.endswith(".json"):
                                    try:
                                        with open(os.path.join(date_dir, fn), 'r', encoding='utf-8') as sf:
                                            snap = json_module.load(sf)
                                        snapshots.append({
                                            "datetime": snap.get("datetime", fn[:-5]),
                                            "timestamp": snap.get("timestamp", 0),
                                            "total_aircraft": snap.get("total_aircraft", 0),
                                            "active_aircraft": snap.get("active_aircraft", 0),
                                            "time": fn[:-5],
                                            "label": snap.get("datetime", fn[:-5]),
                                        })
                                    except Exception:
                                        pass
                        payload = _json.dumps({"date": date, "snapshots": snapshots}, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path.startswith("/pos_history.json"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    from_date = qs.get("from", [None])[0]
                    to_date = qs.get("to", [None])[0]
                    if from_date and to_date:
                        try:
                            result = get_pos_history_range(from_date, to_date)
                            payload = _json.dumps(result, ensure_ascii=False).encode("utf-8")
                        except Exception as e:
                            payload = _json.dumps({"error": str(e)}, ensure_ascii=False).encode("utf-8")
                    else:
                        dates = []
                        if os.path.isdir(POS_HISTORY_DIR):
                            for d in sorted(os.listdir(POS_HISTORY_DIR), reverse=True):
                                if os.path.isdir(os.path.join(POS_HISTORY_DIR, d)):
                                    dates.append(d)
                        payload = _json.dumps({"dates": dates, "positions": []}, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path.startswith("/stats_snapshot.json"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    date = qs.get("date", [None])[0]
                    time_str = qs.get("time", [None])[0]
                    if date and time_str:
                        time_safe = time_str.replace(":", "-")
                        filepath = os.path.join(STATS_HISTORY_DIR, date, f"{time_safe}.json")
                        if not os.path.exists(filepath):
                            filepath = os.path.join(STATS_HISTORY_DIR, date, f"{date}_{time_safe}.json")
                        if os.path.exists(filepath):
                            with open(filepath, 'r', encoding='utf-8') as sf:
                                payload = sf.read().encode("utf-8")
                            self.send_response(200)
                            self.send_header("Content-Type", "application/json; charset=utf-8")
                            self.send_header("Content-Length", str(len(payload)))
                            self.end_headers()
                            self.wfile.write(payload)
                            return
                    self.send_response(404)
                    self.end_headers()
                    return

                if self.path.startswith("/stats_range.json"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    from_date = qs.get("from", [None])[0]
                    to_date = qs.get("to", [None])[0]
                    if from_date and to_date:
                        try:
                            result = get_history_range(db, from_date, to_date)
                            payload = _json.dumps(result, ensure_ascii=False).encode("utf-8")
                        except Exception as e:
                            payload = _json.dumps({"error": str(e)}, ensure_ascii=False).encode("utf-8")
                    else:
                        payload = _json.dumps({"error": "from and to required"}, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path.startswith("/sound_events.json"):
                    events = []
                    while _sound_event_queue:
                        try:
                            events.append(_sound_event_queue.popleft())
                        except Exception:
                            break
                    resp = {"events": events}
                    payload = _json.dumps(resp, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path.startswith("/sound_file"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    stype = qs.get("type", [None])[0]
                    fname = SOUND_TYPE_TO_FILE.get(stype)
                    if fname and os.path.exists(_sound_path(fname)):
                        with open(_sound_path(fname), "rb") as sf:
                            payload = sf.read()
                        self.send_response(200)
                        self.send_header("Content-Type", "audio/wav")
                        self.send_header("Content-Length", str(len(payload)))
                        self.end_headers()
                        self.wfile.write(payload)
                    else:
                        self.send_response(404)
                        self.end_headers()
                    return

                if self.path.startswith("/aircraft_photo"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    icao_hex = (qs.get("icao", [None])[0] or "").strip().upper()
                    reg = (qs.get("reg", [None])[0] or "").strip()
                    if not icao_hex or len(icao_hex) != 6:
                        self.send_response(400)
                        self.send_header("Content-Type", "application/json")
                        self.end_headers()
                        self.wfile.write(b'{"error":"missing icao"}')
                        return
                    photo = fetch_aircraft_photo(icao_hex, reg or None)
                    if photo:
                        payload = _json.dumps({"photo": photo}, ensure_ascii=False).encode("utf-8")
                    else:
                        payload = b'{"photo":null}'
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Cache-Control", "public, max-age=3600")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path.startswith("/sound"):
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    action = qs.get("action", ["status"])[0]
                    _g = globals()
                    if action == "toggle":
                        _g["ENABLE_SOUND"] = not _g.get("ENABLE_SOUND", True)
                        resp = {"enabled": _g["ENABLE_SOUND"]}
                    elif action == "toggle_type":
                        t = qs.get("type", [None])[0]
                        if t and t in SOUND_TYPES:
                            var_name = SOUND_TYPES[t][1]
                            _g[var_name] = not _g.get(var_name, True)
                            resp = {"enabled": _g.get("ENABLE_SOUND", True), "type": t, "value": _g[var_name]}
                        else:
                            resp = {"error": "unknown type"}
                    elif action == "set_type":
                        t = qs.get("type", [None])[0]
                        v = qs.get("value", ["true"])[0] == "true"
                        if t and t in SOUND_TYPES:
                            var_name = SOUND_TYPES[t][1]
                            _g[var_name] = v
                            resp = {"enabled": _g.get("ENABLE_SOUND", True), "type": t, "value": _g[var_name]}
                        else:
                            resp = {"error": "unknown type"}
                    elif action == "browser_takeover":
                        _g["_browser_sound_mode"] = True
                        _g["_browser_heartbeat_time"] = time.time()
                        resp = {"enabled": _g.get("ENABLE_SOUND", True), "browser_mode": True, "types": {
                            k: _g.get(vn, True) for k, (_, vn) in SOUND_TYPES.items()
                        }}
                    elif action == "heartbeat":
                        _g["_browser_heartbeat_time"] = time.time()
                        resp = {"ok": True}
                    elif action == "browser_release":
                        _g["_browser_sound_mode"] = False
                        resp = {"enabled": _g.get("ENABLE_SOUND", True), "browser_mode": False}
                    else:
                        types = {}
                        for k, (_, var_name) in SOUND_TYPES.items():
                            types[k] = _g.get(var_name, True)
                        resp = {
                            "enabled": _g.get("ENABLE_SOUND", True),
                            "types": types,
                            "browser_mode": _g.get("_browser_sound_mode", False),
                        }
                    payload = _json.dumps(resp, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path == "/events.json":
                    _ESC = chr(27)
                    events = []
                    for e in db.new_events[-100:]:
                        clean = e
                        while True:
                            i = clean.find(_ESC + "[")
                            if i < 0:
                                break
                            j = clean.find("m", i)
                            if j < 0:
                                break
                            clean = clean[:i] + clean[j+1:]
                        clean = clean.strip()
                        etype = "other"
                        if "[RETURN]" in clean:
                            etype = "return"
                        elif "[NEW]" in clean:
                            etype = "new"
                        elif "[LOSS]" in clean:
                            etype = "loss"
                        elif "[MAC LOSS]" in clean:
                            etype = "loss"
                        elif "SQUAWK" in clean and ("7700" in clean or "7600" in clean or "7500" in clean):
                            etype = "emergency"
                        elif "[EMERGENCY]" in clean:
                            etype = "emergency"
                        elif "[TCAS" in clean:
                            etype = "emergency"
                        events.append({"text": clean, "type": etype, "time": time.time()})
                    payload = _json.dumps(events, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                # GZ export
                if self.path == "/export.gz":
                    try:
                        dl = []
                        with db._lock:
                            ac_snapshot = dict(db.ac)
                        for icao in sorted(ac_snapshot, key=lambda ic: db._sort_key_safe(ic, ac_snapshot)):
                            a = ac_snapshot[icao]
                            lat = a.get("lat_dr", a.get("lat"))
                            lon = a.get("lon_dr", a.get("lon"))
                            dl.append({"icao": icao, "callsign": a.get("callsign", ""),
                                       "lat": lat, "lon": lon, "altitude": a.get("altitude"),
                                       "speed": a.get("speed_gs") or a.get("speed"),
                                       "heading": a.get("heading"), "squawk": a.get("squawk", ""),
                                       "rssi": a.get("rssi"), "distance": a.get("distance"),
                                       "msg_count": a.get("msg_count", 0)})
                        raw = _json.dumps({"aircraft": dl, "home_lat": HOME_LAT, "home_lon": HOME_LON},
                                          ensure_ascii=False).encode("utf-8")
                        payload = _gz.compress(raw, 6)
                        self.send_response(200)
                        self.send_header("Content-Type", "application/gzip")
                        self.send_header("Content-Disposition", 'attachment; filename="adsb_export.gz"')
                        self.send_header("Content-Length", str(len(payload)))
                        self.end_headers()
                        self.wfile.write(payload)
                        log.info("GZ export: %d aircraft", len(dl))
                    except Exception as e:
                        self.send_response(500); self.end_headers()
                        log.error("GZ export: %s", e)
                    return

                # GPX export
                if self.path == "/export.gpx":
                    try:
                        L = ['<?xml version="1.0" encoding="UTF-8"?>',
                             '<gpx version="1.1" creator="DX1090" xmlns="http://www.topografix.com/GPX/1/1">']
                        cnt = 0
                        with db._lock:
                            ac_snapshot = dict(db.ac)
                        for icao in sorted(ac_snapshot, key=lambda ic: db._sort_key_safe(ic, ac_snapshot)):
                            a = ac_snapshot[icao]
                            lat = a.get("lat_dr", a.get("lat"))
                            lon = a.get("lon_dr", a.get("lon"))
                            if lat is None or lon is None: continue
                            cs = a.get("callsign") or icao
                            alt_m = None
                            if a.get("altitude") is not None:
                                alt_m = int(a["altitude"] * 0.3048)
                            L.append(f'  <wpt lat="{lat:.6f}" lon="{lon:.6f}">')
                            if alt_m is not None: L.append(f"    <ele>{alt_m}</ele>")
                            L.append(f"    <name>{cs}</name>")
                            desc = f"ICAO:{icao}"
                            if a.get("squawk"): desc += f" SQK:{a['squawk']}"
                            if a.get("speed_gs") or a.get("speed"):
                                desc += f" SPD:{a.get('speed_gs') or a.get('speed')}kt"
                            if a.get("heading") is not None:
                                desc += f" HDG:{a['heading']:.0f}"
                            em = a.get("emergency", "")
                            if em and em != "No emergency": desc += f" EMER:{em}"
                            L.append(f"    <desc>{desc}</desc>")
                            L.append("  </wpt>")
                            cnt += 1
                        L.append("</gpx>")
                        payload = "\n".join(L).encode("utf-8")
                        self.send_response(200)
                        self.send_header("Content-Type", "application/gpx+xml; charset=utf-8")
                        self.send_header("Content-Disposition", 'attachment; filename="adsb_export.gpx"')
                        self.send_header("Content-Length", str(len(payload)))
                        self.end_headers()
                        self.wfile.write(payload)
                        log.info("GPX export: %d points", cnt)
                    except Exception as e:
                        self.send_response(500); self.end_headers()
                        log.error("GPX export: %s", e)
                    return

                # Session stats export
                if self.path == "/export_session.json":
                    try:
                        _s = db.stats
                        sd = {
                            "exported_at": datetime.datetime.now().isoformat(),
                            "session": {
                                "total_aircraft": _s.get("total_aircraft", 0),
                                "active_aircraft": len(db.ac),
                                "preambles_total": _s.get("preambles_total", 0),
                                "msg_total": _s.get("msg_total", 0),
                                "msg_valid": _s.get("msg_valid", 0),
                                "msg_rate": round(db.get_msg_rate(), 1) if hasattr(db, "get_msg_rate") else 0,
                                "decode_quality_pct": round(db.decode_quality_ema, 1),
                                "receive_quality_pct": round(db.recv_quality_ema, 1),
                                "crc_corrected": _s.get("crc_corrected", 0),
                                "crc_two_bit_corrected": _s.get("crc_two_bit_corrected", 0),
                                "max_range_km": _s.get("max_range_km", 0),
                                "max_speed_kt": _s.get("max_speed_kt", 0),
                                "max_alt_ft": _s.get("max_alt_ft", 0),
                                "ppm": getattr(db, "ppm_adjusted", PPM_CORRECTION),
                            },
                            "config": {
                                "sdr_type": SDR_TYPE, "freq": FREQ, "gain": GAIN,
                                "sample_rate": SAMPLE_RATE, "ppm": PPM_CORRECTION,
                                "home_lat": HOME_LAT, "home_lon": HOME_LON,
                                "unit_alt": UNIT_ALT, "unit_speed": UNIT_SPEED,
                                "unit_distance": UNIT_DISTANCE, "unit_vrate": UNIT_VRATE,
                            },
                        }
                        try:
                            sd["type_counts"] = dict(sorted(db.type_counts_session.items(), key=lambda x: -x[1])[:20])
                        except Exception:
                            sd["type_counts"] = {}
                        payload = _json.dumps(sd, ensure_ascii=False, indent=2).encode("utf-8")
                        self.send_response(200)
                        self.send_header("Content-Type", "application/json; charset=utf-8")
                        self.send_header("Content-Disposition", 'attachment; filename="adsb_session.json"')
                        self.send_header("Content-Length", str(len(payload)))
                        self.end_headers()
                        self.wfile.write(payload)
                        log.info("Session export: %d aircraft", len(db.ac))
                    except Exception as e:
                        self.send_response(500); self.end_headers()
                        log.error("Session export: %s", e)
                    return

                if self.path != "/data.json":
                    self.send_response(404)
                    self.end_headers()
                    return
                data = []
                with db._lock:
                    ac_snapshot = list(db.ac.items())
                for icao, ac in ac_snapshot:
                    info = AIRCRAFT_INFO.get(icao, {})
                    data.append({
                        "icao": icao,
                        "callsign": ac.get("callsign", ""),
                        "lat": ac.get("lat"),
                        "lon": ac.get("lon"),
                        "alt": ac.get("altitude"),
                        "speed": get_display_speed(ac),
                        "heading": ac.get("heading"),
                        "type": info.get("type", ""),
                        "registration": info.get("reg", ""),
                        "distance": ac.get("distance"),
                        "stale": (time.time() - ac.get("last_seen", 0)) > STALE_TIMEOUT,
                        "emergency": ac.get("emergency", ""),
                        "squawk": ac.get("squawk", ""),
                        "rssi": ac.get("rssi"),
                        "vrate": get_display_vrate(ac),
                        "wind_dir": ac.get("wind_dir"),
                        "wind_spd": ac.get("wind_spd"),
                        "temperature": ac.get("temp"),
                        "nav_title": info.get("model", ""),
                        "operator": info.get("operator", ""),
                        "flag": info.get("flag", ""),
                        "speed_ias": ac.get("speed_ias"),
                        "speed_tas": ac.get("speed_tas"),
                        "category": ac.get("category", ""),
                        "on_ground": ac.get("on_ground", False),
                        "nic": ac.get("nic"),
                        "sil": ac.get("sil"),
                        "nacv": ac.get("nacv"),
                        "adsb_version": ac.get("adsb_version"),
                        "selected_altitude": ac.get("selected_altitude"),
                        "baro_setting": ac.get("baro_setting"),
                        "autopilot": ac.get("autopilot"),
                        "roll": ac.get("roll"),
                        "turn_rate": ac.get("turn_rate"),
                        "true_hdg": ac.get("true_hdg"),
                        "mag_hdg": ac.get("mag_hdg"),
                        "pressure_hpa": ac.get("pressure_hpa"),
                        "trail": ac.get("trail", [])[-20:],
                        "alt_history": ac.get("alt_history", []),
                        "spd_history": ac.get("spd_history", []),
                        "hdg_history": ac.get("hdg_history", []),
                        "vrate_history": ac.get("vrate_history", []),
                        "first_seen": ac.get("first_seen"),
                        "last_seen": ac.get("last_seen"),
                        "gnss_altitude": ac.get("gnss_altitude"),
                        "lat_dr": ac.get("lat_dr"),
                        "lon_dr": ac.get("lon_dr"),
                        "distance_dr": ac.get("distance_dr"),
                        "nac_p": ac.get("nac_p"),
                        "adsb_version": ac.get("adsb_version"),
                        "turbulence": ac.get("turbulence"),
                        "icing": ac.get("icing"),
                        "windshear": ac.get("windshear"),
                        "ground_speed": ac.get("ground_speed"),
                        "cat_desc": ac.get("cat_desc"),
                        "pos_reliable": ac.get("pos_reliable", 0),
                        "vr_source": ac.get("vr_source_tc19"),
                        "selected_heading": ac.get("selected_heading"),
                        "gva": ac.get("gva"),
                        "sda": ac.get("sda"),
                        "nic_supplement": ac.get("nic_supplement"),
                        "sil_supplement": ac.get("sil_supplement"),
                        "transponder_ca": ac.get("transponder_ca"),
                        "ara_vertical": ac.get("ara_vertical"),
                        "ara_horizontal": ac.get("ara_horizontal"),
                        "rac": ac.get("rac"),
                        "microburst": ac.get("microburst"),
                        "wake_vortex": ac.get("wake_vortex"),
                        "ident": ac.get("ident"),
                        "spi": ac.get("spi"),
                        "is_return": icao in db.recently_lost,
                        "just_alerted": ac.get("alerted", False) and ac.get("_just_alerted", False),
                        "msg_count": ac.get("msg_count", 0),
                    })
                payload = _json.dumps(data, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            def do_POST(self):
                global _browser_sound_mode, _browser_heartbeat_time
                from urllib.parse import urlparse, parse_qs
                qs = parse_qs(urlparse(self.path).query)
                action = qs.get("action", [None])[0]
                if action == "set_units":
                    _g = globals()
                    for _uk, _uv in [("distance", "UNIT_DISTANCE"), ("alt", "UNIT_ALT"), ("speed", "UNIT_SPEED"), ("vrate", "UNIT_VRATE"), ("temp", "UNIT_TEMP"), ("map_mode", "MAP_MODE")]:
                        _val = qs.get(_uk, [None])[0]
                        if _val:
                            _g[_uv] = _val
                    try:
                        _dash_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashconf.json")
                        _dcfg = {}
                        if os.path.exists(_dash_path):
                            with open(_dash_path, 'r', encoding='utf-8') as _df:
                                _dcfg = json_module.load(_df)
                        _dcfg["unit_distance"] = _g.get("UNIT_DISTANCE", "km")
                        _dcfg["unit_alt"] = _g.get("UNIT_ALT", "ft")
                        _dcfg["unit_speed"] = _g.get("UNIT_SPEED", "kt")
                        _dcfg["unit_vrate"] = _g.get("UNIT_VRATE", "fpm")
                        _dcfg["unit_temp"] = _g.get("UNIT_TEMP", "F")
                        _dcfg["map_mode"] = _g.get("MAP_MODE", "auto")
                        with open(_dash_path, 'w', encoding='utf-8') as _df:
                            json_module.dump(_dcfg, _df, ensure_ascii=False, indent=2)
                    except Exception as _e:
                        sys.stderr.write("[dashconf] save units: %s\n" % _e)
                    resp = {"ok": True, "unit_distance": _g.get("UNIT_DISTANCE"), "unit_alt": _g.get("UNIT_ALT"), "unit_speed": _g.get("UNIT_SPEED"), "unit_vrate": _g.get("UNIT_VRATE"), "unit_temp": _g.get("UNIT_TEMP")}
                    payload = _json.dumps(resp).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                elif action == "browser_release":
                    _browser_sound_mode = False
                    resp = {"ok": True}
                    payload = _json.dumps(resp).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                else:
                    self.send_response(404)
                    self.end_headers()
        return Handler

    def start(self):
        try:
            self._server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
            self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
            self._thread.start()
        except Exception as e:
            print("JSON OUT: %s" % e)

    def stop(self):
        if self._server:
            try:
                self._server.shutdown()
            except Exception:
                pass


def init_network(db):
    """Initialize network outputs."""
    if ENABLE_BEAST_OUT:
        db.beast_encoder = BeastEncoder(BEAST_OUT_HOST, BEAST_OUT_PORT)
    if ENABLE_SBS1_OUT:
        db.sbs1_encoder = SBS1Encoder(SBS1_OUT_HOST, SBS1_OUT_PORT)
    if ENABLE_JSON_OUT:
        db.json_server = JsonServer(db, JSON_OUT_HOST, JSON_OUT_PORT)
        db.json_server.start()

def shutdown_network(db):
    """Close network connections."""
    if db.beast_encoder:
        db.beast_encoder.close()
    if db.sbs1_encoder:
        db.sbs1_encoder.close()
    if db.json_server:
        db.json_server.stop()

# ==================================================================
# UNIFIED MESSAGE VALIDATION AND PROCESSING
# ==================================================================

_DF_EXPLICIT_ICAO = frozenset({11, 17, 18, 19})
_DF_IMPLICIT_ICAO = frozenset({0, 1, 2, 4, 5, 6, 8, 12, 15, 16, 20, 21, 22, 24, 25})
_DF_SUPPORTED = frozenset({0, 1, 2, 4, 5, 6, 8, 11, 12, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25})

def validate_and_process(bits, db, t, rssi=None, crc_ok=False):
    """CRC validation, error correction and message routing."""
    if not ENABLE_MODE_S: return False
    df = bits_to_int(bits[:5])
    if df not in _DF_SUPPORTED: return False
    if df == 18 and not ENABLE_DF18: return False
    if df == 19 and not ENABLE_DF19: return False
    msglen = 56 if df in (0, 1, 4, 5, 6, 11, 12) else 112

    # If CRC already checked — process directly
    if crc_ok:
        if df in _DF_IMPLICIT_ICAO:
            icao_ap = format(AircraftDB.crc24_icao(bits, msglen), '06X')
            if icao_ap in db.ac or df in (24, 25):
                db.process(bits, t, rssi); return True
            return False
        db.process(bits, t, rssi); return True

    # CRC check
    crc = crc24(bits, msglen)
    if crc == 0:
        if df in _DF_IMPLICIT_ICAO:
            icao_ap = format(AircraftDB.crc24_icao(bits, msglen), '06X')
            if icao_ap in db.ac or df in (24, 25):
                db.process(bits, t, rssi); return True
            return False
        db.process(bits, t, rssi); return True

    # Single-bit correction
    if not ENABLE_CRC_CORRECTION: return False
    syndromes = _FULL_SYNDROMES.get(msglen)
    if syndromes is None: return False
    use_icao_check = df in _DF_IMPLICIT_ICAO

    for i in range(msglen):
        if crc == syndromes[i]:
            bits[i] ^= 1
            if use_icao_check:
                icao_new = format(AircraftDB.crc24_icao(bits, msglen), '06X')
                if icao_new in db.ac or df in (24, 25):
                    db.stats["crc_corrected"] += 1
                    db.process(bits, t, rssi); bits[i] ^= 1; return True
                bits[i] ^= 1
            else:
                db.stats["crc_corrected"] += 1
                db.process(bits, t, rssi); bits[i] ^= 1; return True

    # Two-bit correction (adaptive, dynamic threshold)
    if ENABLE_CRC_TWO_BIT and CRC_TWO_BIT_RANGE > 0:
        if CRC_TWO_BIT_ADAPTIVE:
            if db.get_msg_rate() > CRC_TWO_BIT_MAX_RATE:
                db._two_bit_disabled = True
            else:
                db._two_bit_disabled = False
                db._crc2_time_ema *= 0.95
        if CRC_TWO_BIT_DYNAMIC:
            if not hasattr(db, '_crc2_time_ema'):
                db._crc2_time_ema = 0.0
            _ci = CHUNK_BYTES / SAMPLE_RATE
            if db._crc2_time_ema > CRC_TWO_BIT_TIME_RATIO * _ci:
                db._two_bit_disabled = True
            elif 0 < db._crc2_time_ema < CRC_TWO_BIT_MAX_DECODE_TIME:
                db._two_bit_disabled = False
        if not db._two_bit_disabled:
            _t2_start = time.perf_counter()
            limit = min(CRC_TWO_BIT_RANGE, msglen)
            for i in range(limit):
                si = syndromes[i]
                for j in range(i + 1, limit):
                    if crc == (si ^ syndromes[j]):
                        bits[i] ^= 1; bits[j] ^= 1
                        if use_icao_check:
                            icao_new = format(AircraftDB.crc24_icao(bits, msglen), '06X')
                            if icao_new in db.ac or df in (24, 25):
                                db.stats["crc_corrected"] += 1
                                db.stats["crc_two_bit_corrected"] += 1
                                db.process(bits, t, rssi)
                                bits[i] ^= 1; bits[j] ^= 1; return True
                            bits[i] ^= 1; bits[j] ^= 1
                        else:
                            db.stats["crc_corrected"] += 1
                            db.stats["crc_two_bit_corrected"] += 1
                            db.process(bits, t, rssi)
                            bits[i] ^= 1; bits[j] ^= 1; return True
            _t2_el = time.perf_counter() - _t2_start
            if CRC_TWO_BIT_DYNAMIC:
                db._crc2_time_ema = (CRC_TWO_BIT_TIME_ALPHA * _t2_el
                    + (1 - CRC_TWO_BIT_TIME_ALPHA) * db._crc2_time_ema)
                if db._crc2_time_ema > CRC_TWO_BIT_MAX_DECODE_TIME:
                    db._two_bit_disabled = True
                    log.debug("2-bit CRC off: EMA=%.4fs", db._crc2_time_ema)
    return False

# ==================================================================
# DECODE THREAD
# ==================================================================

class DecodeWorker:
    """Background thread for parallel message decoding."""

    def __init__(self, db):
        self.db = db
        self.queue = queue.Queue(maxsize=DECODE_QUEUE_SIZE)
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.msg_total = 0; self.msg_valid = 0
        self._stop = False
        self.thread.start()

    def _worker(self):
        while not self._stop:
            try:
                item = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None: break
            bits, t, rssi, crc_ok, recv_strong = item
            try:
                ok = validate_and_process(bits, self.db, t, rssi, crc_ok)
                if ok: self.msg_valid += 1
                self.db.record_quality(ok, recv_strong)
            except Exception:
                        log.exception("DecodeWorker: error processing message")

    def submit(self, bits, t, rssi=None, crc_ok=False, recv_strong=False):
        try:
            self.queue.put_nowait((bits, t, rssi, crc_ok, recv_strong))
            self.msg_total += 1
        except queue.Full:
            self.db.stats["queue_drops"] += 1

    def stop(self):
        self._stop = True
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            pass
        self.thread.join(timeout=3)

# ==================================================================
# DATA EXPORT
# ==================================================================

def export_csv(db, filename=None):
    """Export current aircraft data to CSV."""
    if filename is None:
        filename = f"adsb_export_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    active = get_active_columns()
    if not active: return None
    with db._lock:
        ac_snapshot = dict(db.ac)
    with open(filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([col["label"] for _, col in active])
        for icao in sorted(ac_snapshot, key=lambda ic: db._sort_key_safe(ic, ac_snapshot)):
            a = ac_snapshot[icao]
            if FILTER_RADIUS_KM > 0 and not in_range(a): continue
            if FILTER_EMERGENCY_ONLY:
                if a.get("squawk","") not in SQUAWK_SPECIAL and a.get("emergency","No emergency") == "No emergency": continue
            info = AIRCRAFT_INFO.get(icao, {})
            hazard_parts = []
            if a.get("turbulence") and a.get("turbulence") != "None": hazard_parts.append(f"turb:{a['turbulence']}")
            if a.get("icing") and a.get("icing") != "None": hazard_parts.append(f"icing:{a['icing']}")
            if a.get("windshear") and a.get("windshear") != "None": hazard_parts.append(f"wshear:{a['windshear']}")
            row = []
            for k, col in active:
                if k == "icao": row.append(icao)
                elif k == "callsign": row.append(a.get("callsign",""))
                elif k == "reg": row.append(info.get("reg",""))
                elif k == "type": row.append(info.get("type",""))
                elif k == "model": row.append(info.get("model",""))
                elif k == "manufacturer": row.append(info.get("manufacturer",""))
                elif k == "operator": row.append(info.get("operator",""))
                elif k == "owner": row.append(info.get("owner",""))
                elif k == "country": row.append(info.get("country",""))
                elif k == "engines": row.append(info.get("engines",""))
                elif k == "altitude": row.append(a.get("altitude"))
                elif k == "speed": row.append(get_display_speed(a))
                elif k == "heading": row.append(a.get("heading"))
                elif k == "vrate": row.append(get_display_vrate(a))
                elif k == "lat": row.append(a.get("lat_dr", a.get("lat")))
                elif k == "lon": row.append(a.get("lon_dr", a.get("lon")))
                elif k == "distance": row.append(a.get("distance_dr", a.get("distance")))
                elif k == "squawk": row.append(a.get("squawk"))
                elif k == "emergency": row.append(a.get("emergency"))
                elif k == "category": row.append(a.get("category"))
                elif k == "tis_b": row.append("T" if a.get("tis_b") else "")
                elif k == "rssi": row.append(a.get("rssi"))
                elif k == "reliable": row.append(a.get("pos_reliable", 0))
                elif k == "nac": row.append(a.get("nac_p"))
                elif k == "sil": row.append(a.get("sil"))
                elif k == "nacv": row.append(a.get("nacv"))
                elif k == "version": row.append(a.get("adsb_version"))
                elif k == "nic": row.append(a.get("nic"))
                elif k == "weather":
                    row.append(f"{a.get('wind_dir',0):.0f}/{a.get('wind_spd',0)}kt" if a.get("wind_dir") is not None else "")
                elif k == "hazard": row.append(" ".join(hazard_parts))
                else: row.append(a.get(k))
            writer.writerow(row)
    return filename

def export_aeroxml(db, filename=None):
    """Export current aircraft to AeroXML format."""
    if filename is None:
        filename = f"adsb_export_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xml"
    with db._lock:
        ac_snapshot = dict(db.ac)
    with open(filename, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<ADSBSnapshot xmlns="http://www.adsbexchange.com/AeroXML">\n')
        f.write(f'  <Timestamp>{datetime.datetime.now().isoformat()}</Timestamp>\n')
        f.write(f'  <Source>{HOME_LAT:.5f},{HOME_LON:.5f}</Source>\n')
        f.write('  <AircraftList>\n')
        for icao in sorted(ac_snapshot, key=lambda ic: db._sort_key_safe(ic, ac_snapshot)):
            a = ac_snapshot[icao]
            if FILTER_RADIUS_KM > 0 and not in_range(a): continue
            info = AIRCRAFT_INFO.get(icao, {})
            lat = a.get("lat_dr", a.get("lat")); lon = a.get("lon_dr", a.get("lon"))
            f.write('    <Aircraft>\n')
            f.write(f'      <Icao>{icao}</Icao>\n')
            if a.get("callsign"): f.write(f'      <Flight>{a["callsign"]}</Flight>\n')
            if info.get("reg"): f.write(f'      <Registration>{info["reg"]}</Registration>\n')
            if info.get("type"): f.write(f'      <AircraftType>{info["type"]}</AircraftType>\n')
            if lat is not None: f.write(f'      <Latitude>{lat:.6f}</Latitude>\n')
            if lon is not None: f.write(f'      <Longitude>{lon:.6f}</Longitude>\n')
            if a.get("altitude") is not None: f.write(f'      <Altitude units="feet">{a["altitude"]}</Altitude>\n')
            spd = get_display_speed(a)
            if spd is not None: f.write(f'      <GroundSpeed units="knots">{spd}</GroundSpeed>\n')
            if a.get("heading") is not None: f.write(f'      <Heading>{a["heading"]:.1f}</Heading>\n')
            vr = get_display_vrate(a)
            if vr is not None: f.write(f'      <VerticalRate units="fpm">{vr}</VerticalRate>\n')
            if a.get("squawk"): f.write(f'      <Squawk>{a["squawk"]}</Squawk>\n')
            if a.get("emergency","No emergency") != "No emergency": f.write(f'      <Emergency>{a["emergency"]}</Emergency>\n')
            if a.get("category"): f.write(f'      <Category>{a["category"]}</Category>\n')
            if a.get("rssi") is not None: f.write(f'      <RSSI>{a["rssi"]}</RSSI>\n')
            dist = a.get("distance_dr", a.get("distance"))
            if dist is not None: f.write(f'      <Distance units="km">{dist:.1f}</Distance>\n')
            if a.get("wind_dir") is not None:
                f.write(f'      <Wind direction="{a["wind_dir"]:.0f}" speed="{a.get("wind_spd",0)}" />\n')
            if a.get("turbulence") and a.get("turbulence") != "None":
                f.write(f'      <Turbulence>{a["turbulence"]}</Turbulence>\n')
            f.write('    </Aircraft>\n')
        f.write('  </AircraftList>\n')
        f.write(f'  <Stats totalAircraft="{db.stats["total_aircraft"]}" '
                f'maxRangeKm="{db.stats.get("max_range_km",0):.1f}" '
                f'maxSpeedKt="{db.stats.get("max_speed_kt",0)}" '
                f'maxAltFt="{db.stats.get("max_alt_ft",0)}" />\n')
        f.write('</ADSBSnapshot>\n')
    return filename

def export_flightaware(db, filename=None):
    """Export aircraft tracks in FlightAware format (CSV with extended fields)."""
    if filename is None:
        filename = f"flightaware_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with db._lock:
        ac_snapshot = dict(db.ac)
    with open(filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(["hex","flight","registration","aircraft_type","latitude","longitude",
                         "altitude","ground_speed","heading","vertical_rate","squawk",
                         "emergency","category","rssi","distance_km","seen",
                         "nac_p","sil","nic","version","wind_dir","wind_spd","temp",
                         "turbulence","icing","windshear","selected_altitude","baro_setting"])
        now = time.time()
        for icao in sorted(ac_snapshot, key=lambda ic: db._sort_key_safe(ic, ac_snapshot)):
            a = ac_snapshot[icao]
            if FILTER_RADIUS_KM > 0 and not in_range(a): continue
            info = AIRCRAFT_INFO.get(icao, {})
            lat = a.get("lat_dr", a.get("lat")); lon = a.get("lon_dr", a.get("lon"))
            writer.writerow([
                icao, a.get("callsign",""), info.get("reg",""), info.get("type",""),
                f"{lat:.6f}" if lat is not None else "",
                f"{lon:.6f}" if lon is not None else "",
                a.get("altitude",""), get_display_speed(a) or "",
                f"{a['heading']:.1f}" if a.get("heading") is not None else "",
                get_display_vrate(a) or "", a.get("squawk",""),
                a.get("emergency",""), a.get("category",""),
                a.get("rssi",""), a.get("distance_dr", a.get("distance","")),
                int(now - a.get("last_seen", now)),
                a.get("nac_p",""), a.get("sil",""), a.get("nic",""), a.get("adsb_version",""),
                f"{a['wind_dir']:.0f}" if a.get("wind_dir") is not None else "",
                a.get("wind_spd",""), a.get("temp",""),
                a.get("turbulence",""), a.get("icing",""), a.get("windshear",""),
                a.get("selected_altitude",""), a.get("baro_setting",""),
            ])
    return filename

# ==================================================================
# CLI ARGUMENTS AND FLAGS
# ==================================================================

def build_argparser():
    """Build argparse with grouped --help."""
    import argparse
    ap = argparse.ArgumentParser(
        prog="dx1090.py",
        description="ADS-B + Mode S + Mode A/C + UAT decoder\n"
                    "Support: RTL-SDR, Airspy, HackRF, BladeRF\n"
                    "Decoding: DF 0-25, all TC, BDS 1,0-6,0, UAT 978\n"
                    "Network protocols: Beast, SBS1, JSON",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Runtime controls:\n"
            "  Ctrl+C        — exit\n"
            "  SIGUSR1       — reload configuration (Linux/macOS)\n"
            "\n"
            "Web dashboard:  http://127.0.0.1:8080\n"
            "Config:       config.json (all parameters) + dashconf.json (dashboard units)\n"
            "\n"
            "All flags override values from config.json and dashconf.json.\n"
            "Flags do not require values — just toggle on/off."
        ),
        add_help=True,
    )

    g_sdr = ap.add_argument_group("SDR / Receiver")
    g_sdr.add_argument("--airspy", action="store_true", help="Use Airspy R2/Mini")
    g_sdr.add_argument("--hackrf", action="store_true", help="Use HackRF One")
    g_sdr.add_argument("--bladerf", action="store_true", help="Use bladeRF 2.0 micro")
    g_sdr.add_argument("--mode-ac", action="store_true", help="Enable Mode A/C (classic secondary radar)")
    g_sdr.add_argument("--uat", action="store_true", help="Enable UAT 978 MHz (USA, Australia)")
    g_sdr.add_argument("--adaptive-gain", action="store_true", help="Enable adaptive gain adjustment")
    g_sdr.add_argument("--no-watchdog", action="store_true", help="Disable SDR auto-restart on crash")

    g_dec = ap.add_argument_group("Decoding")
    g_dec.add_argument("--no-crc-correction", action="store_true", help="Disable 1-bit CRC correction")
    g_dec.add_argument("--no-crc-two-bit", action="store_true", help="Disable 2-bit CRC correction")
    g_dec.add_argument("--no-phase-correction", action="store_true", help="Disable bit position phase correction")
    g_dec.add_argument("--no-dead-reckoning", action="store_true", help="Disable position extrapolation (DR)")
    g_dec.add_argument("--no-smoothing", action="store_true", help="Disable position smoothing")

    g_ppm = ap.add_argument_group("PPM")
    g_ppm.add_argument("--no-ppm-auto", action="store_true", help="Disable auto-PPM from aircraft positions")

    g_net = ap.add_argument_group("Network")
    g_net.add_argument("--beast", action="store_true", help="Enable Beast output (dump1090 / piaware)")
    g_net.add_argument("--sbs1", action="store_true", help="Enable SBS1 output (BaseStation / VRS)")

    g_ses = ap.add_argument_group("Sessions and history")
    g_ses.add_argument("--no-session-save", action="store_true", help="Do not save session on exit")
    g_ses.add_argument("--no-session-restore", action="store_true", help="Do not restore session on launch")
    g_ses.add_argument("--no-stats-history", action="store_true", help="Disable statistics history saving")

    g_ui = ap.add_argument_group("UI / Sound")
    g_ui.add_argument("--no-sound", action="store_true", help="Fully disable sound notifications")
    g_ui.add_argument("--no-event-log", action="store_true", help="Disable event log on screen")
    g_ui.add_argument("--emergency-only", action="store_true", help="Show emergency aircraft only (7500/7600/7700)")
    g_ui.add_argument("--quiet", action="store_true", help="Minimal output (errors and stats only)")
    g_ui.add_argument("--debug", action="store_true", help="Debug output to stderr")

    g_sys = ap.add_argument_group("System")
    g_sys.add_argument("--config", metavar="FILE", default="config.json", help="Path to config file (default: config.json)")
    g_sys.add_argument("--version", action="version", version=f"ADS-B decoder v{__version__}")

    return ap


def apply_cli_flags(args):
    """Apply CLI flags to global variables (override config.json)."""
    g = globals()

    # SDR / Receiver
    if args.airspy:  g["SDR_TYPE"] = "airspy"
    if args.hackrf:  g["SDR_TYPE"] = "hackrf"
    if args.bladerf: g["SDR_TYPE"] = "bladerf"
    if args.mode_ac:   g["ENABLE_MODE_AC"] = True
    if args.uat:       g["ENABLE_UAT"] = True
    if args.adaptive_gain: g["ADAPTIVE_GAIN"] = True
    if args.no_watchdog:  g["ENABLE_SDR_WATCHDOG"] = False

    # Decoding
    if args.no_crc_correction:    g["ENABLE_CRC_CORRECTION"] = False
    if args.no_crc_two_bit:       g["ENABLE_CRC_TWO_BIT"] = False
    if args.no_phase_correction:  g["ENABLE_PHASE_CORRECTION"] = False
    if args.no_dead_reckoning:    g["ENABLE_DEAD_RECKONING"] = False
    if args.no_smoothing:         g["ENABLE_SMOOTHING"] = False

    # PPM
    if args.no_ppm_auto: g["ENABLE_PPM_AUTO"] = False

    # Network
    if args.beast: g["ENABLE_BEAST_OUT"] = True
    if args.sbs1:  g["ENABLE_SBS1_OUT"] = True

    # Sessions and history
    if args.no_session_save:    g["ENABLE_SESSION_SAVE"] = False
    if args.no_session_restore: g["ENABLE_SESSION_RESTORE"] = False
    if args.no_stats_history:   g["ENABLE_STATS_HISTORY"] = False

    # UI / Sound
    if args.no_sound:        g["ENABLE_SOUND"] = False
    if args.no_event_log:    g["ENABLE_EVENT_LOG"] = False
    if args.emergency_only:  g["FILTER_EMERGENCY_ONLY"] = True
    if args.quiet:           g["QUIET_MODE"] = True
    if args.debug:           g["DEBUG"] = True


# ==================================================================
# SDR MANAGEMENT
# ==================================================================

def _kill_proc(proc, timeout_terminate=5, timeout_kill=3):
    """Terminate process with timeout, fall back to SIGKILL."""
    try:
        proc.terminate()
        proc.wait(timeout=timeout_terminate)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=timeout_kill)
        except subprocess.TimeoutExpired:
            pass

def start_sdr(gain, ppm=None):
    """Start SDR receiver process."""
    ppm_val = ppm if ppm is not None else PPM_CORRECTION
    if SDR_TYPE == "rtl_sdr":
        cmd = ['rtl_sdr', '-f', str(FREQ), '-s', str(SAMPLE_RATE), '-g', str(gain), '-']
        if ppm_val != 0: cmd[3:3] = ['-p', str(ppm_val)]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    elif SDR_TYPE == "airspy":
        cmd = ['airspy_rx', '-f', str(FREQ / 1e6), '-a', str(SAMPLE_RATE),
               '-g', f'linearity:{gain // 10}', '-r', 'raw', '-']
        if ppm_val != 0: cmd[2:2] = ['-p', str(ppm_val)]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    elif SDR_TYPE == "hackrf":
        cmd = ['hackrf_transfer', '-r', '-', '-f', str(FREQ), '-s', str(SAMPLE_RATE),
               '-g', str(gain), '-a', '0', '-l', '1', '-w']
        if ppm_val != 0: cmd[2:2] = ['-C', str(ppm_val)]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    elif SDR_TYPE == "bladerf":
        cmd = ['bladerf-cli', '-s', 'auto', '-c', f'rx fpga {FREQ} {SAMPLE_RATE}',
               '-c', f'set gain {gain}', '-']
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    raise ValueError(f"Unknown SDR_TYPE: {SDR_TYPE}")
def restart_sdr(proc, gain, ppm=None):
    """Restart SDR process."""
    _kill_proc(proc)
    new_proc = start_sdr(gain, ppm); time.sleep(0.3)
    if new_proc.poll() is not None:
        sys.stderr.write(f"[sdr] {SDR_TYPE} error (gain={gain})\n")
        new_proc = start_sdr(0, ppm); time.sleep(0.3)
    return new_proc

def convert_iq(raw):
    """Convert raw IQ data to complex numpy array."""
    if SDR_TYPE in ("rtl_sdr", "hackrf"):
        arr = np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 127.5
        return arr[0::2] + 1j * arr[1::2]
    elif SDR_TYPE in ("airspy", "bladerf"):
        arr = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
        if SDR_TYPE == "airspy": arr = (arr - 2048.0) / 2048.0
        else: arr /= 32768.0
        return arr[0::2] + 1j * arr[1::2]
    arr = np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 127.5
    return arr[0::2] + 1j * arr[1::2]

def adjust_gain(db, current_gain):
    """Adaptive gain adjustment based on message rate."""
    if not ADAPTIVE_GAIN: return current_gain
    rate = db.get_msg_rate()
    idx = _find_gain_index(current_gain); old_idx = idx
    if rate < GAIN_MIN_MSG_RATE and idx < len(GAIN_STEPS) - 1: idx += 1
    elif rate > GAIN_GOOD_MSG_RATE and idx > 0: idx -= 1
    if idx != old_idx:
        new_gain = GAIN_STEPS[idx]
        db.new_events.append(f"  {c('[GAIN]', Color.YELLOW)} {rate:.1f} msg/s -> gain {current_gain/10:.1f} -> {new_gain/10:.1f} dB")
        return new_gain
    return current_gain

# ==================================================================
# UAT 978 MHz DECODER
# ==================================================================

UAT_SYNC_WORD = 0xEACDDA4E2
UAT_SHORT_LEN = 96
UAT_LONG_LEN = 288

def _uat_crc32(bits, msg_len=None):
    """Compute CRC-32 for UAT messages (polynomial 0x04C11DB7)."""
    if msg_len is None: msg_len = len(bits)
    poly = 0x04C11DB7
    crc = 0
    for i in range(msg_len):
        if (crc >> 31) & 1 ^ bits[i]:
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
        else:
            crc = (crc << 1) & 0xFFFFFFFF
    for _ in range(32):
        if (crc >> 31) & 1:
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
        else:
            crc = (crc << 1) & 0xFFFFFFFF
    return crc

def find_uat_preambles(mag, noise_floor=0.0):
    """Search for UAT preambles in amplitude array."""
    min_len = int(UAT_SHORT_LEN * SAMPLES_PER_BIT) + 64
    n = len(mag) - min_len
    if n <= 0: return []
    threshold = max(noise_floor * 2.0, 0.001) if noise_floor > 0 else 0.4
    win = int(8 * SAMPLES_PER_BIT)
    high_mask = mag[0:n] > threshold
    for p in range(1, win):
        high_mask &= mag[p:p + n] > threshold
    idx = np.where(high_mask)[0]
    if len(idx) > 1:
        keep = np.concatenate(([True], np.diff(idx) > int(40 * SAMPLES_PER_BIT)))
        idx = idx[keep]
    return idx

def decode_uat_message(mag, start, db):
    """Decode UAT message (TC 2/3)."""
    spb = SAMPLES_PER_BIT
    total_bits = UAT_LONG_LEN + 36
    end = start + int(total_bits * spb) + 8
    if end > len(mag):
        total_bits = UAT_SHORT_LEN + 36
        end = start + int(total_bits * spb) + 8
        if end > len(mag): return None
    bits = []
    for i in range(total_bits):
        pos = start + int(i * spb)
        pos_prev = start + int((i - 1) * spb) if i > 0 else pos
        if pos >= len(mag) or pos_prev >= len(mag): bits.append(0); continue
        bits.append(1 if float(mag[pos]) - float(mag[pos_prev]) > 0 else 0)
    sync = bits_to_int(bits[:36])
    if sync != UAT_SYNC_WORD: return None
    msg_bits = bits[36:]
    msg_len_for_crc = UAT_SHORT_LEN if len(msg_bits) < UAT_LONG_LEN else UAT_LONG_LEN
    if len(msg_bits) < msg_len_for_crc: return None
    crc_rem = _uat_crc32(msg_bits[:msg_len_for_crc])
    if crc_rem != 0: return None
    if len(msg_bits) < UAT_SHORT_LEN: return None
    is_long = len(msg_bits) >= UAT_LONG_LEN
    msg_len = UAT_LONG_LEN if is_long else UAT_SHORT_LEN
    tc = bits_to_int(msg_bits[0:8])
    if tc == 0:
        result = {"msg_type": tc, "is_long": is_long, "uat_type": "heartbeat"}
        if msg_len >= 12: result["utc_offset"] = bits_to_int(msg_bits[8:12])
        if msg_len >= 28: result["slots"] = bits_to_int(msg_bits[12:28])
        if msg_len >= 32: result["gs_count"] = bits_to_int(msg_bits[28:32])
        db.stats["uat_count"] += 1
        return result
    elif tc == 1:
        result = {"msg_type": tc, "is_long": is_long, "uat_type": "ground_station"}
        if msg_len >= 24: result["gs_id"] = bits_to_int(msg_bits[8:24])
        if msg_len >= 40:
            gl = bits_to_int(msg_bits[24:40])
            result["gs_lat"] = gl * 180.0 / (1<<15) - 90.0 if gl else None
        if msg_len >= 56:
            gl = bits_to_int(msg_bits[40:56])
            result["gs_lon"] = gl * 360.0 / (1<<15) - 180.0 if gl else None
        if msg_len >= 57: result["gs_operational"] = bool(msg_bits[56])
        db.stats["uat_count"] += 1
        return result
    result = {"msg_type": tc, "is_long": is_long}; icao = None
    if tc == 2:
        if msg_len < 32: return None
        addr = bits_to_int(msg_bits[8:32]); icao = f"U{addr:06X}"
        result["icao"] = icao; result["callsign"] = f"UAT-{addr:06X}"
        if msg_len >= 80:
            lat_raw = bits_to_int(msg_bits[32:56]); lon_raw = bits_to_int(msg_bits[56:80])
            if lat_raw == 0 and lon_raw == 0: result["lat"] = result["lon"] = None
            else:
                result["lat"] = lat_raw * 180.0 / (1 << 23) - 90.0
                result["lon"] = lon_raw * 360.0 / (1 << 23) - 180.0
        if msg_len >= 92:
            alt_raw = bits_to_int(msg_bits[80:92])
            if alt_raw > 0: result["altitude"] = (alt_raw - 1) * 25 - 1000
    elif tc == 3:
        if msg_len < 32: return None
        addr = bits_to_int(msg_bits[8:32]); icao = f"U{addr:06X}"
        result["icao"] = icao; result["callsign"] = f"UAT-{addr:06X}"
        if msg_len >= 80:
            lat_raw = bits_to_int(msg_bits[32:56]); lon_raw = bits_to_int(msg_bits[56:80])
            if lat_raw != 0 or lon_raw != 0:
                result["lat"] = lat_raw * 180.0 / (1 << 23) - 90.0
                result["lon"] = lon_raw * 360.0 / (1 << 23) - 180.0
        if msg_len >= 92:
            alt_raw = bits_to_int(msg_bits[80:92])
            if alt_raw > 0: result["altitude"] = (alt_raw - 1) * 25 - 1000
        if msg_len >= 104:
            spd_raw = bits_to_int(msg_bits[92:102])
            if spd_raw > 0: result["speed_gs"] = spd_raw
            hdg_raw = bits_to_int(msg_bits[102:112])
            if hdg_raw > 0: result["heading"] = hdg_raw * 360.0 / 1024.0
        if msg_len >= 122:
            vr = bits_to_int(msg_bits[112:122])
            if vr > 0:
                v = (vr - 1) * 64
                if bits_to_int(msg_bits[111:112]): v = -v
                if abs(v) <= 6000: result["vrate"] = v
        if msg_len >= 125:
            result["nic_uat"] = NIC_RADIUS.get(bits_to_int(msg_bits[122:125]), "?")
        if msg_len >= 128:
            nacp = bits_to_int(msg_bits[125:128])
            result["nacp_uat"] = {0:">10 NM",1:"<4 NM",2:"<2 NM",3:"<1 NM",4:"<0.5 NM",
                5:"<0.3 NM",6:"<0.1 NM",7:"<0.05 NM",8:"<30 m",9:"<10 m",
                10:"<3 m",11:"<1 m"}.get(nacp, f"?{nacp}")
        if msg_len >= 130:
            result["sil_uat"] = SIL_TABLE.get(bits_to_int(msg_bits[128:130]), "?")
        if msg_len >= 137:
            em = bits_to_int(msg_bits[130:137])
            result["emitter_type"] = {0:"unspecified",1:"light",2:"medium",3:"heavy",
                4:"high-vortex",5:"fighter",6:"helicopter",7:"glider",
                8:"ultralight",9:"UAV",10:"space",11:"ground vehicle"}.get(em, f"?{em}")
    if icao is None: return None
    db.stats["uat_count"] += 1
    update_kw = {"callsign": result.get("callsign", "")}
    if result.get("lat") is not None and result.get("lon") is not None:
        if hasattr(db, '_validate_and_update') and db._validate_and_update(icao, result["lat"], result["lon"], ground=False):
            pass  # position passed validation and saved via _validate_and_update
        else:
            update_kw.pop("lat", None); update_kw.pop("lon", None); update_kw.pop("distance", None)
            update_kw.pop("pos_time", None); update_kw.pop("pos_reliable", None)
    if result.get("altitude") is not None: update_kw["altitude"] = result["altitude"]
    if result.get("speed_gs") is not None: update_kw["speed_gs"] = result["speed_gs"]
    if result.get("heading") is not None: update_kw["heading"] = result["heading"]
    db.update(icao, **update_kw)
    db.check_alert(icao, result.get("callsign", ""))
    parts = [f"[UAT] TC={tc}"]
    if result.get("lat") is not None: parts.append(f"{result['lat']:.5f},{result['lon']:.5f}")
    if result.get("altitude") is not None: parts.append(fmt_alt(result["altitude"]))
    if result.get("speed_gs") is not None: parts.append(fmt_speed(result["speed_gs"]))
    db.event(icao, " ".join(parts))
    return result

# ==================================================================
# FINAL REPORT
# ==================================================================

def final_report(db, msg_total, msg_valid, t_start):
    """Print final statistics on exit."""
    log.info("Session end: aircraft=%d valid=%d/%d decode=%.0f%% recv=%.0f%%",
             db.stats.get("total_aircraft", 0), msg_valid, msg_total,
             db.decode_quality_ema, db.recv_quality_ema)
    print(f"\n{'━' * 50}")
    print(f"Session: {time.time()-t_start:.0f}s | Valid: {msg_valid} of {msg_total}")
    print(f"Aircraft: {db.stats['total_aircraft']}")
    print(f"Quality: decode={db.decode_quality_ema:.0f}% | receive={db.recv_quality_ema:.0f}%")
    if db.ppm_adjusted != PPM_CORRECTION:
        print(f"PPM: {PPM_CORRECTION:+d} -> {db.ppm_adjusted:+d} (auto)")
    if db.stats.get("crc_corrected", 0) > 0:
        one_bit = db.stats['crc_corrected'] - db.stats.get('crc_two_bit_corrected', 0)
        two_bit = db.stats.get('crc_two_bit_corrected', 0)
        print(f"CRC corrected: {db.stats['crc_corrected']} (1-bit: {one_bit}, 2-bit: {two_bit})")
    if db.stats["bds_attempts"] > 0:
        bds_rate = db.stats["bds_success"] / db.stats["bds_attempts"] * 100
        print(f"BDS: {db.stats['bds_success']}/{db.stats['bds_attempts']} ({bds_rate:.0f}%) | Unrecognized: {db.stats['bds_fail']}")
    if db.stats.get("mode_ac_count", 0) > 0:
        print(f"Mode A/C: {db.stats['mode_ac_count']} | Emerg.sqwk: {db.stats.get('mode_ac_emergency', 0)}")
    if db.stats.get("queue_drops", 0) > 0:
        print(f"Queue: lost {db.stats['queue_drops']} messages")
    if db.stats.get("uat_count", 0) > 0:
        print(f"UAT 978: {db.stats['uat_count']} messages")
    if db.stats.get("max_range_km", 0) > 0:
        print(f"Max range: {fmt_distance(db.stats['max_range_km'])}")
    if db.stats.get("max_speed_kt", 0) > 0:
        print(f"Max speed: {fmt_speed(db.stats['max_speed_kt'])}")
    if db.stats.get("max_alt_ft", 0) > 0:
        print(f"Max altitude: {fmt_alt(db.stats['max_alt_ft'])}")
    parts = []
    for k, label in [("nic_decoded","NIC"),("sil_decoded","SIL"),("nacv_decoded","NACv"),("version_decoded","Ver")]:
        if db.stats.get(k, 0) > 0: parts.append(f"{label}={db.stats[k]}")
    if parts: print(f"Integrity: {' | '.join(parts)}")
    if db.stats.get("bds45_count", 0) > 0:
        print(f"BDS 4,5 (weather): {db.stats['bds45_count']}")
    rej = db.stats
    if rej["pos_jumps_rejected"] or rej["alt_rejected"] or rej.get("vel_rejected",0) or rej.get("hdg_rejected",0) or rej.get("bds_rejected",0):
        print(f"Filters: pos={rej['pos_jumps_rejected']} alt={rej['alt_rejected']} spd={rej.get('vel_rejected',0)} hdg={rej.get('hdg_rejected',0)} BDS={rej.get('bds_rejected',0)}")
    ext_parts = []
    for k, label in [("long_range_count","long range"),("overhead_count","overhead"),
                     ("rapid_descent_count","descents"),("squawk_change_count","squawk changes"),
                     ("return_count","returns"),("level_off_count","level-offs"),
                     ("proximity_count","proximity"),("headon_count","head-on")]:
        if db.stats.get(k, 0) > 0: ext_parts.append(f"{label}={db.stats[k]}")
    if ext_parts: print(f"Events: {' | '.join(ext_parts)}")
    if db.stats.get("preambles_total", 0) > 0:
        strong_pct = db.stats.get("preambles_strong", 0) / db.stats["preambles_total"] * 100
        print(f"Preambles: total={db.stats['preambles_total']} strong={strong_pct:.0f}%")
    print(f"Logs: {LOG_DIR}/ | Sounds: {SOUND_DIR}/ | Session: {SESSION_FILE}")
    print(f"{'━' * 50}")

# ==================================================================
# MAIN LOOP
# ==================================================================



def load_config(config_path="config.json"):
    """Load settings from config.json. Overrides values in code."""
    g = globals()
    _int_keys = {
        "gain", "ppm_correction", "freq", "sample_rate", "chunk_bytes",
        "min_signal_level", "uat_freq", "uat_sample_rate", "uat_gain", "uat_ppm",
        "json_out_port", "beast_out_port", "sbs1_out_port",
        "gain_check_interval", "gain_min_msg_rate", "gain_good_msg_rate",
        "position_reliable_min", "position_reliable_max",
        "filter_radius", "max_alt_rate_fpm",
        "stale_timeout", "loss_timeout", "cpr_timeout",
        "trail_max_points", "decode_queue_size",
        "mlat_timestamp_precision_us", "session_max_age",
        "sdr_watchdog_timeout", "sdr_watchdog_max_retries",
        "stats_history_interval", "stats_history_retention_days",
        "long_range_threshold_km", "overhead_threshold_km",
        "speed_record_threshold", "proximity_horiz_km",
        "headon_course_diff", "proximity_alt_diff_ft", "proximity_vert_ft",
        "low_alt_threshold_ft", "rapid_descent_fpm",
        "rapid_descent_time_sec", "squawk_change_alert",
        "return_time_sec", "level_off_alt_change_ft",
        "ppm_auto_interval", "ppm_auto_min_samples",
        "crc_two_bit_range", "crc_two_bit_max_rate",
        "headon_dist_km", "alt_record_threshold", "level_off_fpm",
        "level_off_min_vrate", "return_memory_time", "silence_timeout",
        "ppm_auto_min_valid", "ppm_auto_max_adjust",
        "ppm_auto_max_pos_age",
    }
    _float_keys = {
        "home_lat", "home_lon", "noise_floor_alpha",
        "pulse_blanking_ratio", "max_position_jump_km",
        "max_range_km",
        "smoothing_alpha", "trail_min_interval", "dr_max_age", "ppm_auto_tolerance_km",
        "ppm_auto_smoothing", "crc_two_bit_time_ratio",
        "crc_two_bit_max_decode_time", "crc_two_bit_time_alpha",
    }
    _set_keys = {"mil_categories", "rare_types"}
    _list_keys = {
        "phase_offsets", "tracked_callsigns", "tracked_icao",
        "col_show", "gain_steps",
    }
    _bool_keys = {
        "enable_sound", "enable_uat", "enable_mode_s", "enable_mode_ac",
        "adaptive_gain", "enable_phase_correction", "enable_rssi",
        "enable_dc_removal", "enable_noise_floor", "enable_pulse_blanking",
        "enable_mlat_timestamps", "enable_smoothing", "enable_dead_reckoning",
        "enable_bds10", "enable_bds17", "enable_bds20", "enable_bds30",
        "enable_bds40", "enable_bds44", "enable_bds45", "enable_bds50", "enable_bds60",
        "enable_bds51", "enable_bds52", "enable_bds53", "enable_bds61",
        "enable_nic", "enable_sil", "enable_nacv", "enable_adsb_version",
        "enable_crc_correction", "enable_crc_two_bit",
        "crc_two_bit_adaptive", "enable_df18", "enable_df19",
        "enable_event_log", "enable_session_save", "enable_session_restore",
        "session_save_stats", "session_save_ppm",
        "enable_csv_export", "enable_aeroxml_export", "enable_flightaware_export",
        "filter_emergency_only", "filter_remove_out_of_range",
        "enable_aircraft_lookup", "lookup_registration", "lookup_typecode",
        "lookup_operator", "lookup_model", "lookup_manufacturer",
        "lookup_owner", "lookup_country", "lookup_engines",
        "enable_decode_thread", "enable_beast_out", "enable_json_out",
        "enable_sbs1_out", "enable_sdr_watchdog", "enable_ppm_auto",
        "enable_doppler",
        "enable_stats_history", "enable_sound_new", "enable_sound_emergency",
        "enable_sound_loss", "enable_sound_tcas", "enable_sound_low_alt",
        "enable_sound_mil", "enable_sound_tracked", "enable_sound_tracked_loss",
        "enable_sound_long_range", "enable_sound_overhead",
        "enable_sound_rapid_descent", "enable_sound_squawk_change",
        "enable_sound_return", "enable_sound_level_off",
        "enable_sound_proximity", "enable_sound_headon",
        "enable_sound_speed_record", "enable_sound_alt_record",
        "enable_sound_range_record", "enable_sound_stall",
        "enable_sound_fuel", "enable_sound_vrate_record",
        "enable_sound_bds45", "enable_sound_uat",
        "enable_sound_mlat", "enable_sound_df18",
        "enable_sound_version", "enable_sound_sil",
        "enable_sound_nacv", "enable_sound_nic",
        "enable_sound_record", "enable_sound_silence", "enable_sound_turb",
        "enable_sound_squawk", "enable_sound_helicopter", "enable_sound_rare_type",
        "enable_sound_ground_vehicle", "enable_sound_breakthrough",
        "ppm_auto_restart_sdr", "crc_two_bit_dynamic",
    }
    _str_keys = {
        "sdr_type", "filter_radius_unit", "filter_no_position",
        "unit_distance", "unit_alt", "unit_speed", "unit_vrate",
        "unit_pressure", "unit_temp", "speed_source", "vrate_source",
        "sort_by", "session_file", "aircraft_db_file",
        "beast_out_host", "json_out_host", "sbs1_out_host",
        "log_dir", "log_mode", "sound_dir",
        "unit_input_dist", "unit_input_alt", "unit_input_speed", "unit_input_vrate",
        "stats_history_dir",
    }
    _key_to_var = {
        "gain": "GAIN", "ppm_correction": "PPM_CORRECTION",
        "home_lat": "HOME_LAT", "home_lon": "HOME_LON",
        "sdr_type": "SDR_TYPE", "freq": "FREQ", "sample_rate": "SAMPLE_RATE",
        "chunk_bytes": "CHUNK_BYTES", "min_signal_level": "MIN_SIGNAL_LEVEL",
        "enable_uat": "ENABLE_UAT", "uat_freq": "UAT_FREQ",
        "uat_sample_rate": "UAT_SAMPLE_RATE", "uat_gain": "UAT_GAIN",
        "uat_ppm": "UAT_PPM",
        "enable_mode_s": "ENABLE_MODE_S", "enable_mode_ac": "ENABLE_MODE_AC",
        "adaptive_gain": "ADAPTIVE_GAIN",
        "gain_check_interval": "GAIN_CHECK_INTERVAL",
        "gain_min_msg_rate": "GAIN_MIN_MSG_RATE",
        "gain_good_msg_rate": "GAIN_GOOD_MSG_RATE",
        "enable_phase_correction": "ENABLE_PHASE_CORRECTION",
        "phase_offsets": "PHASE_OFFSETS",
        "enable_rssi": "ENABLE_RSSI",
        "position_reliable_min": "POSITION_RELIABLE_MIN",
        "position_reliable_max": "POSITION_RELIABLE_MAX",
        "filter_radius": "FILTER_RADIUS", "filter_radius_unit": "FILTER_RADIUS_UNIT",
        "filter_no_position": "FILTER_NO_POSITION",
        "filter_remove_out_of_range": "FILTER_REMOVE_OUT_OF_RANGE",
        "unit_distance": "UNIT_DISTANCE", "unit_alt": "UNIT_ALT",
        "unit_speed": "UNIT_SPEED", "unit_vrate": "UNIT_VRATE",
        "unit_pressure": "UNIT_PRESSURE", "unit_temp": "UNIT_TEMP",
        "map_mode": "MAP_MODE", "tile_cache_dir": "TILE_CACHE_DIR",
        "speed_source": "SPEED_SOURCE", "vrate_source": "VRATE_SOURCE",
        "enable_dc_removal": "ENABLE_DC_REMOVAL",
        "enable_noise_floor": "ENABLE_NOISE_FLOOR",
        "noise_floor_alpha": "NOISE_FLOOR_ALPHA",
        "enable_pulse_blanking": "ENABLE_PULSE_BLANKING",
        "pulse_blanking_ratio": "PULSE_BLANKING_RATIO",
        "enable_mlat_timestamps": "ENABLE_MLAT_TIMESTAMPS",
        "mlat_timestamp_precision_us": "MLAT_TIMESTAMP_PRECISION_US",
        "max_position_jump_km": "MAX_POSITION_JUMP_KM",
        "max_range_km": "MAX_RANGE_KM",
        "enable_smoothing": "ENABLE_SMOOTHING",
        "smoothing_alpha": "SMOOTHING_ALPHA",
        "enable_dead_reckoning": "ENABLE_DEAD_RECKONING",
        "dr_max_age": "DR_MAX_AGE", "max_alt_rate_fpm": "MAX_ALT_RATE_FPM",
        "trail_max_points": "TRAIL_MAX_POINTS",
        "trail_min_interval": "TRAIL_MIN_INTERVAL",
        "enable_bds10": "ENABLE_BDS10", "enable_bds17": "ENABLE_BDS17",
        "enable_bds20": "ENABLE_BDS20", "enable_bds30": "ENABLE_BDS30",
        "enable_bds40": "ENABLE_BDS40", "enable_bds44": "ENABLE_BDS44",
        "enable_bds45": "ENABLE_BDS45", "enable_bds50": "ENABLE_BDS50",
        "enable_bds60": "ENABLE_BDS60",
        "enable_nic": "ENABLE_NIC", "enable_sil": "ENABLE_SIL",
        "enable_nacv": "ENABLE_NACV", "enable_adsb_version": "ENABLE_ADSB_VERSION",
        "enable_crc_correction": "ENABLE_CRC_CORRECTION",
        "enable_crc_two_bit": "ENABLE_CRC_TWO_BIT",
        "crc_two_bit_range": "CRC_TWO_BIT_RANGE",
        "crc_two_bit_adaptive": "CRC_TWO_BIT_ADAPTIVE",
        "crc_two_bit_max_rate": "CRC_TWO_BIT_MAX_RATE",
        "enable_df18": "ENABLE_DF18", "enable_df19": "ENABLE_DF19",
        "log_dir": "LOG_DIR", "enable_event_log": "ENABLE_EVENT_LOG",
        "log_mode": "LOG_MODE", "sound_dir": "SOUND_DIR",
        "stale_timeout": "STALE_TIMEOUT", "loss_timeout": "LOSS_TIMEOUT",
        "cpr_timeout": "CPR_TIMEOUT",
        "enable_sound": "ENABLE_SOUND",
        "enable_sound_new": "ENABLE_SOUND_NEW",
        "enable_sound_emergency": "ENABLE_SOUND_EMERGENCY",
        "enable_sound_loss": "ENABLE_SOUND_LOSS",
        "enable_sound_tcas": "ENABLE_SOUND_TCAS",
        "enable_sound_low_alt": "ENABLE_SOUND_LOW_ALT",
        "enable_sound_mil": "ENABLE_SOUND_MIL",
        "enable_sound_tracked": "ENABLE_SOUND_TRACKED",
        "enable_sound_tracked_loss": "ENABLE_SOUND_TRACKED_LOSS",
        "enable_sound_long_range": "ENABLE_SOUND_LONG_RANGE",
        "enable_sound_overhead": "ENABLE_SOUND_OVERHEAD",
        "enable_sound_rapid_descent": "ENABLE_SOUND_RAPID_DESCENT",
        "enable_sound_squawk_change": "ENABLE_SOUND_SQUAWK_CHANGE",
        "enable_sound_return": "ENABLE_SOUND_RETURN",
        "enable_sound_level_off": "ENABLE_SOUND_LEVEL_OFF",
        "enable_sound_proximity": "ENABLE_SOUND_PROXIMITY",
        "enable_sound_headon": "ENABLE_SOUND_HEADON",
        "enable_sound_speed_record": "ENABLE_SOUND_SPEED_RECORD",
        "enable_sound_alt_record": "ENABLE_SOUND_ALT_RECORD",
        "enable_sound_range_record": "ENABLE_SOUND_RECORD",
        "enable_sound_stall": "ENABLE_SOUND_STALL",
        "enable_sound_fuel": "ENABLE_SOUND_FUEL",
        "enable_sound_vrate_record": "ENABLE_SOUND_VRATE_RECORD",
        "enable_sound_bds45": "ENABLE_SOUND_BDS45",
        "enable_sound_uat": "ENABLE_SOUND_UAT",
        "enable_sound_mlat": "ENABLE_SOUND_MLAT",
        "enable_sound_df18": "ENABLE_SOUND_DF18",
        "enable_sound_version": "ENABLE_SOUND_VERSION",
        "enable_sound_sil": "ENABLE_SOUND_SIL",
        "enable_sound_nacv": "ENABLE_SOUND_NACV",
        "enable_sound_nic": "ENABLE_SOUND_NIC",
        "long_range_threshold_km": "LONG_RANGE_THRESHOLD_KM",
        "overhead_threshold_km": "OVERHEAD_THRESHOLD_KM",
        "speed_record_threshold": "SPEED_RECORD_THRESHOLD",
        "proximity_horiz_km": "PROXIMITY_HORIZ_KM",
        "headon_course_diff": "HEADON_COURSE_DIFF",
        "proximity_alt_diff_ft": "PROXIMITY_ALT_DIFF_FT",
        "proximity_vert_ft": "PROXIMITY_VERT_FT",
        "low_alt_threshold_ft": "LOW_ALT_THRESHOLD_FT",
        "rapid_descent_fpm": "RAPID_DESCENT_FPM",
        "rapid_descent_time_sec": "RAPID_DESCENT_TIME_SEC",
        "squawk_change_alert": "SQUAWK_CHANGE_ALERT",
        "return_time_sec": "RETURN_TIME_SEC",
        "level_off_alt_change_ft": "LEVEL_OFF_ALT_CHANGE_FT",
        "tracked_callsigns": "TRACKED_CALLSIGNS",
        "tracked_icao": "TRACKED_ICAO",
        "mil_categories": "MIL_CATEGORIES",
        "rare_types": "RARE_TYPES",
        "enable_ppm_auto": "ENABLE_PPM_AUTO",
        "enable_doppler": "ENABLE_DOPPLER",
        "ppm_auto_interval": "PPM_AUTO_INTERVAL",
        "ppm_auto_min_samples": "PPM_AUTO_MIN_SAMPLES",
        "ppm_auto_tolerance_km": "PPM_AUTO_TOLERANCE_KM",
        "enable_session_save": "ENABLE_SESSION_SAVE",
        "enable_session_restore": "ENABLE_SESSION_RESTORE",
        "session_file": "SESSION_FILE",
        "session_max_age": "SESSION_MAX_AGE",
        "session_save_stats": "SESSION_SAVE_STATS",
        "session_save_ppm": "SESSION_SAVE_PPM",
        "enable_csv_export": "ENABLE_CSV_EXPORT",
        "enable_aeroxml_export": "ENABLE_AEROXML_EXPORT",
        "enable_flightaware_export": "ENABLE_FLIGHTAWARE_EXPORT",
        "sort_by": "SORT_BY", "filter_emergency_only": "FILTER_EMERGENCY_ONLY",
        "enable_aircraft_lookup": "ENABLE_AIRCRAFT_LOOKUP",
        "aircraft_db_file": "AIRCRAFT_DB_FILE",
        "lookup_registration": "LOOKUP_REGISTRATION",
        "lookup_typecode": "LOOKUP_TYPECODE",
        "lookup_operator": "LOOKUP_OPERATOR",
        "lookup_model": "LOOKUP_MODEL",
        "lookup_manufacturer": "LOOKUP_MANUFACTURER",
        "lookup_owner": "LOOKUP_OWNER",
        "lookup_country": "LOOKUP_COUNTRY",
        "lookup_engines": "LOOKUP_ENGINES",
        "enable_decode_thread": "ENABLE_DECODE_THREAD",
        "decode_queue_size": "DECODE_QUEUE_SIZE",
        "enable_beast_out": "ENABLE_BEAST_OUT",
        "beast_out_host": "BEAST_OUT_HOST",
        "beast_out_port": "BEAST_OUT_PORT",
        "enable_json_out": "ENABLE_JSON_OUT",
        "json_out_host": "JSON_OUT_HOST",
        "json_out_port": "JSON_OUT_PORT",
        "enable_sbs1_out": "ENABLE_SBS1_OUT",
        "sbs1_out_host": "SBS1_OUT_HOST",
        "sbs1_out_port": "SBS1_OUT_PORT",
        "enable_sdr_watchdog": "ENABLE_SDR_WATCHDOG",
        "sdr_watchdog_timeout": "SDR_WATCHDOG_TIMEOUT",
        "sdr_watchdog_max_retries": "SDR_WATCHDOG_MAX_RETRIES",
        "enable_stats_history": "ENABLE_STATS_HISTORY",
        "stats_history_dir": "STATS_HISTORY_DIR",
        "stats_history_interval": "STATS_HISTORY_INTERVAL",
        "stats_history_retention_days": "STATS_HISTORY_RETENTION_DAYS",
        "col_show": "COL_SHOW",
        "unit_input_dist": "UNIT_INPUT_DIST",
        "unit_input_alt": "UNIT_INPUT_ALT",
        "unit_input_speed": "UNIT_INPUT_SPEED",
        "unit_input_vrate": "UNIT_INPUT_VRATE",
        # --- Added by patch ---
        "enable_sound_record": "ENABLE_SOUND_RECORD",
        "enable_sound_silence": "ENABLE_SOUND_SILENCE",
        "enable_sound_turb": "ENABLE_SOUND_TURB",
        "enable_bds51": "ENABLE_BDS51",
        "enable_bds52": "ENABLE_BDS52",
        "enable_bds53": "ENABLE_BDS53",
        "enable_bds61": "ENABLE_BDS61",
        "enable_sound_squawk": "ENABLE_SOUND_SQUAWK",
        "enable_sound_helicopter": "ENABLE_SOUND_HELICOPTER",
        "enable_sound_rare_type": "ENABLE_SOUND_RARE_TYPE",
        "enable_sound_ground_vehicle": "ENABLE_SOUND_GROUND_VEHICLE",
        "enable_sound_breakthrough": "ENABLE_SOUND_BREAKTHROUGH",
        "headon_dist_km": "HEADON_DIST_KM",
        "alt_record_threshold": "ALT_RECORD_THRESHOLD",
        "level_off_fpm": "LEVEL_OFF_FPM",
        "level_off_min_vrate": "LEVEL_OFF_MIN_VRATE",
        "return_memory_time": "RETURN_MEMORY_TIME",
        "silence_timeout": "SILENCE_TIMEOUT",
        "ppm_auto_min_valid": "PPM_AUTO_MIN_VALID",
        "ppm_auto_max_adjust": "PPM_AUTO_MAX_ADJUST",
        "ppm_auto_smoothing": "PPM_AUTO_SMOOTHING",
        "ppm_auto_max_pos_age": "PPM_AUTO_MAX_POS_AGE",
        "ppm_auto_restart_sdr": "PPM_AUTO_RESTART_SDR",
        "crc_two_bit_dynamic": "CRC_TWO_BIT_DYNAMIC",
        "crc_two_bit_time_ratio": "CRC_TWO_BIT_TIME_RATIO",
        "crc_two_bit_max_decode_time": "CRC_TWO_BIT_MAX_DECODE_TIME",
        "crc_two_bit_time_alpha": "CRC_TWO_BIT_TIME_ALPHA",
        "gain_steps": "GAIN_STEPS",
    }
    if not os.path.exists(config_path):
        return
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = json_module.load(f)
        _loaded_keys = set()
        for k, v in cfg.items():
            if k not in _key_to_var:
                continue
            var = _key_to_var[k]
            if k in _bool_keys:
                g[var] = bool(v)
            elif k in _int_keys:
                g[var] = int(v)
            elif k in _float_keys:
                g[var] = float(v)
            elif k in _set_keys:
                g[var] = set(v) if isinstance(v, list) else v
            elif k in _list_keys:
                g[var] = list(v) if isinstance(v, list) else v
            else:
                g[var] = v
            _loaded_keys.add(k)
            print(f"  config: {k} = {v}")
        # Recompute derived units after config load
        if g.get("FILTER_RADIUS", 0) > 0:
            _fru = g.get("FILTER_RADIUS_UNIT", "km")
            if _fru == "mi":   g["FILTER_RADIUS_KM"] = g["FILTER_RADIUS"] / 0.621371
            elif _fru == "nm": g["FILTER_RADIUS_KM"] = g["FILTER_RADIUS"] / 0.539957
            else:              g["FILTER_RADIUS_KM"] = g["FILTER_RADIUS"]
        else:
            g["FILTER_RADIUS_KM"] = 0
        _uid = g.get("UNIT_INPUT_DIST", "km")
        def _rc_dist(v):
            if _uid == "mi": return v / 0.621371
            if _uid == "nm": return v / 0.539957
            return v
        if "max_range_km" in _loaded_keys:
            g["MAX_RANGE_KM"]            = _rc_dist(g.get("MAX_RANGE_KM", 400.0))
        if "max_position_jump_km" in _loaded_keys:
            g["MAX_POSITION_JUMP_KM"]    = _rc_dist(g.get("MAX_POSITION_JUMP_KM", 50.0))
        if "long_range_threshold_km" in _loaded_keys:
            g["LONG_RANGE_THRESHOLD_KM"] = _rc_dist(g.get("LONG_RANGE_THRESHOLD_KM", 300.0))
        if "overhead_threshold_km" in _loaded_keys:
            g["OVERHEAD_THRESHOLD_KM"]   = _rc_dist(g.get("OVERHEAD_THRESHOLD_KM", 5.0))
        if "proximity_horiz_km" in _loaded_keys:
            g["PROXIMITY_HORIZ_KM"]      = _rc_dist(g.get("PROXIMITY_HORIZ_KM", 10.0))
        if "headon_dist_km" in _loaded_keys:
            g["HEADON_DIST_KM"]          = _rc_dist(g.get("HEADON_DIST_KM", 30.0))
        if "ppm_auto_tolerance_km" in _loaded_keys and "PPM_AUTO_TOLERANCE_KM" in g:
            g["PPM_AUTO_TOLERANCE_KM"] = _rc_dist(g["PPM_AUTO_TOLERANCE_KM"])
        _uia = g.get("UNIT_INPUT_ALT", "ft")
        def _rc_alt(v):
            return v * 3.28084 if _uia == "m" else v
        if "low_alt_threshold_ft" in _loaded_keys:
            g["LOW_ALT_THRESHOLD_FT"] = _rc_alt(g.get("LOW_ALT_THRESHOLD_FT", 10000.0))
        if "proximity_vert_ft" in _loaded_keys:
            g["PROXIMITY_VERT_FT"] = _rc_alt(g.get("PROXIMITY_VERT_FT", 1000.0))
        _uis = g.get("UNIT_INPUT_SPEED", "kt")
        def _rc_spd(v):
            if _uis == "kmh": return v / 1.852
            if _uis == "mph": return v / 1.15078
            return v
        if "speed_record_threshold" in _loaded_keys:
            g["SPEED_RECORD_THRESHOLD_KT"] = _rc_spd(g.get("SPEED_RECORD_THRESHOLD", 500))
        _uiv = g.get("UNIT_INPUT_VRATE", "fpm")
        def _rc_vr(v):
            return v * 196.85 if _uiv == "ms" else v
        if "rapid_descent_fpm" in _loaded_keys:
            g["RAPID_DESCENT_FPM"] = _rc_vr(g.get("RAPID_DESCENT_FPM", 3000))
        if "max_alt_rate_fpm" in _loaded_keys:
            g["MAX_ALT_RATE_FPM"]  = _rc_vr(g.get("MAX_ALT_RATE_FPM", 10000))
        # --- Added by patch: additional thresholds ---
        if "alt_record_threshold" in _loaded_keys:
            g["ALT_RECORD_THRESHOLD"] = _rc_alt(g.get("ALT_RECORD_THRESHOLD", 40000))
        if "level_off_fpm" in _loaded_keys:
            g["LEVEL_OFF_FPM"] = _rc_vr(g.get("LEVEL_OFF_FPM", 200))
        if "level_off_min_vrate" in _loaded_keys:
            g["LEVEL_OFF_MIN_VRATE"] = _rc_vr(g.get("LEVEL_OFF_MIN_VRATE", 500))
        if "low_alt_threshold_ft" in _loaded_keys:
            _ua = g.get("UNIT_ALT", "m")
            if _ua == "m":
                g["LOW_ALT_THRESHOLD"] = g["LOW_ALT_THRESHOLD_FT"] * 0.3048
            else:
                g["LOW_ALT_THRESHOLD"] = g["LOW_ALT_THRESHOLD_FT"]
    except Exception as e:
        sys.stderr.write(f"[config] {e}\n")


def load_dashconf(dash_path="dashconf.json"):
    """Load dashboard settings from dashconf.json. Overrides config.json for units/map."""
    if not os.path.exists(dash_path):
        return
    try:
        with open(dash_path, 'r', encoding='utf-8') as f:
            cfg = json_module.load(f)
        g = globals()
        _dash_keys = {
            "unit_distance": "UNIT_DISTANCE",
            "unit_alt": "UNIT_ALT",
            "unit_speed": "UNIT_SPEED",
            "unit_vrate": "UNIT_VRATE",
            "unit_temp": "UNIT_TEMP",
            "map_mode": "MAP_MODE",
        }
        for k, var in _dash_keys.items():
            if k in cfg:
                g[var] = cfg[k]
                print(f"  dashconf: {k} = {cfg[k]}")
    except Exception as e:
        sys.stderr.write(f"[dashconf] {e}\n")


def main():
    """Main function — initialization and main receive loop."""
    global PPM_CORRECTION
    
    ap = build_argparser()
    args = ap.parse_args()
    
    # Load config file
    load_config(args.config)

    # Load dashboard settings (overrides config.json for units/map)
    load_dashconf()

    # CLI flags override config.json and dashconf.json
    apply_cli_flags(args)
    

    sdr_name = {"rtl_sdr": "RTL-SDR", "airspy": "Airspy",
                "hackrf": "HackRF", "bladerf": "BladeRF"}.get(SDR_TYPE, SDR_TYPE)

    print("━" * 60)
    print(f"  {c('ADS-B + Mode S + Mode A/C + UAT decoder by R4SCS', Color.BOLD)}")
    print(f"  Receiver: {sdr_name} | PPM: {PPM_CORRECTION:+d} | gain: {GAIN/10:.1f}dB")
    print(f"  {SAMPLE_RATE/1e6:.1f} MHz ({SAMPLES_PER_BIT:.1f} samp/bit) | {FREQ/1e6:.1f} MHz")
    modes = []
    if ENABLE_MODE_S: modes.append("Mode S")
    if ENABLE_MODE_AC: modes.append("Mode A/C")
    if ENABLE_UAT: modes.append("UAT")
    print(f"  Modes: {', '.join(modes)}")
    print(f"  Location: {HOME_LAT}°N, {HOME_LON}°E")
    if FILTER_RADIUS_KM > 0:
        print(f"  Radius: {fmt_radius(FILTER_RADIUS_KM)} ({FILTER_RADIUS_UNIT})")
    print(f"  Sound: {'ON (26 types)' if ENABLE_SOUND else 'OFF'} | Logs: {LOG_DIR}/ | Session: {SESSION_FILE}")
    if ENABLE_BEAST_OUT: print(f"  Beast → {BEAST_OUT_HOST}:{BEAST_OUT_PORT}")
    if ENABLE_SBS1_OUT: print(f"  SBS1 → {SBS1_OUT_HOST}:{SBS1_OUT_PORT}")
    if ENABLE_JSON_OUT: print(f"  Dashboard → http://{JSON_OUT_HOST}:{JSON_OUT_PORT}")
    
    if ENABLE_SDR_WATCHDOG: print(f"  Watchdog: ON (timeout {SDR_WATCHDOG_TIMEOUT}s)")
    print("━" * 60)
    sys.stdout.flush()

    player = find_player()
    if player and ENABLE_SOUND:
        print(f"Sound player: {player}")
    elif not player:
        print("Sound: aplay/paplay not found (sudo apt install alsa-utils)")

    db = create_db(player)
    threading.Thread(target=_heartbeat_checker, daemon=True).start()

    # Startup sound
    if ENABLE_SOUND and db.startup_sound:
        play_sound(db.player, db.startup_sound)
    if ENABLE_SESSION_RESTORE: db.restore_session()
    atexit.register(db.logger.close_all)
    atexit.register(db.save_session)
    init_network(db)

    worker = DecodeWorker(db) if ENABLE_DECODE_THREAD else None

    current_gain = GAIN
    proc = None
    sdr_retries = 0

    # Start SDR with watchdog
    while True:
        try:
            proc = start_sdr(current_gain)
            break
        except FileNotFoundError:
            sdr_cmd = {"rtl_sdr": "rtl_sdr", "airspy": "airspy_rx",
                       "hackrf": "hackrf_transfer", "bladerf": "bladerf-cli"}.get(SDR_TYPE, SDR_TYPE)
            print(c(f"\n{sdr_cmd} not found! Install driver for {sdr_name}.", Color.RED))
            sys.exit(1)
        except Exception as e:
            sdr_retries += 1
            if sdr_retries > SDR_WATCHDOG_MAX_RETRIES:
                print(c(f"SDR: retry limit exceeded ({SDR_WATCHDOG_MAX_RETRIES})", Color.RED))
                sys.exit(1)
            print(c(f"SDR: start error ({e}), retry {sdr_retries}/{SDR_WATCHDOG_MAX_RETRIES}...", Color.YELLOW))
            time.sleep(3)

    time.sleep(0.5)
    if proc.poll() is not None:
        print(c("SDR error:", Color.RED))
        print(proc.stderr.read().decode(errors='replace'))
        sys.exit(1)

    print(f"\nCapture: {FREQ/1e6:.1f} MHz, {SAMPLE_RATE/1e6:.1f} MHz, gain {current_gain/10:.1f} dB")
    print(f"Controls: Ctrl+C — exit  •  SIGUSR1 — reload config")
    print(f"{'━' * 50}")
    sys.stdout.flush()
    
    # Wait for Enter key before starting (except replay mode)
    try:
        input("  Press Enter to start (Ctrl+C — cancel)... ")
    except KeyboardInterrupt:
        print(c("\n  Cancelled.", Color.YELLOW))
        sys.exit(0)
    clear_screen()

    prev_tail = np.array([], dtype=np.complex64)
    msg_total = 0; msg_valid = 0
    t_start = time.time()
    last_gain_check = time.time()
    last_ppm_check = time.time()
    last_data_time = time.time()
    sdr_restart_count = 0

    try:
        while True:
            # Read data from SDR or replay
            # Non-blocking read with 1s timeout (watchdog-friendly)
            _ready, _, _ = select.select([proc.stdout], [], [], 1.0)
            if _ready:
                raw = proc.stdout.read(CHUNK_BYTES)
            else:
                raw = b''

            # Watchdog: check for data
            if (not raw or len(raw) < 4):
                if ENABLE_SDR_WATCHDOG:
                    if proc.poll() is not None or (time.time() - last_data_time > SDR_WATCHDOG_TIMEOUT):
                        sdr_restart_count += 1
                        if sdr_restart_count > SDR_WATCHDOG_MAX_RETRIES:
                            print(c(f"SDR Watchdog: retry limit exceeded ({SDR_WATCHDOG_MAX_RETRIES})", Color.RED))
                            break
                        print(c(f"SDR Watchdog: restart {sdr_restart_count}/{SDR_WATCHDOG_MAX_RETRIES}...", Color.YELLOW))
                        db.new_events.append(f"  {c('[SDR RESTART]', Color.YELLOW)} auto-restart #{sdr_restart_count}")
                        try:
                            proc = restart_sdr(proc, current_gain)
                            prev_tail = np.array([], dtype=np.complex64)
                            last_data_time = time.time()
                            time.sleep(1)
                        except Exception as e:
                            print(c(f"SDR Watchdog: restart error: {e}", Color.RED))
                            time.sleep(5)
                        continue
                    time.sleep(0.1)
                    continue
                else:
                    break

            last_data_time = time.time()
            if len(raw) % 2: raw = raw[:-1]

            # IQ conversion
            iq = convert_iq(raw)
            iq = np.concatenate([prev_tail, iq])
            if ENABLE_DC_REMOVAL: iq = iq - np.mean(iq)
            mag = np.abs(iq)

            # Noise floor estimation
            if ENABLE_NOISE_FLOOR:
                k = max(1, len(mag) // 5)
                current_noise = np.partition(mag, k)[k]
                db.noise_floor = db.noise_floor * (1 - NOISE_FLOOR_ALPHA) + float(current_noise) * NOISE_FLOOR_ALPHA

            # Pulse blanking
            if ENABLE_PULSE_BLANKING:
                med = np.median(mag)
                np.clip(mag, 0, med * PULSE_BLANKING_RATIO, out=mag)

            msg_time = time.time()

            # Search and decode Mode S
            if ENABLE_MODE_S:
                _iq_buf = iq
                preambles = find_preambles(mag, db.noise_floor)
                db.collect_analytics()
                db.stats["preambles_total"] += len(preambles)
                for start in preambles:
                    try:
                        if start + MSG_SAMPLES > len(mag): continue
                        bits, crc_ok = decode_bits_best(mag, int(start))
                        rssi = compute_rssi(mag, int(start), db.noise_floor)
                        msg_total += 1
                        sig_strong = rssi is not None and rssi > 5
                        db.stats["preambles_strong"] += 1 if sig_strong else 0
                        if ENABLE_DOPPLER and crc_ok and _iq_buf is not None:
                            try:
                                _df = bits_to_int(bits[:5])
                                if _df in (17, 18):
                                    _icao_d = format(bits_to_int(bits[8:32]), '06X') if _df in (17, 18) else ""
                                    _ac_d = db.ac.get(_icao_d)
                                    if _ac_d and _ac_d.get('speed_gs'):
                                        _now_d = time.time()
                                        if _now_d - _ac_d.get('_last_doppler', 0) >= DOPPLER_MIN_INTERVAL:
                                            _ac_d['_last_doppler'] = _now_d
                                            _bearing = None
                                            _ac_lat = _ac_d.get('lat')
                                            _ac_lon = _ac_d.get('lon')
                                            if _ac_lat is not None and _ac_lon is not None:
                                                _dlon = math.radians(_ac_lon - HOME_LON)
                                                _home_lat_r = math.radians(HOME_LAT)
                                                _ac_lat_r = math.radians(_ac_lat)
                                                _bearing = math.degrees(math.atan2(
                                                    math.sin(_dlon) * math.cos(_ac_lat_r),
                                                    math.cos(_home_lat_r) * math.sin(_ac_lat_r) -
                                                    math.sin(_home_lat_r) * math.cos(_ac_lat_r) * math.cos(_dlon)
                                                ))
                                                _bearing = (_bearing + 360) % 360
                                            _msg_iq = _iq_buf[int(start):int(start) + MSG_SAMPLES]
                                            if len(_msg_iq) >= DOPPLER_MIN_SAMPLES:
                                                db.compute_doppler(_msg_iq, None, _ac_d['speed_gs'], _bearing, _icao_d)
                            except Exception:
                                log.exception("Doppler processing error")

                        if worker:
                            worker.submit(bits, msg_time, rssi, crc_ok, sig_strong)
                        else:
                            ok = validate_and_process(bits, db, msg_time, rssi, crc_ok)
                            if ok: msg_valid += 1
                            db.record_quality(ok, sig_strong)
                    except Exception:
                        log.exception("Main loop: error processing ADS-B message")

            # Search and decode Mode A/C
            if ENABLE_MODE_AC:
                for start in find_mode_ac_preambles(mag, db.noise_floor):
                    try:
                        if start + MODE_AC_MSG_SAMPLES > len(mag): continue
                        sq, alt_ft, valid, spi = decode_mode_ac(mag, int(start))
                        if valid:
                            rssi = compute_rssi(mag, int(start), db.noise_floor)
                            db.process_mode_ac(sq, alt_ft, rssi, spi=spi)
                    except Exception:
                        log.exception("Main loop: error decoding Mode A/C")

            # Search and decode UAT
            if ENABLE_UAT:
                for start in find_uat_preambles(mag, db.noise_floor):
                    try: decode_uat_message(mag, int(start), db)
                    except Exception:
                        log.exception("Main loop: error decoding UAT message")
            # Check silence and reset flag
            db.check_silence()

            # Check proximity and head-on courses
            db.check_proximity()

            # Save tail for next chunk
            prev_tail = iq[-OVERLAP:] if len(iq) >= OVERLAP else iq
            now = time.time()
            db.dead_reckon()

            # Adaptive gain
            if ADAPTIVE_GAIN and now - last_gain_check >= GAIN_CHECK_INTERVAL:
                new_gain = adjust_gain(db, current_gain)
                if new_gain != current_gain:
                    current_gain = new_gain
                    proc = restart_sdr(proc, current_gain)
                    prev_tail = np.array([], dtype=np.complex64)
                last_gain_check = now

            # Auto-PPM
            if ENABLE_PPM_AUTO and now - last_ppm_check >= PPM_AUTO_INTERVAL:
                old_ppm = db.ppm_adjusted
                new_ppm, proc = db.auto_ppm(proc, db.ppm_adjusted, current_gain)
                if new_ppm != old_ppm:
                    PPM_CORRECTION = new_ppm
                    prev_tail = np.array([], dtype=np.complex64)
                last_ppm_check = now

            # Print table every 5 seconds
            if ENABLE_STATS_HISTORY and now - db._stats_hist_last >= STATS_HISTORY_INTERVAL:
                db._stats_hist_last = now
                db.save_stats_snapshot()
                db.cleanup_stats_history()
            if ENABLE_POS_HISTORY and now - db._pos_hist_last >= POS_HISTORY_INTERVAL:
                db._pos_hist_last = now
                db.save_pos_snapshot()
                db.cleanup_pos_history()

            if now - db.last_print >= 5.0:
                db.cleanup()
                clear_screen()
                q = db.get_quality()
                gain_disp = "AUTO" if current_gain == 0 else f"{current_gain/10:.1f}dB"
                ppm_disp = f" | PPM: {db.ppm_adjusted:+d}" if db.ppm_adjusted != PPM_CORRECTION else ""
                print(f"  {c('ADS-B decoder', Color.BOLD)} — {c(datetime.datetime.now().strftime('%H:%M:%S'), Color.GRAY)} {c(f'gain:{gain_disp}', Color.GRAY)}{ppm_disp}")
                if ENABLE_JSON_OUT: print(f"  {c('Dashboard:', Color.CYAN)} http://{JSON_OUT_HOST}:{JSON_OUT_PORT}")
                print()
                db.print_table(quality=q)
                print()
                if worker:
                    msg_total = worker.msg_total
                    msg_valid = worker.msg_valid
                db.stats["msg_total"] = msg_total
                db.stats["msg_valid"] = msg_valid
                db.print_stats(msg_total, msg_valid, t_start)
                print()
                db.print_events()
                sys.stdout.write("\033[J")  # clear old text tail
                sys.stdout.flush()
                db.last_print = now

    except KeyboardInterrupt: pass
    finally:
        try:
            if worker:
                worker.stop()
                msg_total = worker.msg_total
                msg_valid = worker.msg_valid
            shutdown_network(db)
            if ENABLE_SESSION_SAVE: db.save_session()
            clear_screen()
            if proc:
                _kill_proc(proc)
            print(f"  {c('ADS-B decoder by R4SCS', Color.BOLD)} — {c('Shutting down', Color.RED)}")
            print()
            db.print_table()
            print()
            db.print_stats(msg_total, msg_valid, t_start)
            print()
            db.print_events()
            db.logger.close_all()
            final_report(db, msg_total, msg_valid, t_start)

        except KeyboardInterrupt:
            if proc:
                _kill_proc(proc)
            print("\n  Forced exit.")
def get_sysinfo():
    """Collect system metrics: CPU, temperature, memory, disk."""
    import time as _t
    result = {"cpu_pct": 0, "temp_c": 0, "ram_used_gb": 0,
               "ram_total_gb": 0, "disk_used_gb": 0, "disk_total_gb": 0,
               "uptime_sec": 0, "load_avg": [0, 0, 0]}
    try:
        with open("/proc/stat", "r") as f:
            cpu1 = f.readline().split()[1:5]
            cpu1 = [int(x) for x in cpu1]
        _t.sleep(0.1)
        with open("/proc/stat", "r") as f:
            cpu2 = f.readline().split()[1:5]
            cpu2 = [int(x) for x in cpu2]
        d = [cpu2[i] - cpu1[i] for i in range(4)]
        total = sum(d)
        if total > 0:
            result["cpu_pct"] = round(100.0 * (total - d[3]) / total, 1)
    except Exception:
        pass
    try:
        for z in range(10):
            p = "/sys/class/thermal/thermal_zone" + str(z) + "/temp"
            if os.path.exists(p):
                with open(p, "r") as f:
                    val = f.read().strip()
                    result["temp_c"] = int(val) / 1000.0
                break
    except Exception:
        pass
    try:
        with open("/proc/meminfo", "r") as f:
            mi = f.read()
        for line in mi.split("\n"):
            if line.startswith("MemTotal:"):
                result["ram_total_gb"] = round(int(line.split()[1]) / 1048576.0, 1)
            elif line.startswith("MemAvailable:"):
                avail = round(int(line.split()[1]) / 1048576.0, 1)
                result["ram_used_gb"] = round(result["ram_total_gb"] - avail, 1)
    except Exception:
        pass
    try:
        st = os.statvfs("/")
        total_b = st.f_blocks * st.f_frsize
        free_b = st.f_bavail * st.f_frsize
        result["disk_total_gb"] = round(total_b / 1073741824.0, 1)
        result["disk_used_gb"] = round((total_b - free_b) / 1073741824.0, 1)
    except Exception:
        pass
    try:
        with open("/proc/uptime", "r") as f:
            result["uptime_sec"] = int(float(f.read().split()[0]))
    except Exception:
        pass
    try:
        la = os.getloadavg()
        result["load_avg"] = [round(x, 2) for x in la]
    except Exception:
        pass
    return result


if __name__ == '__main__':
    main()
