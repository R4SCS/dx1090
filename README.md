# DX1090 — ADS-B / Mode S / Mode A/C / UAT Decoder

Copyright (c) 2026-present, R4SCS — https://github.com/r4scs

This project is licensed under the GNU General Public License v3.

---

## Overview

dx1090 is a Python-based real-time ADS-B decoder with a built-in web dashboard. It receives and decodes Mode S (ADS-B), Mode A/C, and UAT 978 MHz signals from RTL-SDR, Airspy, HackRF, or bladeRF receivers — all from a single file, no compilation, no external services required.

The decoder processes raw I/Q samples directly from the SDR hardware, performs full Mode S demodulation with multi-phase bit recovery, CRC validation and single/double-bit error correction, and decodes every Downlink Format (DF 0–25) defined in ICAO Annex 10. ADS-B messages (TC 1–31) are parsed for callsign, position (CPR Global and Local), altitude (Gillham-coded barometric and GNSS), ground speed, vertical rate, squawk, and aircraft category. In addition, thirteen Comm-B BDS registers (1.0 through 6.1) are decoded, providing access to selected altitude, meteorological data (wind, temperature, pressure), weather hazards (turbulence, icing), track-and-turn reports, selected heading, TCAS track data, and heading/speed parameters.

Beyond raw decoding, dx1090 maintains a live aircraft database with position smoothing, dead-reckoning extrapolation, reliability scoring, and range/jump filters. A dual-quality metric system (RSSI-based and position-based) tracks each aircraft's signal integrity. Automatic PPM calibration compensates for crystal drift in real time, while adaptive gain control adjusts receiver sensitivity based on message rate.

The built-in web dashboard provides a real-time aircraft table, an interactive map with trails, coverage circles, and 3D altitude visualization, live charts, polar range and RSSI plots, and four analytics tabs with per-chart history mode, Doppler detection, conflict detection, and session comparison. Historical statistics and position snapshots are saved to disk for session replay and long-term analysis.

Sound notifications cover 26 event types — from basic new-aircraft and emergency alerts to TCAS RA, turbulence, rapid descent, head-on proximity, and range records — each with a distinct acoustic signature. The sound system can run on the server or be handed off to the browser.

Data export supports seven formats: CSV (aircraft table), GZ (compressed JSON dump), GPX (GPS), KML (Google Earth), JSON (full session statistics), AeroXML, and FlightAware format. Sessions can be saved and restored, preserving the full aircraft database and statistics between runs.

---

## Table of Contents

- [Features](#features)
- [Third-Party Dependencies](#third-party-dependencies)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Supported Receivers](#supported-receivers)
- [How Decoding Works](#how-decoding-works)
  - [Signal Reception](#signal-reception)
  - [Preamble Detection](#preamble-detection)
  - [Bit Demodulation & Phase Correction](#bit-demodulation--phase-correction)
  - [CRC Validation & Error Correction](#crc-validation--error-correction)
  - [Downlink Format (DF) Processing](#downlink-format-df-processing)
  - [ADS-B Type Codes (TC)](#ads-b-type-codes-tc)
  - [CPR Position Decoding](#cpr-position-decoding)
  - [BDS Comm-B Registers](#bds-comm-b-registers)
  - [Mode A/C Decoding](#mode-ac-decoding)
  - [UAT 978 MHz Decoding](#uat-978-mhz-decoding)
- [Decoding Settings You Can Tune](#decoding-settings-you-can-tune)
  - [Receiver Configuration](#receiver-configuration)
  - [Signal Processing](#signal-processing)
  - [CRC Correction](#crc-correction)
  - [Position Filters & Smoothing](#position-filters--smoothing)
  - [BDS Register Selection](#bds-register-selection)
  - [ADS-B Integrity Fields](#ads-b-integrity-fields)
  - [Auto-PPM Calibration](#auto-ppm-calibration)
  - [Adaptive Gain](#adaptive-gain)
- [Network Output Protocols](#network-output-protocols)
- [HTTP API](#http-api)
- [Web Dashboard](#web-dashboard)
- [Per-Chart History Mode](#per-chart-history-mode)
- [Sound Notifications](#sound-notifications)
- [Data Export](#data-export)
- [Session Save & Restore](#session-save--restore)
- [Historical Statistics](#historical-statistics)
- [Aircraft Database & Photos](#aircraft-database--photos)
- [Configuration System](#configuration-system)
- [CLI Reference](#cli-reference)
- [Runtime Controls](#runtime-controls)
- [Directory Structure](#directory-structure)
- [Project Status](#project-status)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Features

### Reception
- **Four SDR backends**: RTL-SDR (RTL2832U), Airspy R2/Mini, HackRF One, bladeRF 2.0 micro
- **Dual-band**: 1090 MHz (ADS-B / Mode S / Mode A/C) and 978 MHz (UAT, US/Australia)
- **Watchdog**: automatic SDR restart on crash or data stall
- **Decode thread**: optional separate decoding thread with queue for non-blocking reception

### Decoding
- **Mode S**: all Downlink Formats DF 0–25 (ADS-B, interrogation replies, Comm-B, military)
- **ADS-B**: all Type Codes (TC 1–19) — identification, position (airborne & ground), velocity, squawk, emergency, category, integrity fields
- **CPR**: both Global and Local decoding for airborne and surface positions
- **BDS registers**: 1,0 / 1,7 / 2,0 / 3,0 / 4,0 / 4,4 / 4,5 / 5,0 / 5,1 / 5,2 / 5,3 / 6,0 / 6,1
- **Mode A/C**: classic secondary surveillance radar (squawk + altitude)
- **UAT 978 MHz**: Automatic Dependent Surveillance—Broadcast on UAT (US/Australia)
- **CRC correction**: single-bit (syndrome lookup) and two-bit (brute-force) with adaptive throttling
- **Doppler detector**: estimates Doppler shift from I/Q samples and compares with expected values to assess receiver PPM accuracy
- **Conflict detector**: identifies close-approach pairs and head-on courses between tracked aircraft

### Tracking & Quality
- **Dual quality metrics**: separate decode quality (%) and receive quality (%), each tracked on a 10-second rolling window with EMA smoothing
- **Position reliability counter**: each aircraft accumulates a reliability score based on successful position decodings; positions below the threshold are not used for dead-reckoning or smoothing
- **Dead-reckoning**: extrapolates aircraft position from last known speed and heading when position updates are stale (up to a configurable age limit)
- **Position smoothing**: exponential moving average filter on raw decoded lat/lon, adjustable coefficient
- **Auto-PPM**: calibrates the receiver's crystal frequency error by comparing dead-reckon positions against actual decoded positions, then restarts the SDR with the corrected PPM value

### Dashboard
- **Real-time web UI**: aircraft table, live map with trails, charts, analytics, event feed, polar range/RSSI plots
- **Four analytics tabs**: Graphs, Analytics, Profile, Special — each with per-chart history mode
- **Per-chart history mode**: every chart can be toggled to show historical data with presets (Session, 1 hour, Today, custom range)
- **Compare sessions**: side-by-side comparison of two session snapshots with metric diff
- **Browser sound takeover**: audio notifications can play in the browser instead of the server, with heartbeat-based fallback
- **Export from UI**: CSV, GZ (compressed JSON), GPX, KML, JSON session statistics, AeroXML, FlightAware format

### Sound
- **26 notification types**: new aircraft, emergency squawks (7500/7600/7700), loss of contact, TCAS RA, low altitude, military, tracked flight, tracked loss, range record, silence, turbulence, long-range, overhead pass, speed/altitude records, helicopter, rare type, rapid descent, return, level-off, ground vehicle, proximity, head-on, breakthrough, squawk change
- Each sound type can be individually enabled/disabled
- WAV files with embedded metadata (author, software, comment tags)

### Data & Integration
- **Aircraft database**: OpenSky `aircraft.csv` lookup for registration, type code, operator, model, manufacturer, owner, country, engines
- **Aircraft photos**: automatic fetching from Planespotters API with local disk caching (7-day TTL)
- **Historical statistics**: periodic snapshots of types, histograms, and max values saved to disk with configurable retention
- **Position history**: per-aircraft position snapshots for chart history mode
- **Session persistence**: aircraft database, statistics, and PPM are saved on exit and restored on next launch

---

## Third-Party Dependencies

dx1090 is designed to run with minimal external dependencies. The Python decoder itself only requires **NumPy** — everything else is either part of the Python standard library or an optional system tool. However, the web dashboard and aircraft lookup features rely on several third-party projects. Below is a complete inventory.

### Python Package

| Dependency | Required? | Install | Purpose |
|---|---|---|---|
| [NumPy](https://numpy.org/) | **Yes** | `pip install numpy` | I/Q sample processing, array math, FFT |

All other Python modules used by the decoder — `subprocess`, `socket`, `http.server`, `logging`, `gzip`, `csv`, `wave`, `struct`, `signal`, `select`, `atexit`, `argparse`, `urllib.request`, `collections`, `datetime`, `threading`, `queue`, `json`, `math`, `os`, `sys`, `time`, `shutil` — ship with Python 3.8+ and require no installation.

### `static/` Directory — Dashboard Libraries

The `static/` folder contains local copies of the JavaScript and CSS libraries that power the web dashboard. These are bundled so the dashboard works **without internet access**. When an internet connection is available, the dashboard automatically falls back to CDN-hosted versions if a local file is missing or corrupted.

| File | Project | Version | License | Local Path | CDN Fallback |
|---|---|---|---|---|---|
| `leaflet.js` | [Leaflet](https://leafletjs.com/) | 1.9.4 | BSD-2-Clause | `static/leaflet.js` | [unpkg.com](https://unpkg.com/leaflet@1.9.4/dist/leaflet.js) |
| `leaflet.css` | [Leaflet](https://leafletjs.com/) | 1.9.4 | BSD-2-Clause | `static/leaflet.css` | [unpkg.com](https://unpkg.com/leaflet@1.9.4/dist/leaflet.css) |
| `chart.umd.min.js` | [Chart.js](https://www.chartjs.org/) | 4.4.1 | MIT | `static/chart.umd.min.js` | [cdn.jsdelivr.net](https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js) |

**Download links:**

- Leaflet 1.9.4: [leaflet.js](https://unpkg.com/leaflet@1.9.4/dist/leaflet.js) · [leaflet.css](https://unpkg.com/leaflet@1.9.4/dist/leaflet.css)
- Chart.js 4.4.1: [chart.umd.min.js](https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js)

To update these libraries, download the new version from the official source and replace the corresponding file in `static/`. No code changes needed — the dashboard loads local files first and falls back to CDN automatically.

> **Attribution**: Leaflet is © 2010–2024 Volodymyr Agafonkin. Chart.js is © 2014–2024 Chart.js Contributors.

### `aircraft.csv` — Aircraft Database

The decoder optionally uses an aircraft metadata database in OpenSky Network format. This CSV file provides registration, type code, operator, model, manufacturer, owner, country, and engine count for each ICAO 24-bit address.

- **Source**: [OpenSky Network Aircraft Database](https://opensky-network.org/data/aircraft)
- **License**: CC-BY-SA 4.0 (OpenSky Network)
- **File name**: `aircraft.csv` (place next to `dx1090.py`)
- **Configurable**: `aircraft_db_file` in `config.json`

The file is **not included** in the repository due to its size (~100 MB). Without it, the decoder still works — you just won't see registration, type, and operator lookups.

#### Downloading

The database is hosted on OpenSky's S3 bucket. Files are named `aircraft-database-complete-YYYY-MM.csv` (monthly snapshots). OpenSky updates the database irregularly, so the latest available version may not be the current month.

```bash
# Download the latest available snapshot (try current month first, then go back)
wget https://s3.opensky-network.org/data-samples/metadata/aircraft-database-complete-2025-02.csv -O aircraft.csv
```

> **Important**: Use the `aircraft-database-complete-*` format, **not** `aircraftDatabase-*`. The `complete` version has all columns and is the one the decoder expects. The non-complete version has fewer columns and will not work correctly.

#### Format

The file is a CSV with single-quote delimiters and 31 columns. The decoder reads 9 of them:

| Column | Purpose |
|---|---|
| `icao24` | ICAO 24-bit address (dictionary key) |
| `registration` | Aircraft registration number |
| `typecode` | ICAO type code (A320, B738, etc.) |
| `operator` | Airline / operator name |
| `model` | Full aircraft model name |
| `manufacturerName` | Manufacturer (fallback: `manufacturerIcao`) |
| `owner` | Owner |
| `country` | Country (mapped to flag emoji) |
| `engines` | Engine type description |

Each lookup field can be individually toggled via config keys (`lookup_registration`, `lookup_typecode`, `lookup_operator`, `lookup_model`, `lookup_manufacturer`, `lookup_owner`, `lookup_country`, `lookup_engines`).

### SDR Drivers (CLI Tools)

The decoder communicates with SDR hardware by launching the manufacturer's command-line tool as a subprocess. These tools must be installed separately and available in your system `PATH`.

| Receiver | CLI Tool | Project / Source | License | Download |
|---|---|---|---|---|
| RTL-SDR | `rtl_sdr` | [osmocom/rtl-sdr](https://github.com/osmocom/rtl-sdr) | GPL-2.0 | [Releases](https://github.com/osmocom/rtl-sdr/releases) |
| Airspy | `airspy_rx` | [airspy/airspyone_host](https://github.com/airspy/airspyone_host) | BSD-3-Clause | [Releases](https://github.com/airspy/airspyone_host/releases) |
| HackRF | `hackrf_transfer` | [greatscottgadgets/hackrf](https://github.com/greatscottgadgets/hackrf) | GPL-2.0 | [Releases](https://github.com/greatscottgadgets/hackrf/releases) |
| bladeRF | `bladerf-cli` | [Nuand/bladeRF](https://github.com/Nuand/bladeRF) | GPL-2.0 | [Releases](https://github.com/Nuand/bladeRF/releases) |

### Audio Players (optional)

For sound notifications, the decoder auto-detects the first available audio player:

| Player | Platform | Project | License | Download |
|---|---|---|---|---|
| `paplay` | Linux | [PulseAudio](https://gitlab.freedesktop.org/pulseaudio/pulseaudio) | GPL-2.0 | `sudo apt install pulseaudio-utils` |
| `aplay` | Linux | [ALSA](https://www.alsa-project.org/) | GPL-2.0 | `sudo apt install alsa-utils` |
| `ffplay` | All | [FFmpeg](https://ffmpeg.org/) | GPL-2.0+ | [ffmpeg.org/download](https://ffmpeg.org/download.html) |

If no player is found, the browser-based sound mode is used automatically (no system dependency needed).

### Aircraft Photos — Planespotters API

The dashboard fetches aircraft photos from the [Planespotters](https://www.planespotters.net/) REST API by ICAO hex address or registration. This is a free, no-API-key service. Results are cached on disk for 7 days (positive cache) and 1 hour (negative cache — no photo found).

- **API**: `https://api.planespotters.net/`
- **Docs**: [Planespotters Photo API](https://www.planespotters.net/photo/api)
- **No key required**: a descriptive `User-Agent` header is sent automatically

### Map Tiles — OpenStreetMap

When the dashboard runs in Leaflet mode with online tiles, it proxies tile requests through the built-in HTTP server to [OpenStreetMap](https://www.openstreetmap.org/) tile servers. Tiles are cached on disk in the `tile_cache/` directory to reduce bandwidth and enable faster loads.

- **Source**: © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors
- **License**: [ODbL](https://www.openstreetmap.org/copyright)

When no internet is available, the dashboard can switch to canvas mode — a simple offline radar view that draws aircraft positions directly on an HTML canvas without any tile server.

---

## Installation

### 1. System Requirements

| Component | Minimum | Recommended |
|---|---|---|
| **Python** | 3.8+ | 3.11+ |
| **RAM** | 512 MB | 1 GB+ |
| **CPU** | Any dual-core | 2.4 GHz+ for 2-bit CRC at high rates |
| **SDR** | RTL-SDR dongle | RTL-SDR + 1090 MHz antenna |
| **OS** | Linux, macOS, Windows | Linux (best SDR driver support) |

### 2. Install Python and NumPy

**Ubuntu / Debian:**

```bash
sudo apt update
sudo apt install python3 python3-pip
pip3 install numpy
```

**macOS (Homebrew):**

```bash
brew install python numpy
```

**Windows:**

Download Python 3.8+ from [python.org](https://www.python.org/downloads/), then:

```powershell
pip install numpy
```

### 3. Install SDR Drivers

Install the command-line tool for your receiver:

| Receiver | Package / Tool | Install (Ubuntu/Debian) |
|---|---|---|
| RTL-SDR | `rtl-sdr` | `sudo apt install rtl-sdr` |
| Airspy | `airspy_rx` | [Airspy Host Tools](https://github.com/airspy/airspyone_host) |
| HackRF | `hackrf_transfer` | `sudo apt install hackrf` |
| bladeRF | `bladerf-cli` | `sudo apt install bladerf` |

On Windows, install the respective driver from the manufacturer's website and ensure the CLI tool is in your `PATH`.

### 4. Install Audio Player (optional)

| Platform | Tool | Install |
|---|---|---|
| Linux | `paplay` | `sudo apt install pulseaudio-utils` |
| Linux | `aplay` | `sudo apt install alsa-utils` |
| Any | `ffplay` | `sudo apt install ffmpeg` |

The decoder auto-detects available players. If none is found, browser-based sound is used.

### 5. Download Aircraft Database (optional)

```bash
wget https://s3.opensky-network.org/data-samples/metadata/aircraft-database-complete-2025-02.csv -O aircraft.csv
```

Place `aircraft.csv` next to `dx1090.py`, or set `aircraft_db_file` in `config.json`.

> If the `2025-02` snapshot is unavailable, try more recent months: replace `2025-02` with `2025-03`, `2025-04`, etc. See the [Aircraft Database](#aircraft-database--photos) section for details.

### 6. Get the Code

```bash
git clone https://github.com/r4scs/dx1090.git
cd dx1090
```

The repository includes the `static/` folder with Leaflet and Chart.js bundled — no extra downloads needed for the dashboard.

---

## Quick Start

1. **Create a config file.** The repository ships with `config1.json` — a ready-to-use example configuration. Simply rename it:

   ```bash
   cp config1.json config.json
   ```

   Then edit `config.json` and set your receiver coordinates:

   ```json
   {
     "home_lat": 46.856614,
     "home_lon": 2.213749
   }
   ```

   At minimum, set your latitude and longitude. All other parameters have sensible defaults — the program runs out of the box with zero configuration if you're fine with the built-in values.

2. **Plug in your SDR** (e.g. RTL-SDR dongle with 1090 MHz antenna).

3. **Run the decoder:**

   ```bash
   python3 dx1090.py
   ```

4. **Open the dashboard** in your browser: `http://127.0.0.1:8080`

---

## Supported Receivers

| Receiver | `sdr_type` | Sample Format | Gain Unit | CLI Tool |
|---|---|---|---|---|
| RTL-SDR (RTL2832U) | `rtl_sdr` | uint8 IQ (offset) | tenths of dB (0–496) | `rtl_sdr` |
| Airspy R2/Mini | `airspy` | int16 IQ | dB (0–21) | `airspy_rx` |
| HackRF One | `hackrf` | uint8 IQ (offset) | dB (0–14, steps of 8) | `hackrf_transfer` |
| bladeRF 2.0 micro | `bladerf` | int16 IQ | dB (0–60) | `bladerf-cli` |

The decoder auto-detects the SDR type via the `--airspy`, `--hackrf`, or `--bladerf` CLI flags, or via the `sdr_type` key in `config.json`. If none is specified, RTL-SDR is used by default.

IQ sample conversion is handled automatically — each receiver type has its own normalization (uint8 offset for RTL-SDR/HackRF, int16 for Airspy/bladeRF), so you don't need to worry about raw data formats.

---

## How Decoding Works

### Signal Reception

The decoder launches the SDR CLI tool as a subprocess and reads raw I/Q bytes from its standard output. Each chunk is converted to a complex NumPy array (float32 I + float32 Q). The following signal processing steps are applied:

1. **DC removal** — subtracts the mean of I and Q to remove the ADC's DC offset
2. **Pulse blanking** — clips samples whose magnitude exceeds `median × pulse_blanking_ratio` to suppress impulse noise
3. **Noise floor estimation** — 20th percentile of signal magnitudes, smoothed with an exponential moving average (EMA, α = 0.005)

### Preamble Detection

ADS-B messages begin with an 8-microsecond preamble — a specific pattern of high/low pulses. The detector slides over the signal and identifies preamble positions by checking the expected high/low pattern at sample-level resolution. At 2 MS/s, each bit spans exactly 2 samples, so the preamble occupies 16 samples.

Detection is threshold-based: the minimum signal level is either auto-determined from the noise floor or set explicitly via `min_signal_level`. A minimum interval between consecutive preambles prevents duplicate detections.

### Bit Demodulation & Phase Correction

Each ADS-B bit is 1 microsecond long. At 2 MS/s, that's 2 samples per bit. The demodulator uses **Pulse Position Modulation (PPM)**: a logic 1 has higher amplitude in the first half-bit, a logic 0 in the second.

**Phase correction** compensates for sampling clock drift. When a message fails CRC, the decoder retries with fractional sample offsets (±0.5 samples). Integer offsets (±1.0) are excluded because a whole-bit shift can produce false CRC passes.

### CRC Validation & Error Correction

Every Mode S message ends with a 24-bit CRC (polynomial 0x1FFF409). The decoder performs three levels of error correction:

1. **No correction** — if CRC passes on the first try, the message is accepted as-is
2. **1-bit correction** — syndrome lookup table; if the syndrome matches a known single-bit error pattern, the bit is flipped and the message is accepted
3. **2-bit correction** — brute-force search over all pairs of bit positions in the first 56 bits. This is computationally expensive and is dynamically throttled:

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Dynamic throttling | `crc_two_bit_dynamic` | `true` | Enable adaptive 2-bit correction |
| Time ratio threshold | `crc_two_bit_time_ratio` | `0.15` | Max fraction of decode time for 2-bit |
| Max decode time | `crc_two_bit_max_decode_time` | `50` | ms — if exceeded, 2-bit disabled |
| EMA smoothing | `crc_two_bit_time_alpha` | `0.05` | EMA coefficient for time estimation |

When the message rate exceeds ~200 msg/sec or decode time exceeds the threshold, 2-bit correction is automatically disabled to prevent data loss in the receive buffer. It re-enables when the load drops.

### Downlink Format (DF) Processing

Each Mode S message contains a 5-bit Downlink Format identifier. The decoder handles all DF types defined in ICAO Annex 10:

| DF | Name | Description |
|---|---|---|
| 0 | Short ACAS | TCAS interrogation reply (short) |
| 4 | Altitude reply | Barometric altitude (Mode S) |
| 5 | Ident reply | Squawk code (Mode S ident) |
| 11 | All-call reply | Capability flags + ICAO address |
| 16 | Long ACAS | TCAS interrogation reply (long) |
| 17 | Extended Squitter | ADS-B (ADS-B message payload) |
| 18 | Extended Squitter (non-ICAO) | TIS-B / ADS-R messages |
| 19 | Military Extended Squitter | Military ADS-B |
| 20 | Comm-B (ELM) | BDS register data (56-bit payload) |
| 21 | Comm-B (ELM) | BDS register data (56-bit payload) |
| 24 | Comm-D (ELM) | Multi-segment data link |
| 25 | Comm-D (ELM) | Multi-segment data link |

For DF 0, 4, 5, 11, 16, 20, 21, 24, and 25, the ICAO 24-bit address is recovered from the CRC parity bits (interrogator code subtraction).

### ADS-B Type Codes (TC)

ADS-B messages (DF 17/18) carry a 5-bit Type Code that determines the message content:

| TC | Content |
|---|---|
| 1–4 | Aircraft identification (callsign, 8 chars ICAO alphabet) |
| 5–8 | Surface position (CPR, ground mode) |
| 9–18 | Airborne position (CPR, airborne) + barometric altitude |
| 19 | Airborne velocity (ground speed, heading, vertical rate) |
| 20–22 | GNSS altitude + CPR position |
| 23 | Test messages |
| 28 | Aircraft status (DO-260 version, emergency squawk) |
| 29 | Target state and status |
| 31–32 | Operation status |

### CPR Position Decoding

Compact Position Reporting (CPR) is the position encoding scheme used in ADS-B. Each position message is either "even" or "odd" — a pair of opposite parity is needed to decode a position globally. The decoder implements:

- **Global decoding** — uses one even + one odd message to compute an unambiguous position worldwide
- **Local decoding** — uses a single message plus a reference position (previously decoded or dead-reckoned) for incremental updates
- **NZ = 15** — standard ADS-B zone count
- **CPR timeout** — 10 seconds; if no matching parity message arrives within this window, the pair is discarded
- **Position validation** — `MAX_POSITION_JUMP_KM` (50 km) rejects unrealistic jumps; `MAX_RANGE_KM` (400 km) rejects positions beyond expected reception range
- **Reliability counter** — each successful position decode increments a per-aircraft counter; positions below `POSITION_RELIABLE_MIN` (1) are not used for dead-reckoning or smoothing

### BDS Comm-B Registers

DF 20 and DF 21 messages carry a 56-bit Comm-B payload (BDS register). The decoder attempts to identify the register type and extract the corresponding data:

| BDS | Name | Data |
|---|---|---|
| 1,0 | Data Link Capability | Supported BDS registers |
| 1,7 | Common Usage GICB | Capabilities report |
| 2,0 | Aircraft Identification | Callsign (same as TC 1–4) |
| 3,0 | ACAS Resolution Advisory | TCAS RA (vertical/horizontal guidance) |
| 4,0 | Selected Altitude | FMS selected altitude, QNH |
| 4,4 | Meteorological | Wind speed/direction, temperature |
| 4,5 | Meteorological | Turbulence, icing, windshear |
| 5,0 | Track and Turn | Roll angle, heading, GS, TAS |
| 5,1–5,3 | Extended Reports | Additional track/turn data |
| 6,0 | Heading and Speed | Magnetic heading, IAS, Mach, vertical rate |
| 6,1 | Extended Heading/Speed | Additional speed data |

Because the same 56-bit payload can be interpreted as different BDS registers, the decoder uses a scoring algorithm (`_score_bds60`) that compares decoded values against already-known aircraft data to disambiguate.

### Mode A/C Decoding

Mode A/C is the classic secondary surveillance radar format, transmitted on the same 1090 MHz frequency. The decoder optionally processes Mode A/C replies (enabled via `--mode-ac` or `enable_mode_ac` in config).

- **Mode A** — 4-digit squawk code (octal, 0000–7777)
- **Mode C** — barometric altitude (Gillham-coded)
- Special squawk codes: 7500 (hijack), 7600 (radio failure), 7700 (general emergency) trigger sound alerts

Mode A/C decoding uses a simpler threshold-based demodulator (no CRC), with median-based bit detection.

### UAT 978 MHz Decoding

UAT (Universal Access Transceiver) is a second ADS-B channel used in the US and Australia. It operates at 978 MHz and requires a separate SDR receiver.

The decoder supports:

- CRC-32 validation (polynomial 0x04C11DB7)
- Sync word detection
- Short (272-bit) and long (272+144-bit) messages
- TC 0 — heartbeat, TC 1 — ground station, TC 2 — airborne position, TC 3 — airborne velocity
- Position, altitude, speed, heading, vertical rate, NIC/NACp/SIL, emitter category

Configure via `enable_uat`, `uat_freq`, `uat_sample_rate`, `uat_gain`, `uat_ppm` in config or `--uat` CLI flag.

---

## Decoding Settings You Can Tune

All settings are defined as constants at the top of `dx1090.py` with detailed comments. They can be overridden via `config.json` (see [Configuration System](#configuration-system)).

### Receiver Configuration

| Setting | Config Key | Default | Description |
|---|---|---|---|
| SDR type | `sdr_type` | `rtl_sdr` | `rtl_sdr`, `airspy`, `hackrf`, `bladerf` |
| Frequency | `freq` | `1090000000` | Hz (1090 MHz standard) |
| Sample rate | `sample_rate` | `2000000` | Hz (2 MS/s = 2 samples/bit) |
| Gain | `gain` | `0` | RTL-SDR: tenths of dB (0=AGC, 496=max) |
| PPM correction | `ppm_correction` | `0` | Crystal frequency deviation, ppm |
| Chunk size | `chunk_bytes` | `262144` | SDR read block size, bytes |
| Min signal level | `min_signal_level` | `0` | 0 = auto (noise floor), >0 = fixed threshold |

### Signal Processing

| Setting | Config Key | Default | Description |
|---|---|---|---|
| DC removal | `enable_dc_removal` | `true` | Remove ADC DC offset |
| Noise floor estimation | `enable_noise_floor` | `true` | Adaptive noise floor (20th percentile) |
| Noise floor alpha | `noise_floor_alpha` | `0.005` | EMA smoothing coefficient |
| Pulse blanking | `enable_pulse_blanking` | `true` | Impulse noise suppression |
| Pulse blanking ratio | `pulse_blanking_ratio` | `50` | Median × ratio = clip threshold |

### CRC Correction

| Setting | Config Key | Default | Description |
|---|---|---|---|
| 1-bit correction | `enable_crc_correction` | `true` | Syndrome lookup, always fast |
| 2-bit correction | `enable_crc_two_bit` | `true` | Brute-force, CPU-intensive |
| Dynamic throttling | `crc_two_bit_dynamic` | `true` | Auto-disable under high load |
| Time ratio | `crc_two_bit_time_ratio` | `0.15` | Max fraction of decode time |
| Max decode time | `crc_two_bit_max_decode_time` | `50` | ms threshold |
| Time EMA alpha | `crc_two_bit_time_alpha` | `0.05` | Smoothing for time estimate |

### Position Filters & Smoothing

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Max position jump | `max_position_jump_km` | `50.0` | km — reject larger jumps |
| Max range | `max_range_km` | `400.0` | km — reject positions beyond |
| Max vertical rate | `max_alt_rate_fpm` | (from code) | fpm — reject unrealistic climb/descent |
| Smoothing | `enable_smoothing` | `true` | Exponential position filter |
| Smoothing alpha | `smoothing_alpha` | `0.6` | 0.1 = very smooth, 1.0 = no smoothing |
| Dead-reckoning | `enable_dead_reckoning` | `true` | Extrapolate position when stale |
| DR max age | `dr_max_age` | (from code) | sec — max age for extrapolation |
| Stale timeout | `stale_timeout` | `60` | sec — aircraft shown in gray |
| Loss timeout | `loss_timeout` | `180` | sec — aircraft removed + sound |
| CPR timeout | `cpr_timeout` | `10` | sec — even/odd message pair validity |

### BDS Register Selection

| Setting | Config Key | Default | Description |
|---|---|---|---|
| BDS attempts | `enable_bds` | `true` | Try to decode Comm-B payloads |
| BDS 4,4 (meteo) | `enable_bds44` | `true` | Wind, temperature |
| BDS 4,5 (hazards) | `enable_bds45` | `true` | Turbulence, icing, windshear |
| BDS 5,0 | `enable_bds50` | `true` | Track and turn |
| BDS 6,0 | `enable_bds60` | `true` | Heading and speed |

### ADS-B Integrity Fields

The decoder tracks NIC (Navigation Integrity Category), NACp (Navigation Accuracy Category — Position), NACv (Navigation Accuracy Category — Velocity), and SIL (Source Integrity Level) from ADS-B messages. These are displayed in the aircraft detail panel and used for quality assessment.

### Auto-PPM Calibration

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Auto-PPM | `enable_ppm_auto` | `true` | Calibrate crystal drift from positions |
| Max position age | `ppm_auto_max_pos_age` | (from code) | sec — max age for calibration |
| Restart SDR | `ppm_auto_restart_sdr` | `true` | Restart SDR with corrected PPM |
| Doppler estimation | `enable_doppler` | `true` | Estimate PPM from I/Q phase slope |
| Min IQ samples | `doppler_min_samples` | `100` | Minimum samples for Doppler |
| Throttle interval | `doppler_min_interval` | `3` | sec between measurements per aircraft |

### Adaptive Gain

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Adaptive gain | `adaptive_gain` | `false` | Auto-adjust gain by message rate |
| Check interval | `gain_check_interval` | `30` | sec between gain checks |
| Min msg rate | `gain_min_msg_rate` | `3` | msg/sec — below this, gain increases |
| Good msg rate | `gain_good_msg_rate` | `15` | msg/sec — above this, gain decreases |

### Geofilter

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Filter radius | `filter_radius` | `0` | 0 = disabled, >0 = discard beyond radius |
| Radius unit | `filter_radius_unit` | `mi` | `km`, `mi`, `nm` |
| No-position action | `filter_no_position` | `show` | `show` or `hide` |
| Remove out of range | `filter_remove_out_of_range` | `true` | Remove from database when out |

---

## Network Output Protocols

The decoder can output decoded data in three network formats simultaneously:

### Beast (binary, port 30005)

Raw Mode S messages in Beast binary format, compatible with `dump1090`, `piaware`, `readsb`, and other ADS-B tools. Includes MLAT timestamps (12 MHz timer).

| Setting | Config Key | Default |
|---|---|---|
| Enable | `enable_beast_out` | `false` |
| Host | `beast_out_host` | `127.0.0.1` |
| Port | `beast_out_port` | `30005` |

CLI: `--beast`

### SBS1 (text, port 30003)

SBS1/BaseStation text format, compatible with Virtual Radar Server and BaseStation.

| Setting | Config Key | Default |
|---|---|---|
| Enable | `enable_sbs1_out` | `false` |
| Host | `sbs1_out_host` | `127.0.0.1` |
| Port | `sbs1_out_port` | `30003` |

CLI: `--sbs1`

### JSON (HTTP, port 8080)

Built-in HTTP server serving the web dashboard and a JSON API in `dump1090`-compatible format.

| Setting | Config Key | Default |
|---|---|---|
| Enable | `enable_json_out` | `true` |
| Host | `json_out_host` | `127.0.0.1` |
| Port | `json_out_port` | `8080` |

---

## HTTP API

All responses include CORS headers (`Access-Control-Allow-Origin: *`).

### GET Endpoints

| Path | Parameters | Description |
|---|---|---|
| `/` | — | Serve dashboard HTML (`dashboard.html`) |
| `/data.json` | — | Aircraft list in dump1090 format |
| `/analytics.json` | — | Extended statistics (DF counts, histograms, conflicts, Doppler, etc.) |
| `/stats_snapshot.json` | `date=YYYY-MM-DD` | Historical statistics snapshot for given date |
| `/stats_range.json` | `from=ISO`, `to=ISO` | Historical statistics for date range |
| `/pos_history.json` | — | Available dates with position history |
| `/export.gpx` | — | GPX export (Garmin, Ozi) |
| `/export.kml` | — | KML export (Google Earth) |
| `/export.gz` | — | Compressed JSON dump (full session) |
| `/photo.json` | `icao=XXXXXX` | Aircraft photo from Planespotters API |
| `/tiles/{z}/{x}/{y}.png` | — | OSM tile proxy with disk cache |
| `/static/{file}` | — | Static files (JS, CSS) |
| `/sound_events.json` | — | Queued sound events for browser playback |

### GET/POST — Sound Control

| Action | Method | Parameters | Description |
|---|---|---|---|
| `status` | GET | — | Get sound status and all type toggles |
| `toggle` | GET | — | Toggle master sound on/off |
| `toggle_type` | GET | `type=NAME` | Toggle individual sound type |
| `set_type` | GET | `type=NAME&value=true\|false` | Set individual sound type |
| `browser_takeover` | GET | — | Switch to browser sound mode |
| `heartbeat` | GET | — | Keep browser sound alive |
| `browser_release` | GET/POST | — | Release browser sound back to server |

### POST — Dashboard Settings

| Action | Parameters | Description |
|---|---|---|
| `set_units` | `distance`, `alt`, `speed`, `vrate`, `temp`, `map_mode` | Update display units and map mode; saved to `dashconf.json` |

---

## Web Dashboard

The dashboard is a single-page application built with vanilla JavaScript, Leaflet, Chart.js, and Canvas 2D. No framework, no build step — just open the URL in a browser.

### Top Bar

- Title with live clock, session duration, and system info
- Buttons: Stats, Map, Analytics, Sound, Settings, Export, Theme toggle

### Main Area

- **Aircraft table** — sortable, filterable, with configurable columns
- **Chart panel** — altitude/speed/distance/vrate/heading for selected aircraft
- **Event feed** — live event log with filters (All, New, Emergency, etc.)
- **Map** — Leaflet (online tiles) or Canvas (offline radar) with aircraft markers, trails, coverage circle, and 3D altitude visualization

### Aircraft Table

Columns (toggleable in Settings):

| Column | Key | Always visible |
|---|---|---|
| ICAO | `icao` | ✓ |
| Flag | `flag` | |
| Flight | `callsign` | ✓ |
| Name | `nav_title` | |
| Operator | `operator` | |
| Altitude | `altitude` | ✓ |
| Speed | `speed` | ✓ |
| Heading | `heading` | ✓ |
| V/S | `vrate` | |
| Lat | `lat` | ✓ |
| Lon | `lon` | ✓ |
| RSSI | `rssi` | |
| Type | `type` | |
| Registration | `registration` | |
| Squawk | `squawk` | |
| Distance | `distance` | |
| Last seen | `last_seen` | |
| Category | `category` | |
| NIC/SIL | `nic_sil` | |
| Messages | `messages` | |
| Turbulence | `turbulence` | |
| Reliability | `reliability` | |

Color coding: gray = stale (>60 sec), red = emergency, green = callsign, purple = coordinates.

### Map Modes

| Mode | Config Value | Description |
|---|---|---|
| Leaflet | `leaflet` | Online OSM tiles via proxy with disk cache |
| Canvas | `canvas` | Offline radar — draws on HTML canvas, no internet needed |
| Auto | `auto` | Leaflet with Canvas fallback when tiles unavailable |

### Aircraft Detail Panel

Clicking an aircraft opens a side panel with:

- Aircraft photo (Planespotters API)
- Full parameter grid: ICAO, callsign, registration, type, operator, country, coordinates, altitude, speed, heading, vertical rate, squawk, RSSI, NIC/SIL/NACv, category, distance
- Meteorological data (BDS 4,4/4,5): wind, temperature, turbulence
- Altitude/speed chart (Canvas) from trail history
- Doppler data: measured frequency, expected frequency, PPM

### Analytics — Four Tabs

#### Tab 1: Graphs

- **Message rate by DF types** — bar chart, per-chart history
- **Aircraft dynamics by categories** — line chart with tooltip descriptions, per-chart history
- **Message rate timeline** — line chart
- **RSSI timeline** — line chart
- **CRC error rate** — two charts (CRC errors + CRC fixed)
- **Reception range by sector** — Canvas radar with zoom, sector table (azimuth, max range, avg RSSI)

#### Tab 2: Analytics

- **Aircraft type distribution** — bar chart (top-15), per-chart history
- **Country distribution** — bar chart (flags), per-chart history
- **Altitude histogram** — bar chart, per-chart history
- **Speed histogram** — bar chart, per-chart history
- **Unique aircraft over time** — line chart, per-chart history
- **Squawk distribution** — bar chart (7500/7600/7700 highlighted in red), per-chart history
- **ADS-B version statistics** — pie chart (DO-260/A/B), per-chart history

#### Tab 3: Profile

- **Altitude profile** — per-aircraft vertical track (time vs altitude), per-chart history
- **Speed profile** — per-aircraft speed over time
- **Meteo profile** — wind and temperature by altitude (BDS 4,4/4,5), dual-axis line chart, per-chart history

#### Tab 4: Special

- **Doppler detector** — table (ICAO, measured Hz, expected Hz, PPM) + PPM chart, per-chart history
- **Conflict detector** — table (aircraft pairs, distance, altitude diff, heading diff, converging/diverging) + conflict map (Canvas)
- **"Quiet hour" silence map** — silence periods table with badges + visualization
- **Compare two sessions** — side-by-side metric comparison table with diff column
- **Session summary** — current session statistics overview

---

## Per-Chart History Mode

Every chart in the analytics panel supports an individual history mode. Click the gear icon (⚙) on any chart to open the date filter panel.

### Presets

| Pres | Description |
|---|---|
| Session | Full current session data |
| 1 hour | Last 60 minutes |
| Today | From midnight to now |
| Custom | Arbitrary date range (datetime-local pickers) |

When history mode is active, a "HISTORY" badge appears on the chart. Data is sourced from:

- **Server**: `stats_history` snapshots (saved every 5 min, retained 30 days) — for historical dates
- **Browser buffer**: in-memory snapshots (saved every 10 sec, up to 360 points) — for recent data within the session

Click "Live" to return to real-time mode.

---

## Sound Notifications

26 distinct audio alerts for various events. Each can be individually enabled. The decoder generates WAV files programmatically (22050 Hz, 16-bit, mono) with embedded RIFF INFO metadata. Playback uses the first available audio player (`paplay`, `aplay`, or `ffplay`), or can be handed off to the browser via Web Audio API.

### Basic Events (12 types)

| Sound | Config Key | Description |
|---|---|---|
| New aircraft | `enable_sound_new` | First appearance of a new ICAO address |
| Emergency (7700) | `enable_sound_emergency` | General emergency squawk |
| Loss of contact | `enable_sound_loss` | Aircraft signal lost (timeout) |
| TCAS RA | `enable_sound_tcas` | Resolution Advisory from BDS 3,0 |
| Low aircraft | `enable_sound_low_alt` | Aircraft below configured altitude threshold |
| Military | `enable_sound_mil` | Military category detected |
| Tracked | `enable_sound_tracked` | Aircraft now has reliable position |
| Tracked loss | `enable_sound_tracked_loss` | Aircraft lost reliable tracking |
| Range record | `enable_sound_record` | New maximum reception distance |
| Silence | `enable_sound_silence` | No messages for configured period |
| Turbulence | `enable_sound_turb` | BDS 4,5 turbulence report |
| Squawk emergency | `enable_sound_squawk` | Squawk 7500/7600/7700 (distinct tones per code) |

### Extended Events (14 types)

| Sound | Config Key | Description |
|---|---|---|
| Long range | `enable_sound_long_range` | Aircraft beyond long-range threshold |
| Overhead pass | `enable_sound_overhead` | Aircraft directly overhead |
| Speed record | `enable_sound_speed_record` | New maximum speed |
| Altitude record | `enable_sound_alt_record` | New maximum altitude |
| Helicopter | `enable_sound_helicopter` | Rotorcraft detected (rotor sound) |
| Rare type | `enable_sound_rare_type` | Uncommon aircraft type |
| Rapid descent | `enable_sound_rapid_descent` | High vertical descent rate |
| Return | `enable_sound_return` | Aircraft returns after being lost |
| Level-off | `enable_sound_level_off` | Vertical rate approaches zero |
| Ground vehicle | `enable_sound_ground_vehicle` | Surface vehicle category |
| Proximity | `enable_sound_proximity` | Two aircraft within proximity threshold |
| Head-on | `enable_sound_headon` | Converging courses detected |
| Breakthrough | `enable_sound_breakthrough` | Signal breaks through noise floor |
| Squawk change | `enable_sound_squawk_change` | Aircraft changed squawk code |

### Additional Sound Config Keys

These config keys control extended sound behavior (no separate sound type, but enable/disable specific triggers):

| Config Key | Default | Description |
|---|---|---|
| `enable_sound_vrate_record` | `true` | Vertical rate record trigger |
| `enable_sound_bds45` | `true` | BDS 4,5 weather hazard trigger |
| `enable_sound_uat` | `true` | UAT message trigger |
| `enable_sound_mlat` | `true` | MLAT message trigger |
| `enable_sound_df18` | `true` | DF 18 (TIS-B/ADS-R) trigger |
| `enable_sound_version` | `true` | ADS-B version change trigger |
| `enable_sound_sil` | `true` | SIL value change trigger |
| `enable_sound_nacv` | `true` | NACv value change trigger |
| `enable_sound_nic` | `true` | NIC value change trigger |
| `enable_sound_range_record` | `true` | Range record (alias) |
| `enable_sound_stall` | `true` | Speed stall trigger |
| `enable_sound_fuel` | `true` | Fuel-related trigger |

### Sound Thresholds

Sound thresholds are specified in the **display units** you've configured. Use `unit_input_*` keys to tell the decoder which units your thresholds are in:

```json
{
  "unit_input_dist": "km",
  "unit_input_alt": "ft",
  "unit_input_speed": "kt",
  "unit_input_vrate": "fpm",
  "long_range_threshold_km": 300,
  "overhead_threshold_km": 5,
  "speed_record_threshold": 500,
  "proximity_horiz_km": 10
}
```

The decoder converts threshold values from input units to internal units automatically.

---

## Data Export

Seven export formats are available:

| Format | Source | Description |
|---|---|---|
| **CSV** | Browser | Aircraft table (generated client-side) |
| **GZ** | Server | Compressed JSON dump (full session data) |
| **GPX** | Server | GPS format (Garmin, Ozi Explorer) — `GET /export.gpx` |
| **KML** | Server | Google Earth format — `GET /export.kml` |
| **JSON** | Server | Full session statistics |
| **AeroXML** | Server | AeroXML format |
| **FlightAware** | Server | FlightAware format |

### Config Keys

| Setting | Config Key | Default |
|---|---|---|
| Enable CSV | `enable_csv_export` | `true` |
| Enable AeroXML | `enable_aeroxml_export` | `true` |
| Enable FlightAware | `enable_flightaware_export` | `true` |

---

## Session Save & Restore

The decoder can save its state on exit and restore it on the next launch:

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Save on exit | `enable_session_save` | `true` | Save session on exit |
| Restore on launch | `enable_session_restore` | `true` | Restore previous session |
| Session file | `session_file` | `session.json` | Path to session file |
| Max age | `session_max_age` | `3600` | sec — don't restore older sessions |
| Save statistics | `session_save_stats` | `true` | Include statistics in session |
| Save PPM | `session_save_ppm` | `true` | Include calibrated PPM value |

CLI: `--no-session-save`, `--no-session-restore`

---

## Historical Statistics

Periodic snapshots of statistics are saved to disk for long-term analysis:

| Setting | Config Key | Default | Description |
|---|---|---|---|
| Enable | `enable_stats_history` | `true` | Save periodic snapshots |
| Directory | `stats_history_dir` | `stats_history/` | Structure: `YYYY-MM-DD/HH-MM.json` |
| Interval | `stats_history_interval` | `300` | sec (5 min) |
| Retention | `stats_history_retention_days` | `30` | days — older snapshots deleted |

Position history (for chart replay) is stored separately with its own interval and retention.

---

## Aircraft Database & Photos

### OpenSky Aircraft Database

The decoder can look up aircraft metadata from an OpenSky-format CSV file (`aircraft.csv`). Fields available: registration, type code, operator, model, manufacturer, owner, country, engines. Each field can be individually toggled.

Download (see [Third-Party Dependencies](#aircraftcsv--aircraft-database) for details):

```bash
wget https://s3.opensky-network.org/data-samples/metadata/aircraft-database-complete-2025-02.csv -O aircraft.csv
```

> **Warning**: Use the `aircraft-database-complete-*` format, not `aircraftDatabase-*`. The complete version has all 31 columns; the non-complete version is missing columns the decoder needs.

### Aircraft Photos

Photos are fetched from the Planespotters API (`api.planespotters.net`) by ICAO hex address or registration. Results are cached on disk for 7 days (positive) and 1 hour (negative — no photo found). A descriptive `User-Agent` header is sent automatically.

---

## Configuration System

dx1090 uses a **three-tier configuration system**. Settings are loaded in order, with each tier overriding the previous one for any parameter it specifies:

```
  ┌─────────────────────────────────────────────────────┐
  │  3. CLI flags        (highest priority)             │
  │  Overrides everything for the parameters it sets     │
  ├─────────────────────────────────────────────────────┤
  │  2. config.json      (overrides hardcoded defaults)  │
  │  + dashconf.json     (overrides units/map only)      │
  ├─────────────────────────────────────────────────────┤
  │  1. Source code      (always available)              │
  │  Hardcoded defaults with full comments               │
  └─────────────────────────────────────────────────────┘
```

### Tier 1 — Built-in Defaults

Every setting is defined as a constant at the top of `dx1090.py` with detailed comments — description, valid range, and default value. The program runs out of the box with zero configuration. Open the source file and scroll through the configuration sections to see all available parameters.

### Tier 2 — config.json

If a `config.json` file exists next to the script (or at a path specified with `--config FILE`), the decoder loads it on startup and overrides the hardcoded defaults with the values found in the file. Only the keys you include are overridden — everything else stays at the built-in defaults.

**The repository ships with `config1.json`** — a ready-to-use example configuration file. To activate it, simply rename or copy it:

```bash
cp config1.json config.json
```

Then edit `config.json` with your favorite text editor. A minimal config only needs your receiver coordinates:

```json
{
  "home_lat": 46.856614,
  "home_lon": 2.213749
}
```

You can also specify a custom path at runtime:

```bash
python3 dx1090.py --config /path/to/my_config.json
```

The decoder automatically handles type conversion for config values (bool, int, float, string, set) and validates them. Unit conversion is also supported — you can specify distances in km/mi/nm, altitudes in ft/m, speeds in kt/kmh/mph/ms, and vertical rates in fpm/ms using the `unit_input_*` keys:

```json
{
  "unit_input_dist": "km",
  "unit_input_alt": "m",
  "unit_input_speed": "kmh",
  "unit_input_vrate": "ms",
  "filter_radius": 350,
  "long_range_threshold_km": 300
}
```

The decoder converts threshold values from the specified input units to internal units automatically.

#### Config Key Groups

All config keys map to constants in the source code. Here's a summary by group:

**SDR / Receiver:** `sdr_type`, `freq`, `sample_rate`, `gain`, `ppm_correction`, `chunk_bytes`, `min_signal_level`

**UAT:** `enable_uat`, `uat_freq`, `uat_sample_rate`, `uat_gain`, `uat_ppm`

**Modes:** `enable_mode_s`, `enable_mode_ac`

**Adaptive Gain:** `adaptive_gain`, `gain_check_interval`, `gain_min_msg_rate`, `gain_good_msg_rate`, `gain_steps`

**Phase Correction:** `enable_phase_correction`

**RSSI / Reliability:** `enable_rssi`, `position_reliable_min`, `position_reliable_max`

**Coordinates:** `home_lat`, `home_lon`

**Geofilter:** `filter_radius`, `filter_radius_unit`, `filter_no_position`, `filter_remove_out_of_range`

**Units:** `unit_alt`, `unit_speed`, `unit_distance`, `unit_vrate`, `unit_pressure`, `unit_temp`

**Input Units:** `unit_input_dist`, `unit_input_alt`, `unit_input_speed`, `unit_input_vrate`

**Map:** `map_mode`, `tile_cache_dir`

**Data Sources:** `speed_source`, `vrate_source`

**Signal Processing:** `enable_dc_removal`, `enable_noise_floor`, `noise_floor_alpha`, `enable_pulse_blanking`, `pulse_blanking_ratio`

**MLAT:** `enable_mlat_timestamps`, `mlat_timestamp_precision_us`

**Doppler / PPM:** `enable_doppler`, `doppler_min_samples`, `doppler_min_interval`, `enable_ppm_auto`, `ppm_auto_max_pos_age`, `ppm_auto_restart_sdr`

**Filters / Smoothing:** `max_position_jump_km`, `max_range_km`, `enable_smoothing`, `smoothing_alpha`, `enable_dead_reckoning`, `dr_max_age`, `stale_timeout`, `loss_timeout`, `cpr_timeout`, `max_alt_rate_fpm`

**CRC:** `enable_crc_correction`, `enable_crc_two_bit`, `crc_two_bit_dynamic`, `crc_two_bit_time_ratio`, `crc_two_bit_max_decode_time`, `crc_two_bit_time_alpha`

**BDS:** `enable_bds`, `enable_bds44`, `enable_bds45`, `enable_bds50`, `enable_bds60`

**Network:** `enable_beast_out`, `beast_out_host`, `beast_out_port`, `enable_json_out`, `json_out_host`, `json_out_port`, `enable_sbs1_out`, `sbs1_out_host`, `sbs1_out_port`

**Watchdog:** `enable_sdr_watchdog`, `sdr_watchdog_timeout`, `sdr_watchdog_max_retries`

**Decode Thread:** `enable_decode_thread`, `decode_queue_size`

**Sessions:** `enable_session_save`, `enable_session_restore`, `session_file`, `session_max_age`, `session_save_stats`, `session_save_ppm`

**Stats History:** `enable_stats_history`, `stats_history_dir`, `stats_history_interval`, `stats_history_retention_days`

**Aircraft DB:** `enable_aircraft_lookup`, `aircraft_db_file`, `lookup_registration`, `lookup_typecode`, `lookup_operator`, `lookup_model`, `lookup_manufacturer`, `lookup_owner`, `lookup_country`, `lookup_engines`

**Export:** `enable_csv_export`, `enable_aeroxml_export`, `enable_flightaware_export`

**Table:** `col_show`, `sort_by`, `filter_emergency_only`

**Sound (master):** `enable_sound`

**Sound types (26):** `enable_sound_new`, `enable_sound_emergency`, `enable_sound_loss`, `enable_sound_tcas`, `enable_sound_low_alt`, `enable_sound_mil`, `enable_sound_tracked`, `enable_sound_tracked_loss`, `enable_sound_record`, `enable_sound_silence`, `enable_sound_turb`, `enable_sound_squawk`, `enable_sound_long_range`, `enable_sound_overhead`, `enable_sound_speed_record`, `enable_sound_alt_record`, `enable_sound_helicopter`, `enable_sound_rare_type`, `enable_sound_rapid_descent`, `enable_sound_return`, `enable_sound_level_off`, `enable_sound_ground_vehicle`, `enable_sound_proximity`, `enable_sound_headon`, `enable_sound_breakthrough`, `enable_sound_squawk_change`

**Sound triggers (extended):** `enable_sound_vrate_record`, `enable_sound_bds45`, `enable_sound_uat`, `enable_sound_mlat`, `enable_sound_df18`, `enable_sound_version`, `enable_sound_sil`, `enable_sound_nacv`, `enable_sound_nic`, `enable_sound_range_record`, `enable_sound_stall`, `enable_sound_fuel`

**Sound thresholds:** `long_range_threshold_km`, `overhead_threshold_km`, `speed_record_threshold`, `proximity_horiz_km`, `low_alt_threshold_ft`

**Logging:** `log_dir`, `log_mode`

**Trails:** `trail_max_points`

### Tier 2b — dashconf.json (optional)

A separate `dashconf.json` file can be used to override dashboard display settings independently of `config.json`. This file is loaded **after** `config.json` and overrides only these keys:

| Key | Type | Default | Description |
|---|---|---|---|
| `unit_distance` | string | `km` | `km`, `mi`, `nm` |
| `unit_alt` | string | `ft` | `ft`, `m` |
| `unit_speed` | string | `kt` | `kt`, `kmh`, `mph`, `ms` |
| `unit_vrate` | string | `fpm` | `fpm`, `ms` |
| `unit_temp` | string | `C` | `C`, `F` |
| `map_mode` | string | `auto` | `leaflet`, `canvas`, `auto` |

Example `dashconf.json`:

```json
{
  "unit_distance": "nm",
  "unit_alt": "ft",
  "unit_speed": "kt",
  "unit_vrate": "fpm",
  "unit_temp": "C",
  "map_mode": "leaflet"
}
```

**Why a separate file?** When you change units in the dashboard Settings panel, the decoder saves them to `dashconf.json` — not `config.json`. This keeps your main config clean and lets different users on the same machine have different dashboard preferences without touching the decoder config.

### Tier 3 — CLI Flags (highest priority)

Command-line flags override both `config.json` and the built-in defaults. Most flags are toggle-style (no value required) — they only affect the parameters they explicitly control, and everything else falls back to `config.json` or the defaults. One flag accepts a value: `--config FILE` (path to a configuration file).

See the [CLI Reference](#cli-reference) section for the full list of flags.

### Priority Summary

```
CLI flag ──> config.json ──> dashconf.json* ──> source code defaults
             │                                    │
             └── overrides source code ───────────┘

* dashconf.json overrides config.json ONLY for:
  unit_distance, unit_alt, unit_speed, unit_vrate, unit_temp, map_mode
```

### Runtime Reload

No restart needed — edit `config.json` and send a signal to reload the configuration at runtime without interrupting reception:

| Platform | Signal | Effect |
|---|---|---|
| Linux / macOS | `SIGUSR1` | Reload `config.json` |
| Windows | `SIGTERM` | Reload `config.json` |

Example:

```bash
kill -USR1 $(pgrep -f dx1090.py)
```

Refer to the source code comments for the full list of configurable parameters — every constant has a description, valid range, and default value.

---

## CLI Reference

```
usage: dx1090.py [-h] [--airspy] [--hackrf] [--bladerf]
                [--mode-ac] [--uat] [--adaptive-gain] [--no-watchdog]
                [--no-crc-correction] [--no-crc-two-bit]
                [--no-phase-correction] [--no-dead-reckoning] [--no-smoothing]
                [--no-ppm-auto] [--beast] [--sbs1]
                [--no-session-save] [--no-session-restore]
                [--no-stats-history]
                [--no-sound] [--no-event-log] [--emergency-only]
                [--quiet] [--debug] [--config FILE] [--version]
```

### SDR / Receiver

| Flag | Description |
|---|---|
| `--airspy` | Use Airspy R2/Mini |
| `--hackrf` | Use HackRF One |
| `--bladerf` | Use bladeRF 2.0 micro |
| `--mode-ac` | Enable Mode A/C decoding |
| `--uat` | Enable UAT 978 MHz decoding |
| `--adaptive-gain` | Enable adaptive gain adjustment |
| `--no-watchdog` | Disable SDR auto-restart on crash |

### Decoding

| Flag | Description |
|---|---|
| `--no-crc-correction` | Disable 1-bit CRC correction |
| `--no-crc-two-bit` | Disable 2-bit CRC correction |
| `--no-phase-correction` | Disable bit position phase correction |
| `--no-dead-reckoning` | Disable position extrapolation (DR) |
| `--no-smoothing` | Disable position smoothing |

### PPM

| Flag | Description |
|---|---|
| `--no-ppm-auto` | Disable auto-PPM from aircraft positions |

### Network

| Flag | Description |
|---|---|
| `--beast` | Enable Beast output (dump1090 / piaware) |
| `--sbs1` | Enable SBS1 output (BaseStation / VRS) |

### Sessions and History

| Flag | Description |
|---|---|
| `--no-session-save` | Do not save session on exit |
| `--no-session-restore` | Do not restore session on launch |
| `--no-stats-history` | Disable statistics history saving |

### UI / Sound

| Flag | Description |
|---|---|
| `--no-sound` | Fully disable sound notifications |
| `--no-event-log` | Disable event log on screen |
| `--emergency-only` | Show emergency aircraft only (7500/7600/7700) |
| `--quiet` | Minimal output (errors and stats only) |
| `--debug` | Debug output to stderr |

### System

| Flag | Description |
|---|---|
| `--config FILE` | Path to config file (default: config.json) |
| `--version` | Show version and exit |
| `-h`, `--help` | Show help and exit |

### Example Commands

```bash
# Basic run with RTL-SDR (uses config.json or defaults)
python3 dx1090.py

# Use Airspy receiver
python3 dx1090.py --airspy

# Enable Beast output to piaware on default port
python3 dx1090.py --beast

# Emergency-only mode, no sound, minimal output
python3 dx1090.py --emergency-only --no-sound --quiet

# Enable Mode A/C and UAT alongside Mode S
python3 dx1090.py --mode-ac --uat

# Use a custom config file
python3 dx1090.py --config /path/to/my_config.json

# Disable 2-bit CRC correction for maximum speed
python3 dx1090.py --no-crc-two-bit

# Enable adaptive gain with Beast and SBS1 output
python3 dx1090.py --adaptive-gain --beast --sbs1
```

---

## Runtime Controls

| Action | Effect |
|---|---|
| `Ctrl+C` | Exit (saves session if enabled) |
| `SIGUSR1` | Reload configuration from `config.json` (Linux/macOS) |
| `SIGTERM` | Reload configuration from `config.json` (Windows) |

---

## Directory Structure

```
dx1090/
├── dx1090.py              # Main decoder script (single file)
├── dashboard.html         # Web dashboard HTML (served at /)
├── config1.json           # Example configuration (rename to config.json)
├── config.json            # Your configuration (created by you)
├── dashconf.json          # Dashboard settings (auto-created by UI)
├── aircraft.csv           # OpenSky aircraft database (downloaded separately)
├── session.json           # Saved session state (auto-created)
├── static/                # Dashboard libraries (bundled)
│   ├── leaflet.js         # Leaflet 1.9.4
│   ├── leaflet.css        # Leaflet styles
│   └── chart.umd.min.js   # Chart.js 4.4.1
├── sounds/                # Generated WAV files (auto-created)
│   ├── new.wav
│   ├── emergency.wav
│   ├── loss.wav
│   ├── tcas.wav
│   └── ... (26 types)
├── logs/                  # Log files
│   └── dx1090.log          # Rotating log (5 MB, 3 backups)
├── stats_history/         # Historical statistics snapshots
│   └── YYYY-MM-DD/
│       └── HH-MM.json
├── pos_history/          # Position history (for chart replay)
├── tile_cache/           # OSM tile cache (auto-created)
└── photo_cache/          # Aircraft photo cache (auto-created)
```

---

## Project Status

**This project is under active development.**

The following areas are being worked on or are planned for future releases:

- Improved UAT message support (weather uplinks, FIS-B)
- MLAT client support for position triangulation from multiple receivers
- Web dashboard performance optimizations for large aircraft counts
- Additional export formats (e.g. SBS1 replay, APRS)
- Enhanced Doppler analysis with multi-pass averaging
- Web-based configuration editor
- Docker container support
- Automated testing and CI/CD pipeline
- Documentation for API integration (third-party apps)

Bug reports, feature requests, and contributions are welcome via GitHub Issues and Pull Requests.

---

## License

Copyright (c) 2026-present, R4SCS — https://github.com/r4scs

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

---

## Acknowledgements

This project builds upon the work of many open-source projects and organizations:

- [OpenSky Network](https://opensky-network.org/) — aircraft database (CC-BY-SA 4.0)
- [Planespotters](https://www.planespotters.net/) — aircraft photo API
- [OpenStreetMap](https://www.openstreetmap.org/) — map tiles (ODbL)
- [Leaflet](https://leafletjs.com/) — JavaScript map library (BSD-2-Clause)
- [Chart.js](https://www.chartjs.org/) — JavaScript charting library (MIT)
- [RTL-SDR](https://osmocom.org/projects/rtl-sdr) — RTL2832U driver (GPL-2.0)
- [Airspy](https://airspy.com/) — Airspy SDR tools (BSD-3-Clause)
- [HackRF](https://greatscottgadgets.com/hackrf/) — HackRF SDR tools (GPL-2.0)
- [bladeRF](https://www.nuand.com/) — bladeRF SDR tools (GPL-2.0)
- [PulseAudio](https://gitlab.freedesktop.org/pulseaudio/pulseaudio) — audio playback (GPL-2.0)
- [ALSA](https://www.alsa-project.org/) — audio playback (GPL-2.0)
- [FFmpeg](https://ffmpeg.org/) — audio playback (GPL-2.0+)

ADS-B and Mode S specifications are defined in ICAO Annex 10, Volume III, Chapter 9.
