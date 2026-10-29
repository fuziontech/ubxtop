# `ubxtop` — GNSS & u-blox Hardware Diagnostic TUI

[![License: MIT](https://img.shields.io/badge/License-MIT-cyan.svg)](LICENSE)
[![Python 3.9+](https://img.shields.io/badge/python-3.9%2B-blue.svg)](https://www.python.org/)
[![Zero Dependencies](https://img.shields.io/badge/dependencies-zero-brightgreen.svg)](#installation)

**`ubxtop`** is a zero-dependency, 24-bit TrueColor terminal diagnostic suite for **u-blox** and **NMEA 0183** GNSS receivers.

Built with a pure-ANSI cell-grid engine and **DEC Mode 2026 Synchronized Output** (designed for modern GPU terminals like **Ghostty**, Kitty, WezTerm, and iTerm2), `ubxtop` simultaneously demultiplexes ASCII **NMEA 0183** sentences and binary **u-blox UBX** frames to expose everything happening inside your GNSS receiver—from satellite orbital physics and pseudorange residuals to quartz oscillator drift and baseband I/Q mixer balance.

```text
╔══════════════════════════════════════════════════════════════════════════════════════════════╗
║ ubxtop — GNSS & u-blox HARDWARE DIAGNOSTIC SNAPSHOT                                         ║
╚══════════════════════════════════════════════════════════════════════════════════════════════╝
  Device & Link  : /dev/cu.usbmodem2101 (u-blox GNSS Receiver / u-blox AG)
  Silicon / FW   : UBX-M8130 (00080000) │ Core: ROM CORE 3.01 (107888) │ FW: SPG 3.01 │ UBX Prot: 18.00
  Constellations : GPS;GLO;BDS;QZSS │ Tracking Channels: 32/32
  Navigation Fix : 3D FIX │ SVs Used: 12 / 21 in view │ TTFF: 1.42s
  UTC Timestamp  : 2026-10-28 22:37:49.999509527 UTC (tAcc: ±11 ns)
  GPS Time & Clk : Week 2442 │ iTOW 340688.000s │ Leap +18s │ Bias +490,472 ns │ Drift +1210 ns/s (ppb)
  WGS84 Lat/Lon  : +37.9874582°, -122.5780629°  (37°59'14.85"N, 122°34'41.03"W)
  Grid / Geohash : Maidenhead: CM87rx06 │ Geohash: 9qbb4emku │ UTM: 10S 537051.3E 4204507.5N
  Altitudes & Acc: MSL: 22.21m │ HAE: -7.39m │ Geoid: -29.60m │ hAcc: ±0.85m │ vAcc: ±1.33m
  3D ECEF (XYZ)  : X=-2,710,160.75m  Y=-4,241,337.13m  Z=3,904,342.34m  (3D pAcc: ±1.58m)
  7-Axis DOP     : GDOP=1.62  PDOP=1.47  HDOP=0.77  VDOP=1.25  TDOP=0.70  NDOP=0.55  EDOP=0.54
  RF & Baseband  : ANT=OK (ON) │ AGC=48.6% │ Noise=101/ms │ Jam=29/255 │ I=(+6,151) Q=(+49,139)
  Sidereal & Sun : LMST 16:56:39.5 │ Solar Elevation +25.9° Azimuth 225.1° │ CEP50=0.05m R95=0.11m

  POLAR SKYPLOT (N=0°, Zenith=Center)     SV   CONST USED  C/N0         ELEV  AZIM   PR_RES   SLANT_RNG   DELAY
  │                ⣀⣀N⣀⡀               │  B20  BDS   ●YES  36 █████━━━   31°  308°   +0.6m   24,078km  80.32ms
  │           ⣀⠤⠒⠉·B09 ⠈⠉⠑⠢⢄⡀          │  G21  GPS   ●YES  36 █████━━━   20°  249°   -1.2m   23,688km  79.02ms
  │         ⣠⠚⠁ ·B06⣀⣂⣀⣀    ⠙⢢⡀        │  G09  GPS   ●YES  34 █████━━━   24°  158°   +3.2m   23,314km  77.77ms
  │        ⡜⠁  ●B12  ⡂  ⠉⠲⣀   ⠙⡄       │  B23  BDS   ●YES  32 █████━━━   67°   24°   -6.2m   21,923km  73.13ms
  │       ⡜   ⡠●B20⣀⡠⡦⣀⡀   ●G08⠘⡄      │  G08  GPS   ●YES  28 ████━━━━   30°   53°  +12.2m   22,786km  76.01ms
  │      ⢠⠃ ●G05 ⢠⠊  ●●B23  ⢣●B25      │  B28  BDS   ●YES  27 ████━━━━    8°  161°   -6.5m   26,290km  87.69ms
  │W     ⢸⠠⠠⠠⢸⠠⠠⠠⡮⠠⠠⠠⡢⠠⠠⠨●G07⠠⠠⠠•G02  E│  B25  BDS   ●YES  26 ████━━━━   25°   67°  -23.0m   24,602km  82.06ms
  │      ⠸·B04⡀  ⠸●G14  ⡸   ⡸   ⡸      │  G22  GPS   ●YES  26 ████━━━━   45°  242°  -15.9m   21,661km  72.25ms
  │       ⢇  ●G21G22⠢⡦⠒⠊   ⡠⠃  ⢀⠇      │  G14  GPS   ●YES  25 ████━━━━   66°  259°   -1.1m   20,604km  68.73ms
  │       ⠈⢆   *SUN  ⡂   ⣠⠒⠁  ⢀⠎       │  B12  BDS   ●YES  24 ████━━━━   15°  320°   -6.5m   25,563km  85.27ms
  │        ⠈⠳⣀   ⠈⠑⠒⠒⡒⠒⠒●G09B13        │  G05  GPS   ●YES  21 ███━━━━━   17°  283°   -1.6m   23,980km  79.99ms
  │          ⠈⠓⠤⣀    ●G17B28⠋          │  G07  GPS   ●YES  21 ███━━━━━   56°   75°  +22.5m   21,029km  70.15ms
  │              ⠉·G19⠒⠊⠉⠁             │  B13  BDS   ●YES  19 ███━━━━━   19°  135°  -13.6m   39,650km  132.26ms
```

---

## Features

- **5 Interactive Diagnostic Views**:
  1. **`[1] Overview (Mission Control)`**:
     - **Geodesy & 3D ECEF**: WGS84 Lat/Lon (Decimal, DMS, NMEA DMM), **Maidenhead Ham Grid Locator** (8-char), **Geohash** (9-char), **UTM** Zone/Easting/Northing, **3D ECEF ($X, Y, Z$)** coordinates & velocities, MSL vs. WGS84 Ellipsoid Height (HAE) & Geoid Separation, NED velocities, Local Mean Sidereal Time (LMST), and Solar Elevation/Azimuth.
     - **High-Res Braille Polar Skyplot**: $360^\circ$ azimuth / $0^\circ\text{–}90^\circ$ elevation radar (`2×4` subpixel Unicode Braille) showing active navigation SVs (`●`), tracked SVs (`•`), and the Sun (`*SUN`).
     - **C/N0 Signal Spectrum**: Per-SV carrier-to-noise (`dB-Hz`) bars, elevation, azimuth, pseudorange residual, slant range, and per-constellation channel usage.
     - **7-Axis DOP & Precision Oscillator Timing**: `GDOP`, `PDOP`, `HDOP`, `VDOP`, `TDOP`, `NDOP`, `EDOP`, GPS Week, `iTOW`, Leap Seconds, nanosecond UTC accuracy (`±tAcc`), receiver quartz **clock bias (`ns`)**, and **clock drift (`ns/s` / ppb)**.
     - **RF Frontend & Jamming Monitor**: Antenna Supervisor (`OK`/`SHORT`/`OPEN`), AGC gain %, GPS core noise (`counts/ms`), **Continuous-Wave (CW) Jamming Indicator (`0–255`)**, **Baseband I/Q Mixer offset & magnitude**, and anti-spoofing status.
  2. **`[2] Satellites & Orbits` (`UBX-NAV-SAT` + `UBX-NAV-ORB`)**:
     - Full table of all satellites in view with signal lock quality (`Code+Carr`, `Code Lock`, `Acquired`, `Searching`), `C/N0`, `Elev`, `Azim`, **Pseudorange Residual (`prRes` in meters)**, geometric **Slant Range ($\text{km}$)**, **Speed-of-Light Propagation Delay ($\text{ms}$)**, Orbit Source (`Ephemeris`, `Almanac`, `AssistNow`), Carrier Smoothing (`SMTH`), and Health flags.
  3. **`[3] Scatter & Drift`**:
     - Live 2D East-North-Up (ENU) subpixel Braille position scatter plot with real-time **CEP50** (50% Circular Error Probable), **R95**, **2DRMS**, standard deviations ($\sigma_E, \sigma_N, \sigma_U$), and 4 live Braille time-series charts (`Altitude MSL`, `hAcc`, `Mean C/N0`, `PDOP`).
  4. **`[4] RF, Clock & Hardware`**:
     - Low-level silicon diagnostics: Baseband I/Q mixer balance (`UBX-MON-HW2`), 32-channel hardware constellation allocation (`UBX-CFG-GNSS`), per-port hardware I/O byte & framing/parity/overrun error counters across `I2C`, `UART1`, `UART2`, `USB`, and `SPI` (`UBX-MON-IO`), and ROM/firmware extensions (`UBX-MON-VER`).
     - Live interactive controls to change measurement rate (`1 / 2 / 5 / 10 Hz` via `UBX-CFG-RATE`) and navigation dynamic platform model (`Portable`, `Stationary`, `Pedestrian`, `Automotive`, `Sea`, `Airborne` via `UBX-CFG-NAV5`).
  5. **`[5] Protocol Sniffer`**:
     - Real-time packet inspector with per-message counters, age, checksum error tracking, protocol filtering (`ALL` / `NMEA` / `UBX`), stream freeze/pause, and disk logging.

---

## Installation & Quick Start

`ubxtop` uses only the Python 3 standard library—**zero external packages required**.

### Run Directly with `uvx` or `pipx`
```bash
uvx --from git+https://github.com/fuziontech/ubxtop ubxtop
```

### Or Clone and Run
```bash
git clone https://github.com/fuziontech/ubxtop.git
cd ubxtop

# Launch interactive full-screen TUI (auto-detects USB GPS on macOS & Linux):
./ubxtop

# Print a 1-shot colorized CLI diagnostic snapshot and exit:
./ubxtop --snapshot

# Run with simulated u-blox M8 telemetry (no hardware required):
./ubxtop --demo

# Specify a custom serial port:
./ubxtop -d /dev/cu.usbmodem2101
```

---

## Keyboard Shortcuts

| Key | Action |
|---|---|
| `1` – `5` / `Tab` / `←` `→` | Switch between the 5 diagnostic views |
| `s` | Cycle satellite sort (`Used + C/N0`, `Constellation/PRN`, `Elevation`, `\|Pseudorange Residual\|`) |
| `c` | Clear 2D ENU position scatter & CEP/R95 drift history |
| `a` | Toggle scatter reference between running Mean Centroid and Locked Anchor |
| `r` | Cycle receiver measurement rate (`1 Hz` / `2 Hz` / `5 Hz` / `10 Hz` via `UBX-CFG-RATE`) |
| `m` | Cycle navigation dynamic model (`Portable`, `Stationary`, `Pedestrian`, `Automotive`, `Sea`, `Airborne` via `UBX-CFG-NAV5`) |
| `f` | Cycle Protocol Sniffer filter (`ALL`, `NMEA`, `UBX`) |
| `Space` / `p` | Pause / resume live packet stream in Protocol Sniffer |
| `l` | Toggle packet logging to timestamped `gps_diag_*.log` file |
| `v` | Re-poll all UBX hardware & configuration registers |
| `q` / `Esc` | Quit |

---

## Supported Protocols & Messages

| Protocol | Decoded Messages |
|---|---|
| **u-blox Binary (`UBX`)** | `NAV-PVT` (`0x01 0x07`), `NAV-DOP` (`0x01 0x04`), `NAV-SAT` (`0x01 0x35`), `NAV-ORB` (`0x01 0x34`), `NAV-CLOCK` (`0x01 0x22`), `NAV-STATUS` (`0x01 0x03`), `NAV-TIMEGPS` (`0x01 0x20`), `NAV-POSECEF` (`0x01 0x01`), `NAV-VELECEF` (`0x01 0x11`), `NAV-SBAS` (`0x01 0x32`), `NAV-AOPSTATUS` (`0x01 0x60`), `MON-VER` (`0x0A 0x04`), `MON-HW` (`0x0A 0x09`), `MON-HW2` (`0x0A 0x0B`), `MON-IO` (`0x0A 0x02`), `MON-GNSS` (`0x0A 0x28`), `CFG-NAV5` (`0x06 0x24`), `CFG-RATE` (`0x06 0x08`), `CFG-GNSS` (`0x06 0x3E`), `ACK-ACK` / `ACK-NAK` |
| **NMEA 0183 (v4.10)** | `$GNRMC`, `$GNGGA`, `$GNGSA`, `$GPGSV` / `$GLGSV` / `$GAGSV` / `$GBGSV` / `$GQGSV`, `$GNVTG`, `$GNGLL`, `$GNTXT` |

---

## License

Released under the [MIT License](LICENSE).
