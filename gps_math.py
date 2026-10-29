"""Geodesy, orbital physics, astronomy, and statistical accuracy calculations for GPS diagnostics."""

from __future__ import annotations
import math
from typing import List, Tuple, Optional


# WGS84 ellipsoid constants
WGS84_A = 6378137.0               # Semi-major axis (m)
WGS84_F = 1.0 / 298.257223563     # Flattening
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # Eccentricity squared
SPEED_OF_LIGHT = 299792458.0      # m/s
EARTH_MEAN_RADIUS = 6371008.8     # m

# Nominal orbital altitudes above Earth surface (meters) by constellation
CONSTELLATION_ALTITUDE_M = {
    "GPS": 20180_000.0,
    "GLO": 19130_000.0,
    "GAL": 23222_000.0,
    "BDS": 21528_000.0,   # MEO default (GEO/IGSO handled by SV ID)
    "SBAS": 35786_000.0,  # Geostationary
    "QZSS": 35786_000.0,  # Quasi-zenith (~32,600 - 39,000 km)
    "IMES": 0.0,          # Indoor Messaging System
}


def format_dms(deg: float, is_lat: bool = True) -> str:
    """Format decimal degrees into compact Degrees°Minutes'Seconds.ss\"H."""
    if math.isnan(deg):
        return "---"
    hemi = ("N" if deg >= 0 else "S") if is_lat else ("E" if deg >= 0 else "W")
    val = abs(deg)
    d = int(val)
    m_full = (val - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    width = 2 if is_lat else 3
    return f"{d:0{width}d}°{m:02d}'{s:05.2f}\"{hemi}"


def format_dmm(deg: float, is_lat: bool = True) -> str:
    """Format decimal degrees into NMEA-style DDD°MM.MMMMM'H."""
    if math.isnan(deg):
        return "---"
    hemi = ("N" if deg >= 0 else "S") if is_lat else ("E" if deg >= 0 else "W")
    val = abs(deg)
    d = int(val)
    m = (val - d) * 60.0
    width = 2 if is_lat else 3
    return f"{d:0{width}d}°{m:08.5f}'{hemi}"


def latlon_to_maidenhead(lat: float, lon: float, precision: int = 8) -> str:
    """Convert WGS84 latitude/longitude to a Maidenhead Grid Locator (e.g. CM87px36)."""
    if math.isnan(lat) or math.isnan(lon):
        return "---"
    adj_lon = (lon + 180.0) % 360.0
    adj_lat = max(0.0, min(179.999999, lat + 90.0))

    # 1st pair: Field (A-R, 20° lon x 10° lat)
    f_lon = int(adj_lon // 20)
    f_lat = int(adj_lat // 10)
    grid = chr(ord("A") + f_lon) + chr(ord("A") + f_lat)

    # 2nd pair: Square (0-9, 2° lon x 1° lat)
    rem_lon = adj_lon - f_lon * 20
    rem_lat = adj_lat - f_lat * 10
    s_lon = int(rem_lon // 2)
    s_lat = int(rem_lat // 1)
    grid += f"{s_lon}{s_lat}"

    if precision <= 4:
        return grid

    # 3rd pair: Subsquare (a-x, 5' lon x 2.5' lat)
    rem_lon = (rem_lon - s_lon * 2) * 12.0
    rem_lat = (rem_lat - s_lat * 1) * 24.0
    sub_lon = min(23, int(rem_lon))
    sub_lat = min(23, int(rem_lat))
    grid += chr(ord("a") + sub_lon) + chr(ord("a") + sub_lat)

    if precision <= 6:
        return grid

    # 4th pair: Extended square (0-9, 30" lon x 15" lat)
    rem_lon = (rem_lon - sub_lon) * 10.0
    rem_lat = (rem_lat - sub_lat) * 10.0
    ext_lon = min(9, int(rem_lon))
    ext_lat = min(9, int(rem_lat))
    grid += f"{ext_lon}{ext_lat}"

    if precision >= 10:
        rem_lon = (rem_lon - ext_lon) * 24.0
        rem_lat = (rem_lat - ext_lat) * 24.0
        ss_lon = min(23, int(rem_lon))
        ss_lat = min(23, int(rem_lat))
        grid += chr(ord("a") + ss_lon) + chr(ord("a") + ss_lat)

    return grid


_GEOHASH_BASE32 = "0123456789bcdefghjkmnpqrstuvwxyz"


def latlon_to_geohash(lat: float, lon: float, precision: int = 9) -> str:
    """Encode WGS84 latitude/longitude into a Geohash string."""
    if math.isnan(lat) or math.isnan(lon):
        return "---"
    lat_interval = [-90.0, 90.0]
    lon_interval = [-180.0, 180.0]
    geohash = []
    bits = [16, 8, 4, 2, 1]
    bit = 0
    ch = 0
    even = True
    while len(geohash) < precision:
        if even:
            mid = (lon_interval[0] + lon_interval[1]) / 2.0
            if lon > mid:
                ch |= bits[bit]
                lon_interval[0] = mid
            else:
                lon_interval[1] = mid
        else:
            mid = (lat_interval[0] + lat_interval[1]) / 2.0
            if lat > mid:
                ch |= bits[bit]
                lat_interval[0] = mid
            else:
                lat_interval[1] = mid
        even = not even
        if bit < 4:
            bit += 1
        else:
            geohash.append(_GEOHASH_BASE32[ch])
            bit = 0
            ch = 0
    return "".join(geohash)


def latlon_to_utm(lat: float, lon: float) -> Tuple[int, str, float, float]:
    """Convert WGS84 lat/lon to UTM (zone_number, zone_letter, easting_m, northing_m)."""
    if math.isnan(lat) or math.isnan(lon) or lat < -80.0 or lat > 84.0:
        return (0, "?", 0.0, 0.0)

    zone_number = int((lon + 180.0) // 6) + 1
    letters = "CDEFGHJKLMNPQRSTUVWXX"
    idx = min(len(letters) - 1, max(0, int((lat + 80.0) // 8)))
    zone_letter = letters[idx]

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lon_origin = (zone_number - 1) * 6 - 180 + 3
    lon_origin_rad = math.radians(lon_origin)

    k0 = 0.9996
    e2 = WGS84_E2
    ep2 = e2 / (1.0 - e2)

    N = WGS84_A / math.sqrt(1.0 - e2 * math.sin(lat_rad) ** 2)
    T = math.tan(lat_rad) ** 2
    C = ep2 * math.cos(lat_rad) ** 2
    A = math.cos(lat_rad) * (lon_rad - lon_origin_rad)

    M = WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e2 ** 2 / 64.0 - 5.0 * e2 ** 3 / 256.0) * lat_rad
        - (3.0 * e2 / 8.0 + 3.0 * e2 ** 2 / 32.0 + 45.0 * e2 ** 3 / 1024.0) * math.sin(2.0 * lat_rad)
        + (15.0 * e2 ** 2 / 256.0 + 45.0 * e2 ** 3 / 1024.0) * math.sin(4.0 * lat_rad)
        - (35.0 * e2 ** 3 / 3072.0) * math.sin(6.0 * lat_rad)
    )

    easting = (
        k0
        * N
        * (
            A
            + (1.0 - T + C) * A ** 3 / 6.0
            + (5.0 - 18.0 * T + T ** 2 + 72.0 * C - 58.0 * ep2) * A ** 5 / 120.0
        )
        + 500000.0
    )

    northing = k0 * (
        M
        + N
        * math.tan(lat_rad)
        * (
            A ** 2 / 2.0
            + (5.0 - T + 9.0 * C + 4.0 * C ** 2) * A ** 4 / 24.0
            + (61.0 - 58.0 * T + T ** 2 + 600.0 * C - 330.0 * ep2) * A ** 6 / 720.0
        )
    )
    if lat < 0:
        northing += 10000000.0

    return (zone_number, zone_letter, easting, northing)


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_ellipsoid_m: float) -> Tuple[float, float, float]:
    """Convert WGS84 Lat/Lon/Height-above-ellipsoid to ECEF X, Y, Z in meters."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (N + alt_ellipsoid_m) * cos_lat * math.cos(lon)
    y = (N + alt_ellipsoid_m) * cos_lat * math.sin(lon)
    z = (N * (1.0 - WGS84_E2) + alt_ellipsoid_m) * sin_lat
    return (x, y, z)


def sat_slant_range_and_delay(constellation: str, sv_id: int, elev_deg: float) -> Tuple[float, float]:
    """
    Estimate satellite slant range (km) and signal propagation time-of-flight (ms)
    from nominal constellation orbital altitude and elevation angle.
    """
    if elev_deg is None or elev_deg < -5:
        return (0.0, 0.0)
    alt_m = CONSTELLATION_ALTITUDE_M.get(constellation, 20180_000.0)
    # BeiDou GEO satellites are SV 1..5 and 59..63; IGSO are 6..10, 13, 16, 38..40
    if constellation == "BDS" and (sv_id <= 10 or sv_id in (13, 16, 38, 39, 40) or sv_id >= 59):
        alt_m = 35786_000.0
    if alt_m <= 0:
        return (0.0, 0.0)

    el_rad = math.radians(max(0.0, float(elev_deg)))
    r_e = EARTH_MEAN_RADIUS
    r_sat = r_e + alt_m
    # Law of cosines in Earth-Receiver-Satellite triangle:
    # r_sat^2 = r_e^2 + d^2 - 2*r_e*d*cos(90° + el) = r_e^2 + d^2 + 2*r_e*d*sin(el)
    sin_el = math.sin(el_rad)
    cos_el = math.cos(el_rad)
    d_m = math.sqrt(r_sat ** 2 - (r_e * cos_el) ** 2) - r_e * sin_el
    delay_ms = (d_m / SPEED_OF_LIGHT) * 1000.0
    return (d_m / 1000.0, delay_ms)


def compute_astro_info(
    year: int, month: int, day: int, hour: int, minute: int, sec: float, lat_deg: float, lon_deg: float
) -> Tuple[str, float, float]:
    """
    Compute Local Mean Sidereal Time (HH:MM:SS), Solar Elevation (deg), and Solar Azimuth (deg)
    for the current UTC timestamp and GPS coordinates.
    """
    if year < 2000 or math.isnan(lat_deg) or math.isnan(lon_deg):
        return ("--:--:--", 0.0, 0.0)

    # Julian Date
    y, m = year, month
    if m <= 2:
        y -= 1
        m += 12
    A = y // 100
    B = 2 - A + (A // 4)
    day_frac = (hour + minute / 60.0 + sec / 3600.0) / 24.0
    jd = int(365.25 * (y + 4716)) + int(30.6001 * (m + 1)) + day + day_frac + B - 1524.5
    d_j2000 = jd - 2451545.0

    # Greenwich Mean Sidereal Time (degrees)
    gmst_deg = (280.46061837 + 360.98564736629 * d_j2000) % 360.0
    lmst_deg = (gmst_deg + lon_deg) % 360.0
    lmst_hours = lmst_deg / 15.0
    lh = int(lmst_hours)
    lm = int((lmst_hours - lh) * 60.0)
    ls = ((lmst_hours - lh) * 60.0 - lm) * 60.0
    lmst_str = f"{lh:02d}:{lm:02d}:{ls:04.1f}"

    # Approximate solar position
    mean_lon = (280.460 + 0.9856474 * d_j2000) % 360.0
    mean_anom = math.radians((357.528 + 0.9856003 * d_j2000) % 360.0)
    ecl_lon = math.radians((mean_lon + 1.915 * math.sin(mean_anom) + 0.020 * math.sin(2 * mean_anom)) % 360.0)
    obliquity = math.radians(23.439 - 0.0000004 * d_j2000)

    ra = math.degrees(math.atan2(math.cos(obliquity) * math.sin(ecl_lon), math.cos(ecl_lon))) % 360.0
    dec = math.asin(math.sin(obliquity) * math.sin(ecl_lon))

    ha = math.radians((lmst_deg - ra) % 360.0)
    lat_rad = math.radians(lat_deg)

    sin_el = math.sin(lat_rad) * math.sin(dec) + math.cos(lat_rad) * math.cos(dec) * math.cos(ha)
    sun_el = math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))

    y_az = -math.sin(ha)
    x_az = math.tan(dec) * math.cos(lat_rad) - math.sin(lat_rad) * math.cos(ha)
    sun_az = math.degrees(math.atan2(y_az, x_az)) % 360.0

    return (lmst_str, sun_el, sun_az)


def compass_rose(deg: float) -> str:
    """Return 16-point compass cardinal direction for a heading in degrees."""
    if math.isnan(deg):
        return "---"
    dirs = [
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    ]
    idx = int(((deg % 360.0) + 11.25) // 22.5) % 16
    return dirs[idx]


def dop_rating(dop: float) -> Tuple[str, int]:
    """Return qualitative DOP rating and severity level (0=good, 1=ok, 2=warn, 3=bad)."""
    if dop <= 0 or math.isnan(dop) or dop >= 99.0:
        return ("NONE", 3)
    if dop < 1.0:
        return ("IDEAL", 0)
    if dop < 2.0:
        return ("EXCELLENT", 0)
    if dop < 5.0:
        return ("GOOD", 1)
    if dop < 10.0:
        return ("MODERATE", 2)
    if dop < 20.0:
        return ("FAIR", 2)
    return ("POOR", 3)


class PositionDriftTracker:
    """Tracks 3D position history in a local East-North-Up (ENU) tangent plane and computes CEP/R95/2DRMS."""

    def __init__(self, max_points: int = 600):
        self.max_points = max_points
        self.samples: List[Tuple[float, float, float]] = []  # (lat_deg, lon_deg, alt_m)
        self.anchor: Optional[Tuple[float, float, float]] = None
        self.total_distance_m: float = 0.0

    def reset(self) -> None:
        self.samples.clear()
        self.anchor = None
        self.total_distance_m = 0.0

    def add_fix(self, lat: float, lon: float, alt: float) -> None:
        if math.isnan(lat) or math.isnan(lon) or abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return
        if self.samples:
            prev_lat, prev_lon, _ = self.samples[-1]
            de, dn = self._latlon_delta_m(prev_lat, prev_lon, lat, lon)
            step = math.hypot(de, dn)
            if step < 5000.0:  # Ignore wild initial jump
                self.total_distance_m += step
        self.samples.append((lat, lon, alt if not math.isnan(alt) else 0.0))
        if len(self.samples) > self.max_points:
            self.samples.pop(0)

    @staticmethod
    def _latlon_delta_m(lat0: float, lon0: float, lat1: float, lon1: float) -> Tuple[float, float]:
        """Return (delta_east_m, delta_north_m) from (lat0, lon0) to (lat1, lon1)."""
        dlat = math.radians(lat1 - lat0)
        dlon = math.radians(lon1 - lon0)
        mean_lat = math.radians((lat0 + lat1) * 0.5)
        dn = dlat * EARTH_MEAN_RADIUS
        de = dlon * EARTH_MEAN_RADIUS * math.cos(mean_lat)
        return (de, dn)

    def get_enu_points_and_stats(self) -> dict:
        """Return ENU offsets (meters) relative to mean centroid and statistical precision metrics."""
        n = len(self.samples)
        if n == 0:
            return {
                "count": 0,
                "mean_lat": float("nan"),
                "mean_lon": float("nan"),
                "mean_alt": float("nan"),
                "enu_points": [],
                "std_e": 0.0,
                "std_n": 0.0,
                "std_u": 0.0,
                "cep50": 0.0,
                "r95": 0.0,
                "drms2": 0.0,
                "max_wander": 0.0,
                "cur_e": 0.0,
                "cur_n": 0.0,
                "cur_u": 0.0,
            }

        mean_lat = sum(p[0] for p in self.samples) / n
        mean_lon = sum(p[1] for p in self.samples) / n
        mean_alt = sum(p[2] for p in self.samples) / n

        ref_lat, ref_lon, ref_alt = self.anchor if self.anchor else (mean_lat, mean_lon, mean_alt)

        enu_points: List[Tuple[float, float, float]] = []
        radii: List[float] = []
        for lat, lon, alt in self.samples:
            de, dn = self._latlon_delta_m(ref_lat, ref_lon, lat, lon)
            du = alt - ref_alt
            enu_points.append((de, dn, du))
            radii.append(math.hypot(de, dn))

        if n > 1:
            var_e = sum(p[0] ** 2 for p in enu_points) / n
            var_n = sum(p[1] ** 2 for p in enu_points) / n
            var_u = sum(p[2] ** 2 for p in enu_points) / n
            std_e = math.sqrt(var_e)
            std_n = math.sqrt(var_n)
            std_u = math.sqrt(var_u)
        else:
            std_e = std_n = std_u = 0.0

        sorted_r = sorted(radii)
        cep50 = sorted_r[int(0.50 * (n - 1))] if n >= 2 else 0.5887 * (std_e + std_n)
        r95 = sorted_r[int(0.95 * (n - 1))] if n >= 5 else 2.08 * cep50
        drms2 = 2.0 * math.hypot(std_e, std_n)
        max_wander = sorted_r[-1] if sorted_r else 0.0
        cur_e, cur_n, cur_u = enu_points[-1]

        return {
            "count": n,
            "mean_lat": ref_lat,
            "mean_lon": ref_lon,
            "mean_alt": ref_alt,
            "enu_points": enu_points,
            "std_e": std_e,
            "std_n": std_n,
            "std_u": std_u,
            "cep50": cep50,
            "r95": r95,
            "drms2": drms2,
            "max_wander": max_wander,
            "cur_e": cur_e,
            "cur_n": cur_n,
            "cur_u": cur_u,
        }
