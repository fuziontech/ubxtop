"""NMEA 0183 and u-blox UBX binary protocol parser, device auto-discovery, and telemetry state."""

from __future__ import annotations
import collections
import glob
import math
import os
import select
import struct
import subprocess
import termios
import time
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Set, Tuple

from gps_math import PositionDriftTracker, lla_to_ecef, sat_slant_range_and_delay


GNSS_NAMES = {
    0: "GPS",
    1: "SBAS",
    2: "GAL",
    3: "BDS",
    4: "IMES",
    5: "QZSS",
    6: "GLO",
}

GNSS_PREFIX = {
    "GPS": "G",
    "SBAS": "S",
    "GAL": "E",
    "BDS": "B",
    "IMES": "I",
    "QZSS": "Q",
    "GLO": "R",
}

QUALITY_IND_NAMES = {
    0: "No Signal",
    1: "Searching",
    2: "Acquired",
    3: "Unusable",
    4: "Code Lock",
    5: "Code+Carr",
    6: "Code+Carr",
    7: "Code+Carr",
}

ORBIT_SOURCE_NAMES = {
    0: "None",
    1: "Ephemeris",
    2: "Almanac",
    3: "AssistNow Off",
    4: "AssistNow Auto",
    5: "Other",
    6: "Other",
    7: "Other",
}

FIX_TYPE_NAMES = {
    0: "NO FIX",
    1: "DEAD RECKONING",
    2: "2D FIX",
    3: "3D FIX",
    4: "GNSS + DR",
    5: "TIME ONLY",
}

DYN_MODEL_NAMES = {
    0: "Portable",
    2: "Stationary",
    3: "Pedestrian",
    4: "Automotive",
    5: "Sea",
    6: "Airborne <1g",
    7: "Airborne <2g",
    8: "Airborne <4g",
    9: "Wrist Watch",
    10: "Motorbike",
}

ANT_STATUS_NAMES = {
    0: "INIT",
    1: "UNKNOWN",
    2: "OK",
    3: "SHORT",
    4: "OPEN",
}

ANT_POWER_NAMES = {
    0: "OFF",
    1: "ON",
    2: "UNKNOWN",
}

JAMMING_STATE_NAMES = {
    0: "Unknown",
    1: "Clean (OK)",
    2: "Warning",
    3: "CRITICAL!",
}

SPOOF_STATE_NAMES = {
    0: "Deactivated/Unk",
    1: "Clean (No Spoof)",
    2: "Spoofing Indicated!",
    3: "Multiple Spoofing!",
}

SBAS_SYS_NAMES = {
    -1: "Unknown",
    0: "WAAS (North America)",
    1: "EGNOS (Europe)",
    2: "MSAS (Japan)",
    3: "GAGAN (India)",
    16: "GPS",
}

UBX_MSG_NAMES: Dict[Tuple[int, int], str] = {
    (0x01, 0x01): "UBX-NAV-POSECEF",
    (0x01, 0x03): "UBX-NAV-STATUS",
    (0x01, 0x04): "UBX-NAV-DOP",
    (0x01, 0x07): "UBX-NAV-PVT",
    (0x01, 0x11): "UBX-NAV-VELECEF",
    (0x01, 0x20): "UBX-NAV-TIMEGPS",
    (0x01, 0x21): "UBX-NAV-TIMEUTC",
    (0x01, 0x22): "UBX-NAV-CLOCK",
    (0x01, 0x23): "UBX-NAV-TIMEGLO",
    (0x01, 0x24): "UBX-NAV-TIMEBDS",
    (0x01, 0x32): "UBX-NAV-SBAS",
    (0x01, 0x34): "UBX-NAV-ORB",
    (0x01, 0x35): "UBX-NAV-SAT",
    (0x01, 0x60): "UBX-NAV-AOPSTATUS",
    (0x05, 0x00): "UBX-ACK-NAK",
    (0x05, 0x01): "UBX-ACK-ACK",
    (0x06, 0x08): "UBX-CFG-RATE",
    (0x06, 0x24): "UBX-CFG-NAV5",
    (0x06, 0x3E): "UBX-CFG-GNSS",
    (0x0A, 0x02): "UBX-MON-IO",
    (0x0A, 0x04): "UBX-MON-VER",
    (0x0A, 0x09): "UBX-MON-HW",
    (0x0A, 0x0B): "UBX-MON-HW2",
    (0x0A, 0x28): "UBX-MON-GNSS",
}


@dataclass
class SatelliteInfo:
    key: str               # e.g., "G09", "B25", "R14"
    constellation: str     # "GPS", "BDS", "GLO", "GAL", "SBAS", "QZSS"
    sv_id: int
    cno: int = 0           # dB-Hz (SNR)
    elev: Optional[int] = None   # degrees (-90..90)
    azim: Optional[int] = None   # degrees (0..360)
    used: bool = False
    quality_ind: int = 0   # 0..7
    pr_res_m: Optional[float] = None  # Pseudorange residual in meters (from UBX-NAV-SAT)
    health: int = 0        # 0=unk, 1=healthy, 2=unhealthy
    diff_corr: bool = False
    smoothed: bool = False
    orbit_source: int = 0
    eph_avail: bool = False
    alm_avail: bool = False
    aop_avail: bool = False
    sbas_corr: bool = False
    pr_corr: bool = False
    cr_corr: bool = False
    do_corr: bool = False
    slant_range_km: float = 0.0
    prop_delay_ms: float = 0.0
    last_seen: float = field(default_factory=time.time)

    @property
    def quality_str(self) -> str:
        return QUALITY_IND_NAMES.get(self.quality_ind, f"Q{self.quality_ind}")

    @property
    def orbit_str(self) -> str:
        return ORBIT_SOURCE_NAMES.get(self.orbit_source, "Other")

    @property
    def health_str(self) -> str:
        if self.health == 1:
            return "HLTH"
        if self.health == 2:
            return "UNHL"
        return "UNK"


@dataclass
class GnssChannelBlock:
    gnss_name: str
    res_trk_ch: int
    max_trk_ch: int
    enabled: bool
    sig_cfg_mask: int


@dataclass
class IoPortStats:
    port_id: int
    port_name: str
    rx_bytes: int = 0
    tx_bytes: int = 0
    parity_errs: int = 0
    framing_errs: int = 0
    overrun_errs: int = 0
    break_cond: int = 0
    rx_busy: int = 0
    tx_busy: int = 0


@dataclass
class PacketRecord:
    timestamp: float
    proto: str         # "NMEA" or "UBX"
    msg_type: str      # e.g., "$GNGGA" or "UBX-NAV-PVT"
    size_bytes: int
    summary: str
    raw_preview: str


class GPSState:
    """Comprehensive diagnostic telemetry state combining NMEA 0183 and u-blox UBX binary data."""

    def __init__(self) -> None:
        self.device_path: str = "Searching..."
        self.usb_product: str = "u-blox GNSS Receiver"
        self.usb_vendor: str = "u-blox AG"
        self.connected: bool = False
        self.start_time: float = time.time()
        self.last_rx_time: float = 0.0

        # Link & throughput counters
        self.rx_bytes_total: int = 0
        self.tx_bytes_total: int = 0
        self.rx_rate_bps: float = 0.0
        self._rate_window_bytes: int = 0
        self._rate_window_t0: float = time.time()

        self.nmea_valid_count: int = 0
        self.nmea_checksum_err_count: int = 0
        self.ubx_valid_count: int = 0
        self.ubx_checksum_err_count: int = 0
        self.msg_counts: Dict[str, int] = collections.Counter()
        self.msg_last_seen: Dict[str, float] = {}
        self.packet_log: Deque[PacketRecord] = collections.deque(maxlen=250)
        self.txt_messages: Deque[str] = collections.deque(maxlen=20)

        # Hardware / Firmware identity (UBX-MON-VER & $GNTXT)
        self.hw_version: str = "UBX-M8130"
        self.sw_version: str = "ROM CORE 3.01"
        self.fw_version: str = "SPG 3.01"
        self.prot_version: str = "18.00"
        self.supported_gnss_str: str = "GPS;GLO;BDS;QZSS"
        self.ver_extensions: List[str] = []

        # Fix & Navigation state
        self.fix_type: int = 0           # 0=No, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=Time
        self.fix_ok: bool = False
        self.diff_soln: bool = False
        self.carr_soln: int = 0          # 0=None, 1=Float RTK, 2=Fixed RTK
        self.gga_quality: int = 0        # 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
        self.faa_mode: str = "N"         # A=Autonomous, D=Differential, E=Estimated, N=Not valid
        self.nav_status_char: str = "V"
        self.num_sv_used: int = 0
        self.num_sv_visible: int = 0
        self.ttff_ms: int = 0
        self.uptime_ms: int = 0

        # Position & Geodesy
        self.lat: float = float("nan")
        self.lon: float = float("nan")
        self.alt_msl_m: float = float("nan")
        self.alt_hae_m: float = float("nan")
        self.geoid_sep_m: float = float("nan")
        self.h_acc_m: float = float("nan")
        self.v_acc_m: float = float("nan")
        self.p_acc_m: float = float("nan")

        # ECEF Coordinates & Velocities
        self.ecef_x_m: float = float("nan")
        self.ecef_y_m: float = float("nan")
        self.ecef_z_m: float = float("nan")
        self.ecef_vx_ms: float = float("nan")
        self.ecef_vy_ms: float = float("nan")
        self.ecef_vz_ms: float = float("nan")

        # Kinematics & Velocity
        self.speed_ms: float = 0.0
        self.speed_knots: float = 0.0
        self.speed_kmh: float = 0.0
        self.vel_n_ms: float = 0.0
        self.vel_e_ms: float = 0.0
        self.vel_d_ms: float = 0.0
        self.course_deg: float = float("nan")
        self.s_acc_ms: float = float("nan")
        self.head_acc_deg: float = float("nan")
        self.mag_dec_deg: float = float("nan")
        self.mag_acc_deg: float = float("nan")

        # Dilution of Precision (7-axis from UBX-NAV-DOP + NMEA GSA)
        self.gdop: float = float("nan")
        self.pdop: float = float("nan")
        self.tdop: float = float("nan")
        self.vdop: float = float("nan")
        self.hdop: float = float("nan")
        self.ndop: float = float("nan")
        self.edop: float = float("nan")

        # Precision Timing & Oscillator Clock (UBX-NAV-PVT, NAV-CLOCK, NAV-TIMEGPS)
        self.utc_year: int = 0
        self.utc_month: int = 0
        self.utc_day: int = 0
        self.utc_hour: int = 0
        self.utc_min: int = 0
        self.utc_sec: int = 0
        self.utc_nano: int = 0
        self.valid_date: bool = False
        self.valid_time: bool = False
        self.fully_resolved_time: bool = False
        self.itow_ms: int = 0
        self.ftow_ns: int = 0
        self.gps_week: int = 0
        self.leap_sec: int = 18
        self.leap_sec_valid: bool = False
        self.clk_bias_ns: int = 0        # Receiver clock bias (ns)
        self.clk_drift_nss: int = 0      # Receiver clock drift (ns/s = ppb)
        self.t_acc_ns: int = 0           # Time accuracy estimate (ns)
        self.f_acc_pss: int = 0          # Frequency accuracy estimate (ps/s)

        # RF Frontend, Hardware & Security (UBX-MON-HW, MON-HW2, NAV-STATUS)
        self.noise_per_ms: int = 0
        self.agc_cnt: int = 0            # 0..8191
        self.ant_status: int = 2         # 2=OK
        self.ant_power: int = 1          # 1=ON
        self.jamming_state: int = 1      # 1=OK
        self.jam_ind: int = 0            # 0..255 CW jamming indicator
        self.spoof_state: int = 0        # 0..3
        self.rtc_calibrated: bool = False
        self.xtal_absent: bool = False
        self.ofs_i: int = 0              # I-channel DC offset (-128..127)
        self.mag_i: int = 0              # I-channel magnitude (0..255)
        self.ofs_q: int = 0              # Q-channel DC offset (-128..127)
        self.mag_q: int = 0              # Q-channel magnitude (0..255)

        # Receiver Engine Configuration (CFG-NAV5, CFG-RATE, CFG-GNSS, MON-GNSS, NAV-SBAS)
        self.dyn_model: int = 0
        self.fix_mode_cfg: int = 3
        self.min_elev_deg: int = 5
        self.meas_rate_ms: int = 1000
        self.nav_rate_cycles: int = 1
        self.time_ref: int = 1
        self.num_trk_ch_hw: int = 32
        self.num_trk_ch_use: int = 32
        self.gnss_blocks: List[GnssChannelBlock] = []
        self.io_ports: List[IoPortStats] = []
        self.sbas_mode: int = 0
        self.sbas_sys: int = -1
        self.sbas_geo_prn: int = 0
        self.aop_cfg: int = 0
        self.aop_status: int = 0
        self.last_ack: str = ""

        # Satellites dictionary keyed by e.g. "G09", "B25"
        self.satellites: Dict[str, SatelliteInfo] = {}
        self.gsa_active_prns: Dict[str, Set[int]] = collections.defaultdict(set)

        # Position scatter & time-series history
        self.drift_tracker = PositionDriftTracker(max_points=600)
        self.hist_alt: Deque[float] = collections.deque(maxlen=180)
        self.hist_hacc: Deque[float] = collections.deque(maxlen=180)
        self.hist_pdop: Deque[float] = collections.deque(maxlen=180)
        self.hist_snr: Deque[float] = collections.deque(maxlen=180)
        self.hist_sv_used: Deque[float] = collections.deque(maxlen=180)
        self.hist_clk_drift: Deque[float] = collections.deque(maxlen=180)
        self.hist_clk_bias: Deque[float] = collections.deque(maxlen=180)
        self.hist_noise: Deque[float] = collections.deque(maxlen=180)
        self.hist_agc: Deque[float] = collections.deque(maxlen=180)
        self.hist_jam: Deque[float] = collections.deque(maxlen=180)
        self._last_hist_sample_t: float = 0.0

    def update_throughput(self, nbytes: int) -> None:
        now = time.time()
        self.rx_bytes_total += nbytes
        self.last_rx_time = now
        self._rate_window_bytes += nbytes
        dt = now - self._rate_window_t0
        if dt >= 1.0:
            self.rx_rate_bps = self._rate_window_bytes / dt
            self._rate_window_bytes = 0
            self._rate_window_t0 = now

    def record_history_tick(self) -> None:
        now = time.time()
        if now - self._last_hist_sample_t < 0.8:
            return
        self._last_hist_sample_t = now
        if not math.isnan(self.alt_msl_m):
            self.hist_alt.append(self.alt_msl_m)
        if not math.isnan(self.h_acc_m):
            self.hist_hacc.append(self.h_acc_m)
        if not math.isnan(self.pdop):
            self.hist_pdop.append(self.pdop)

        active_cnos = [s.cno for s in self.satellites.values() if s.cno > 0]
        mean_cno = sum(active_cnos) / len(active_cnos) if active_cnos else 0.0
        self.hist_snr.append(mean_cno)
        self.hist_sv_used.append(float(self.num_sv_used))

        if self.clk_drift_nss != 0 or self.clk_bias_ns != 0:
            self.hist_clk_drift.append(float(self.clk_drift_nss))
            self.hist_clk_bias.append(float(self.clk_bias_ns))
        if self.noise_per_ms > 0:
            self.hist_noise.append(float(self.noise_per_ms))
        if self.agc_cnt > 0:
            self.hist_agc.append((self.agc_cnt / 8191.0) * 100.0)
        self.hist_jam.append(float(self.jam_ind))

        # Expire stale satellites not seen for >20s
        stale_keys = [k for k, v in self.satellites.items() if now - v.last_seen > 20.0]
        for k in stale_keys:
            del self.satellites[k]
        self.num_sv_visible = len(self.satellites)

    @property
    def fix_badge(self) -> str:
        if self.carr_soln == 2:
            return "RTK FIXED"
        if self.carr_soln == 1:
            return "RTK FLOAT"
        base = FIX_TYPE_NAMES.get(self.fix_type, "NO FIX")
        if self.diff_soln or self.gga_quality == 2:
            return f"{base} + DGPS/SBAS"
        return base

    @property
    def update_rate_hz(self) -> float:
        if self.meas_rate_ms > 0:
            return 1000.0 / (self.meas_rate_ms * max(1, self.nav_rate_cycles))
        return 1.0

    @property
    def agc_percent(self) -> float:
        return (self.agc_cnt / 8191.0) * 100.0 if self.agc_cnt else 0.0

    @property
    def jam_percent(self) -> float:
        return (self.jam_ind / 255.0) * 100.0

    @property
    def utc_iso_str(self) -> str:
        if self.utc_year > 0:
            frac_s = self.utc_nano / 1e9
            sec_total = self.utc_sec + frac_s
            sec_int = int(sec_total)
            ns_rem = int(round((sec_total - sec_int) * 1e9))
            if ns_rem < 0:
                ns_rem = 0
            return (
                f"{self.utc_year:04d}-{self.utc_month:02d}-{self.utc_day:02d} "
                f"{self.utc_hour:02d}:{self.utc_min:02d}:{sec_int:02d}.{ns_rem:09d} UTC"
            )
        return "Waiting for UTC lock..."


def discover_gps_device(preferred: Optional[str] = None) -> Tuple[Optional[str], str, str]:
    """
    Auto-detect connected GPS serial device on macOS / Linux.
    Returns (device_path, product_name, vendor_name).
    """
    if preferred and os.path.exists(preferred):
        return (preferred, "Custom Serial Device", "User Specified")

    product_name = "u-blox GNSS Receiver"
    vendor_name = "u-blox AG"

    # Try ioreg on macOS to identify USB GPS product/vendor and callout device
    try:
        out = subprocess.check_output(
            ["ioreg", "-p", "IOUSB", "-l", "-w", "0"],
            stderr=subprocess.DEVNULL,
            timeout=1.5,
        ).decode("latin1", errors="ignore")
        if "u_blox" in out or "u-blox" in out:
            product_name = "u-blox GNSS Receiver"
            vendor_name = "u-blox AG"
    except Exception:
        pass

    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    for pat in patterns:
        matches = sorted(glob.glob(pat))
        if matches:
            return (matches[0], product_name, vendor_name)

    return (None, product_name, vendor_name)


def build_ubx_packet(cls_id: int, msg_id: int, payload: bytes = b"") -> bytes:
    """Construct a binary UBX packet with sync chars (0xB5 0x62) and Fletcher-8 checksum."""
    header = struct.pack("<BBBBH", 0xB5, 0x62, cls_id, msg_id, len(payload))
    ck_a = 0
    ck_b = 0
    for b in header[2:] + payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + payload + bytes([ck_a, ck_b])


class GPSConnection:
    """Non-blocking serial connection and dual NMEA 0183 + binary UBX stream parser."""

    def __init__(
        self,
        state: GPSState,
        device_path: Optional[str] = None,
        baudrate: int = 9600,
        log_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.state = state
        self.preferred_device = device_path
        self.baudrate = baudrate
        self.log_callback = log_callback
        self.fd: Optional[int] = None
        self.rx_buf: bytearray = bytearray()
        self.last_poll_fast: float = 0.0
        self.last_poll_slow: float = 0.0
        self.last_reconnect_attempt: float = 0.0
        self.paused: bool = False

    def connect(self) -> bool:
        self.close()
        dev, prod, vend = discover_gps_device(self.preferred_device)
        self.state.usb_product = prod
        self.state.usb_vendor = vend
        if not dev:
            self.state.device_path = "Disconnected (No USB GPS found)"
            self.state.connected = False
            return False

        try:
            fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self._configure_termios(fd, self.baudrate)
            self.fd = fd
            self.state.device_path = dev
            self.state.connected = True
            # Immediately poll static hardware/firmware & config blocks
            self.poll_static_info()
            self.poll_fast_telemetry()
            return True
        except Exception as exc:
            self.state.device_path = f"{dev} ({exc})"
            self.state.connected = False
            return False

    def _configure_termios(self, fd: int, baud: int) -> None:
        try:
            attrs = termios.tcgetattr(fd)
            baud_map = {
                4800: termios.B4800,
                9600: termios.B9600,
                19200: termios.B19200,
                38400: termios.B38400,
                57600: termios.B57600,
                115200: termios.B115200,
                230400: termios.B230400,
            }
            bconst = baud_map.get(baud, termios.B9600)
            # Raw 8N1 mode without echo or line buffering
            attrs[0] &= ~(
                termios.IGNBRK | termios.BRKINT | termios.PARMRK
                | termios.ISTRIP | termios.INLCR | termios.IGNCR
                | termios.ICRNL | termios.IXON
            )
            attrs[1] &= ~termios.OPOST
            attrs[2] &= ~(termios.CSIZE | termios.PARENB)
            attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
            attrs[4] = bconst
            attrs[5] = bconst
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception:
            # USB CDC ACM devices often ignore baudrate settings anyway
            pass

    def close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None
        self.state.connected = False

    def send_ubx(self, cls_id: int, msg_id: int, payload: bytes = b"") -> bool:
        if self.fd is None:
            return False
        pkt = build_ubx_packet(cls_id, msg_id, payload)
        try:
            written = os.write(self.fd, pkt)
            self.state.tx_bytes_total += written
            return True
        except Exception:
            return False

    def poll_static_info(self) -> None:
        """Poll hardware/firmware version and receiver engine configurations."""
        for cls_id, msg_id in [
            (0x0A, 0x04),  # MON-VER
            (0x0A, 0x28),  # MON-GNSS
            (0x06, 0x24),  # CFG-NAV5
            (0x06, 0x08),  # CFG-RATE
            (0x06, 0x3E),  # CFG-GNSS
            (0x01, 0x32),  # NAV-SBAS
            (0x01, 0x60),  # NAV-AOPSTATUS
        ]:
            self.send_ubx(cls_id, msg_id)

    def poll_fast_telemetry(self) -> None:
        """Poll 1Hz high-precision UBX navigation, clock, satellite, and RF hardware diagnostics."""
        for cls_id, msg_id in [
            (0x01, 0x07),  # NAV-PVT
            (0x01, 0x04),  # NAV-DOP
            (0x01, 0x35),  # NAV-SAT
            (0x01, 0x22),  # NAV-CLOCK
            (0x01, 0x03),  # NAV-STATUS
            (0x01, 0x20),  # NAV-TIMEGPS
            (0x01, 0x01),  # NAV-POSECEF
            (0x01, 0x11),  # NAV-VELECEF
            (0x0A, 0x09),  # MON-HW
            (0x0A, 0x0B),  # MON-HW2
        ]:
            self.send_ubx(cls_id, msg_id)

    def poll_slow_telemetry(self) -> None:
        """Poll 0.25Hz orbital availability, I/O port statistics, and SBAS status."""
        for cls_id, msg_id in [
            (0x01, 0x34),  # NAV-ORB
            (0x0A, 0x02),  # MON-IO
            (0x01, 0x32),  # NAV-SBAS
            (0x06, 0x08),  # CFG-RATE
            (0x06, 0x24),  # CFG-NAV5
        ]:
            self.send_ubx(cls_id, msg_id)

    def set_measurement_rate_hz(self, hz: int) -> None:
        """Set receiver measurement rate (e.g. 1, 2, 5, 10 Hz) via UBX-CFG-RATE."""
        meas_ms = max(100, int(1000 // max(1, hz)))
        payload = struct.pack("<HHH", meas_ms, 1, 1)  # measRate(ms), navRate=1, timeRef=GPS
        self.send_ubx(0x06, 0x08, payload)
        self.send_ubx(0x06, 0x08)  # Poll back to confirm

    def set_dynamic_model(self, dyn_model: int) -> None:
        """Set receiver navigation dynamic platform model via UBX-CFG-NAV5."""
        # Mask bit 0 (0x0001) = apply dynamic model only
        payload = struct.pack(
            "<HBBiIbbHHHHBBBBHBB6s",
            0x0001,            # mask: dynModel only
            dyn_model,         # dynModel
            self.state.fix_mode_cfg or 3,
            0, 10000,
            self.state.min_elev_deg,
            0,
            250, 250, 100, 300,
            0, 60, 0, 0,
            0, 0, 0, b"\x00" * 6,
        )
        self.send_ubx(0x06, 0x24, payload)
        self.send_ubx(0x06, 0x24)  # Poll back to confirm

    def poll_step(self, timeout_s: float = 0.02) -> None:
        """Perform non-blocking read/parse and scheduled UBX polls."""
        now = time.time()
        if self.fd is None:
            if now - self.last_reconnect_attempt >= 1.5:
                self.last_reconnect_attempt = now
                self.connect()
            return

        if now - self.last_poll_fast >= 0.95:
            self.last_poll_fast = now
            self.poll_fast_telemetry()

        if now - self.last_poll_slow >= 4.0:
            self.last_poll_slow = now
            self.poll_slow_telemetry()

        try:
            r, _, _ = select.select([self.fd], [], [], timeout_s)
            if r:
                chunk = os.read(self.fd, 8192)
                if not chunk:
                    self.close()
                    return
                self.state.update_throughput(len(chunk))
                self.rx_buf.extend(chunk)
                self._consume_buffer()
                self.state.record_history_tick()
        except OSError:
            self.close()

    def _consume_buffer(self) -> None:
        """Demultiplex interleaved binary UBX packets (0xB5 0x62) and ASCII NMEA sentences ($...\\r\\n)."""
        buf = self.rx_buf
        while len(buf) > 0:
            b0 = buf[0]

            # Case 1: UBX Sync Header (0xB5 0x62)
            if b0 == 0xB5:
                if len(buf) < 2:
                    break
                if buf[1] != 0x62:
                    del buf[0]
                    continue
                if len(buf) < 6:
                    break
                cls_id, msg_id, length = struct.unpack_from("<BBH", buf, 2)
                if length > 4096:
                    # Bogus length, skip sync byte
                    del buf[0]
                    continue
                total_len = 6 + length + 2
                if len(buf) < total_len:
                    break
                frame = bytes(buf[:total_len])
                del buf[:total_len]
                self._handle_ubx_frame(cls_id, msg_id, frame[6:-2], frame[-2], frame[-1])
                continue

            # Case 2: NMEA Sentence ('$')
            if b0 == 0x24:  # '$'
                nl_idx = buf.find(b"\n")
                # Also check if a binary UBX header got injected mid-line before \n
                ubx_idx = buf.find(b"\xb5\x62", 1)
                if ubx_idx != -1 and (nl_idx == -1 or ubx_idx < nl_idx):
                    # Discard partial line before UBX header
                    del buf[:ubx_idx]
                    continue
                if nl_idx == -1:
                    if len(buf) > 256:
                        del buf[0]
                    break
                line_bytes = bytes(buf[: nl_idx + 1])
                del buf[: nl_idx + 1]
                line_str = line_bytes.decode("ascii", errors="replace").strip()
                if line_str:
                    self._handle_nmea_line(line_str)
                continue

            # Skip any whitespace or stray byte until next '$' or 0xB5
            next_ubx = buf.find(b"\xb5")
            next_nmea = buf.find(b"$")
            candidates = [idx for idx in (next_ubx, next_nmea) if idx != -1]
            if not candidates:
                buf.clear()
                break
            del buf[: min(candidates)]

    def _record_packet(self, proto: str, msg_type: str, size_bytes: int, summary: str, raw_preview: str) -> None:
        now = time.time()
        self.state.msg_counts[msg_type] += 1
        self.state.msg_last_seen[msg_type] = now
        if not self.paused:
            self.state.packet_log.append(
                PacketRecord(
                    timestamp=now,
                    proto=proto,
                    msg_type=msg_type,
                    size_bytes=size_bytes,
                    summary=summary,
                    raw_preview=raw_preview,
                )
            )
        if self.log_callback:
            self.log_callback(f"{now:.3f}\t{proto}\t{msg_type}\t{summary}\t{raw_preview}")

    # -------------------------------------------------------------------------
    # UBX Binary Frame Decoders
    # -------------------------------------------------------------------------
    def _handle_ubx_frame(self, cls_id: int, msg_id: int, payload: bytes, ck_a: int, ck_b: int) -> None:
        calc_a = 0
        calc_b = 0
        header_tail = struct.pack("<BBH", cls_id, msg_id, len(payload))
        for b in header_tail + payload:
            calc_a = (calc_a + b) & 0xFF
            calc_b = (calc_b + calc_a) & 0xFF

        msg_name = UBX_MSG_NAMES.get((cls_id, msg_id), f"UBX-0x{cls_id:02X}-0x{msg_id:02X}")
        if calc_a != ck_a or calc_b != ck_b:
            self.state.ubx_checksum_err_count += 1
            return

        self.state.ubx_valid_count += 1
        summary = f"len={len(payload)}B"

        try:
            if (cls_id, msg_id) == (0x01, 0x07) and len(payload) >= 92:
                summary = self._decode_ubx_nav_pvt(payload)
            elif (cls_id, msg_id) == (0x01, 0x04) and len(payload) >= 18:
                summary = self._decode_ubx_nav_dop(payload)
            elif (cls_id, msg_id) == (0x01, 0x35) and len(payload) >= 8:
                summary = self._decode_ubx_nav_sat(payload)
            elif (cls_id, msg_id) == (0x01, 0x34) and len(payload) >= 8:
                summary = self._decode_ubx_nav_orb(payload)
            elif (cls_id, msg_id) == (0x01, 0x22) and len(payload) >= 20:
                summary = self._decode_ubx_nav_clock(payload)
            elif (cls_id, msg_id) == (0x01, 0x03) and len(payload) >= 16:
                summary = self._decode_ubx_nav_status(payload)
            elif (cls_id, msg_id) == (0x01, 0x20) and len(payload) >= 16:
                summary = self._decode_ubx_nav_timegps(payload)
            elif (cls_id, msg_id) == (0x01, 0x01) and len(payload) >= 20:
                summary = self._decode_ubx_nav_posecef(payload)
            elif (cls_id, msg_id) == (0x01, 0x11) and len(payload) >= 20:
                summary = self._decode_ubx_nav_velecef(payload)
            elif (cls_id, msg_id) == (0x01, 0x32) and len(payload) >= 12:
                summary = self._decode_ubx_nav_sbas(payload)
            elif (cls_id, msg_id) == (0x01, 0x60) and len(payload) >= 16:
                summary = self._decode_ubx_nav_aopstatus(payload)
            elif (cls_id, msg_id) == (0x0A, 0x04) and len(payload) >= 40:
                summary = self._decode_ubx_mon_ver(payload)
            elif (cls_id, msg_id) == (0x0A, 0x09) and len(payload) >= 60:
                summary = self._decode_ubx_mon_hw(payload)
            elif (cls_id, msg_id) == (0x0A, 0x0B) and len(payload) >= 28:
                summary = self._decode_ubx_mon_hw2(payload)
            elif (cls_id, msg_id) == (0x0A, 0x02) and len(payload) >= 20:
                summary = self._decode_ubx_mon_io(payload)
            elif (cls_id, msg_id) == (0x0A, 0x28) and len(payload) >= 8:
                summary = self._decode_ubx_mon_gnss(payload)
            elif (cls_id, msg_id) == (0x06, 0x24) and len(payload) >= 36:
                summary = self._decode_ubx_cfg_nav5(payload)
            elif (cls_id, msg_id) == (0x06, 0x08) and len(payload) >= 6:
                summary = self._decode_ubx_cfg_rate(payload)
            elif (cls_id, msg_id) == (0x06, 0x3E) and len(payload) >= 4:
                summary = self._decode_ubx_cfg_gnss(payload)
            elif (cls_id, msg_id) in ((0x05, 0x01), (0x05, 0x00)) and len(payload) >= 2:
                ack_cls, ack_id = struct.unpack_from("<BB", payload, 0)
                target = UBX_MSG_NAMES.get((ack_cls, ack_id), f"0x{ack_cls:02X}-0x{ack_id:02X}")
                kind = "ACK" if msg_id == 0x01 else "NAK"
                self.state.last_ack = f"{kind} {target}"
                summary = self.state.last_ack
        except Exception as exc:
            summary = f"decode err: {exc}"

        hex_prev = payload[:20].hex(" ").upper()
        if len(payload) > 20:
            hex_prev += " ..."
        self._record_packet("UBX", msg_name, len(payload) + 8, summary, hex_prev)

    def _decode_ubx_nav_pvt(self, payload: bytes) -> str:
        (
            itow, year, month, day, hour, minute, sec, valid,
            t_acc, nano, fix_type, flags, flags2, num_sv,
            lon_1e7, lat_1e7, height_mm, h_msl_mm, h_acc_mm, v_acc_mm,
            vel_n_mm, vel_e_mm, vel_d_mm, g_speed_mm, head_mot_1e5,
            s_acc_mm, head_acc_1e5, p_dop_100, _res1, head_veh_1e5,
            mag_dec_1e2, mag_acc_1e2
        ) = struct.unpack_from("<IHBBBBBBIiBBBBiiiiIIiiiiiIIH6sihH", payload, 0)

        s = self.state
        s.itow_ms = itow
        s.utc_year = year
        s.utc_month = month
        s.utc_day = day
        s.utc_hour = hour
        s.utc_min = minute
        s.utc_sec = sec
        s.utc_nano = nano
        s.valid_date = bool(valid & 0x01)
        s.valid_time = bool(valid & 0x02)
        s.fully_resolved_time = bool(valid & 0x04)
        s.t_acc_ns = t_acc
        s.fix_type = fix_type
        s.fix_ok = bool(flags & 0x01)
        s.diff_soln = bool(flags & 0x02)
        s.carr_soln = (flags >> 6) & 0x03
        s.num_sv_used = num_sv

        if fix_type >= 2 and s.fix_ok:
            s.lat = lat_1e7 * 1e-7
            s.lon = lon_1e7 * 1e-7
            s.alt_hae_m = height_mm / 1000.0
            s.alt_msl_m = h_msl_mm / 1000.0
            s.geoid_sep_m = s.alt_hae_m - s.alt_msl_m
            s.drift_tracker.add_fix(s.lat, s.lon, s.alt_msl_m)
            if math.isnan(s.ecef_x_m):
                s.ecef_x_m, s.ecef_y_m, s.ecef_z_m = lla_to_ecef(s.lat, s.lon, s.alt_hae_m)

        s.h_acc_m = h_acc_mm / 1000.0
        s.v_acc_m = v_acc_mm / 1000.0
        s.vel_n_ms = vel_n_mm / 1000.0
        s.vel_e_ms = vel_e_mm / 1000.0
        s.vel_d_ms = vel_d_mm / 1000.0
        s.speed_ms = g_speed_mm / 1000.0
        s.speed_kmh = s.speed_ms * 3.6
        s.speed_knots = s.speed_ms * 1.943844
        s.course_deg = head_mot_1e5 * 1e-5
        s.s_acc_ms = s_acc_mm / 1000.0
        s.head_acc_deg = head_acc_1e5 * 1e-5
        if p_dop_100 < 9900:
            s.pdop = p_dop_100 * 0.01
        s.mag_dec_deg = mag_dec_1e2 * 1e-2
        s.mag_acc_deg = mag_acc_1e2 * 1e-2

        return (
            f"fix={s.fix_type}D sv={num_sv} lat={s.lat:.6f} lon={s.lon:.6f} "
            f"hAcc={s.h_acc_m:.2f}m vAcc={s.v_acc_m:.2f}m tAcc={t_acc}ns"
        )

    def _decode_ubx_nav_dop(self, payload: bytes) -> str:
        itow, gdop, pdop, tdop, vdop, hdop, ndop, edop = struct.unpack_from("<I7H", payload, 0)
        s = self.state
        s.gdop = gdop * 0.01
        s.pdop = pdop * 0.01
        s.tdop = tdop * 0.01
        s.vdop = vdop * 0.01
        s.hdop = hdop * 0.01
        s.ndop = ndop * 0.01
        s.edop = edop * 0.01
        return f"G={s.gdop:.2f} P={s.pdop:.2f} H={s.hdop:.2f} V={s.vdop:.2f} T={s.tdop:.2f}"

    def _decode_ubx_nav_sat(self, payload: bytes) -> str:
        itow, version, num_svs, _ = struct.unpack_from("<IBBH", payload, 0)
        now = time.time()
        used_count = 0
        for i in range(num_svs):
            offset = 8 + i * 12
            if offset + 12 > len(payload):
                break
            gnss_id, sv_id, cno, elev, azim, pr_res_10cm, flags = struct.unpack_from(
                "<BBBbhhi", payload, offset
            )
            constellation = GNSS_NAMES.get(gnss_id, f"GN{gnss_id}")
            prefix = GNSS_PREFIX.get(constellation, "U")
            key = f"{prefix}{sv_id:02d}"

            sat = self.state.satellites.get(key)
            if not sat:
                sat = SatelliteInfo(key=key, constellation=constellation, sv_id=sv_id)
                self.state.satellites[key] = sat

            sat.cno = cno
            sat.elev = elev if -90 <= elev <= 90 else None
            sat.azim = azim if 0 <= azim <= 360 else None
            sat.pr_res_m = pr_res_10cm * 0.1
            sat.quality_ind = flags & 0x07
            sat.used = bool((flags >> 3) & 0x01)
            sat.health = (flags >> 4) & 0x03
            sat.diff_corr = bool((flags >> 6) & 0x01)
            sat.smoothed = bool((flags >> 7) & 0x01)
            sat.orbit_source = (flags >> 8) & 0x07
            sat.eph_avail = bool((flags >> 11) & 0x01)
            sat.alm_avail = bool((flags >> 12) & 0x01)
            sat.aop_avail = bool((flags >> 14) & 0x01)
            sat.sbas_corr = bool((flags >> 16) & 0x01)
            sat.pr_corr = bool((flags >> 20) & 0x01)
            sat.cr_corr = bool((flags >> 21) & 0x01)
            sat.do_corr = bool((flags >> 22) & 0x01)
            sat.last_seen = now
            if sat.elev is not None:
                sat.slant_range_km, sat.prop_delay_ms = sat_slant_range_and_delay(
                    constellation, sv_id, sat.elev
                )
            if sat.used:
                used_count += 1

        self.state.num_sv_visible = len(self.state.satellites)
        return f"numSvs={num_svs} navUsed={used_count}"

    def _decode_ubx_nav_orb(self, payload: bytes) -> str:
        itow, version, num_sv, _ = struct.unpack_from("<IBBH", payload, 0)
        eph_cnt = 0
        alm_cnt = 0
        for i in range(num_sv):
            offset = 8 + i * 6
            if offset + 6 > len(payload):
                break
            gnss_id, sv_id, sv_flag, eph, alm, other_orb = struct.unpack_from("<BBBBBB", payload, offset)
            constellation = GNSS_NAMES.get(gnss_id, f"GN{gnss_id}")
            prefix = GNSS_PREFIX.get(constellation, "U")
            key = f"{prefix}{sv_id:02d}"
            sat = self.state.satellites.get(key)
            if sat:
                sat.eph_avail = (eph & 0x1F) not in (0, 31)
                sat.alm_avail = (alm & 0x1F) not in (0, 31)
                sat.aop_avail = (other_orb & 0x1F) not in (0, 31)
            if (eph & 0x1F) not in (0, 31):
                eph_cnt += 1
            if (alm & 0x1F) not in (0, 31):
                alm_cnt += 1
        return f"numSv={num_sv} eph={eph_cnt} alm={alm_cnt}"

    def _decode_ubx_nav_clock(self, payload: bytes) -> str:
        itow, clk_b, clk_d, t_acc, f_acc = struct.unpack_from("<IiiII", payload, 0)
        s = self.state
        s.clk_bias_ns = clk_b
        s.clk_drift_nss = clk_d
        s.t_acc_ns = t_acc
        s.f_acc_pss = f_acc
        return f"bias={clk_b}ns drift={clk_d}ns/s tAcc={t_acc}ns fAcc={f_acc}ps/s"

    def _decode_ubx_nav_status(self, payload: bytes) -> str:
        itow, gps_fix, flags, fix_stat, flags2, ttff, msss = struct.unpack_from("<IBBBBII", payload, 0)
        s = self.state
        s.ttff_ms = ttff
        s.uptime_ms = msss
        s.spoof_state = (flags2 >> 3) & 0x03
        return f"ttff={ttff/1000.0:.2f}s uptime={msss/1000.0:.0f}s spoof={s.spoof_state}"

    def _decode_ubx_nav_timegps(self, payload: bytes) -> str:
        itow, ftow, week, leap_s, valid, t_acc = struct.unpack_from("<IihbBI", payload, 0)
        s = self.state
        s.itow_ms = itow
        s.ftow_ns = ftow
        s.gps_week = week
        s.leap_sec = leap_s
        s.leap_sec_valid = bool(valid & 0x04)
        return f"week={week} iTOW={itow/1000.0:.3f}s leapS={leap_s}s"

    def _decode_ubx_nav_posecef(self, payload: bytes) -> str:
        itow, x_cm, y_cm, z_cm, p_acc_cm = struct.unpack_from("<IiiiI", payload, 0)
        s = self.state
        s.ecef_x_m = x_cm / 100.0
        s.ecef_y_m = y_cm / 100.0
        s.ecef_z_m = z_cm / 100.0
        s.p_acc_m = p_acc_cm / 100.0
        return f"X={s.ecef_x_m:.2f}m Y={s.ecef_y_m:.2f}m Z={s.ecef_z_m:.2f}m pAcc={s.p_acc_m:.2f}m"

    def _decode_ubx_nav_velecef(self, payload: bytes) -> str:
        itow, vx_cm, vy_cm, vz_cm, s_acc_cm = struct.unpack_from("<IiiiI", payload, 0)
        s = self.state
        s.ecef_vx_ms = vx_cm / 100.0
        s.ecef_vy_ms = vy_cm / 100.0
        s.ecef_vz_ms = vz_cm / 100.0
        return f"VX={s.ecef_vx_ms:.2f} VY={s.ecef_vy_ms:.2f} VZ={s.ecef_vz_ms:.2f} m/s"

    def _decode_ubx_nav_sbas(self, payload: bytes) -> str:
        itow, geo, mode, sys_id, service, cnt = struct.unpack_from("<IBBbBB", payload, 0)
        s = self.state
        s.sbas_geo_prn = geo
        s.sbas_mode = mode
        s.sbas_sys = sys_id
        return f"sys={SBAS_SYS_NAMES.get(sys_id, str(sys_id))} mode={mode} geo={geo} svs={cnt}"

    def _decode_ubx_nav_aopstatus(self, payload: bytes) -> str:
        itow, aop_cfg, status = struct.unpack_from("<IBB", payload, 0)
        self.state.aop_cfg = aop_cfg
        self.state.aop_status = status
        return f"aopCfg={aop_cfg} status={'Running' if status else 'Idle'}"

    def _decode_ubx_mon_ver(self, payload: bytes) -> str:
        sw_raw = payload[:30].split(b"\x00", 1)[0].decode("ascii", errors="ignore").strip()
        hw_raw = payload[30:40].split(b"\x00", 1)[0].decode("ascii", errors="ignore").strip()
        exts: List[str] = []
        offset = 40
        while offset + 30 <= len(payload):
            ext = payload[offset : offset + 30].split(b"\x00", 1)[0].decode("ascii", errors="ignore").strip()
            if ext:
                exts.append(ext)
            offset += 30

        s = self.state
        if sw_raw:
            s.sw_version = sw_raw
        if hw_raw:
            hw_map = {"00080000": "UBX-M8130 (00080000)", "00070000": "UBX-M7020 (00070000)"}
            s.hw_version = hw_map.get(hw_raw, f"UBX-{hw_raw}")
        s.ver_extensions = exts
        gnss_parts = []
        for ext in exts:
            if ext.startswith("FWVER="):
                s.fw_version = ext.split("=", 1)[1]
            elif ext.startswith("PROTVER="):
                s.prot_version = ext.split("=", 1)[1]
            elif any(tok in ext for tok in ("GPS", "GLO", "BDS", "GAL", "QZSS", "SBAS")) and "=" not in ext:
                gnss_parts.append(ext)
        if gnss_parts:
            s.supported_gnss_str = ";".join(gnss_parts)
        return f"HW={s.hw_version} FW={s.fw_version} PROT={s.prot_version}"

    def _decode_ubx_mon_hw(self, payload: bytes) -> str:
        (
            pin_sel, pin_bank, pin_dir, pin_val,
            noise_per_ms, agc_cnt, a_status, a_power, flags, _res1,
            used_mask, vp, jam_ind
        ) = struct.unpack_from("<IIIIHHBBBBI17sB", payload, 0)
        s = self.state
        s.noise_per_ms = noise_per_ms
        s.agc_cnt = agc_cnt
        s.ant_status = a_status
        s.ant_power = a_power
        s.rtc_calibrated = bool(flags & 0x01)
        s.jamming_state = (flags >> 2) & 0x03
        s.xtal_absent = bool(flags & 0x10)
        s.jam_ind = jam_ind
        return (
            f"noise={noise_per_ms} agc={s.agc_percent:.1f}% "
            f"ant={ANT_STATUS_NAMES.get(a_status, '?')} jam={jam_ind}/255"
        )

    def _decode_ubx_mon_hw2(self, payload: bytes) -> str:
        ofs_i, mag_i, ofs_q, mag_q, cfg_src = struct.unpack_from("<bBbBB", payload, 0)
        s = self.state
        s.ofs_i = ofs_i
        s.mag_i = mag_i
        s.ofs_q = ofs_q
        s.mag_q = mag_q
        return f"I=({ofs_i:+d}, mag {mag_i}) Q=({ofs_q:+d}, mag {mag_q})"

    def _decode_ubx_mon_io(self, payload: bytes) -> str:
        port_names = ["I2C/DDC", "UART1", "UART2", "USB", "SPI", "Port5"]
        ports: List[IoPortStats] = []
        n_ports = len(payload) // 20
        for i in range(n_ports):
            rx_b, tx_b, par_e, frm_e, ovr_e, brk_c, rx_busy, tx_busy = struct.unpack_from(
                "<IIHHHBB", payload, i * 20
            )
            pname = port_names[i] if i < len(port_names) else f"Port{i}"
            ports.append(
                IoPortStats(
                    port_id=i,
                    port_name=pname,
                    rx_bytes=rx_b,
                    tx_bytes=tx_b,
                    parity_errs=par_e,
                    framing_errs=frm_e,
                    overrun_errs=ovr_e,
                    break_cond=brk_c,
                    rx_busy=rx_busy,
                    tx_busy=tx_busy,
                )
            )
        self.state.io_ports = ports
        usb_tx = ports[3].tx_bytes if len(ports) > 3 else 0
        return f"ports={n_ports} USB_TX={usb_tx}B"

    def _decode_ubx_mon_gnss(self, payload: bytes) -> str:
        ver, supported, default_g, enabled, simul = struct.unpack_from("<BBBBB", payload, 0)
        return f"supported=0x{supported:02X} enabled=0x{enabled:02X} maxSimul={simul}"

    def _decode_ubx_cfg_nav5(self, payload: bytes) -> str:
        mask, dyn_model, fix_mode, fixed_alt, fixed_alt_var, min_elev = struct.unpack_from(
            "<HBBiIb", payload, 0
        )
        self.state.dyn_model = dyn_model
        self.state.fix_mode_cfg = fix_mode
        self.state.min_elev_deg = min_elev
        return f"dynModel={DYN_MODEL_NAMES.get(dyn_model, str(dyn_model))} minElev={min_elev}°"

    def _decode_ubx_cfg_rate(self, payload: bytes) -> str:
        meas_rate, nav_rate, time_ref = struct.unpack_from("<HHH", payload, 0)
        self.state.meas_rate_ms = meas_rate
        self.state.nav_rate_cycles = nav_rate
        self.state.time_ref = time_ref
        return f"meas={meas_rate}ms ({self.state.update_rate_hz:.1f}Hz) navRate={nav_rate}"

    def _decode_ubx_cfg_gnss(self, payload: bytes) -> str:
        msg_ver, num_hw, num_use, num_blocks = struct.unpack_from("<BBBB", payload, 0)
        self.state.num_trk_ch_hw = num_hw
        self.state.num_trk_ch_use = num_use
        blocks: List[GnssChannelBlock] = []
        for i in range(num_blocks):
            offset = 4 + i * 8
            if offset + 8 > len(payload):
                break
            gnss_id, res_ch, max_ch, _res, flags = struct.unpack_from("<BBBBI", payload, offset)
            gname = GNSS_NAMES.get(gnss_id, f"GN{gnss_id}")
            enabled = bool(flags & 0x01)
            sig_mask = (flags >> 16) & 0xFF
            blocks.append(
                GnssChannelBlock(
                    gnss_name=gname,
                    res_trk_ch=res_ch,
                    max_trk_ch=max_ch,
                    enabled=enabled,
                    sig_cfg_mask=sig_mask,
                )
            )
        self.state.gnss_blocks = blocks
        active = [b.gnss_name for b in blocks if b.enabled]
        return f"hwCh={num_hw} active={','.join(active)}"

    # -------------------------------------------------------------------------
    # NMEA 0183 Sentence Decoders
    # -------------------------------------------------------------------------
    def _handle_nmea_line(self, line: str) -> None:
        if not line.startswith("$"):
            return

        # Verify NMEA XOR checksum if '*' present
        if "*" in line:
            body, ck_str = line[1:].rsplit("*", 1)
            try:
                expected_ck = int(ck_str[:2], 16)
            except ValueError:
                self.state.nmea_checksum_err_count += 1
                return
            calc_ck = 0
            for ch in body:
                calc_ck ^= ord(ch)
            if calc_ck != expected_ck:
                self.state.nmea_checksum_err_count += 1
                return
        else:
            body = line[1:]

        self.state.nmea_valid_count += 1
        fields = body.split(",")
        talker_sentence = fields[0]
        msg_tag = f"${talker_sentence}"
        talker = talker_sentence[:2]
        sentence_type = talker_sentence[2:]

        summary = ""
        try:
            if sentence_type == "RMC":
                summary = self._decode_nmea_rmc(fields)
            elif sentence_type == "GGA":
                summary = self._decode_nmea_gga(fields)
            elif sentence_type == "GSA":
                summary = self._decode_nmea_gsa(talker, fields)
            elif sentence_type == "GSV":
                summary = self._decode_nmea_gsv(talker, fields)
            elif sentence_type == "VTG":
                summary = self._decode_nmea_vtg(fields)
            elif sentence_type == "GLL":
                summary = self._decode_nmea_gll(fields)
            elif sentence_type == "TXT":
                summary = self._decode_nmea_txt(fields)
        except Exception as exc:
            summary = f"err: {exc}"

        self._record_packet("NMEA", msg_tag, len(line) + 2, summary, line)

    @staticmethod
    def _parse_nmea_latlon(val_str: str, hemi: str) -> float:
        if not val_str or not hemi:
            return float("nan")
        dot = val_str.find(".")
        if dot < 2:
            return float("nan")
        deg_Part = float(val_str[: dot - 2])
        min_part = float(val_str[dot - 2 :])
        dec = deg_Part + min_part / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec

    def _decode_nmea_rmc(self, f: List[str]) -> str:
        # $GNRMC,220228.00,A,3759.24511,N,12234.68369,W,0.010,,281026,,,A,V
        status = f[2] if len(f) > 2 else "V"
        lat = self._parse_nmea_latlon(f[3], f[4]) if len(f) > 4 else float("nan")
        lon = self._parse_nmea_latlon(f[5], f[6]) if len(f) > 6 else float("nan")
        sog_kn = float(f[7]) if len(f) > 7 and f[7] else 0.0
        cog = float(f[8]) if len(f) > 8 and f[8] else float("nan")
        if len(f) > 12 and f[12]:
            self.state.faa_mode = f[12]
        if len(f) > 13 and f[13]:
            self.state.nav_status_char = f[13]

        # If UBX-NAV-PVT hasn't populated coordinates yet, use NMEA
        if not math.isnan(lat) and not math.isnan(lon) and math.isnan(self.state.lat):
            self.state.lat = lat
            self.state.lon = lon
            self.state.speed_knots = sog_kn
            self.state.speed_ms = sog_kn / 1.943844
            self.state.speed_kmh = self.state.speed_ms * 3.6
            if not math.isnan(cog):
                self.state.course_deg = cog
        return f"status={status} mode={self.state.faa_mode} sog={sog_kn:.3f}kn"

    def _decode_nmea_gga(self, f: List[str]) -> str:
        # $GNGGA,220228.00,3759.24511,N,12234.68369,W,1,12,0.73,28.3,M,-29.6,M,,
        qual = int(f[6]) if len(f) > 6 and f[6] else 0
        num_sv = int(f[7]) if len(f) > 7 and f[7] else 0
        hdop = float(f[8]) if len(f) > 8 and f[8] else float("nan")
        alt_msl = float(f[9]) if len(f) > 9 and f[9] else float("nan")
        geoid = float(f[11]) if len(f) > 11 and f[11] else float("nan")

        self.state.gga_quality = qual
        if not math.isnan(geoid):
            self.state.geoid_sep_m = geoid
        if math.isnan(self.state.alt_msl_m) and not math.isnan(alt_msl):
            self.state.alt_msl_m = alt_msl
        if math.isnan(self.state.hdop) and not math.isnan(hdop):
            self.state.hdop = hdop
        return f"qual={qual} sv={num_sv} hdop={hdop:.2f} alt={alt_msl:.1f}m geoid={geoid:.1f}m"

    def _decode_nmea_gsa(self, talker: str, f: List[str]) -> str:
        # $GNGSA,A,3,14,04,05,09,21,07,30,08,22,27,,,1.37,0.73,1.16,1
        fix_mode = int(f[2]) if len(f) > 2 and f[2] else 1
        prns = [int(x) for x in f[3:15] if x.isdigit()]
        pdop = float(f[15]) if len(f) > 15 and f[15] else float("nan")
        hdop = float(f[16]) if len(f) > 16 and f[16] else float("nan")
        vdop = float(f[17]) if len(f) > 17 and f[17] else float("nan")
        sys_id = int(f[18]) if len(f) > 18 and f[18].isdigit() else 0

        sys_map = {1: "GPS", 2: "GLO", 3: "GAL", 4: "BDS", 5: "QZSS"}
        talker_map = {"GP": "GPS", "GL": "GLO", "GA": "GAL", "GB": "BDS", "BD": "BDS", "GQ": "QZSS"}
        constellation = sys_map.get(sys_id) or talker_map.get(talker, "GPS")
        self.state.gsa_active_prns[constellation] = set(prns)

        if self.state.fix_type == 0 and fix_mode >= 2:
            self.state.fix_type = fix_mode
        return f"{constellation} {fix_mode}D active={len(prns)} P/H/V={pdop:.2f}/{hdop:.2f}/{vdop:.2f}"

    def _decode_nmea_gsv(self, talker: str, f: List[str]) -> str:
        # $GPGSV,3,1,12, 04,09,143,25, 05,19,299,26, ..., 0
        talker_map = {
            "GP": "GPS",
            "GL": "GLO",
            "GA": "GAL",
            "GB": "BDS",
            "BD": "BDS",
            "GQ": "QZSS",
        }
        constellation = talker_map.get(talker, "GPS")
        prefix = GNSS_PREFIX.get(constellation, "G")
        total_msgs = int(f[1]) if len(f) > 1 and f[1].isdigit() else 1
        msg_num = int(f[2]) if len(f) > 2 and f[2].isdigit() else 1
        total_sv = int(f[3]) if len(f) > 3 and f[3].isdigit() else 0

        # NMEA 4.10 appends a trailing signalId field if (len(f) - 4) % 4 == 1
        end_idx = len(f) - 1 if (len(f) - 4) % 4 == 1 else len(f)
        now = time.time()
        idx = 4
        parsed = 0
        while idx + 3 < end_idx:
            prn_s, el_s, az_s, snr_s = f[idx : idx + 4]
            idx += 4
            if not prn_s.isdigit():
                continue
            sv_id = int(prn_s)
            key = f"{prefix}{sv_id:02d}"
            sat = self.state.satellites.get(key)
            if not sat:
                sat = SatelliteInfo(key=key, constellation=constellation, sv_id=sv_id)
                self.state.satellites[key] = sat
            if el_s.lstrip("-").isdigit():
                sat.elev = int(el_s)
                sat.slant_range_km, sat.prop_delay_ms = sat_slant_range_and_delay(
                    constellation, sv_id, sat.elev
                )
            if az_s.isdigit():
                sat.azim = int(az_s)
            if snr_s.isdigit():
                sat.cno = int(snr_s)
                if sat.quality_ind == 0 and sat.cno > 0:
                    sat.quality_ind = 4
            if sv_id in self.state.gsa_active_prns.get(constellation, set()):
                sat.used = True
            sat.last_seen = now
            parsed += 1

        self.state.num_sv_visible = len(self.state.satellites)
        return f"{constellation} msg {msg_num}/{total_msgs} inView={total_sv} (+{parsed})"

    def _decode_nmea_vtg(self, f: List[str]) -> str:
        sog_kn = float(f[5]) if len(f) > 5 and f[5] else 0.0
        sog_kmh = float(f[7]) if len(f) > 7 and f[7] else 0.0
        return f"sog={sog_kn:.3f}kn ({sog_kmh:.3f}km/h)"

    def _decode_nmea_gll(self, f: List[str]) -> str:
        status = f[6] if len(f) > 6 else "?"
        return f"status={status} utc={f[5] if len(f) > 5 else ''}"

    def _decode_nmea_txt(self, f: List[str]) -> str:
        txt = f[4] if len(f) > 4 else ""
        if txt and txt not in self.state.txt_messages:
            self.state.txt_messages.append(txt)
        return txt


class DemoGenerator:
    """Realistic u-blox M8 GNSS simulator when run with --demo."""

    def __init__(self, state: GPSState) -> None:
        self.state = state
        self.t0 = time.time()
        self.last_step = 0.0
        s = self.state
        s.device_path = "/dev/cu.usbmodem2101 (SIMULATED)"
        s.connected = True
        s.hw_version = "UBX-M8130"
        s.sw_version = "ROM CORE 3.01 (107888)"
        s.fw_version = "SPG 3.01"
        s.prot_version = "18.00"
        s.supported_gnss_str = "GPS;GLO;BDS;QZSS"
        s.fix_type = 3
        s.fix_ok = True
        s.gga_quality = 1
        s.faa_mode = "A"
        s.ttff_ms = 1420
        s.gps_week = 2442
        s.leap_sec = 18
        s.leap_sec_valid = True
        s.valid_date = True
        s.valid_time = True
        s.fully_resolved_time = True
        s.gnss_blocks = [
            GnssChannelBlock("GPS", 8, 16, True, 1),
            GnssChannelBlock("SBAS", 1, 3, True, 1),
            GnssChannelBlock("GAL", 0, 8, False, 1),
            GnssChannelBlock("BDS", 8, 16, True, 1),
            GnssChannelBlock("IMES", 0, 8, False, 1),
            GnssChannelBlock("QZSS", 0, 3, True, 1),
            GnssChannelBlock("GLO", 8, 14, False, 1),
        ]

    def poll_step(self, timeout_s: float = 0.02) -> None:
        time.sleep(timeout_s)
        now = time.time()
        if now - self.last_step < 0.25:
            return
        self.last_step = now
        dt = now - self.t0
        s = self.state
        s.update_throughput(540)
        s.nmea_valid_count += 9
        s.ubx_valid_count += 10
        s.uptime_ms = int((3600 + dt) * 1000)

        # Simulated position near Marin County, CA (matching real device fix)
        base_lat = 37.9874185
        base_lon = -122.5780615
        wander_n = 0.35 * math.sin(dt * 0.3) + 0.15 * math.cos(dt * 1.1)
        wander_e = 0.30 * math.cos(dt * 0.25) + 0.12 * math.sin(dt * 0.9)
        s.lat = base_lat + (wander_n / 6371000.0) * (180.0 / math.pi)
        s.lon = base_lon + (wander_e / (6371000.0 * math.cos(math.radians(base_lat)))) * (180.0 / math.pi)
        s.alt_msl_m = 28.3 + 0.4 * math.sin(dt * 0.15)
        s.geoid_sep_m = -29.6
        s.alt_hae_m = s.alt_msl_m + s.geoid_sep_m
        s.h_acc_m = 0.85 + 0.05 * math.sin(dt * 0.4)
        s.v_acc_m = 1.25 + 0.08 * math.cos(dt * 0.3)
        s.p_acc_m = math.hypot(s.h_acc_m, s.v_acc_m)
        s.ecef_x_m, s.ecef_y_m, s.ecef_z_m = lla_to_ecef(s.lat, s.lon, s.alt_hae_m)
        s.drift_tracker.add_fix(s.lat, s.lon, s.alt_msl_m)

        s.gdop = 1.62
        s.pdop = 1.37
        s.hdop = 0.73
        s.vdop = 1.16
        s.tdop = 0.86
        s.ndop = 0.54
        s.edop = 0.49

        gm = time.gmtime(now)
        s.utc_year, s.utc_month, s.utc_day = gm.tm_year, gm.tm_mon, gm.tm_mday
        s.utc_hour, s.utc_min, s.utc_sec = gm.tm_hour, gm.tm_min, gm.tm_sec
        s.utc_nano = int((now - int(now)) * 1e9)
        s.clk_bias_ns = int(142500 + 180 * math.sin(dt * 0.2))
        s.clk_drift_nss = int(-42 + 4 * math.cos(dt * 0.5))
        s.t_acc_ns = 14
        s.f_acc_pss = 260
        s.noise_per_ms = int(98 + 4 * math.sin(dt * 0.7))
        s.agc_cnt = int(3850 + 60 * math.cos(dt * 0.4))
        s.jam_ind = int(6 + 2 * math.sin(dt * 0.6))
        s.ofs_i, s.mag_i, s.ofs_q, s.mag_q = -2, 168, 1, 167

        demo_sats = [
            ("G04", "GPS", 4, 25, 9, 143, True, -0.4),
            ("G05", "GPS", 5, 27, 19, 299, True, 0.3),
            ("G07", "GPS", 7, 14, 64, 48, True, 1.1),
            ("G08", "GPS", 8, 24, 37, 69, True, -0.2),
            ("G09", "GPS", 9, 37, 41, 153, True, 0.1),
            ("G14", "GPS", 14, 29, 53, 237, True, -0.5),
            ("G17", "GPS", 17, 23, 8, 182, False, 1.8),
            ("G21", "GPS", 21, 30, 28, 264, True, 0.2),
            ("G22", "GPS", 22, 34, 31, 229, True, -0.1),
            ("G27", "GPS", 27, 21, 10, 39, True, 0.7),
            ("G30", "GPS", 30, 23, 61, 319, True, -0.6),
            ("B13", "BDS", 13, 17, 8, 145, False, 2.1),
            ("B20", "BDS", 20, 27, 21, 318, True, -0.3),
            ("B23", "BDS", 23, 12, 73, 346, True, 0.9),
            ("B25", "BDS", 25, 27, 36, 58, True, 0.0),
        ]
        for key, const, sv_id, cno, el, az, used, pr_res in demo_sats:
            sr_km, delay_ms = sat_slant_range_and_delay(const, sv_id, el)
            s.satellites[key] = SatelliteInfo(
                key=key,
                constellation=const,
                sv_id=sv_id,
                cno=max(5, int(cno + 2 * math.sin(dt + sv_id))),
                elev=el,
                azim=az,
                used=used,
                quality_ind=7 if used else 4,
                pr_res_m=pr_res,
                health=1,
                smoothed=used,
                orbit_source=1,
                eph_avail=True,
                alm_avail=True,
                slant_range_km=sr_km,
                prop_delay_ms=delay_ms,
                last_seen=now,
            )
        s.num_sv_used = sum(1 for x in s.satellites.values() if x.used)
        s.num_sv_visible = len(s.satellites)
        s.record_history_tick()
