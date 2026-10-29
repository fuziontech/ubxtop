#!/usr/bin/env python3
"""
GPS-TUI: Geek-Grade GNSS & u-blox UBX / NMEA-0183 Terminal Diagnostic Suite.
Uses a pure-ANSI 24-bit TrueColor Cell-Grid Renderer with DEC Mode 2026 Synchronized
Output for Ghostty and modern GPU terminals (zero libncurses UTF-8 width bugs).
"""

from __future__ import annotations
import argparse
import math
import os
import select
import shutil
import signal
import sys
import termios
import time
import tty
from typing import List, Optional, Tuple

from gps_math import (
    compass_rose,
    compute_astro_info,
    dop_rating,
    format_dmm,
    format_dms,
    latlon_to_geohash,
    latlon_to_maidenhead,
    latlon_to_utm,
)
from gps_protocol import (
    ANT_POWER_NAMES,
    ANT_STATUS_NAMES,
    DYN_MODEL_NAMES,
    JAMMING_STATE_NAMES,
    SBAS_SYS_NAMES,
    SPOOF_STATE_NAMES,
    DemoGenerator,
    GPSConnection,
    GPSState,
)
from gps_widgets import (
    STYLE_BAD,
    STYLE_BAD_BOLD,
    STYLE_BANNER,
    STYLE_BOLD,
    STYLE_BORDER,
    STYLE_CYAN,
    STYLE_CYAN_BOLD,
    STYLE_DEFAULT,
    STYLE_DIM,
    STYLE_FOOTER,
    STYLE_GOOD,
    STYLE_GOOD_BOLD,
    STYLE_MAGENTA,
    STYLE_MUTED,
    STYLE_RESET,
    STYLE_TAB_ACTIVE,
    STYLE_TAB_BAR_BG,
    STYLE_TAB_INACTIVE,
    STYLE_TBL_HEADER,
    STYLE_TITLE,
    STYLE_TOPBAR,
    STYLE_TOPBAR_FIX_BAD,
    STYLE_TOPBAR_FIX_OK,
    STYLE_TOPBAR_FIX_WARN,
    STYLE_WARN,
    STYLE_WARN_BOLD,
    ScreenBuffer,
    build_scatter_canvas,
    build_skyplot_canvas,
    draw_sparkline,
    progress_bar_str,
    render_braille_timeseries,
)


class GPSTuiApp:
    """Pure-ANSI 24-bit TrueColor TUI for deep GNSS & u-blox hardware diagnostics."""

    TABS = [
        "1:Overview",
        "2:Satellites & Orbits",
        "3:Scatter & Drift",
        "4:RF, Clock & HW",
        "5:Protocol Sniffer",
    ]

    SORT_MODES = ["used_snr", "sv_id", "elev", "pr_res"]
    SORT_LABELS = {
        "used_snr": "Used + C/N0",
        "sv_id": "Constellation/PRN",
        "elev": "Elevation",
        "pr_res": "|Pseudorange Res|",
    }

    RATES_HZ = [1, 2, 5, 10]
    DYN_MODELS = [0, 2, 3, 4, 5, 6, 7, 8]

    def __init__(
        self,
        state: GPSState,
        conn: Optional[GPSConnection],
        demo: Optional[DemoGenerator] = None,
    ) -> None:
        self.state = state
        self.conn = conn
        self.demo = demo
        self.active_tab: int = 0
        self.sat_sort_idx: int = 0
        self.proto_filter: str = "ALL"  # "ALL", "NMEA", "UBX"
        self.logging_path: Optional[str] = None
        self.log_fp = None
        self.rate_idx: int = 0
        self.dyn_idx: int = 0
        self.status_banner: str = ""
        self.status_banner_t: float = 0.0
        self.running: bool = True

    def set_banner(self, msg: str) -> None:
        self.status_banner = msg
        self.status_banner_t = time.time()

    def toggle_logging(self) -> None:
        if self.log_fp is not None:
            try:
                self.log_fp.close()
            except Exception:
                pass
            self.log_fp = None
            if self.conn:
                self.conn.log_callback = None
            self.set_banner(f"Stopped logging ({self.logging_path})")
            self.logging_path = None
        else:
            fname = f"gps_diag_{time.strftime('%Y%m%d_%H%M%S')}.log"
            try:
                self.log_fp = open(fname, "a", encoding="utf-8")
                self.logging_path = fname
                if self.conn:
                    self.conn.log_callback = self._write_log_line
                self.set_banner(f"Logging packets to {fname}")
            except Exception as exc:
                self.set_banner(f"Log error: {exc}")

    def _write_log_line(self, line: str) -> None:
        if self.log_fp:
            try:
                self.log_fp.write(line + "\n")
                self.log_fp.flush()
            except Exception:
                pass

    @staticmethod
    def snr_style(cno: int, used: bool = False) -> str:
        if cno >= 32:
            return STYLE_GOOD_BOLD
        if cno >= 24:
            return STYLE_CYAN_BOLD if used else STYLE_CYAN
        if cno >= 15:
            return STYLE_WARN_BOLD if used else STYLE_WARN
        if cno > 0:
            return STYLE_BAD
        return STYLE_DIM

    @staticmethod
    def dop_style(dop: float) -> str:
        _, sev = dop_rating(dop)
        if sev == 0:
            return STYLE_GOOD_BOLD
        if sev == 1:
            return STYLE_CYAN_BOLD
        if sev == 2:
            return STYLE_WARN_BOLD
        return STYLE_BAD_BOLD

    def run(self) -> None:
        stdin_fd = sys.stdin.fileno()
        old_termios = termios.tcgetattr(stdin_fd)

        def _on_sig(signum, frame):
            self.running = False

        signal.signal(signal.SIGINT, _on_sig)
        signal.signal(signal.SIGTERM, _on_sig)

        # Enter alternate screen buffer, hide cursor, clear screen
        sys.stdout.write("\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H")
        sys.stdout.flush()

        try:
            tty.setcbreak(stdin_fd)
            screen: Optional[ScreenBuffer] = None

            while self.running:
                if self.demo:
                    self.demo.poll_step(0.02)
                elif self.conn:
                    self.conn.poll_step(0.02)

                self._poll_keyboard(stdin_fd)
                if not self.running:
                    break

                tsize = shutil.get_terminal_size((120, 36))
                w, h = tsize.columns, tsize.lines
                if screen is None or screen.w != w or screen.h != h:
                    screen = ScreenBuffer(w, h)
                else:
                    screen.clear()

                self.render_frame(screen)
                screen.flush_to_terminal()
        finally:
            try:
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_termios)
            except Exception:
                pass
            # Exit synchronized output, show cursor, restore normal screen buffer
            sys.stdout.write("\x1b[?2026l\x1b[0m\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()

    def _poll_keyboard(self, stdin_fd: int) -> None:
        while True:
            r, _, _ = select.select([stdin_fd], [], [], 0.0)
            if not r:
                break
            try:
                data = os.read(stdin_fd, 32)
            except OSError:
                break
            if not data:
                break
            if data in (b"q", b"Q", b"\x03"):
                self.running = False
                return
            if data == b"\x1b":
                self.running = False
                return
            if data in (b"\x1b[C", b"\t"):  # Right arrow or Tab
                self.active_tab = (self.active_tab + 1) % len(self.TABS)
            elif data in (b"\x1b[D", b"\x1b[Z"):  # Left arrow or Shift-Tab
                self.active_tab = (self.active_tab - 1) % len(self.TABS)
            else:
                ch = chr(data[0])
                self.handle_char(ch)

    def handle_char(self, ch: str) -> None:
        if ch in ("1", "2", "3", "4", "5"):
            self.active_tab = int(ch) - 1
        elif ch in ("s", "S"):
            self.sat_sort_idx = (self.sat_sort_idx + 1) % len(self.SORT_MODES)
            mode = self.SORT_MODES[self.sat_sort_idx]
            self.set_banner(f"Satellite sort: {self.SORT_LABELS[mode]}")
        elif ch in ("c", "C"):
            self.state.drift_tracker.reset()
            self.set_banner("Cleared position scatter & drift history")
        elif ch in ("a", "A"):
            if not math.isnan(self.state.lat) and not math.isnan(self.state.lon):
                if self.state.drift_tracker.anchor is None:
                    self.state.drift_tracker.anchor = (
                        self.state.lat,
                        self.state.lon,
                        self.state.alt_msl_m if not math.isnan(self.state.alt_msl_m) else 0.0,
                    )
                    self.set_banner("Locked scatter reference anchor to current fix")
                else:
                    self.state.drift_tracker.anchor = None
                    self.set_banner("Switched scatter reference to running mean centroid")
        elif ch in ("r", "R"):
            if self.conn:
                self.rate_idx = (self.rate_idx + 1) % len(self.RATES_HZ)
                hz = self.RATES_HZ[self.rate_idx]
                self.conn.set_measurement_rate_hz(hz)
                self.set_banner(f"Sent UBX-CFG-RATE -> {hz} Hz measurement rate")
        elif ch in ("m", "M"):
            if self.conn:
                self.dyn_idx = (self.dyn_idx + 1) % len(self.DYN_MODELS)
                dm = self.DYN_MODELS[self.dyn_idx]
                self.conn.set_dynamic_model(dm)
                self.set_banner(f"Sent UBX-CFG-NAV5 -> Dynamic Model: {DYN_MODEL_NAMES.get(dm, str(dm))}")
        elif ch in (" ", "p", "P"):
            if self.conn:
                self.conn.paused = not self.conn.paused
                self.set_banner("Packet stream PAUSED" if self.conn.paused else "Packet stream RESUMED")
        elif ch in ("f", "F"):
            order = ["ALL", "NMEA", "UBX"]
            self.proto_filter = order[(order.index(self.proto_filter) + 1) % len(order)]
            self.set_banner(f"Protocol sniffer filter: {self.proto_filter}")
        elif ch in ("l", "L"):
            self.toggle_logging()
        elif ch in ("v", "V"):
            if self.conn:
                self.conn.poll_static_info()
                self.conn.poll_fast_telemetry()
                self.set_banner("Polled all UBX hardware & navigation registers")

    def render_frame(self, scr: ScreenBuffer) -> None:
        if scr.h < 16 or scr.w < 60:
            scr.put_str(
                0, 0,
                f"Terminal too small ({scr.w}x{scr.h}). Please resize to at least 80x24.",
                STYLE_WARN_BOLD,
            )
            return

        self.draw_top_bar(scr)
        content_y = 2
        content_h = scr.h - 3
        if self.active_tab == 0:
            self.draw_tab_overview(scr, content_y, content_h, scr.w)
        elif self.active_tab == 1:
            self.draw_tab_satellites(scr, content_y, content_h, scr.w)
        elif self.active_tab == 2:
            self.draw_tab_scatter(scr, content_y, content_h, scr.w)
        elif self.active_tab == 3:
            self.draw_tab_rf_hw(scr, content_y, content_h, scr.w)
        elif self.active_tab == 4:
            self.draw_tab_protocol(scr, content_y, content_h, scr.w)
        self.draw_footer(scr, scr.h - 1)

    # -------------------------------------------------------------------------
    # Header & Footer
    # -------------------------------------------------------------------------
    def draw_top_bar(self, scr: ScreenBuffer) -> None:
        s = self.state
        w = scr.w
        scr.fill_row(0, 0, w, " ", STYLE_TOPBAR)
        title = f" ubxtop │ {s.hw_version} ({s.fw_version} / UBX {s.prot_version}) │ {s.device_path} "
        scr.put_str(0, 0, title, STYLE_TOPBAR, max_x=w)

        fix_str = f" {s.fix_badge} ({s.num_sv_used}/{s.num_sv_visible} SVs) "
        fix_style = (
            STYLE_TOPBAR_FIX_OK
            if s.fix_type >= 3
            else (STYLE_TOPBAR_FIX_WARN if s.fix_type == 2 else STYLE_TOPBAR_FIX_BAD)
        )
        right_str = (
            f" RX:{s.rx_rate_bps:5.0f}B/s │ NMEA:{s.nmea_valid_count} UBX:{s.ubx_valid_count} "
            f"│ {s.update_rate_hz:.0f}Hz "
        )
        rx_x = w - len(right_str)
        fix_x = rx_x - len(fix_str)
        if fix_x > len(title):
            scr.put_str(0, fix_x, fix_str, fix_style, max_x=w)
        if rx_x > 0:
            scr.put_str(0, rx_x, right_str, STYLE_TOPBAR, max_x=w)

        # Row 1: Navigation Tabs + UTC timestamp
        scr.fill_row(1, 0, w, " ", STYLE_TAB_BAR_BG)
        x = 1
        for i, tab_name in enumerate(self.TABS):
            if i == self.active_tab:
                lbl = f" ▸ {tab_name} "
                scr.put_str(1, x, lbl, STYLE_TAB_ACTIVE, max_x=w)
            else:
                lbl = f"   {tab_name} "
                scr.put_str(1, x, lbl, STYLE_TAB_INACTIVE, max_x=w)
            x += len(lbl) + 1

        utc_str = f" {s.utc_iso_str} "
        if w - len(utc_str) > x:
            scr.put_str(1, w - len(utc_str), utc_str, STYLE_TAB_ACTIVE, max_x=w)

    def draw_footer(self, scr: ScreenBuffer, y: int) -> None:
        w = scr.w
        scr.fill_row(y, 0, w, " ", STYLE_FOOTER)
        keys = (
            " [1-5/Tab] View  [s] Sort SVs  [c] Clear Scatter  [a] Anchor  "
            "[r] Rate(Hz)  [m] DynModel  [f] Filter  [Space] Pause  [l] Log  [q] Quit "
        )
        scr.put_str(y, 0, keys, STYLE_FOOTER, max_x=w)
        if self.status_banner and time.time() - self.status_banner_t < 4.0:
            b = f" * {self.status_banner} "
            scr.put_str(y, max(0, w - len(b)), b, STYLE_BANNER, max_x=w)

    # -------------------------------------------------------------------------
    # TAB 1: OVERVIEW / MISSION CONTROL
    # -------------------------------------------------------------------------
    def draw_tab_overview(self, scr: ScreenBuffer, top_y: int, h: int, w: int) -> None:
        # Proportional 3-column grid that scales smoothly across wide Ghostty windows
        col1_w = max(44, int(w * 0.34))
        col2_w = max(36, int(w * 0.31))
        col3_w = max(28, w - col1_w - col2_w)

        top_h = max(14, int(h * 0.56))
        bot_h = max(7, h - top_h)

        self._draw_panel_geodesy(scr, top_y, 0, top_h, col1_w)
        self._draw_panel_skyplot(scr, top_y, col1_w, top_h, col2_w)
        self._draw_panel_snr_bars(scr, top_y, col1_w + col2_w, h, col3_w)
        self._draw_panel_dop_and_clock(scr, top_y + top_h, 0, bot_h, col1_w)
        self._draw_panel_rf_summary(scr, top_y + top_h, col1_w, bot_h, col2_w)

    def _draw_panel_geodesy(self, scr: ScreenBuffer, y: int, x: int, h: int, w: int) -> None:
        s = self.state
        badge = f"±{s.h_acc_m:.2f}m H / ±{s.v_acc_m:.2f}m V" if not math.isnan(s.h_acc_m) else "Acquiring..."
        scr.draw_box(
            y, x, h, w,
            title="GEODESY, 3D ECEF & KINEMATICS",
            right_badge=badge,
            badge_style=STYLE_GOOD_BOLD if not math.isnan(s.h_acc_m) else STYLE_WARN,
        )

        ix = x + 2
        rx = x + w - 1
        row = y + 1

        lat_dd = f"{s.lat:+.7f}°" if not math.isnan(s.lat) else "---"
        lon_dd = f"{s.lon:+.7f}°" if not math.isnan(s.lon) else "---"
        scr.put_str(row, ix, "Lat WGS84 :", STYLE_DIM, rx)
        scr.put_str(row, ix + 12, f"{lat_dd}  ({format_dms(s.lat, True)})", STYLE_BOLD, rx)
        row += 1

        scr.put_str(row, ix, "Lon WGS84 :", STYLE_DIM, rx)
        scr.put_str(row, ix + 12, f"{lon_dd} ({format_dms(s.lon, False)})", STYLE_BOLD, rx)
        row += 1

        if row < y + h - 1:
            scr.put_str(row, ix, "NMEA DMM  :", STYLE_DIM, rx)
            scr.put_str(row, ix + 12, f"{format_dmm(s.lat, True)}  {format_dmm(s.lon, False)}", STYLE_DEFAULT, rx)
            row += 1

        if row < y + h - 1:
            grid_sq = latlon_to_maidenhead(s.lat, s.lon, 8)
            ghash = latlon_to_geohash(s.lat, s.lon, 9)
            scr.put_str(row, ix, "Grid/Hash :", STYLE_DIM, rx)
            scr.put_str(row, ix + 12, f"{grid_sq} │ Geohash: {ghash}", STYLE_CYAN_BOLD, rx)
            row += 1

        if row < y + h - 1:
            zn, zl, east, north = latlon_to_utm(s.lat, s.lon)
            utm_str = f"{zn}{zl}  {east:8.1f}m E  {north:9.1f}m N" if zn else "---"
            scr.put_str(row, ix, "UTM Coord :", STYLE_DIM, rx)
            scr.put_str(row, ix + 12, utm_str, STYLE_DEFAULT, rx)
            row += 1

        if row < y + h - 1:
            msl_s = f"{s.alt_msl_m:.1f}m" if not math.isnan(s.alt_msl_m) else "---"
            hae_s = f"{s.alt_hae_m:.1f}m" if not math.isnan(s.alt_hae_m) else "---"
            geo_s = f"{s.geoid_sep_m:+.1f}m" if not math.isnan(s.geoid_sep_m) else "---"
            scr.put_str(row, ix, "Altitude :", STYLE_DIM, rx)
            scr.put_str(row, ix + 11, f"MSL {msl_s} │ HAE {hae_s} │ Geoid {geo_s}", STYLE_BOLD, rx)
            row += 1

        if row < y + h - 1:
            ex = f"{s.ecef_x_m / 1000.0:+.2f}" if not math.isnan(s.ecef_x_m) else "---"
            ey = f"{s.ecef_y_m / 1000.0:+.2f}" if not math.isnan(s.ecef_y_m) else "---"
            ez = f"{s.ecef_z_m / 1000.0:+.2f}" if not math.isnan(s.ecef_z_m) else "---"
            scr.put_str(row, ix, "ECEF (km):", STYLE_DIM, rx)
            scr.put_str(row, ix + 11, f"X:{ex} Y:{ey} Z:{ez}", STYLE_CYAN, rx)
            row += 1

        if row < y + h - 1:
            cog_s = f"{s.course_deg:5.1f}°({compass_rose(s.course_deg)})" if not math.isnan(s.course_deg) else "---"
            scr.put_str(row, ix, "Gnd Speed:", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"{s.speed_ms:.2f}m/s │ {s.speed_knots:.2f}kn │ {s.speed_kmh:.2f}km/h",
                STYLE_BOLD, rx,
            )
            row += 1

        if row < y + h - 1:
            scr.put_str(row, ix, "NED / COG:", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"N:{s.vel_n_ms:+.2f} E:{s.vel_e_ms:+.2f} D:{s.vel_d_ms:+.2f} │ {cog_s}",
                STYLE_DEFAULT, rx,
            )
            row += 1

        if row < y + h - 1:
            lmst, sun_el, sun_az = compute_astro_info(
                s.utc_year, s.utc_month, s.utc_day, s.utc_hour, s.utc_min, s.utc_sec, s.lat, s.lon
            )
            scr.put_str(row, ix, "Astro/Sun:", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"LMST {lmst} │ Sun El {sun_el:+.1f}° Az {sun_az:.0f}°",
                STYLE_WARN if sun_el > 0 else STYLE_CYAN, rx,
            )
            row += 1

        if row < y + h - 1:
            dstats = s.drift_tracker.get_enu_points_and_stats()
            scr.put_str(row, ix, "Precision:", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"CEP50:{dstats['cep50']:.2f}m R95:{dstats['r95']:.2f}m 2DRMS:{dstats['drms2']:.2f}m",
                STYLE_GOOD_BOLD, rx,
            )
            row += 1

        # If terminal is tall enough, draw a multi-row Braille Altitude MSL chart in the remaining space!
        rem_rows = (y + h - 1) - row
        if rem_rows >= 4 and len(s.hist_alt) >= 2:
            scr.draw_hsep(row, x, w)
            lo_a, hi_a = min(s.hist_alt), max(s.hist_alt)
            scr.put_str(
                row, ix,
                f" Altitude MSL History [{lo_a:.1f}m .. {hi_a:.1f}m] ",
                STYLE_TITLE, rx,
            )
            row += 1
            chart = render_braille_timeseries(
                s.hist_alt, max(8, w - 4), max(1, (y + h - 1) - row), STYLE_CYAN_BOLD, STYLE_MUTED
            )
            chart.render_to_screen(scr, row, ix, STYLE_CYAN)
        elif rem_rows >= 1:
            scr.put_str(row, ix, "Alt Trend :", STYLE_DIM, rx)
            draw_sparkline(scr, row, ix + 12, max(6, w - 16), s.hist_alt, STYLE_CYAN, STYLE_MUTED)

    def _draw_panel_skyplot(self, scr: ScreenBuffer, y: int, x: int, h: int, w: int) -> None:
        s = self.state
        _, sun_el, sun_az = compute_astro_info(
            s.utc_year, s.utc_month, s.utc_day, s.utc_hour, s.utc_min, s.utc_sec, s.lat, s.lon
        )
        scr.draw_box(
            y, x, h, w,
            title="POLAR SKYPLOT (AZIM/ELEV)",
            right_badge="●Used •Trk *SUN",
            badge_style=STYLE_GOOD_BOLD,
        )
        cw = max(8, w - 4)
        ch = max(4, h - 2)
        canvas = build_skyplot_canvas(
            s.satellites.values(),
            cw,
            ch,
            grid_style=STYLE_MUTED,
            cardinal_style=STYLE_TITLE,
            used_style=STYLE_GOOD_BOLD,
            tracked_style=STYLE_CYAN,
            weak_style=STYLE_WARN,
            sun_el=sun_el,
            sun_az=sun_az,
            sun_style=STYLE_WARN_BOLD,
        )
        canvas.render_to_screen(scr, y + 1, x + 2, STYLE_DEFAULT)

    def _draw_panel_snr_bars(self, scr: ScreenBuffer, y: int, x: int, h: int, w: int) -> None:
        s = self.state
        active_cnos = [sat.cno for sat in s.satellites.values() if sat.cno > 0]
        mean_cno = sum(active_cnos) / len(active_cnos) if active_cnos else 0.0
        peak_cno = max(active_cnos) if active_cnos else 0
        scr.draw_box(
            y, x, h, w,
            title="C/N0 SPECTRUM (dB-Hz)",
            right_badge=f"Avg {mean_cno:.1f} / Max {peak_cno}",
            badge_style=STYLE_GOOD_BOLD,
        )

        sats = sorted(
            s.satellites.values(),
            key=lambda sat: (not sat.used, -sat.cno, sat.key),
        )
        ix = x + 2
        rx = x + w - 1

        # Reserve bottom 7 rows for Constellation summary
        const_rows = 7
        max_sat_rows = max(1, h - 2 - const_rows)
        # Keep the progress bar a clean, readable width (14..28 chars) so wide screens show extra orbital columns!
        bar_w = min(26, max(10, w - 32))

        last_sat_row = y + 1
        for idx, sat in enumerate(sats[:max_sat_rows]):
            row = y + 1 + idx
            if row >= y + h - const_rows:
                break
            last_sat_row = row + 1
            mark = "●" if sat.used else ("•" if sat.cno > 0 else "·")
            st = self.snr_style(sat.cno, sat.used)
            scr.put_str(row, ix, f"{mark}{sat.key:<3} {sat.cno:2d} ", st, rx)
            scr.draw_progress_bar(row, ix + 8, bar_w, sat.cno / 52.0, fill_style=st, empty_style=STYLE_MUTED)

            el_s = f"{sat.elev:2d}°" if sat.elev is not None else "--°"
            az_s = f"{sat.azim:3d}°" if sat.azim is not None else "---°"
            pr_s = f"{sat.pr_res_m:+5.1f}m" if sat.pr_res_m is not None else "   -- "
            sr_s = f"{sat.slant_range_km:6,.0f}km" if sat.slant_range_km > 0 else ""

            meta_x = ix + 8 + bar_w + 1
            avail_meta = rx - meta_x
            if avail_meta >= 26:
                meta = f"El:{el_s} Az:{az_s} Res:{pr_s} {sr_s}"
            elif avail_meta >= 18:
                meta = f"El:{el_s} Az:{az_s} {pr_s}"
            else:
                meta = f"{el_s} {pr_s}"
            scr.put_str(row, meta_x, meta, STYLE_DIM if not sat.used else STYLE_DEFAULT, rx)

        # Constellation summary footer inside right panel
        sep_y = y + h - const_rows
        if sep_y - last_sat_row >= 5 and len(s.hist_snr) >= 2:
            scr.draw_hsep(last_sat_row, x, w)
            lo_s, hi_s = min(s.hist_snr), max(s.hist_snr)
            scr.put_str(
                last_sat_row, ix,
                f" Mean C/N0 Signal History [{lo_s:.1f} .. {hi_s:.1f} dB-Hz] ",
                STYLE_TITLE, rx,
            )
            c_chart = render_braille_timeseries(
                s.hist_snr, max(8, w - 4), max(1, sep_y - (last_sat_row + 1)), STYLE_GOOD_BOLD, STYLE_MUTED
            )
            c_chart.render_to_screen(scr, last_sat_row + 1, ix, STYLE_GOOD)
        if sep_y > y + 2:
            scr.draw_hsep(sep_y, x, w)
            scr.put_str(sep_y, ix, " CONSTELLATION TRACKING SUMMARY ", STYLE_TITLE, rx)
            by_const = {}
            for sat in s.satellites.values():
                u, tot, c_sum, c_n = by_const.get(sat.constellation, (0, 0, 0, 0))
                by_const[sat.constellation] = (
                    u + (1 if sat.used else 0),
                    tot + 1,
                    c_sum + (sat.cno if sat.cno > 0 else 0),
                    c_n + (1 if sat.cno > 0 else 0),
                )
            r = sep_y + 1
            for cname in ("GPS", "BDS", "GLO", "GAL", "SBAS", "QZSS"):
                if r >= y + h - 1:
                    break
                if cname in by_const:
                    u, tot, c_sum, c_n = by_const[cname]
                    avg_c = c_sum / c_n if c_n else 0.0
                    st = STYLE_CYAN_BOLD if u > 0 else STYLE_DIM
                    scr.put_str(
                        r, ix,
                        f"{cname:<4}: {u:2d} used / {tot:2d} view │ Avg C/N0: {avg_c:4.1f} dB-Hz",
                        st, rx,
                    )
                    r += 1

    def _draw_panel_dop_and_clock(self, scr: ScreenBuffer, y: int, x: int, h: int, w: int) -> None:
        s = self.state
        rating_str, _ = dop_rating(s.pdop)
        scr.draw_box(
            y, x, h, w,
            title="7-AXIS DOP & OSCILLATOR TIMING",
            right_badge=f"PDOP: {rating_str}",
            badge_style=self.dop_style(s.pdop),
        )
        ix = x + 2
        rx = x + w - 1
        row = y + 1

        dops1 = [("GDOP", s.gdop), ("PDOP", s.pdop), ("HDOP", s.hdop), ("VDOP", s.vdop)]
        dx = ix
        for name, val in dops1:
            vs = f"{val:.2f}" if not math.isnan(val) else "--"
            scr.put_str(row, dx, f"{name}:", STYLE_DIM, rx)
            scr.put_str(row, dx + 5, vs, self.dop_style(val), rx)
            dx += 11
        row += 1

        if row < y + h - 1:
            dops2 = [("TDOP", s.tdop), ("NDOP", s.ndop), ("EDOP", s.edop)]
            dx = ix
            for name, val in dops2:
                vs = f"{val:.2f}" if not math.isnan(val) else "--"
                scr.put_str(row, dx, f"{name}:", STYLE_DIM, rx)
                scr.put_str(row, dx + 5, vs, self.dop_style(val), rx)
                dx += 11
            scr.put_str(row, dx, f"TTFF:{s.ttff_ms/1000.0:.2f}s", STYLE_GOOD, rx)
            row += 1

        if row < y + h - 1:
            scr.draw_hsep(row, x, w)
            row += 1

        if row < y + h - 1:
            tow_s = s.itow_ms / 1000.0 + s.ftow_ns / 1e9
            scr.put_str(row, ix, "GPS Time :", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"Wk {s.gps_week} │ TOW {tow_s:9.2f}s │ Leap {s.leap_sec:+d}s",
                STYLE_BOLD, rx,
            )
            row += 1

        if row < y + h - 1:
            scr.put_str(row, ix, "Clk Bias :", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"{s.clk_bias_ns:+,d}ns ({s.clk_bias_ns/1000.0:+.2f}µs) │ tAcc ±{s.t_acc_ns}ns",
                STYLE_CYAN_BOLD, rx,
            )
            row += 1

        if row < y + h - 1:
            scr.put_str(row, ix, "Clk Drift:", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"{s.clk_drift_nss:+d} ns/s (ppb) │ fAcc ±{s.f_acc_pss} ps/s",
                STYLE_GOOD_BOLD, rx,
            )
            row += 1

        rem_rows = (y + h - 1) - row
        if rem_rows >= 4 and len(s.hist_clk_drift) >= 2:
            scr.draw_hsep(row, x, w)
            lo_d, hi_d = min(s.hist_clk_drift), max(s.hist_clk_drift)
            scr.put_str(
                row, ix,
                f" Quartz Clock Drift (ppb) [{lo_d:+.0f} .. {hi_d:+.0f} ns/s] ",
                STYLE_TITLE, rx,
            )
            row += 1
            chart = render_braille_timeseries(
                s.hist_clk_drift, max(8, w - 4), max(1, (y + h - 1) - row), STYLE_GOOD_BOLD, STYLE_MUTED
            )
            chart.render_to_screen(scr, row, ix, STYLE_GOOD)
        elif rem_rows >= 1:
            scr.put_str(row, ix, "Drift Spk:", STYLE_DIM, rx)
            draw_sparkline(scr, row, ix + 11, max(6, w - 15), s.hist_clk_drift, STYLE_CYAN, STYLE_MUTED)

    def _draw_panel_rf_summary(self, scr: ScreenBuffer, y: int, x: int, h: int, w: int) -> None:
        s = self.state
        ant_str = ANT_STATUS_NAMES.get(s.ant_status, "?")
        ant_style = STYLE_GOOD_BOLD if s.ant_status == 2 else STYLE_WARN_BOLD
        scr.draw_box(
            y, x, h, w,
            title="RF FRONTEND, JAMMING & ENGINE",
            right_badge=f"ANT:{ant_str} ({ANT_POWER_NAMES.get(s.ant_power, '?')})",
            badge_style=ant_style,
        )
        ix = x + 2
        rx = x + w - 1
        row = y + 1

        if row < y + h - 1:
            scr.put_str(row, ix, "AGC/Noise:", STYLE_DIM, rx)
            scr.draw_progress_bar(row, ix + 11, 10, s.agc_percent / 100.0, STYLE_CYAN_BOLD, STYLE_MUTED)
            scr.put_str(row, ix + 22, f"{s.agc_percent:4.1f}% │ Noise: {s.noise_per_ms}/ms", STYLE_CYAN_BOLD, rx)
            row += 1

        if row < y + h - 1:
            j_style = STYLE_GOOD_BOLD if s.jam_ind < 35 else (STYLE_WARN_BOLD if s.jam_ind < 100 else STYLE_BAD_BOLD)
            scr.put_str(row, ix, "Jamming  :", STYLE_DIM, rx)
            scr.draw_progress_bar(row, ix + 11, 10, s.jam_ind / 255.0, j_style, STYLE_MUTED)
            scr.put_str(
                row, ix + 22,
                f"{s.jam_ind:3d}/255 ({JAMMING_STATE_NAMES.get(s.jamming_state, '?')})",
                j_style, rx,
            )
            row += 1

        if row < y + h - 1:
            scr.put_str(row, ix, "I/Q Mixer:", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"I(ofs={s.ofs_i:+d}, mag={s.mag_i}) │ Q(ofs={s.ofs_q:+d}, mag={s.mag_q})",
                STYLE_DEFAULT, rx,
            )
            row += 1

        if row < y + h - 1:
            sp_style = STYLE_GOOD if s.spoof_state <= 1 else STYLE_BAD_BOLD
            scr.put_str(row, ix, "Security :", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"{SPOOF_STATE_NAMES.get(s.spoof_state, '?')} │ SBAS: {SBAS_SYS_NAMES.get(s.sbas_sys, 'Auto')}",
                sp_style, rx,
            )
            row += 1

        if row < y + h - 1:
            dyn_s = DYN_MODEL_NAMES.get(s.dyn_model, str(s.dyn_model))
            scr.put_str(row, ix, "Engine   :", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"{dyn_s} │ Mask:{s.min_elev_deg}° │ Ch:{s.num_trk_ch_use}/{s.num_trk_ch_hw}",
                STYLE_CYAN, rx,
            )
            row += 1

        if row < y + h - 1:
            up_s = s.uptime_ms // 1000
            uh, um, us = up_s // 3600, (up_s % 3600) // 60, up_s % 60
            scr.put_str(row, ix, "Uptime   :", STYLE_DIM, rx)
            scr.put_str(
                row, ix + 11,
                f"{uh:02d}h {um:02d}m {us:02d}s │ {s.supported_gnss_str}",
                STYLE_DEFAULT, rx,
            )
            row += 1

        rem_rows = (y + h - 1) - row
        if rem_rows >= 5:
            dstats = s.drift_tracker.get_enu_points_and_stats()
            scr.draw_hsep(row, x, w)
            scr.put_str(
                row, ix,
                f" Live 2D Drift Scatter (CEP50={dstats['cep50']:.2f}m R95={dstats['r95']:.2f}m) ",
                STYLE_TITLE, rx,
            )
            row += 1
            scat_canvas, _ = build_scatter_canvas(
                dstats["enu_points"],
                dstats["cep50"],
                dstats["r95"],
                max(8, w - 4),
                max(2, (y + h - 1) - row),
            )
            scat_canvas.render_to_screen(scr, row, ix, STYLE_DEFAULT)

    # -------------------------------------------------------------------------
    # TAB 2: SATELLITES & ORBITAL PHYSICS TABLE
    # -------------------------------------------------------------------------
    def draw_tab_satellites(self, scr: ScreenBuffer, top_y: int, h: int, w: int) -> None:
        s = self.state
        sort_mode = self.SORT_MODES[self.sat_sort_idx]
        scr.draw_box(
            top_y, 0, h, w,
            title="SATELLITE TELEMETRY, PSEUDORANGE RESIDUALS & ORBITAL PHYSICS (UBX-NAV-SAT + NAV-ORB)",
            right_badge=f"Sort [s]: {self.SORT_LABELS[sort_mode]}",
            badge_style=STYLE_WARN_BOLD,
        )

        hdr = (
            " SV    CONST  USED   SIGNAL QUALITY   C/N0 (dB-Hz)         ELEV   AZIM   "
            "PR RESID    SLANT RANGE   PROP DELAY   ORBIT SRC      FLAGS (HEALTH/EPH/ALM/AOP/SMTH)"
        )
        scr.fill_row(top_y + 1, 1, w - 2, " ", STYLE_TBL_HEADER)
        scr.put_str(top_y + 1, 2, hdr, STYLE_TBL_HEADER, w - 1)

        sats = list(s.satellites.values())
        if sort_mode == "used_snr":
            sats.sort(key=lambda x: (not x.used, -x.cno, x.key))
        elif sort_mode == "sv_id":
            sats.sort(key=lambda x: (x.constellation, x.sv_id))
        elif sort_mode == "elev":
            sats.sort(key=lambda x: (-(x.elev if x.elev is not None else -99), -x.cno))
        elif sort_mode == "pr_res":
            sats.sort(key=lambda x: (-(abs(x.pr_res_m) if x.pr_res_m is not None else -1.0), -x.cno))

        max_rows = max(1, h - 3)
        for idx, sat in enumerate(sats[:max_rows]):
            r = top_y + 2 + idx
            used_str = "●YES" if sat.used else "  NO"
            el_str = f"{sat.elev:3d}°" if sat.elev is not None else " --°"
            az_str = f"{sat.azim:3d}°" if sat.azim is not None else " --°"
            pr_str = f"{sat.pr_res_m:+6.1f} m" if sat.pr_res_m is not None else "    --  "
            sr_str = f"{sat.slant_range_km:8,.0f} km" if sat.slant_range_km > 0 else "       --  "
            dl_str = f"{sat.prop_delay_ms:6.2f} ms" if sat.prop_delay_ms > 0 else "    --  "

            flags_list = [sat.health_str]
            if sat.eph_avail:
                flags_list.append("EPH")
            if sat.alm_avail:
                flags_list.append("ALM")
            if sat.aop_avail:
                flags_list.append("AOP")
            if sat.smoothed:
                flags_list.append("SMTH")
            if sat.sbas_corr or sat.diff_corr:
                flags_list.append("DGPS")
            flags_str = " ".join(flags_list)

            st = self.snr_style(sat.cno, sat.used)
            prefix = f" {sat.key:<4}  {sat.constellation:<5}  {used_str:<4}   {sat.quality_str:<15}  {sat.cno:2d} "
            scr.put_str(r, 2, prefix, st, w - 1)
            bar_x = 2 + len(prefix)
            scr.draw_progress_bar(r, bar_x, 14, sat.cno / 52.0, st, STYLE_MUTED)
            tail = (
                f"   {el_str:<5}  {az_str:<5}  {pr_str:<8}    {sr_str:<11}   "
                f"{dl_str:<9}    {sat.orbit_str:<13}  {flags_str}"
            )
            scr.put_str(r, bar_x + 14, tail, st, w - 1)

    # -------------------------------------------------------------------------
    # TAB 3: POSITION SCATTER & TIME-SERIES DRIFT
    # -------------------------------------------------------------------------
    def draw_tab_scatter(self, scr: ScreenBuffer, top_y: int, h: int, w: int) -> None:
        s = self.state
        dstats = s.drift_tracker.get_enu_points_and_stats()
        left_w = max(44, w // 2)
        right_w = max(32, w - left_w)

        canvas, scale_m = build_scatter_canvas(
            dstats["enu_points"],
            dstats["cep50"],
            dstats["r95"],
            max(10, left_w - 4),
            max(6, h - 8),
        )

        ref_mode = "Locked Anchor" if s.drift_tracker.anchor else "Mean Centroid"
        scr.draw_box(
            top_y, 0, h, left_w,
            title=f"2D ENU POSITION SCATTER (±{scale_m:g}m Scale)",
            right_badge=f"{dstats['count']} pts │ {ref_mode}",
            badge_style=STYLE_GOOD_BOLD,
        )
        canvas.render_to_screen(scr, top_y + 1, 2, STYLE_DEFAULT)

        fy = top_y + h - 6
        if fy > top_y + 4:
            scr.draw_hsep(fy, 0, left_w)
            rx = left_w - 1
            scr.put_str(
                fy + 1, 2,
                f"Ref Lat/Lon: {dstats['mean_lat']:+.7f}°, {dstats['mean_lon']:+.7f}°",
                STYLE_BOLD, rx,
            )
            scr.put_str(
                fy + 2, 2,
                f"Cur Offset : ΔE {dstats['cur_e']:+.2f}m │ ΔN {dstats['cur_n']:+.2f}m │ ΔU {dstats['cur_u']:+.2f}m",
                STYLE_CYAN_BOLD, rx,
            )
            scr.put_str(
                fy + 3, 2,
                f"Precision  : CEP50={dstats['cep50']:.2f}m │ R95={dstats['r95']:.2f}m │ 2DRMS={dstats['drms2']:.2f}m",
                STYLE_GOOD_BOLD, rx,
            )
            scr.put_str(
                fy + 4, 2,
                f"StdDev ENU : σE={dstats['std_e']:.2f}m σN={dstats['std_n']:.2f}m σU={dstats['std_u']:.2f}m │ Max={dstats['max_wander']:.2f}m",
                STYLE_WARN, rx,
            )

        chart_h = max(4, h // 4)
        charts = [
            ("ALTITUDE MSL (m)", s.hist_alt, "m", STYLE_CYAN_BOLD),
            ("HORIZONTAL ACCURACY hAcc (m)", s.hist_hacc, "m", STYLE_GOOD_BOLD),
            ("MEAN C/N0 SIGNAL (dB-Hz)", s.hist_snr, "dB-Hz", STYLE_WARN_BOLD),
            ("POSITION DOP (PDOP)", s.hist_pdop, "", STYLE_MAGENTA),
        ]
        for i, (title, seq, unit, st_col) in enumerate(charts):
            cy = top_y + i * chart_h
            ch_box = h - i * chart_h if i == 3 else chart_h
            if ch_box < 4:
                break
            if seq:
                cur_v = seq[-1]
                lo_v, hi_v = min(seq), max(seq)
                badge = f"Now:{cur_v:.2f}{unit} [{lo_v:.1f}..{hi_v:.1f}]"
            else:
                badge = "Waiting..."
            scr.draw_box(
                cy, left_w, ch_box, right_w,
                title=title,
                right_badge=badge,
                badge_style=st_col,
            )
            ts_canvas = render_braille_timeseries(
                seq,
                max(8, right_w - 4),
                max(1, ch_box - 2),
                line_style=st_col,
                axis_style=STYLE_MUTED,
            )
            ts_canvas.render_to_screen(scr, cy + 1, left_w + 2, st_col)

    # -------------------------------------------------------------------------
    # TAB 4: RF FRONTEND, OSCILLATOR CLOCK & SILICON HARDWARE
    # -------------------------------------------------------------------------
    def draw_tab_rf_hw(self, scr: ScreenBuffer, top_y: int, h: int, w: int) -> None:
        s = self.state
        left_w = max(46, w // 2)
        right_w = max(34, w - left_w)
        top_h = max(12, h // 2)
        bot_h = max(8, h - top_h)

        scr.draw_box(
            top_y, 0, top_h, left_w,
            title="RF FRONTEND, I/Q MIXER & JAMMING (UBX-MON-HW/HW2)",
        )
        ix = 2
        rx = left_w - 1
        r = top_y + 1
        scr.put_str(r, ix, "Antenna Supv :", STYLE_DIM, rx)
        scr.put_str(
            r, ix + 15,
            f"State={ANT_STATUS_NAMES.get(s.ant_status, '?')} │ Power={ANT_POWER_NAMES.get(s.ant_power, '?')} │ RTC Calib={s.rtc_calibrated}",
            STYLE_GOOD_BOLD, rx,
        )
        r += 1
        scr.put_str(r, ix, "AGC Monitor  :", STYLE_DIM, rx)
        scr.draw_progress_bar(r, ix + 15, 16, s.agc_percent / 100.0, STYLE_CYAN_BOLD, STYLE_MUTED)
        scr.put_str(r, ix + 32, f"{s.agc_percent:5.1f}% (raw {s.agc_cnt}/8191)", STYLE_CYAN_BOLD, rx)
        r += 1
        scr.put_str(r, ix, "GPS Core Noise:", STYLE_DIM, rx)
        scr.put_str(r, ix + 15, f"{s.noise_per_ms} counts/ms │ Trend: ", STYLE_BOLD, rx)
        draw_sparkline(scr, r, ix + 38, max(6, left_w - 42), s.hist_noise, STYLE_CYAN, STYLE_MUTED)
        r += 1
        j_st = STYLE_GOOD_BOLD if s.jam_ind < 35 else STYLE_WARN_BOLD
        scr.put_str(r, ix, "CW Jamming   :", STYLE_DIM, rx)
        scr.draw_progress_bar(r, ix + 15, 16, s.jam_ind / 255.0, j_st, STYLE_MUTED)
        scr.put_str(
            r, ix + 32,
            f"{s.jam_ind:3d}/255 ({JAMMING_STATE_NAMES.get(s.jamming_state, '?')})",
            j_st, rx,
        )
        r += 1
        scr.put_str(r, ix, "Baseband I/Q :", STYLE_DIM, rx)
        scr.put_str(
            r, ix + 15,
            f"I-Ofs:{s.ofs_i:+3d} I-Mag:{s.mag_i:3d}/255 │ Q-Ofs:{s.ofs_q:+3d} Q-Mag:{s.mag_q:3d}/255",
            STYLE_CYAN, rx,
        )
        r += 1
        scr.put_str(r, ix, "Oscillator   :", STYLE_DIM, rx)
        scr.put_str(
            r, ix + 15,
            f"Bias:{s.clk_bias_ns:+,d} ns │ Drift:{s.clk_drift_nss:+d} ns/s │ tAcc:±{s.t_acc_ns}ns fAcc:±{s.f_acc_pss}ps/s",
            STYLE_GOOD, rx,
        )
        r += 1
        if r < top_y + top_h - 1:
            scr.put_str(r, ix, "Bias Spark   :", STYLE_DIM, rx)
            draw_sparkline(scr, r, ix + 15, max(6, left_w - 19), s.hist_clk_bias, STYLE_CYAN, STYLE_MUTED)
            r += 1
        if r < top_y + top_h - 1:
            scr.put_str(r, ix, "Controls     :", STYLE_DIM, rx)
            scr.put_str(
                r, ix + 15,
                f"[r] Rate={s.update_rate_hz:.0f}Hz ({s.meas_rate_ms}ms)  [m] Dyn={DYN_MODEL_NAMES.get(s.dyn_model, '?')}  {s.last_ack}",
                STYLE_WARN_BOLD, rx,
            )

        # Bottom-Left: 32-Channel Hardware Constellation Allocation (UBX-CFG-GNSS)
        scr.draw_box(
            top_y + top_h, 0, bot_h, left_w,
            title=f"HARDWARE TRACKING CHANNELS ({s.num_trk_ch_use}/{s.num_trk_ch_hw} Ch - UBX-CFG-GNSS)",
        )
        scr.fill_row(top_y + top_h + 1, 1, left_w - 2, " ", STYLE_TBL_HEADER)
        scr.put_str(
            top_y + top_h + 1, 2,
            " CONSTELLATION   STATE     RES CH   MAX CH   SIG MASK   ALLOCATION",
            STYLE_TBL_HEADER, left_w - 1,
        )
        for idx, blk in enumerate(s.gnss_blocks[: max(1, bot_h - 3)]):
            br = top_y + top_h + 2 + idx
            st_s = "ENABLED " if blk.enabled else "DISABLED"
            col = STYLE_GOOD_BOLD if blk.enabled else STYLE_DIM
            txt = f" {blk.gnss_name:<13}   {st_s}  {blk.res_trk_ch:4d}     {blk.max_trk_ch:4d}     0x{blk.sig_cfg_mask:02X}       "
            scr.put_str(br, 2, txt, col, left_w - 1)
            scr.draw_progress_bar(
                br, 2 + len(txt), 14, blk.max_trk_ch / max(1, s.num_trk_ch_hw), col, STYLE_MUTED
            )

        # Top-Right: Hardware I/O Port Telemetry (UBX-MON-IO)
        scr.draw_box(
            top_y, left_w, top_h, right_w,
            title="HARDWARE I/O PORTS (UBX-MON-IO)",
        )
        scr.fill_row(top_y + 1, left_w + 1, right_w - 2, " ", STYLE_TBL_HEADER)
        scr.put_str(
            top_y + 1, left_w + 2,
            " PORT      RX BYTES     TX BYTES   PAR FRM OVR",
            STYLE_TBL_HEADER, left_w + right_w - 1,
        )
        for idx, p in enumerate(s.io_ports[: max(1, top_h - 3)]):
            pr = top_y + 2 + idx
            active = p.rx_bytes > 0 or p.tx_bytes > 0
            col = (STYLE_GOOD_BOLD if p.port_name == "USB" else STYLE_CYAN) if active else STYLE_DIM
            scr.put_str(
                pr, left_w + 2,
                f" {p.port_name:<7} {p.rx_bytes:10,d}   {p.tx_bytes:10,d}   {p.parity_errs:3d} {p.framing_errs:3d} {p.overrun_errs:3d}",
                col, left_w + right_w - 1,
            )

        # Bottom-Right: Silicon ROM/Firmware Extensions & Boot Text ($GNTXT + UBX-MON-VER)
        scr.draw_box(
            top_y + top_h, left_w, bot_h, right_w,
            title="SILICON VERSION & BOOT ROM LOG (MON-VER / GNTXT)",
        )
        lines = [
            f"HW Version : {s.hw_version}",
            f"SW Core    : {s.sw_version}",
            f"Firmware   : {s.fw_version} (Protocol {s.prot_version})",
            f"GNSS Caps  : {s.supported_gnss_str}",
        ]
        for ext in s.ver_extensions:
            lines.append(f"ROM Ext    : {ext}")
        for txt in s.txt_messages:
            lines.append(f"$GNTXT     : {txt}")

        for idx, line in enumerate(lines[: max(1, bot_h - 2)]):
            scr.put_str(top_y + top_h + 1 + idx, left_w + 2, line, STYLE_DEFAULT, left_w + right_w - 1)

    # -------------------------------------------------------------------------
    # TAB 5: PROTOCOL INSPECTOR (LIVE NMEA + UBX PACKET SNIFFER)
    # -------------------------------------------------------------------------
    def draw_tab_protocol(self, scr: ScreenBuffer, top_y: int, h: int, w: int) -> None:
        s = self.state
        left_w = max(36, min(46, w // 3))
        right_w = max(36, w - left_w)

        err_badge = f"ChkErr NMEA:{s.nmea_checksum_err_count} UBX:{s.ubx_checksum_err_count}"
        scr.draw_box(
            top_y, 0, h, left_w,
            title="MESSAGE COUNTERS",
            right_badge=err_badge,
            badge_style=STYLE_GOOD_BOLD if (s.nmea_checksum_err_count + s.ubx_checksum_err_count) == 0 else STYLE_BAD_BOLD,
        )
        scr.fill_row(top_y + 1, 1, left_w - 2, " ", STYLE_TBL_HEADER)
        scr.put_str(
            top_y + 1, 2,
            " MESSAGE TYPE       COUNT     AGE",
            STYLE_TBL_HEADER, left_w - 1,
        )
        now = time.time()
        sorted_msgs = sorted(s.msg_counts.items(), key=lambda kv: (not kv[0].startswith("UBX"), kv[0]))
        for idx, (mtype, cnt) in enumerate(sorted_msgs[: max(1, h - 3)]):
            age = now - s.msg_last_seen.get(mtype, now)
            col = STYLE_CYAN_BOLD if mtype.startswith("UBX") else STYLE_GOOD_BOLD
            if age >= 2.0:
                col = STYLE_CYAN if mtype.startswith("UBX") else STYLE_GOOD
            scr.put_str(
                top_y + 2 + idx, 2,
                f" {mtype:<17} {cnt:6d}   {age:4.1f}s",
                col, left_w - 1,
            )

        paused_str = "PAUSED" if (self.conn and self.conn.paused) else "LIVE"
        scr.draw_box(
            top_y, left_w, h, right_w,
            title=f"PACKET STREAM INSPECTOR (Filter [f]: {self.proto_filter})",
            right_badge=f"{paused_str} │ Total RX: {s.rx_bytes_total:,}B",
            badge_style=STYLE_WARN_BOLD if paused_str == "PAUSED" else STYLE_GOOD_BOLD,
        )
        scr.fill_row(top_y + 1, left_w + 1, right_w - 2, " ", STYLE_TBL_HEADER)
        scr.put_str(
            top_y + 1, left_w + 2,
            " TIME       PROTO  MESSAGE           BYTES  DECODED SUMMARY / RAW PAYLOAD",
            STYLE_TBL_HEADER, left_w + right_w - 1,
        )

        filtered = [
            pkt for pkt in s.packet_log
            if self.proto_filter == "ALL" or pkt.proto == self.proto_filter
        ]
        max_rows = max(1, h - 3)
        visible = filtered[-max_rows:]
        for idx, pkt in enumerate(visible):
            r = top_y + 2 + idx
            t_str = time.strftime("%H:%M:%S", time.localtime(pkt.timestamp))
            ms = int((pkt.timestamp - int(pkt.timestamp)) * 1000)
            col = STYLE_CYAN if pkt.proto == "UBX" else STYLE_DEFAULT
            line = (
                f" {t_str}.{ms:03d} {pkt.proto:<4}  {pkt.msg_type:<16} {pkt.size_bytes:4d}B  "
                f"{pkt.summary}  │  {pkt.raw_preview}"
            )
            scr.put_str(r, left_w + 2, line, col, left_w + right_w - 1)


def print_cli_snapshot(state: GPSState) -> None:
    """Render a comprehensive, colorized geek-grade GPS diagnostic report to stdout."""
    B = "\033[1m"
    C = "\033[36m"
    G = "\033[32m"
    Y = "\033[33m"
    M = "\033[35m"
    D = "\033[2m"
    R = "\033[0m"

    s = state
    zn, zl, east, north = latlon_to_utm(s.lat, s.lon)
    lmst, sun_el, sun_az = compute_astro_info(
        s.utc_year, s.utc_month, s.utc_day, s.utc_hour, s.utc_min, s.utc_sec, s.lat, s.lon
    )
    dstats = s.drift_tracker.get_enu_points_and_stats()

    print(f"{B}{C}╔══════════════════════════════════════════════════════════════════════════════════════════════╗{R}")
    print(f"{B}{C}║ ubxtop — GNSS & u-blox HARDWARE DIAGNOSTIC SNAPSHOT                                         ║{R}")
    print(f"{B}{C}╚══════════════════════════════════════════════════════════════════════════════════════════════╝{R}")
    print(f"  {B}Device & Link  :{R} {G}{s.device_path}{R} ({s.usb_product} / {s.usb_vendor})")
    print(f"  {B}Silicon / FW   :{R} {C}{s.hw_version}{R} │ Core: {s.sw_version} │ FW: {s.fw_version} │ UBX Prot: {s.prot_version}")
    print(f"  {B}Constellations :{R} {s.supported_gnss_str} │ Tracking Channels: {s.num_trk_ch_use}/{s.num_trk_ch_hw}")
    print(f"  {B}Navigation Fix :{R} {B}{G}{s.fix_badge}{R} │ SVs Used: {B}{s.num_sv_used}{R} / {s.num_sv_visible} in view │ TTFF: {s.ttff_ms/1000.0:.2f}s")
    print(f"  {B}UTC Timestamp  :{R} {G}{s.utc_iso_str}{R} (tAcc: ±{s.t_acc_ns} ns)")
    print(f"  {B}GPS Time & Clk :{R} Week {s.gps_week} │ iTOW {s.itow_ms/1000.0:.3f}s │ Leap {s.leap_sec:+d}s │ Bias {s.clk_bias_ns:+,d} ns │ Drift {s.clk_drift_nss:+d} ns/s (ppb)")
    print(f"  {B}WGS84 Lat/Lon  :{R} {s.lat:+.7f}°, {s.lon:+.7f}°  ({format_dms(s.lat, True)}, {format_dms(s.lon, False)})")
    print(f"  {B}Grid / Geohash :{R} Maidenhead: {C}{latlon_to_maidenhead(s.lat, s.lon, 8)}{R} │ Geohash: {C}{latlon_to_geohash(s.lat, s.lon, 9)}{R} │ UTM: {zn}{zl} {east:.1f}E {north:.1f}N")
    print(f"  {B}Altitudes & Acc:{R} MSL: {B}{s.alt_msl_m:.2f}m{R} │ HAE: {s.alt_hae_m:.2f}m │ Geoid: {s.geoid_sep_m:+.2f}m │ hAcc: ±{s.h_acc_m:.2f}m │ vAcc: ±{s.v_acc_m:.2f}m")
    print(f"  {B}3D ECEF (XYZ)  :{R} X={s.ecef_x_m:,.2f}m  Y={s.ecef_y_m:,.2f}m  Z={s.ecef_z_m:,.2f}m  (3D pAcc: ±{s.p_acc_m:.2f}m)")
    print(f"  {B}7-Axis DOP     :{R} GDOP={s.gdop:.2f}  PDOP={s.pdop:.2f}  HDOP={s.hdop:.2f}  VDOP={s.vdop:.2f}  TDOP={s.tdop:.2f}  NDOP={s.ndop:.2f}  EDOP={s.edop:.2f}")
    print(f"  {B}RF & Baseband  :{R} ANT={ANT_STATUS_NAMES.get(s.ant_status, '?')} ({ANT_POWER_NAMES.get(s.ant_power, '?')}) │ AGC={s.agc_percent:.1f}% │ Noise={s.noise_per_ms}/ms │ Jam={s.jam_ind}/255 │ I=({s.ofs_i:+d},{s.mag_i}) Q=({s.ofs_q:+d},{s.mag_q})")
    print(f"  {B}Sidereal & Sun :{R} LMST {lmst} │ Solar Elevation {sun_el:+.1f}° Azimuth {sun_az:.1f}° │ CEP50={dstats['cep50']:.2f}m R95={dstats['r95']:.2f}m")
    print()

    sky = build_skyplot_canvas(s.satellites.values(), 36, 13, sun_el=sun_el, sun_az=sun_az)
    sky_lines = sky.render_lines_plain()

    sats = sorted(s.satellites.values(), key=lambda x: (not x.used, -x.cno, x.key))
    print(f"  {B}{C}POLAR SKYPLOT (N=0°, Zenith=Center){R}     {B}{C}SV   CONST USED  C/N0         ELEV  AZIM   PR_RES   SLANT_RNG   DELAY{R}")
    max_lines = max(len(sky_lines), min(16, len(sats)))
    for i in range(max_lines):
        left = f"  │{sky_lines[i]}│" if i < len(sky_lines) else " " * 40
        if i < len(sats):
            sat = sats[i]
            u_s = f"{G}●YES{R}" if sat.used else f"{D}  NO{R}"
            bar = progress_bar_str(sat.cno / 50.0, 8)
            el_s = f"{sat.elev:3d}°" if sat.elev is not None else " --°"
            az_s = f"{sat.azim:3d}°" if sat.azim is not None else " --°"
            pr_s = f"{sat.pr_res_m:+5.1f}m" if sat.pr_res_m is not None else "   -- "
            sr_s = f"{sat.slant_range_km:7,.0f}km" if sat.slant_range_km > 0 else "     --  "
            dl_s = f"{sat.prop_delay_ms:5.2f}ms" if sat.prop_delay_ms > 0 else "   --  "
            right = f"  {B}{sat.key:<4}{R} {sat.constellation:<5} {u_s}  {sat.cno:2d} {C}{bar}{R}  {el_s}  {az_s}  {Y}{pr_s}{R}  {sr_s}  {M}{dl_s}{R}"
        else:
            right = ""
        print(f"{left}{right}")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="ubxtop",
        description="ubxtop: Geek-grade TUI GNSS & u-blox UBX/NMEA Diagnostic Suite",
    )
    parser.add_argument(
        "-d", "--device",
        help="Serial device path (auto-detected if omitted, e.g. /dev/cu.usbmodem2101)",
        default=None,
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=9600,
        help="Baud rate (default: 9600; USB CDC ignores baud rate)",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run with simulated u-blox M8 telemetry",
    )
    parser.add_argument(
        "--snapshot",
        action="store_true",
        help="Collect live GPS telemetry for ~2.2 seconds, print rich diagnostic snapshot, and exit",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=2.2,
        help="Seconds to sample in --snapshot mode (default: 2.2)",
    )
    args = parser.parse_args()

    state = GPSState()
    conn: Optional[GPSConnection] = None
    demo: Optional[DemoGenerator] = None

    if args.demo:
        demo = DemoGenerator(state)
    else:
        conn = GPSConnection(state, device_path=args.device, baudrate=args.baud)
        conn.connect()

    if args.snapshot:
        t0 = time.time()
        while time.time() - t0 < args.wait:
            if demo:
                demo.poll_step(0.05)
            elif conn:
                conn.poll_step(0.05)
        if conn:
            conn.close()
        print_cli_snapshot(state)
        return 0

    app = GPSTuiApp(state, conn, demo=demo)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if conn:
            conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
