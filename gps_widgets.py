"""
Pure-ANSI / 24-bit TrueColor Cell-Grid Screen Buffer, Unicode Braille Canvas,
Polar Skyplot, 2D CEP Scatter Plot, and Sparkline Widgets.
Designed for Ghostty and modern GPU terminals with DEC Mode 2026 Synchronized Output
and zero libncurses UTF-8 width bugs.
"""

from __future__ import annotations
import math
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# TrueColor ANSI style constants (FG + optional BG + bold/dim)
def fg_rgb(r: int, g: int, b: int, bold: bool = False) -> str:
    return f"\x1b[{'1;' if bold else '22;'}38;2;{r};{g};{b}m"


def fg_bg_rgb(fr: int, fg: int, fb: int, br: int, bg: int, bb: int, bold: bool = False) -> str:
    return f"\x1b[{'1;' if bold else '22;'}38;2;{fr};{fg};{fb};48;2;{br};{bg};{bb}m"


STYLE_RESET = "\x1b[0m"
STYLE_DEFAULT = fg_rgb(220, 228, 240)
STYLE_BOLD = fg_rgb(245, 248, 255, bold=True)
STYLE_DIM = fg_rgb(110, 125, 150)
STYLE_MUTED = fg_rgb(65, 78, 102)
STYLE_BORDER = fg_rgb(75, 155, 210)
STYLE_TITLE = fg_rgb(110, 215, 255, bold=True)
STYLE_CYAN = fg_rgb(80, 220, 240)
STYLE_CYAN_BOLD = fg_rgb(100, 235, 255, bold=True)
STYLE_GOOD = fg_rgb(80, 235, 150)
STYLE_GOOD_BOLD = fg_rgb(95, 250, 165, bold=True)
STYLE_WARN = fg_rgb(250, 200, 80)
STYLE_WARN_BOLD = fg_rgb(255, 215, 95, bold=True)
STYLE_BAD = fg_rgb(255, 95, 95)
STYLE_BAD_BOLD = fg_rgb(255, 110, 110, bold=True)
STYLE_MAGENTA = fg_rgb(205, 140, 255, bold=True)

STYLE_TOPBAR = fg_bg_rgb(15, 23, 34, 85, 205, 235, bold=True)
STYLE_TOPBAR_FIX_OK = fg_bg_rgb(10, 25, 18, 80, 235, 150, bold=True)
STYLE_TOPBAR_FIX_WARN = fg_bg_rgb(30, 22, 5, 250, 200, 80, bold=True)
STYLE_TOPBAR_FIX_BAD = fg_bg_rgb(255, 255, 255, 220, 65, 65, bold=True)

STYLE_TAB_ACTIVE = fg_bg_rgb(15, 23, 34, 100, 220, 245, bold=True)
STYLE_TAB_INACTIVE = fg_bg_rgb(160, 185, 210, 24, 32, 46, bold=False)
STYLE_TAB_BAR_BG = fg_bg_rgb(160, 185, 210, 24, 32, 46, bold=False)
STYLE_TBL_HEADER = fg_bg_rgb(210, 235, 255, 32, 48, 72, bold=True)
STYLE_FOOTER = fg_bg_rgb(200, 220, 240, 26, 36, 52, bold=False)
STYLE_BANNER = fg_bg_rgb(20, 20, 10, 250, 205, 75, bold=True)


# Unicode Braille dot bitmask mapping for a 2x4 subpixel cell
_BRAILLE_MAP = (
    (0x01, 0x08),
    (0x02, 0x10),
    (0x04, 0x20),
    (0x40, 0x80),
)

SPARK_BLOCKS = "▁▂▃▄▅▆▇█"


def sanitize_cell_char(ch: str) -> str:
    """Ensure every cell holds a strictly 1-column printable character."""
    if not ch or ch == "\n" or ch == "\r" or ch == "\t":
        return " "
    code = ord(ch[0])
    if code < 32:
        return " "
    return ch[0]


class ScreenBuffer:
    """
    Exact (width x height) terminal character cell grid.
    Eliminates all ncurses UTF-8 multi-byte column drift and ghost artifacts.
    """

    def __init__(self, width: int, height: int) -> None:
        self.w = max(1, width)
        self.h = max(1, height)
        self.chars: List[List[str]] = [[" "] * self.w for _ in range(self.h)]
        self.styles: List[List[str]] = [[STYLE_DEFAULT] * self.w for _ in range(self.h)]

    def clear(self) -> None:
        for y in range(self.h):
            row_c = self.chars[y]
            row_s = self.styles[y]
            for x in range(self.w):
                row_c[x] = " "
                row_s[x] = STYLE_DEFAULT

    def put_char(self, y: int, x: int, ch: str, style: str = STYLE_DEFAULT) -> None:
        if 0 <= y < self.h and 0 <= x < self.w and ch:
            self.chars[y][x] = sanitize_cell_char(ch)
            self.styles[y][x] = style

    def put_str(
        self,
        y: int,
        x: int,
        text: str,
        style: str = STYLE_DEFAULT,
        max_x: Optional[int] = None,
    ) -> None:
        if y < 0 or y >= self.h or not text:
            return
        limit = min(self.w, max_x) if max_x is not None else self.w
        row_c = self.chars[y]
        row_s = self.styles[y]
        cx = x
        for ch in text:
            if cx >= limit:
                break
            if cx >= 0:
                row_c[cx] = sanitize_cell_char(ch)
                row_s[cx] = style
            cx += 1

    def fill_row(self, y: int, x0: int, width: int, ch: str = " ", style: str = STYLE_DEFAULT) -> None:
        if y < 0 or y >= self.h or width <= 0:
            return
        c = sanitize_cell_char(ch)
        x_start = max(0, x0)
        x_end = min(self.w, x0 + width)
        row_c = self.chars[y]
        row_s = self.styles[y]
        for x in range(x_start, x_end):
            row_c[x] = c
            row_s[x] = style

    def draw_box(
        self,
        y: int,
        x: int,
        h: int,
        w: int,
        title: str = "",
        border_style: str = STYLE_BORDER,
        title_style: str = STYLE_TITLE,
        right_badge: str = "",
        badge_style: str = STYLE_GOOD_BOLD,
    ) -> None:
        if h < 2 or w < 4:
            return
        # Clear interior of box first so nothing ever bleeds through
        for row in range(y + 1, y + h - 1):
            self.fill_row(row, x + 1, w - 2, " ", STYLE_DEFAULT)

        # Top and bottom borders
        self.put_char(y, x, "╭", border_style)
        self.fill_row(y, x + 1, w - 2, "─", border_style)
        self.put_char(y, x + w - 1, "╮", border_style)

        for row in range(y + 1, y + h - 1):
            self.put_char(row, x, "│", border_style)
            self.put_char(row, x + w - 1, "│", border_style)

        self.put_char(y + h - 1, x, "╰", border_style)
        self.fill_row(y + h - 1, x + 1, w - 2, "─", border_style)
        self.put_char(y + h - 1, x + w - 1, "╯", border_style)

        if title:
            t_str = f" {title} "
            self.put_str(y, x + 2, t_str, title_style, max_x=x + w - 2)

        if right_badge:
            b_str = f" {right_badge} "
            bx = x + w - 2 - len(b_str)
            min_bx = x + (len(title) + 4 if title else 2)
            if bx >= min_bx:
                self.put_str(y, bx, b_str, badge_style, max_x=x + w - 1)

    def draw_hsep(self, y: int, x: int, w: int, border_style: str = STYLE_BORDER) -> None:
        if w < 3 or y < 0 or y >= self.h:
            return
        self.put_char(y, x, "├", border_style)
        self.fill_row(y, x + 1, w - 2, "─", border_style)
        self.put_char(y, x + w - 1, "┤", border_style)

    def draw_progress_bar(
        self,
        y: int,
        x: int,
        width: int,
        fraction: float,
        fill_style: str = STYLE_GOOD_BOLD,
        empty_style: str = STYLE_MUTED,
    ) -> None:
        """
        Render a high-contrast progress bar using solid/partial blocks for the filled part
        and a clean dim horizontal track ('━') for the empty part (avoids ugly '░' glyphs).
        """
        if width <= 0 or y < 0 or y >= self.h:
            return
        frac = max(0.0, min(1.0, fraction))
        full_float = frac * width
        full_cells = int(full_float)
        rem = int((full_float - full_cells) * 8)
        partials = " ▏▎▍▌▋▊▉█"

        cx = x
        for _ in range(full_cells):
            self.put_char(y, cx, "█", fill_style)
            cx += 1
        if full_cells < width:
            if rem > 0:
                self.put_char(y, cx, partials[rem], fill_style)
                cx += 1
                rem_empty = width - full_cells - 1
            else:
                rem_empty = width - full_cells
            for _ in range(rem_empty):
                self.put_char(y, cx, "━", empty_style)
                cx += 1

    def flush_to_terminal(self) -> None:
        """
        Atomically flush the entire screen grid using DEC Mode 2026 Synchronized Output
        so Ghostty renders every frame with zero tearing or cursor drift.
        """
        out: List[str] = ["\x1b[?2026h\x1b[H"]
        for y in range(self.h):
            out.append(f"\x1b[{y + 1};1H")
            row_c = self.chars[y]
            row_s = self.styles[y]
            cur_style = ""
            for x in range(self.w):
                st = row_s[x]
                if st != cur_style:
                    out.append(st)
                    cur_style = st
                out.append(row_c[x])
            out.append(STYLE_RESET)
        out.append("\x1b[?2026l")
        sys.stdout.write("".join(out))
        sys.stdout.flush()


class BrailleCanvas:
    """
    High-resolution 2D canvas using Unicode Braille characters (2x4 subpixels per terminal cell).
    """

    def __init__(self, char_width: int, char_height: int) -> None:
        self.cw = max(1, char_width)
        self.ch = max(1, char_height)
        self.pw = self.cw * 2
        self.ph = self.ch * 4
        self.grid: List[List[int]] = [[0] * self.cw for _ in range(self.ch)]
        self.styles: List[List[str]] = [[""] * self.cw for _ in range(self.ch)]
        self.overlays: Dict[Tuple[int, int], Tuple[str, str]] = {}

    def set_pixel(self, px: int, py: int, style: str = "") -> None:
        if 0 <= px < self.pw and 0 <= py < self.ph:
            cx = px >> 1
            cy = py >> 2
            dx = px & 1
            dy = py & 3
            self.grid[cy][cx] |= _BRAILLE_MAP[dy][dx]
            if style:
                self.styles[cy][cx] = style

    def draw_line(self, x0: int, y0: int, x1: int, y1: int, style: str = "", step: int = 1) -> None:
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        count = 0
        while True:
            if count % step == 0:
                self.set_pixel(x0, y0, style)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
            count += 1

    def draw_circle(self, cx: float, cy: float, radius: float, style: str = "", step_deg: int = 2) -> None:
        if radius <= 0.5:
            self.set_pixel(int(round(cx)), int(round(cy)), style)
            return
        for deg in range(0, 360, max(1, step_deg)):
            rad = math.radians(deg)
            px = int(round(cx + radius * math.cos(rad)))
            py = int(round(cy + radius * math.sin(rad)))
            self.set_pixel(px, py, style)

    def put_label(self, cx: int, cy: int, text: str, style: str = STYLE_DEFAULT) -> None:
        if not (0 <= cy < self.ch):
            return
        for i, ch in enumerate(text):
            x = cx + i
            if 0 <= x < self.cw:
                self.overlays[(x, cy)] = (sanitize_cell_char(ch), style)

    def render_to_screen(self, screen: ScreenBuffer, top_y: int, left_x: int, default_style: str = STYLE_DEFAULT) -> None:
        for cy in range(self.ch):
            row_bits = self.grid[cy]
            row_styles = self.styles[cy]
            y = top_y + cy
            for cx in range(self.cw):
                x = left_x + cx
                ov = self.overlays.get((cx, cy))
                if ov is not None:
                    ch, st = ov
                    screen.put_char(y, x, ch, st or default_style)
                else:
                    bits = row_bits[cx]
                    ch = chr(0x2800 + bits) if bits else " "
                    st = row_styles[cx] or default_style
                    screen.put_char(y, x, ch, st)

    def render_lines_plain(self) -> List[str]:
        lines: List[str] = []
        for cy in range(self.ch):
            chars: List[str] = []
            for cx in range(self.cw):
                ov = self.overlays.get((cx, cy))
                if ov is not None:
                    chars.append(ov[0])
                else:
                    bits = self.grid[cy][cx]
                    chars.append(chr(0x2800 + bits) if bits else " ")
            lines.append("".join(chars))
        return lines


def draw_sparkline(
    screen: ScreenBuffer,
    y: int,
    x: int,
    width: int,
    values: Sequence[float],
    fill_style: str = STYLE_CYAN,
    empty_style: str = STYLE_MUTED,
    vmin: Optional[float] = None,
    vmax: Optional[float] = None,
) -> None:
    """Draw a clean single-row sparkline with a subtle dim baseline for empty history."""
    if width <= 0 or y < 0 or y >= screen.h:
        return
    if not values:
        screen.fill_row(y, x, width, "─", empty_style)
        return
    tail = list(values)[-width:]
    lo = vmin if vmin is not None else min(tail)
    hi = vmax if vmax is not None else max(tail)
    span = hi - lo if hi > lo else 1.0
    pad_len = max(0, width - len(tail))
    screen.fill_row(y, x, pad_len, "─", empty_style)
    cx = x + pad_len
    for v in tail:
        norm = max(0.0, min(1.0, (v - lo) / span))
        idx = int(round(norm * 7))
        screen.put_char(y, cx, SPARK_BLOCKS[idx], fill_style)
        cx += 1


def progress_bar_str(fraction: float, width: int) -> str:
    """Plain string progress bar for CLI --snapshot mode."""
    if width <= 0:
        return ""
    frac = max(0.0, min(1.0, fraction))
    full = int(round(frac * width))
    return ("█" * full) + ("━" * (width - full))


def render_braille_timeseries(
    values: Sequence[float],
    char_width: int,
    char_height: int,
    line_style: str = STYLE_CYAN_BOLD,
    axis_style: str = STYLE_MUTED,
) -> BrailleCanvas:
    """Render a high-resolution Braille line chart from a sequence of floats."""
    canvas = BrailleCanvas(char_width, char_height)
    for px in range(0, canvas.pw, 2):
        canvas.set_pixel(px, canvas.ph - 1, axis_style)

    if not values:
        return canvas

    pts = list(values)[-canvas.pw :]
    lo = min(pts)
    hi = max(pts)
    if abs(hi - lo) < 1e-6:
        lo -= 1.0
        hi += 1.0
    span = hi - lo

    n = len(pts)
    prev_px: Optional[int] = None
    prev_py: Optional[int] = None
    x_offset = canvas.pw - n
    for i, val in enumerate(pts):
        px = x_offset + i
        norm = (val - lo) / span
        py = int(round((1.0 - norm) * (canvas.ph - 2)))
        py = max(0, min(canvas.ph - 1, py))
        if prev_px is not None and prev_py is not None:
            canvas.draw_line(prev_px, prev_py, px, py, line_style)
        else:
            canvas.set_pixel(px, py, line_style)
        prev_px, prev_py = px, py

    return canvas


def build_skyplot_canvas(
    satellites: Iterable,
    char_width: int,
    char_height: int,
    grid_style: str = STYLE_MUTED,
    cardinal_style: str = STYLE_TITLE,
    used_style: str = STYLE_GOOD_BOLD,
    tracked_style: str = STYLE_CYAN,
    weak_style: str = STYLE_WARN,
    sun_el: Optional[float] = None,
    sun_az: Optional[float] = None,
    sun_style: str = STYLE_WARN_BOLD,
) -> BrailleCanvas:
    """
    Build a high-resolution polar skyplot radar on a BrailleCanvas.
    Cardinal labels N/E/S/W are positioned snugly around the horizon ring.
    """
    canvas = BrailleCanvas(char_width, char_height)
    cx = canvas.pw / 2.0
    cy = canvas.ph / 2.0
    max_r = max(6.0, min(cx - 6.0, cy - 5.0))

    # Concentric elevation rings: 0° (horizon), 30°, 60°
    canvas.draw_circle(cx, cy, max_r, grid_style, step_deg=2)
    canvas.draw_circle(cx, cy, max_r * (60.0 / 90.0), grid_style, step_deg=4)
    canvas.draw_circle(cx, cy, max_r * (30.0 / 90.0), grid_style, step_deg=6)

    # N-S and E-W crosshairs
    canvas.draw_line(int(cx - max_r), int(cy), int(cx + max_r), int(cy), grid_style, step=2)
    canvas.draw_line(int(cx), int(cy - max_r), int(cx), int(cy + max_r), grid_style, step=2)

    # Position N/S/E/W labels right outside the actual circle radius (not at box corners!)
    mid_cx = int(round(cx / 2.0))
    mid_cy = int(round(cy / 4.0))
    n_cy = max(0, int(round((cy - max_r) / 4.0)) - 1)
    s_cy = min(canvas.ch - 1, int(round((cy + max_r) / 4.0)) + 1)
    w_cx = max(0, int(round((cx - max_r) / 2.0)) - 2)
    e_cx = min(canvas.cw - 1, int(round((cx + max_r) / 2.0)) + 1)

    canvas.put_label(mid_cx, n_cy, "N", cardinal_style)
    canvas.put_label(mid_cx, s_cy, "S", cardinal_style)
    canvas.put_label(w_cx, mid_cy, "W", cardinal_style)
    canvas.put_label(e_cx, mid_cy, "E", cardinal_style)

    # Sun marker if above horizon (strictly 1-col ASCII/Unicode)
    if sun_el is not None and sun_az is not None and sun_el >= 0.0:
        r_sun = max_r * ((90.0 - min(90.0, sun_el)) / 90.0)
        az_rad = math.radians(sun_az)
        spx = cx + r_sun * math.sin(az_rad)
        spy = cy - r_sun * math.cos(az_rad)
        scx = max(0, min(canvas.cw - 4, int(round(spx / 2.0))))
        scy = max(0, min(canvas.ch - 1, int(round(spy / 4.0))))
        canvas.put_label(scx, scy, "*SUN", sun_style)

    # Plot satellites (sorted so used satellites draw on top)
    sat_list = [s for s in satellites if s.elev is not None and s.azim is not None and s.elev >= 0]
    sat_list.sort(key=lambda s: (s.used, s.cno))

    for sat in sat_list:
        r = max_r * ((90.0 - max(0.0, min(90.0, float(sat.elev)))) / 90.0)
        az_rad = math.radians(float(sat.azim))
        px = cx + r * math.sin(az_rad)
        py = cy - r * math.cos(az_rad)
        scx = max(1, min(canvas.cw - 5, int(round(px / 2.0))))
        scy = max(0, min(canvas.ch - 1, int(round(py / 4.0))))

        if sat.used:
            st = used_style
            label = f"●{sat.key}"
        elif sat.cno > 0:
            st = tracked_style
            label = f"•{sat.key}"
        else:
            st = weak_style
            label = f"·{sat.key}"
        canvas.put_label(scx, scy, label, st)

    return canvas


def build_scatter_canvas(
    enu_points: Sequence[Tuple[float, float, float]],
    cep50_m: float,
    r95_m: float,
    char_width: int,
    char_height: int,
    grid_style: str = STYLE_MUTED,
    cep_style: str = STYLE_GOOD_BOLD,
    r95_style: str = STYLE_WARN,
    trail_style: str = STYLE_CYAN,
    curr_style: str = STYLE_GOOD_BOLD,
) -> Tuple[BrailleCanvas, float]:
    """
    Build a 2D East-North position wander scatter plot on a BrailleCanvas.
    Returns (canvas, scale_radius_m).
    """
    canvas = BrailleCanvas(char_width, char_height)
    cx = canvas.pw / 2.0
    cy = canvas.ph / 2.0
    max_r_px = max(6.0, min(cx - 6.0, cy - 5.0))

    max_dist = 1.5
    for de, dn, _ in enu_points:
        d = math.hypot(de, dn)
        if d > max_dist:
            max_dist = d
    if r95_m > max_dist:
        max_dist = r95_m

    nice_scales = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 250.0, 500.0, 1000.0]
    scale_m = nice_scales[-1]
    for s in nice_scales:
        if max_dist <= s * 0.90:
            scale_m = s
            break

    px_per_m = max_r_px / scale_m

    # Axes crosshairs & outer scale circle
    canvas.draw_line(int(cx - max_r_px), int(cy), int(cx + max_r_px), int(cy), grid_style, step=2)
    canvas.draw_line(int(cx), int(cy - max_r_px), int(cx), int(cy + max_r_px), grid_style, step=2)
    canvas.draw_circle(cx, cy, max_r_px, grid_style, step_deg=3)

    # R95 & CEP50 error rings
    if r95_m > 0.05:
        canvas.draw_circle(cx, cy, min(max_r_px, r95_m * px_per_m), r95_style, step_deg=4)
    if cep50_m > 0.05:
        canvas.draw_circle(cx, cy, min(max_r_px, cep50_m * px_per_m), cep_style, step_deg=3)

    # Historical fix points
    for i, (de, dn, _) in enumerate(enu_points):
        px = int(round(cx + de * px_per_m))
        py = int(round(cy - dn * px_per_m))
        canvas.set_pixel(px, py, trail_style)
        if i > 0:
            prev_e, prev_n, _ = enu_points[i - 1]
            ppx = int(round(cx + prev_e * px_per_m))
            ppy = int(round(cy - prev_n * px_per_m))
            if math.hypot(px - ppx, py - ppy) <= 12:
                canvas.draw_line(ppx, ppy, px, py, trail_style, step=1)

    mid_cx = int(round(cx / 2.0))
    mid_cy = int(round(cy / 4.0))
    n_cy = max(0, int(round((cy - max_r_px) / 4.0)) - 1)
    s_cy = min(canvas.ch - 1, int(round((cy + max_r_px) / 4.0)) + 1)
    w_cx = max(0, int(round((cx - max_r_px) / 2.0)) - 3)
    e_cx = min(canvas.cw - 3, int(round((cx + max_r_px) / 2.0)) + 1)

    canvas.put_label(mid_cx - 2, n_cy, f"+{scale_m:g}m N", STYLE_TITLE)
    canvas.put_label(mid_cx - 2, s_cy, f"-{scale_m:g}m S", STYLE_TITLE)
    canvas.put_label(w_cx, mid_cy, "-W", STYLE_TITLE)
    canvas.put_label(e_cx, mid_cy, "+E", STYLE_TITLE)
    canvas.put_label(mid_cx, mid_cy, "+", STYLE_TITLE)

    if enu_points:
        cur_e, cur_n, _ = enu_points[-1]
        cpx = cx + cur_e * px_per_m
        cpy = cy - cur_n * px_per_m
        ccx = max(0, min(canvas.cw - 1, int(round(cpx / 2.0))))
        ccy = max(0, min(canvas.ch - 1, int(round(cpy / 4.0))))
        canvas.put_label(ccx, ccy, "●", curr_style)

    return (canvas, scale_m)
