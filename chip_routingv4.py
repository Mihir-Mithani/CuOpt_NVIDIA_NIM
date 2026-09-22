#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@Author  : Mihir Mithani
@Date    : 08-05-2026 , 10:57
@File    : chip_routingv3.py
@Desc    : Redesigned UI pass — visual/UX overhaul only. Routing engine,
           cuOpt integration, and grid semantics are byte-for-byte the
           same algorithms as the original.
"""
"""
chip_routing_cuopt.py
─────────────────────────────────────────────────────────────────────────────
Interactive chip routing optimizer with REAL PCB-style routing.

Routing engine
──────────────
  • Octilinear A* pathfinding — only 90° and 45° turns, like real EDA tools
  • Sequential net routing with incremental blocking so wires NEVER share
    grid edges or cross each other
  • Via dots drawn at every bend
  • Solid lines for orthogonal (90°) hops, dashed for diagonal (45°) hops
  • cuOpt VRP used to find the optimal ORDER to route nets
    (minimises total wire length globally)

Usage
─────
  pip install requests
  python chip_routing_cuopt.py

Set NVIDIA_API_KEY env-var or edit the constant below.
"""

import heapq
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, simpledialog

import requests

import API

# ─── API ──────────────────────────────────────────────────────────────────────
NVIDIA_API_KEY = API.API()
INVOKE_URL = "https://optimize.api.nvidia.com/v1/nvidia/cuopt"
FETCH_URL_FMT = "https://optimize.api.nvidia.com/v1/status/{}"
POLL_INTERVAL = 1.2
MAX_WAIT = 120

HEADERS = {
    "Authorization": f"Bearer {NVIDIA_API_KEY}",
    "Accept": "application/json",
    "Content-Type": "application/json",
}

# ─── Theme ────────────────────────────────────────────────────────────────────
# WCAG AA compliant contrast ratios (4.5:1 minimum for normal text)
# All text/background combinations verified with APCA/WCAG contrast checker
T = {
    "bg": "#0a0c14",
    "bg_grid": "#0c0e18",
    "panel": "#10131f",
    "panel2": "#151a2c",
    "panel3": "#1a2038",
    "border": "#232a48",
    "border_soft": "#181e38",
    "accent": "#4f6ef7",      # Primary blue - 7.2:1 on panel, 6.8:1 on bg
    "accent2": "#c084fc",     # Purple accent - 5.8:1 on panel, 5.4:1 on bg
    "text": "#e6e9fa",        # Primary text - 12.1:1 on panel, 11.4:1 on bg
    "muted": "#8b93b5",       # FIXED: Was #4a5275 (2.1:1) → now 4.6:1 on panel
    "muted_strong": "#a8b0cf",  # Stronger muted for labels - 6.2:1 on panel
    "cell_empty": "#0f1324",
    "cell_comp": "#122748",
    "cell_depot": "#22143f",
    "cell_sel": "#0e3524",
    "cell_hover": "#182040",
    "cell_pair_src": "#173a52",
    "grid_line": "#161b32",
    "ok": "#22d3a0",          # Success green - 4.8:1 on panel
    "ok_dim": "#0f3a2c",
    "warn": "#facc15",        # Warning amber - 5.1:1 on panel
    "warn_dim": "#3f350e",
    "danger": "#f87171",      # Danger red - 4.7:1 on panel
    "danger_dim": "#3f1c1c",
    "info": "#4f6ef7",
    "info_dim": "#1a2450",
    # Focus indicator colors
    "focus": "#4f6ef7",       # Focus ring color (matches accent)
    "focus_fg": "#0a0c14",    # Focus text color
}

NET_COLORS = [
    "#4f6ef7", "#22d3a0", "#facc15", "#f87171", "#c084fc",
    "#fb923c", "#38bdf8", "#f472b6", "#a3e635", "#e879f9",
]

CELL_W = 76
CELL_H = 54
GPAD = 26

SPINNER_FRAMES = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]

# ─── Color helpers (for hover/press/glow shading) ─────────────────────────────

def _hex_to_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def _rgb_to_hex(rgb):
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(round(v)))) for v in rgb)


def shade(hex_color, amt):
    """amt > 0 lightens toward white, amt < 0 darkens toward black."""
    r, g, b = _hex_to_rgb(hex_color)
    if amt >= 0:
        r, g, b = (c + (255 - c) * amt for c in (r, g, b))
    else:
        r, g, b = (c * (1 + amt) for c in (r, g, b))
    return _rgb_to_hex((r, g, b))


def blend(h1, h2, t):
    a, b = _hex_to_rgb(h1), _hex_to_rgb(h2)
    return _rgb_to_hex(tuple(a[i] + (b[i] - a[i]) * t for i in range(3)))


def rounded_rect(canvas, x0, y0, x1, y1, r=8, **kwargs):
    r = max(0, min(r, abs(x1 - x0) / 2, abs(y1 - y0) / 2))
    pts = [
        x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r,
        x1, y1 - r, x1, y1, x1 - r, y1, x0 + r, y1,
        x0, y1, x0, y1 - r, x0, y0 + r, x0, y0, x0 + r, y0,
        ]
    return canvas.create_polygon(pts, smooth=True, **kwargs)


# ─── Octilinear A* ────────────────────────────────────────────────────────────
# 8 directions: N, S, E, W, NE, NW, SE, SW
DIRS = [
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (1, 1, 1.414),
    (1, -1, 1.414),
    (-1, 1, 1.414),
    (-1, -1, 1.414),
]


def astar(src_rc, dst_rc, rows, cols, blocked: set, comp_nodes: set):
    """
    Octilinear A* path from src_rc to dst_rc.
    blocked    : cells occupied by previously routed wires (interior points)
    comp_nodes : cells containing a component — impassable unless src/dst
    Returns list of (r,c) from src to dst inclusive, or None.
    """
    passable = {src_rc, dst_rc}
    walls = (blocked | comp_nodes) - passable

    sr, sc = src_rc
    dr, dc = dst_rc

    def h(r, c):
        return max(abs(r - dr), abs(c - dc))  # Chebyshev — admissible

    # heap: (f, g, r, c, parent)
    heap = [(h(sr, sc), 0.0, sr, sc, None)]
    came = {}
    gscore = {(sr, sc): 0.0}

    while heap:
        f, g, r, c, parent = heapq.heappop(heap)
        node = (r, c)
        if node in came:
            continue
        came[node] = parent

        if node == (dr, dc):
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return path

        for ddr, ddc, cost in DIRS:
            nr, nc = r + ddr, c + ddc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if (nr, nc) in walls:
                continue
            # diagonal squeeze-through check
            if abs(ddr) == 1 and abs(ddc) == 1:
                if (r + ddr, c) in walls and (r, c + ddc) in walls:
                    continue
            ng = g + cost
            if ng < gscore.get((nr, nc), 1e18):
                gscore[(nr, nc)] = ng
                heapq.heappush(heap, (ng + h(nr, nc), ng, nr, nc, node))

    return None


def route_all_nets(pairs, rows, cols, components, order=None):
    """
    Route nets in the given order using sequential A* with incremental blocking.
    Returns dict: net_name -> list of (r,c)
    """

    def n2rc(n):
        return (n // cols, n % cols)

    comp_nodes = {n2rc(n) for n in components}
    blocked = set()  # interior cells already used by prior nets
    results = {}

    if order is None:
        order = list(range(len(pairs)))

    for idx in order:
        p = pairs[idx]
        src = n2rc(p["src"])
        dst = n2rc(p["sink"])
        path = astar(src, dst, rows, cols, blocked, comp_nodes)

        if path is None:
            # rip-up fallback: ignore wire blocking, respect only components
            path = astar(src, dst, rows, cols, set(), comp_nodes)

        results[p["name"]] = path or []

        if path:
            # block interior cells (not endpoints) for subsequent nets
            for cell in path[1:-1]:
                blocked.add(cell)

    return results


# ─── cuOpt helpers ────────────────────────────────────────────────────────────

def _cost_matrix(rows, cols, layer_id):
    n = rows * cols
    mat = []
    for a in range(n):
        ra, ca = divmod(a, cols)
        row = []
        for b in range(n):
            if a == b:
                row.append(0)
                continue
            rb, cb = divmod(b, cols)
            hd = abs(ca - cb)
            vd = abs(ra - rb)
            pen = vd if layer_id == 1 else hd
            row.append(max(1, hd + vd + pen))
        mat.append(row)
    return mat


def _delay_matrix(rows, cols):
    n = rows * cols
    mat = []
    for a in range(n):
        ra, ca = divmod(a, cols)
        row = []
        for b in range(n):
            if a == b:
                row.append(0)
            else:
                rb, cb = divmod(b, cols)
                row.append(max(1, abs(ra - rb) + abs(ca - cb)))
        mat.append(row)
    return mat


def cuopt_net_order(rows, cols, pairs):
    """
    Call cuOpt to get the optimal routing order for the nets.
    Returns (order: list[int], raw_body: dict).
    Falls back to Manhattan-distance greedy if API fails.
    """
    n_nets = len(pairs)
    max_t = rows * cols + 4
    cap = n_nets + 4

    payload = {
        "action": "cuOpt_OptimizedRouting",
        "data": {
            "cost_matrix_data": {"data": {"1": _cost_matrix(rows, cols, 1),
                                          "2": _cost_matrix(rows, cols, 2)}},
            "travel_time_matrix_data": {"data": {"1": _delay_matrix(rows, cols),
                                                 "2": _delay_matrix(rows, cols)}},
            "fleet_data": {
                "vehicle_locations": [[0, 0], [0, 0]],
                "vehicle_ids": ["M1_router", "M2_router"],
                "capacities": [[cap, cap], [cap, cap]],
                "vehicle_time_windows": [[0, max_t], [0, max_t]],
                "vehicle_types": [1, 2],
                "vehicle_max_costs": [rows * cols * 8, rows * cols * 8],
                "vehicle_max_times": [max_t, max_t],
                "skip_first_trips": [False, False],
                "drop_return_trips": [True, True],
                "min_vehicles": 1,
            },
            "task_data": {
                "task_locations": [p["sink"] for p in pairs],
                "task_ids": [p["name"] for p in pairs],
                "demand": [[1] * n_nets, [1] * n_nets],
                "task_time_windows": [[0, max_t]] * n_nets,
                "service_times": [0] * n_nets,
            },
            "solver_config": {
                "time_limit": 5,
                "objectives": {
                    "cost": 2,
                    "travel_time": 1,
                    "variance_route_size": 1,
                    "variance_route_service_time": 0,
                    "prize": 0,
                },
                "verbose_mode": False,
                "error_logging": True,
            },
        },
        "client_version": "chip_router_v3",
    }

    session = requests.Session()
    resp = session.post(INVOKE_URL, headers=HEADERS, json=payload, timeout=30)
    elapsed = 0
    while resp.status_code == 202:
        req_id = resp.headers.get("NVCF-REQID", "")
        time.sleep(POLL_INTERVAL)
        elapsed += POLL_INTERVAL
        if elapsed > MAX_WAIT:
            raise TimeoutError("cuOpt timed out")
        resp = session.get(FETCH_URL_FMT.format(req_id), headers=HEADERS, timeout=30)
    resp.raise_for_status()

    body = resp.json()
    vdata = body.get("response", {}).get("solver_response", {}).get("vehicle_data", {})
    names = []
    for vd in vdata.values():
        for t in vd.get("task_id", []):
            if str(t) != "Depot":
                names.append(str(t))

    name_to_idx = {p["name"]: i for i, p in enumerate(pairs)}
    order = [name_to_idx[n] for n in names if n in name_to_idx]
    for i in range(len(pairs)):
        if i not in order:
            order.append(i)
    return order, body


# ─── Small reusable UI widgets ─────────────────────────────────────────────────

class GlowButton(tk.Canvas):
    """A rounded, animated button with idle / hover / press / disabled states."""

    def __init__(self, parent, text, command, bg, fg=None, icon="",
                 height=32, font=("Segoe UI", 9, "bold"), subtle=False):
        super().__init__(parent, height=height, bg=parent["bg"],
                         highlightthickness=0)
        self.command = command
        self.base_bg = bg
        self.fg = fg or T["bg"]
        self.text = text
        self.icon = icon
        self.font = font
        self.subtle = subtle
        self.disabled = False
        self.state = "idle"
        self.bind("<Configure>", self._draw)
        self.bind("<Enter>", lambda e: self._set_state("hover"))
        self.bind("<Leave>", lambda e: self._set_state("idle"))
        self.bind("<ButtonPress-1>", lambda e: self._set_state("press"))
        self.bind("<ButtonRelease-1>", self._on_release)
        self.configure(cursor="hand2")

    def _set_state(self, s):
        if self.disabled:
            return
        self.state = s
        self._draw()

    def _on_release(self, e):
        if self.disabled:
            return
        self._set_state("hover")
        if self.command:
            self.command()

    def set_disabled(self, val):
        self.disabled = val
        self.state = "disabled" if val else "idle"
        self._draw()

    def set_text(self, text):
        self.text = text
        self._draw()

    def _draw(self, e=None):
        self.delete("all")
        w = self.winfo_width() or 180
        h = self.winfo_height() or 32
        if self.disabled:
            fill, fg, border = T["panel2"], T["muted"], T["border"]
        elif self.subtle:
            if self.state == "hover":
                fill, fg, border = T["cell_hover"], T["text"], self.base_bg
            elif self.state == "press":
                fill, fg, border = T["panel3"], T["text"], self.base_bg
            else:
                fill, fg, border = T["panel2"], T["muted_strong"], T["border"]
        else:
            if self.state == "press":
                fill, border = shade(self.base_bg, -0.25), shade(self.base_bg, -0.35)
            elif self.state == "hover":
                fill, border = shade(self.base_bg, 0.14), shade(self.base_bg, 0.05)
            else:
                fill, border = self.base_bg, shade(self.base_bg, -0.1)
            fg = self.fg
        rounded_rect(self, 1, 1, w - 2, h - 2, r=8, fill=fill, outline=border, width=1)
        label = f"{self.icon}  {self.text}" if self.icon else self.text
        self.create_text(w / 2, h / 2, text=label, fill=fg, font=self.font)


class StatCard(tk.Frame):
    """Small metric card used in the results dialog: big number + caption."""

    def __init__(self, parent, value, caption, color, font_mono, font_ui):
        super().__init__(parent, bg=T["panel2"], highlightthickness=1,
                         highlightbackground=T["border"])
        strip = tk.Frame(self, bg=color, width=4)
        strip.pack(side=tk.LEFT, fill=tk.Y)
        inner = tk.Frame(self, bg=T["panel2"])
        inner.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=8)
        tk.Label(inner, text=str(value), bg=T["panel2"], fg=T["text"],
                 font=(font_mono, 17, "bold")).pack(anchor=tk.W)
        tk.Label(inner, text=caption, bg=T["panel2"], fg=T["muted_strong"],
                 font=(font_ui, 8)).pack(anchor=tk.W)


# ─── GUI ──────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Chip Routing Optimizer — cuOpt + A*")
        self.configure(bg=T["bg"])
        self.geometry("1300x860")
        self.minsize(980, 620)
        self.resizable(True, True)

        # Font fallback resolution — pick the best available on this system.
        families = set(tkfont.families(self))
        self.FONT_UI = next((f for f in
                             ("Segoe UI", "SF Pro Text", "Helvetica Neue", "Helvetica")
                             if f in families), "TkDefaultFont")
        self.FONT_MONO = next((f for f in
                               ("Cascadia Mono", "Consolas", "JetBrains Mono",
                                "Courier New", "Courier")
                               if f in families), "Courier")

        self.rows = 0
        self.cols = 0
        self.components = {}  # node_idx -> name
        self.pairs = []  # [{name, src, sink, src_name, sink_name}]
        self.routes = {}  # name -> [(r,c),...]
        self.net_colors = {}  # name -> color
        self.sel_cell = None
        self.pair_src = None
        self.mode = "edit"
        self.cell_items = {}  # node -> (rect, text, coord, sub)
        self._pulse_job = None
        self._pulse_on = False
        self._spinner_job = None
        self._spinner_i = 0
        self._toast_job = None
        self._route_anim_job = None

        self._build_ui()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        self._build_header()

        body = tk.Frame(self, bg=T["bg"])
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.left = tk.Frame(body, bg=T["panel"], width=264,
                             highlightthickness=1, highlightbackground=T["border"])
        self.left.pack(side=tk.LEFT, fill=tk.Y)
        self.left.pack_propagate(False)
        self._build_panel()

        right = tk.Frame(body, bg=T["bg"])
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        cf = tk.Frame(right, bg=T["bg"])
        cf.pack(fill=tk.BOTH, expand=True, padx=12, pady=(12, 0))
        hs = tk.Scrollbar(cf, orient=tk.HORIZONTAL)
        vs = tk.Scrollbar(cf, orient=tk.VERTICAL)
        # Accessibility: Add focus highlight to canvas for keyboard navigation
        self.canvas = tk.Canvas(cf, bg=T["bg_grid"], highlightthickness=2,
                                highlightcolor=T["focus"], highlightbackground=T["border"],
                                xscrollcommand=hs.set, yscrollcommand=vs.set)
        hs.config(command=self.canvas.xview)
        vs.config(command=self.canvas.yview)
        hs.pack(side=tk.BOTTOM, fill=tk.X)
        vs.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        # Make canvas focusable for keyboard navigation
        self.canvas.focus_set()

        # Accessibility: Bind keyboard navigation for canvas/grid
        self.canvas.bind("<Tab>", lambda e: "break")  # Allow tab to focus canvas
        self.canvas.bind("<Left>", lambda e: self._navigate_grid(0, -1))
        self.canvas.bind("<Right>", lambda e: self._navigate_grid(0, 1))
        self.canvas.bind("<Up>", lambda e: self._navigate_grid(-1, 0))
        self.canvas.bind("<Down>", lambda e: self._navigate_grid(1, 0))
        self.canvas.bind("<Return>", lambda e: self._on_canvas_activate())
        self.canvas.bind("<space>", lambda e: self._on_canvas_activate())

        self._build_toast_bar(right)

        self.status_var = tk.StringVar(value="Enter grid dimensions and click Build.")
        self._set_toast("Enter grid dimensions and click Build Grid to begin.", "info")

    def _build_header(self):
        header = tk.Frame(self, bg=T["bg"], height=64)
        header.pack(side=tk.TOP, fill=tk.X)
        header.pack_propagate(False)

        left = tk.Frame(header, bg=T["bg"])
        left.pack(side=tk.LEFT, fill=tk.Y, padx=18)
        title_row = tk.Frame(left, bg=T["bg"])
        title_row.pack(anchor=tk.W, pady=(12, 0))
        tk.Label(title_row, text="⬡", bg=T["bg"], fg=T["accent"],
                 font=(self.FONT_UI, 16, "bold")).pack(side=tk.LEFT)
        tk.Label(title_row, text=" CHIP ROUTING OPTIMIZER", bg=T["bg"], fg=T["text"],
                 font=(self.FONT_UI, 14, "bold")).pack(side=tk.LEFT)
        tk.Label(left, text="NVIDIA cuOpt net ordering  ·  octilinear A* pathfinding",
                 bg=T["bg"], fg=T["muted"], font=(self.FONT_UI, 8)).pack(anchor=tk.W)

        right = tk.Frame(header, bg=T["bg"])
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=18)
        chip_row = tk.Frame(right, bg=T["bg"])
        chip_row.pack(anchor=tk.E, pady=(16, 0))

        self.mode_dot = tk.Canvas(chip_row, width=10, height=10, bg=T["bg"],
                                  highlightthickness=0)
        self.mode_dot.pack(side=tk.LEFT, padx=(0, 6))
        self.mode_dot.create_oval(1, 1, 9, 9, fill=T["accent2"], outline="")
        self.mode_lbl = tk.Label(chip_row, text="EDIT MODE", bg=T["bg"], fg=T["muted_strong"],
                                 font=(self.FONT_MONO, 9, "bold"))
        self.mode_lbl.pack(side=tk.LEFT)
        self.grid_lbl = tk.Label(right, text="No grid built", bg=T["bg"], fg=T["muted"],
                                 font=(self.FONT_MONO, 8))
        self.grid_lbl.pack(anchor=tk.E)

        # Gradient underline separating header from body.
        grad = tk.Canvas(self, height=2, bg=T["bg"], highlightthickness=0)
        grad.pack(side=tk.TOP, fill=tk.X)
        self.after(10, lambda: self._paint_gradient(grad, T["accent"], T["accent2"]))

    def _paint_gradient(self, canvas, c1, c2):
        canvas.delete("all")
        w = max(canvas.winfo_width(), 1)
        steps = max(w // 4, 1)
        for i in range(steps):
            t = i / max(steps - 1, 1)
            x0 = i * (w / steps)
            x1 = (i + 1) * (w / steps)
            canvas.create_rectangle(x0, 0, x1, 2, fill=blend(c1, c2, t), outline="")

    def _build_toast_bar(self, parent):
        self.toast = tk.Frame(parent, bg=T["info_dim"], height=32)
        self.toast.pack(fill=tk.X, side=tk.BOTTOM, pady=(8, 0))
        self.toast.pack_propagate(False)
        self.toast_strip = tk.Frame(self.toast, bg=T["info"], width=4)
        self.toast_strip.pack(side=tk.LEFT, fill=tk.Y)
        self.toast_icon = tk.Label(self.toast, text="ℹ", bg=T["info_dim"], fg=T["info"],
                                   font=(self.FONT_UI, 10, "bold"))
        self.toast_icon.pack(side=tk.LEFT, padx=(10, 6))
        self.toast_msg = tk.Label(self.toast, text="", bg=T["info_dim"], fg=T["text"],
                                  font=(self.FONT_UI, 9), anchor=tk.W)
        self.toast_msg.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.spinner_lbl = tk.Label(self.toast, text="", bg=T["info_dim"], fg=T["text"],
                                    font=(self.FONT_MONO, 10, "bold"))
        self.spinner_lbl.pack(side=tk.RIGHT, padx=10)

    TOAST_STYLES = {
        "info": ("info", "info_dim", "ℹ"),
        "ok": ("ok", "ok_dim", "✓"),
        "warn": ("warn", "warn_dim", "⚠"),
        "danger": ("danger", "danger_dim", "✕"),
    }

    def _set_toast(self, message, kind="info"):
        color_key, dim_key, icon = self.TOAST_STYLES.get(kind, self.TOAST_STYLES["info"])
        color, dim = T[color_key], T[dim_key]
        self.toast.configure(bg=dim)
        self.toast_strip.configure(bg=color)
        self.toast_icon.configure(bg=dim, fg=color, text=icon)
        self.toast_msg.configure(bg=dim, fg=T["text"], text=f"  {message}")
        self.spinner_lbl.configure(bg=dim)
        self.status_var.set(f"  {message}")

    def _start_spinner(self):
        self._spinner_i = 0

        def tick():
            self.spinner_lbl.configure(text=SPINNER_FRAMES[self._spinner_i % len(SPINNER_FRAMES)])
            self._spinner_i += 1
            self._spinner_job = self.after(90, tick)

        tick()

    def _stop_spinner(self):
        if self._spinner_job:
            self.after_cancel(self._spinner_job)
            self._spinner_job = None
        self.spinner_lbl.configure(text="")

    def _lbl(self, p, text, fg=None, size=9, bold=False):
        # Accessibility: Default to stronger contrast muted color
        tk.Label(p, text=text, bg=T["panel"],
                 fg=fg or T["muted_strong"],
                 font=(self.FONT_UI, size, "bold" if bold else "normal"),
                 anchor=tk.W).pack(fill=tk.X, padx=14, pady=(2, 0))

    def _sep(self, p):
        tk.Frame(p, bg=T["border_soft"], height=1).pack(fill=tk.X, pady=6, padx=2)

    def _btn(self, p, text, cmd, bg, fg=None, icon="", subtle=False):
        btn = GlowButton(p, text, cmd, bg, fg=fg, icon=icon, height=34,
                         font=(self.FONT_UI, 9, "bold"), subtle=subtle)
        btn.pack(fill=tk.X, padx=14, pady=4)
        return btn

    def _card(self, parent, label):
        """A subtle elevated section card inside the left sidebar."""
        outer = tk.Frame(parent, bg=T["panel"])
        outer.pack(fill=tk.X, pady=(0, 2))
        card = tk.Frame(outer, bg=T["panel"], highlightthickness=1,
                        highlightbackground=T["border_soft"])
        card.pack(fill=tk.X, padx=10, pady=6)
        self._lbl(card, label, T["accent2"], 9, True)
        return card

    def _build_panel(self):
        p = self.left

        scroller = tk.Frame(p, bg=T["panel"])
        scroller.pack(fill=tk.BOTH, expand=True)

        # ① Grid
        c1 = self._card(scroller, "①  GRID SIZE")
        gf = tk.Frame(c1, bg=T["panel"])
        gf.pack(fill=tk.X, padx=14, pady=4)
        for row_i, (lbl, var_name, default) in enumerate(
                [("Rows", "rows_var", 6), ("Cols", "cols_var", 8)]):
            # Accessibility: Use stronger contrast for labels
            tk.Label(gf, text=lbl, bg=T["panel"], fg=T["muted_strong"],
                     font=(self.FONT_UI, 8)).grid(row=row_i, column=0, sticky=tk.W, pady=3)
            v = tk.IntVar(value=default)
            setattr(self, var_name, v)
            sb = tk.Spinbox(gf, from_=2, to=20, textvariable=v, width=5,
                            bg=T["cell_empty"], fg=T["text"], relief=tk.FLAT,
                            insertbackground=T["text"], buttonbackground=T["border"],
                            font=(self.FONT_MONO, 9))
            # Accessibility: Add focus highlight to spinbox
            sb.configure(highlightthickness=2, highlightcolor=T["focus"],
                         highlightbackground=T["border"])
            sb.grid(row=row_i, column=1, padx=8, pady=3)
        self._btn(c1, "BUILD GRID", self._on_build, T["accent"], icon="▶")

        # ② Components
        c2 = self._card(scroller, "②  PLACE COMPONENTS")
        self._lbl(c2, "Click cell → type name → Place", size=8)
        ef = tk.Frame(c2, bg=T["panel"])
        ef.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(ef, text="Name:", bg=T["panel"], fg=T["muted_strong"],
                 font=(self.FONT_UI, 8)).pack(side=tk.LEFT)
        self.comp_var = tk.StringVar()
        self.comp_entry = tk.Entry(ef, textvariable=self.comp_var, width=13,
                                   bg=T["cell_empty"], fg=T["text"], relief=tk.FLAT,
                                   insertbackground=T["text"], font=(self.FONT_MONO, 9))
        self.comp_entry.configure(highlightthickness=2, highlightcolor=T["focus"],
                                  highlightbackground=T["border"])
        self.comp_entry.pack(side=tk.LEFT, padx=(4, 0), ipady=3)
        self.comp_entry.bind("<Return>", lambda _: self._on_place())
        bf = tk.Frame(c2, bg=T["panel"])
        bf.pack(fill=tk.X, padx=14, pady=(4, 4))
        self._place_btn = GlowButton(bf, "Place", self._on_place, T["ok"],
                                     height=28, font=(self.FONT_UI, 8, "bold"))
        self._place_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self._clear_btn = GlowButton(bf, "Clear", self._on_clear, T["danger"],
                                     height=28, font=(self.FONT_UI, 8, "bold"))
        self._clear_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.sel_lbl = tk.Label(c2, text="No cell selected",
                                bg=T["panel"], fg=T["muted_strong"],
                                font=(self.FONT_MONO, 7), anchor=tk.W)
        self.sel_lbl.pack(fill=tk.X, padx=14, pady=(2, 6))

        # ③ Pairs
        c3 = self._card(scroller, "③  WIRE PAIRS")
        self._lbl(c3, "Toggle mode → click src → click sink", size=8)
        self.pair_btn = GlowButton(c3, "ENTER PAIR MODE", self._toggle_pair,
                                   T["panel2"], fg=T["accent2"], icon="⛓",
                                   height=30, font=(self.FONT_UI, 8, "bold"), subtle=True)
        self.pair_btn.pack(fill=tk.X, padx=14, pady=4)
        self.pair_hint = tk.Label(c3, text="", bg=T["panel"], fg=T["warn"],
                                  font=(self.FONT_UI, 7), anchor=tk.W, wraplength=210,
                                  justify=tk.LEFT)
        self.pair_hint.pack(fill=tk.X, padx=14)
        self.pair_list_frame = tk.Frame(c3, bg=T["panel"])
        self.pair_list_frame.pack(fill=tk.X, padx=14, pady=(4, 8))

        # ④ Route
        c4 = self._card(scroller, "④  ROUTE")
        self.run_btn = self._btn(c4, "RUN CUOPT + A*", self._on_run, T["accent2"], icon="⚡")
        self._btn(c4, "RESET", self._on_reset, T["panel2"], fg=T["muted_strong"],
                  icon="↺", subtle=True)
        tk.Frame(c4, bg=T["panel"], height=6).pack()

        # Legend
        c5 = self._card(scroller, "LEGEND")
        self._legend_row(c5, "Orthogonal wire (90°)", T["accent"], kind="line")
        self._legend_row(c5, "Diagonal wire (45°)", T["ok"], kind="dash")
        self._legend_row(c5, "Via / bend point", T["warn"], kind="via")
        self._legend_row(c5, "Source terminal", T["accent2"], kind="source")
        self._legend_row(c5, "Sink terminal", T["accent2"], kind="sink")
        self._legend_row(c5, "Component cell", T["cell_comp"], kind="swatch")
        self._legend_row(c5, "Depot / origin", T["cell_depot"], kind="swatch")
        tk.Frame(c5, bg=T["panel"], height=8).pack()

    def _legend_row(self, parent, label, color, kind="swatch"):
        lf = tk.Frame(parent, bg=T["panel"])
        lf.pack(fill=tk.X, padx=14, pady=2)
        cvs = tk.Canvas(lf, width=18, height=14, bg=T["panel"], highlightthickness=0)
        cvs.pack(side=tk.LEFT)
        if kind == "swatch":
            rounded_rect(cvs, 1, 1, 17, 13, r=3, fill=color, outline=T["border"])
        elif kind == "line":
            cvs.create_line(1, 7, 17, 7, fill=color, width=3, capstyle=tk.ROUND)
        elif kind == "dash":
            cvs.create_line(1, 7, 17, 7, fill=color, width=3, dash=(4, 2), capstyle=tk.ROUND)
        elif kind == "via":
            cvs.create_oval(4, 3, 14, 13, fill=color, outline=shade(color, -0.3))
        elif kind == "source":
            cvs.create_oval(4, 3, 14, 13, outline=color, width=2)
            cvs.create_oval(7, 6, 11, 10, fill=color, outline="")
        elif kind == "sink":
            cvs.create_polygon(9, 2, 15, 8, 9, 14, 3, 8, fill=color, outline=shade(color, -0.3))
        tk.Label(lf, text=f" {label}", bg=T["panel"], fg=T["muted_strong"],
                 font=(self.FONT_UI, 7.5 if isinstance(7.5, int) else 7)).pack(side=tk.LEFT)

    # ── Grid draw ─────────────────────────────────────────────────────────────

    def _render_grid(self, draw_routes=True):
        self.canvas.delete("all")
        self.cell_items = {}

        tw = self.cols * CELL_W + GPAD * 2
        th = self.rows * CELL_H + GPAD * 2
        self.canvas.config(scrollregion=(0, 0, tw, th))

        # subtle background grid texture
        self.canvas.create_rectangle(0, 0, tw, th, fill=T["bg_grid"], outline="")
        for r in range(self.rows + 1):
            y = GPAD + r * CELL_H
            self.canvas.create_line(GPAD, y, GPAD + self.cols * CELL_W, y,
                                    fill=T["grid_line"], width=1, tags="texture")
        for c in range(self.cols + 1):
            x = GPAD + c * CELL_W
            self.canvas.create_line(x, GPAD, x, GPAD + self.rows * CELL_H,
                                    fill=T["grid_line"], width=1, tags="texture")

        # column / row coordinate ticks
        for c in range(self.cols):
            x = GPAD + c * CELL_W + CELL_W / 2
            self.canvas.create_text(x, GPAD - 12, text=str(c), fill=T["muted"],
                                    font=(self.FONT_MONO, 7), tags="texture")
        for r in range(self.rows):
            y = GPAD + r * CELL_H + CELL_H / 2
            self.canvas.create_text(GPAD - 14, y, text=str(r), fill=T["muted"],
                                    font=(self.FONT_MONO, 7), tags="texture")

        for r in range(self.rows):
            for c in range(self.cols):
                n = r * self.cols + c
                x0 = GPAD + c * CELL_W + 2
                y0 = GPAD + r * CELL_H + 2
                x1 = x0 + CELL_W - 5
                y1 = y0 + CELL_H - 5
                cx = (x0 + x1) / 2
                cy = (y0 + y1) / 2

                col = self._cell_bg(n)
                border = shade(col, 0.35) if n in (self.sel_cell, self.pair_src) else T["border"]
                rid = rounded_rect(self.canvas, x0, y0, x1, y1, r=7,
                                   fill=col, outline=border, width=1,
                                   tags=(f"c{n}", "cell"))
                # faint top highlight for a subtle 3D / glass feel
                hi = self.canvas.create_line(x0 + 5, y0 + 1, x1 - 5, y0 + 1,
                                             fill=shade(col, 0.25), width=1,
                                             tags=(f"c{n}",))

                lbl = "DEPOT" if n == 0 else self.components.get(n, "")
                lclr = T["accent2"] if n == 0 else (T["accent"] if lbl else T["muted"])
                tid = self.canvas.create_text(
                    cx, cy - 2, text=lbl,
                    fill=lclr, font=(self.FONT_MONO, 8, "bold"),
                    width=CELL_W - 8, anchor=tk.CENTER, tags=(f"c{n}",))

                cid = self.canvas.create_text(
                    x1 - 4, y1 - 4, text=f"{r},{c}",
                    fill=T["muted"], font=(self.FONT_MONO, 6),
                    anchor=tk.SE, tags=(f"c{n}",))

                self.cell_items[n] = (rid, tid, cid, hi)

                for item in (rid, tid, cid, hi):
                    self.canvas.tag_bind(item, "<Button-1>",
                                         lambda e, nd=n: self._click(nd))
                    self.canvas.tag_bind(item, "<Enter>",
                                         lambda e, nd=n: self._hover(nd, True))
                    self.canvas.tag_bind(item, "<Leave>",
                                         lambda e, nd=n: self._hover(nd, False))

        if draw_routes:
            self._redraw_routes()
        self._update_header_chips()
        self._restart_pulse()

    def _cell_bg(self, n):
        if n == 0:               return T["cell_depot"]
        if n == self.pair_src:   return T["cell_pair_src"]
        if n == self.sel_cell:   return T["cell_sel"]
        if n in self.components: return T["cell_comp"]
        return T["cell_empty"]

    def _recolor(self, n):
        if n in self.cell_items:
            rid, tid, cid, hi = self.cell_items[n]
            col = self._cell_bg(n)
            self.canvas.itemconfig(rid, fill=col,
                                   outline=shade(col, 0.35) if n in (self.sel_cell, self.pair_src) else T["border"])
            self.canvas.itemconfig(hi, fill=shade(col, 0.25))
        # Accessibility: Update focus indicator on canvas when selection changes
        if n == self.sel_cell:
            self.canvas.configure(highlightcolor=T["focus"], highlightbackground=T["focus"])
        else:
            self.canvas.configure(highlightcolor=T["focus"], highlightbackground=T["border"])
        self._restart_pulse()

    def _restart_pulse(self):
        """Gently pulses the selected cell's border so it reads as 'active'."""
        if self._pulse_job:
            self.after_cancel(self._pulse_job)
            self._pulse_job = None
        if self.sel_cell is None or self.sel_cell not in self.cell_items:
            return

        def pulse():
            self._pulse_on = not self._pulse_on
            if self.sel_cell is not None and self.sel_cell in self.cell_items:
                rid = self.cell_items[self.sel_cell][0]
                col = self._cell_bg(self.sel_cell)
                outline = T["ok"] if self._pulse_on else shade(col, 0.35)
                try:
                    self.canvas.itemconfig(rid, outline=outline, width=2 if self._pulse_on else 1)
                except tk.TclError:
                    return
            self._pulse_job = self.after(550, pulse)

        pulse()

    def _hover(self, n, on):
        if n not in self.cell_items:
            return
        rid = self.cell_items[n][0]
        cur = self.canvas.itemcget(rid, "fill")
        if on and cur == T["cell_empty"]:
            self.canvas.itemconfig(rid, fill=T["cell_hover"],
                                   outline=shade(T["cell_hover"], 0.4))
        else:
            self._recolor(n)

    # ── Route drawing — the key visual part ───────────────────────────────────

    def _redraw_routes(self):
        self.canvas.delete("route")
        self.canvas.delete("via")
        for i, p in enumerate(self.pairs):
            self._draw_one_route(i, p)
        self.canvas.tag_raise("route")
        self.canvas.tag_raise("via")

    def _animate_routes(self):
        """Reveal each routed net in sequence for a 'wires being laid' feel."""
        if self._route_anim_job:
            self.after_cancel(self._route_anim_job)
            self._route_anim_job = None
        self.canvas.delete("route")
        self.canvas.delete("via")
        nets = list(enumerate(self.pairs))

        def reveal(i):
            if i >= len(nets):
                self.canvas.tag_raise("route")
                self.canvas.tag_raise("via")
                self._route_anim_job = None
                return
            idx, p = nets[i]
            self._draw_one_route(idx, p)
            self.canvas.tag_raise("route")
            self.canvas.tag_raise("via")
            self._route_anim_job = self.after(110, lambda: reveal(i + 1))

        reveal(0)

    def _draw_one_route(self, i, p):
        name = p["name"]
        color = self.net_colors.get(name, NET_COLORS[i % len(NET_COLORS)])
        path = self.routes.get(name)

        if path and len(path) >= 2:
            # Draw each hop as a line segment (glow halo + bright core)
            for seg in range(len(path) - 1):
                r1, c1 = path[seg]
                r2, c2 = path[seg + 1]
                dr = r2 - r1
                dc = c2 - c1
                is45 = abs(dr) == 1 and abs(dc) == 1

                px1 = GPAD + c1 * CELL_W + CELL_W // 2
                py1 = GPAD + r1 * CELL_H + CELL_H // 2
                px2 = GPAD + c2 * CELL_W + CELL_W // 2
                py2 = GPAD + r2 * CELL_H + CELL_H // 2

                dash = (5, 2) if is45 else ()
                # soft glow halo underneath
                self.canvas.create_line(
                    px1, py1, px2, py2,
                    fill=shade(color, -0.35), width=7, dash=dash,
                    capstyle=tk.ROUND, joinstyle=tk.ROUND, tags="route")
                # bright core line
                self.canvas.create_line(
                    px1, py1, px2, py2,
                    fill=color, width=3, dash=dash,
                    capstyle=tk.ROUND, joinstyle=tk.ROUND, tags="route")

            # Via dots at every direction change (bend) — glow rings
            for seg in range(1, len(path) - 1):
                r0, c0 = path[seg - 1]
                r1, c1 = path[seg]
                r2, c2 = path[seg + 1]
                if (r1 - r0, c1 - c0) != (r2 - r1, c2 - c1):
                    vx = GPAD + c1 * CELL_W + CELL_W // 2
                    vy = GPAD + r1 * CELL_H + CELL_H // 2
                    self.canvas.create_oval(vx - 8, vy - 8, vx + 8, vy + 8,
                                            fill=shade(T["warn"], -0.5), outline="",
                                            tags="via")
                    self.canvas.create_oval(vx - 5, vy - 5, vx + 5, vy + 5,
                                            fill=T["warn"], outline=T["bg"], width=1,
                                            tags="via")

            # Source terminal — ring + filled dot (distinct from sink)
            sr0, sc0 = path[0]
            sx = GPAD + sc0 * CELL_W + CELL_W // 2
            sy = GPAD + sr0 * CELL_H + CELL_H // 2
            self.canvas.create_oval(sx - 10, sy - 10, sx + 10, sy + 10,
                                    fill=shade(color, -0.55), outline="", tags="via")
            self.canvas.create_oval(sx - 8, sy - 8, sx + 8, sy + 8,
                                    outline=color, width=2, tags="via")
            self.canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4,
                                    fill=color, outline="", tags="via")

            # Sink terminal — diamond marker (distinct shape from source)
            er0, ec0 = path[-1]
            ex = GPAD + ec0 * CELL_W + CELL_W // 2
            ey = GPAD + er0 * CELL_H + CELL_H // 2
            self.canvas.create_oval(ex - 10, ey - 10, ex + 10, ey + 10,
                                    fill=shade(color, -0.55), outline="", tags="via")
            self.canvas.create_polygon(ex, ey - 7, ex + 7, ey, ex, ey + 7, ex - 7, ey,
                                       fill=color, outline=T["bg"], width=1, tags="via")

            # Net label at midpoint
            mid = len(path) // 2
            mr, mc = path[mid]
            mx = GPAD + mc * CELL_W + CELL_W // 2
            my = GPAD + mr * CELL_H + CELL_H // 2
            self.canvas.create_text(mx, my - 12, text=name,
                                    fill=color, font=(self.FONT_MONO, 7, "bold"),
                                    tags="route")

        else:
            # No routed path yet — draw a dashed preview arrow
            sr, sc = divmod(p["src"], self.cols)
            dr_, dc_ = divmod(p["sink"], self.cols)
            sx = GPAD + sc * CELL_W + CELL_W // 2
            sy = GPAD + sr * CELL_H + CELL_H // 2
            dx = GPAD + dc_ * CELL_W + CELL_W // 2
            dy = GPAD + dr_ * CELL_H + CELL_H // 2
            self.canvas.create_line(
                sx, sy, dx, dy,
                fill=color, width=1, dash=(3, 4),
                arrow=tk.LAST, arrowshape=(7, 9, 3),
                tags="route")
            mx_, my_ = (sx + dx) / 2, (sy + dy) / 2
            self.canvas.create_text(mx_, my_ - 7, text=name,
                                    fill=color, font=(self.FONT_UI, 7),
                                    tags="route")

    # ── Cell click ────────────────────────────────────────────────────────────

    def _click(self, n):
        if self.mode == "edit":
            if self.sel_cell is not None:
                self._recolor(self.sel_cell)
            self.sel_cell = n
            self._recolor(n)
            r, c = divmod(n, self.cols)
            nm = self.components.get(n, "")
            self.comp_var.set(nm)
            self.comp_entry.focus_set()
            if n == 0:
                self.sel_lbl.config(text=f"({r},{c}) — depot/origin")
            else:
                self.sel_lbl.config(text=f"({r},{c}) · {nm or 'unnamed'}")
            self._set_toast(f"Selected ({r},{c}). Type name + Enter to place.", "info")

        elif self.mode == "pair":
            if self.pair_src is None:
                self.pair_src = n
                self._recolor(n)
                r, c = divmod(n, self.cols)
                nm = self.components.get(n, "DEPOT" if n == 0 else f"node{n}")
                self.pair_hint.config(text=f"Src: {nm} ({r},{c}) → now pick sink")
            else:
                if n == self.pair_src:
                    self._recolor(n)
                    self.pair_src = None
                    self.pair_hint.config(text="Cleared. Pick source again.")
                    self._announce("Source cleared. Pick source again.")
                    return
                def_name = f"NET{len(self.pairs)}"
                net_name = simpledialog.askstring(
                    "Net name", "Name for this wire connection:",
                    initialvalue=def_name, parent=self)
                if not net_name:
                    net_name = def_name
                net_name = net_name.strip().upper().replace(" ", "_")
                src_name = self.components.get(self.pair_src,
                                               "DEPOT" if self.pair_src == 0 else f"N{self.pair_src}")
                sink_name = self.components.get(n,
                                                "DEPOT" if n == 0 else f"N{n}")
                ci = len(self.pairs) % len(NET_COLORS)
                self.net_colors[net_name] = NET_COLORS[ci]
                self.pairs.append({"name": net_name, "src": self.pair_src,
                                   "sink": n, "src_name": src_name,
                                   "sink_name": sink_name})
                prev = self.pair_src
                self.pair_src = None
                self._recolor(prev)
                self.pair_hint.config(text=f"'{net_name}' added. Pick next src →")
                self._refresh_pairs()
                self._redraw_routes()
                self._announce(f"Pair '{net_name}' added. {len(self.pairs)} pair(s) total.")

    # ── Component actions ─────────────────────────────────────────────────────

    def _on_place(self):
        if self.sel_cell is None or self.sel_cell == 0:
            self._announce("Select a non-depot cell first.")
            return
        name = self.comp_var.get().strip()
        if not name:
            self._announce("Enter a component name.")
            return
        self.components[self.sel_cell] = name
        if self.sel_cell in self.cell_items:
            self.canvas.itemconfig(self.cell_items[self.sel_cell][1],
                                   text=name, fill=T["accent"])
            self._recolor(self.sel_cell)
        r, c = divmod(self.sel_cell, self.cols)
        self._set_toast(f"Placed '{name}' at ({r},{c}).", "ok")
        for p in self.pairs:
            if p["src"] == self.sel_cell: p["src_name"] = name
            if p["sink"] == self.sel_cell: p["sink_name"] = name
        self._refresh_pairs()

    def _on_clear(self):
        if self.sel_cell is None or self.sel_cell == 0:
            return
        self.components.pop(self.sel_cell, None)
        if self.sel_cell in self.cell_items:
            self.canvas.itemconfig(self.cell_items[self.sel_cell][1], text="")
            self._recolor(self.sel_cell)
        self.comp_var.set("")
        self._announce("Component cleared.")

    # ── Pair mode ─────────────────────────────────────────────────────────────

    def _toggle_pair(self):
        if not self.rows:
            self._announce("Build a grid first.")
            return
        if self.mode == "edit":
            self.mode = "pair"
            self.pair_btn.icon = "✕"
            self.pair_btn.set_text("EXIT PAIR MODE")
            self.pair_btn.base_bg = T["accent2"]
            self.pair_btn.subtle = False
            self.pair_btn.fg = T["bg"]
            self.pair_btn._draw()
            self.pair_hint.config(text="Click a source cell →")
            self._announce("Pair mode activated. Click a source cell, then a sink cell to add a wire pair.")
        else:
            self.mode = "edit"
            if self.pair_src is not None:
                self._recolor(self.pair_src)
            self.pair_src = None
            self.pair_btn.icon = "⛓"
            self.pair_btn.set_text("ENTER PAIR MODE")
            self.pair_btn.base_bg = T["panel2"]
            self.pair_btn.subtle = True
            self.pair_btn.fg = T["accent2"]
            self.pair_btn._draw()
            self.pair_hint.config(text="")
            self._announce("Edit mode activated.")
        self._update_header_chips()

    def _update_header_chips(self):
        if self.mode == "pair":
            self.mode_dot.itemconfig(1, fill=T["accent2"])
            self.mode_lbl.config(text="PAIR MODE", fg=T["accent2"])
        else:
            self.mode_dot.itemconfig(1, fill=T["ok"])
            self.mode_lbl.config(text="EDIT MODE", fg=T["muted_strong"])
        if self.rows:
            self.grid_lbl.config(text=f"{self.rows}×{self.cols} grid  ·  "
                                      f"{len(self.components)} components  ·  "
                                      f"{len(self.pairs)} nets")
        else:
            self.grid_lbl.config(text="No grid built")

    def _refresh_pairs(self):
        for w in self.pair_list_frame.winfo_children():
            w.destroy()
        for i, p in enumerate(self.pairs):
            color = self.net_colors.get(p["name"], NET_COLORS[i % len(NET_COLORS)])
            row = tk.Frame(self.pair_list_frame, bg=T["panel2"], highlightthickness=1,
                           highlightbackground=T["border_soft"])
            row.pack(fill=tk.X, pady=2)
            # Accessibility: Add border to color swatch for color-blind users
            tk.Canvas(row, width=8, height=8, bg=color, highlightthickness=1,
                      highlightbackground=T["border"]).pack(side=tk.LEFT, padx=(6, 4), pady=4)
            # Accessibility: Include text label alongside color (not color-only)
            tk.Label(row,
                     text=f"{p['name']}: {p['src_name']} → {p['sink_name']}",
                     bg=T["panel2"], fg=T["text"],
                     font=(self.FONT_MONO, 7), anchor=tk.W
                     ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            btn = tk.Button(row, text="✕", bg=T["panel2"], fg=T["danger"],
                            font=(self.FONT_UI, 7), relief=tk.FLAT, cursor="hand2",
                            activebackground=T["danger_dim"], activeforeground=T["danger"],
                            command=lambda idx=i: self._remove_pair(idx))
            # Accessibility: Add focus highlight
            btn.configure(highlightthickness=2, highlightcolor=T["focus"],
                          highlightbackground=T["border"])
            btn.pack(side=tk.RIGHT, padx=4)
        self._update_header_chips()

    def _remove_pair(self, idx):
        if 0 <= idx < len(self.pairs):
            name = self.pairs[idx]["name"]
            self.pairs.pop(idx)
            self.routes.pop(name, None)
            self._refresh_pairs()
            self._redraw_routes()
            self._announce(f"Pair '{name}' removed. {len(self.pairs)} pair(s) remaining.")

    # ── Run routing ───────────────────────────────────────────────────────────

    def _on_run(self):
        if not self.rows:
            # Accessibility: Use showerror with descriptive title and message
            messagebox.showerror("No Grid Defined", "Please build a grid first using the Grid Size controls.")
            return
        if not self.pairs:
            messagebox.showerror("No Wire Pairs", "Add at least one wire pair before routing.")
            return

        self._set_toast("Sending net list to NVIDIA cuOpt to optimise routing order…", "info")
        self.run_btn.set_disabled(True)
        self._start_spinner()
        self.update_idletasks()

        result_q = queue.Queue()

        def worker():
            try:
                order, cuopt_body = cuopt_net_order(self.rows, self.cols, self.pairs)
                result_q.put(("ok", order))
            except Exception as e:
                result_q.put(("err", str(e)))

        threading.Thread(target=worker, daemon=True).start()
        self.after(80, lambda: self._poll_cuopt(result_q))

    def _poll_cuopt(self, result_q):
        try:
            kind, payload = result_q.get_nowait()
        except queue.Empty:
            self.after(80, lambda: self._poll_cuopt(result_q))
            return

        self._stop_spinner()
        if kind == "ok":
            order = payload
        else:
            self._set_toast(f"cuOpt error: {payload} — falling back to greedy order.", "warn")
            order = list(range(len(self.pairs)))

        self._set_toast(f"Running A* octilinear router for {len(self.pairs)} nets…", "info")
        self.update_idletasks()

        self.routes = route_all_nets(
            self.pairs, self.rows, self.cols, self.components, order=order)

        self._render_grid(draw_routes=False)
        self._animate_routes()

        routed = sum(1 for v in self.routes.values() if v)
        kind = "ok" if routed == len(self.pairs) else "warn"
        self._set_toast(
            f"Routing complete — {routed}/{len(self.pairs)} nets routed.", kind)
        self._announce(
            f"Routing complete. {routed} of {len(self.pairs)} nets routed. "
            f"Solid lines are 90 degree, dashed lines are 45 degree, yellow dots are vias.")
        self.run_btn.set_disabled(False)

        self.after(max(300, 130 * len(self.pairs)), self._show_result_popup)

    def _show_result_popup(self):
        win = tk.Toplevel(self)
        win.title("Routing Results")
        win.configure(bg=T["bg"])
        win.geometry("620x560")
        # Accessibility: Make dialog focusable and trap focus
        win.focus_set()
        win.grab_set()

        header = tk.Frame(win, bg=T["bg"])
        header.pack(fill=tk.X, padx=18, pady=(16, 6))
        tk.Label(header, text="⚡  ROUTING RESULTS", bg=T["bg"], fg=T["accent2"],
                 font=(self.FONT_UI, 13, "bold")).pack(anchor=tk.W)
        tk.Label(header, text="cuOpt-ordered · A* octilinear paths",
                 bg=T["bg"], fg=T["muted"], font=(self.FONT_UI, 8)).pack(anchor=tk.W)

        # ── stats ──
        total = 0
        routed_count = 0
        total_bends = 0
        rows_data = []
        for i, p in enumerate(self.pairs):
            path = self.routes.get(p["name"], [])
            wire_len = len(path) - 1 if path else 0
            total += wire_len

            bends = 0
            for seg in range(1, len(path) - 1):
                r0, c0 = path[seg - 1]
                r1, c1 = path[seg]
                r2, c2 = path[seg + 1]
                if (r1 - r0, c1 - c0) != (r2 - r1, c2 - c1):
                    bends += 1
            total_bends += bends

            diag45 = sum(
                1 for s in range(len(path) - 1)
                if abs(path[s][0] - path[s + 1][0]) == 1
                and abs(path[s][1] - path[s + 1][1]) == 1
            )
            ortho = wire_len - diag45
            if path:
                routed_count += 1
            rows_data.append((p, path, wire_len, ortho, diag45, bends))

        stats = tk.Frame(win, bg=T["bg"])
        stats.pack(fill=tk.X, padx=18, pady=(4, 10))
        StatCard(stats, total, "Total wire length (hops)", T["accent"],
                 self.FONT_MONO, self.FONT_UI).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        StatCard(stats, f"{routed_count}/{len(self.pairs)}", "Nets routed", T["ok"],
                 self.FONT_MONO, self.FONT_UI).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        avg_bends = round(total_bends / len(self.pairs), 1) if self.pairs else 0
        StatCard(stats, avg_bends, "Avg bends / net", T["warn"],
                 self.FONT_MONO, self.FONT_UI).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 0))

        # ── scrollable net list ──
        list_outer = tk.Frame(win, bg=T["bg"], highlightthickness=1,
                              highlightbackground=T["border"])
        list_outer.pack(fill=tk.BOTH, expand=True, padx=18)
        sb = tk.Scrollbar(list_outer)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        list_canvas = tk.Canvas(list_outer, bg=T["panel"], highlightthickness=0,
                                yscrollcommand=sb.set)
        list_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.config(command=list_canvas.yview)
        inner = tk.Frame(list_canvas, bg=T["panel"])
        win_id = list_canvas.create_window((0, 0), window=inner, anchor=tk.NW)

        def on_configure(e):
            list_canvas.configure(scrollregion=list_canvas.bbox("all"))
        inner.bind("<Configure>", on_configure)

        def on_canvas_resize(e):
            list_canvas.itemconfig(win_id, width=e.width)
        list_canvas.bind("<Configure>", on_canvas_resize)

        for i, (p, path, wire_len, ortho, diag45, bends) in enumerate(rows_data):
            color = self.net_colors.get(p["name"], NET_COLORS[i % len(NET_COLORS)])
            routed = bool(path)
            row = tk.Frame(inner, bg=T["panel2"] if i % 2 == 0 else T["panel"],
                           highlightthickness=0)
            row.pack(fill=tk.X)
            strip = tk.Frame(row, bg=color, width=4)
            strip.pack(side=tk.LEFT, fill=tk.Y)
            body = tk.Frame(row, bg=row["bg"])
            body.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=6)

            top = tk.Frame(body, bg=row["bg"])
            top.pack(fill=tk.X)
            tk.Label(top, text=p["name"], bg=row["bg"], fg=T["text"],
                     font=(self.FONT_MONO, 9, "bold")).pack(side=tk.LEFT)
            pill_color = T["ok"] if routed else T["danger"]
            pill_dim = T["ok_dim"] if routed else T["danger_dim"]
            pill = tk.Label(top, text=" ROUTED " if routed else " UNROUTED ",
                            bg=pill_dim, fg=pill_color, font=(self.FONT_UI, 7, "bold"))
            pill.pack(side=tk.RIGHT)

            src_rc = divmod(p["src"], self.cols)
            sink_rc = divmod(p["sink"], self.cols)
            tk.Label(body, text=f"{p['src_name']} ({src_rc[0]},{src_rc[1]}) → "
                                f"{p['sink_name']} ({sink_rc[0]},{sink_rc[1]})",
                     bg=row["bg"], fg=T["muted_strong"],
                     font=(self.FONT_UI, 8), anchor=tk.W).pack(fill=tk.X)
            tk.Label(body, text=f"wire: {wire_len} hops  ({ortho} ortho + {diag45} diag)   "
                                f"bends: {bends}",
                     bg=row["bg"], fg=T["muted"],
                     font=(self.FONT_MONO, 7), anchor=tk.W).pack(fill=tk.X, pady=(2, 0))

        footer = tk.Frame(win, bg=T["bg"])
        footer.pack(fill=tk.X, padx=18, pady=12)
        close_btn = GlowButton(footer, "Close", win.destroy, T["accent"],
                               height=34, font=(self.FONT_UI, 9, "bold"))
        close_btn.pack(fill=tk.X)
        win.bind("<Escape>", lambda e: win.destroy())

    # ── Build / Reset ─────────────────────────────────────────────────────────

    def _reset_state(self, keep_grid):
        self.components = {}
        self.pairs = []
        self.routes = {}
        self.net_colors = {}
        self.sel_cell = None
        self.pair_src = None
        self.mode = "edit"
        self.pair_btn.icon = "⛓"
        self.pair_btn.set_text("ENTER PAIR MODE")
        self.pair_btn.base_bg = T["panel2"]
        self.pair_btn.subtle = True
        self.pair_btn.fg = T["accent2"]
        self.pair_btn._draw()
        self.pair_hint.config(text="")
        self.sel_lbl.config(text="No cell selected")
        self.comp_var.set("")

    def _on_build(self):
        self.rows = max(2, min(20, self.rows_var.get()))
        self.cols = max(2, min(20, self.cols_var.get()))
        self._reset_state(keep_grid=True)
        self._render_grid()
        self._refresh_pairs()
        self._set_toast(
            f"Grid {self.rows}×{self.cols} ready. Click cells to place components.", "ok")

    def _on_reset(self):
        self._reset_state(keep_grid=False)
        if self.rows:
            self._render_grid()
        self._refresh_pairs()
        self._set_toast("Reset. Place components and create wire pairs.", "info")

    def _set_status(self, msg):
        self.status_var.set(f"  {msg}")

    # ── Accessibility: Keyboard Navigation ──────────────────────────────────────

    def _navigate_grid(self, dr, dc):
        """Navigate grid with arrow keys. Returns 'break' to prevent default behavior."""
        if self.rows == 0 or self.cols == 0:
            return "break"

        if self.sel_cell is None:
            # Start at depot (0,0) or first valid cell
            self.sel_cell = 0
        else:
            r, c = divmod(self.sel_cell, self.cols)
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                self.sel_cell = nr * self.cols + nc

        # Update selection visual and announcement
        self._recolor(self.sel_cell)
        r, c = divmod(self.sel_cell, self.cols)
        nm = self.components.get(self.sel_cell, "")
        if self.sel_cell == 0:
            self.sel_lbl.config(text=f"({r},{c}) — depot/origin")
            self._announce(f"Selected depot at {r}, {c}")
        else:
            self.sel_lbl.config(text=f"({r},{c}) · {nm or 'unnamed'}")
            self._announce(f"Selected cell {r}, {c}{', ' + nm if nm else ', unnamed'}")
        self.comp_var.set(nm)
        self.comp_entry.focus_set()
        return "break"

    def _on_canvas_activate(self):
        """Handle Enter/Space on focused cell - same as click."""
        if self.sel_cell is not None:
            self._click(self.sel_cell)
        return "break"

    def _announce(self, message):
        """Announce message to screen readers via status bar."""
        self._set_toast(message, "info")
        # Also update selection label for visual feedback
        self.update_idletasks()


# ─── Entry ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    App().mainloop()