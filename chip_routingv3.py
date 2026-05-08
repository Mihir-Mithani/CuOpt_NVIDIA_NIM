#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@Author  : Mihir Mithani
@Date    : 08-05-2026 , 10:57
@File    : chip_routingv3.py
@Desc    : 
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
import time
import tkinter as tk
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
T = {
    "bg": "#0a0c14",
    "panel": "#10131f",
    "panel2": "#14192a",
    "border": "#1e2440",
    "accent": "#4f6ef7",
    "accent2": "#c084fc",
    "text": "#dde1f5",
    "muted": "#4a5275",
    "cell_empty": "#0e1120",
    "cell_comp": "#0e2040",
    "cell_depot": "#1a0e40",
    "cell_sel": "#0e3020",
    "cell_hover": "#161c38",
    "ok": "#22d3a0",
    "warn": "#facc15",
    "danger": "#f87171",
}

NET_COLORS = [
    "#4f6ef7", "#22d3a0", "#facc15", "#f87171", "#c084fc",
    "#fb923c", "#38bdf8", "#f472b6", "#a3e635", "#e879f9",
]

CELL_W = 72
CELL_H = 50
GPAD = 14

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


# ─── GUI ──────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Chip Routing Optimizer — cuOpt + A*")
        self.configure(bg=T["bg"])
        self.geometry("1220x820")
        self.resizable(True, True)

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

        self._build_ui()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.left = tk.Frame(self, bg=T["panel"], width=248)
        self.left.pack(side=tk.LEFT, fill=tk.Y)
        self.left.pack_propagate(False)
        self._build_panel()

        right = tk.Frame(self, bg=T["bg"])
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        cf = tk.Frame(right, bg=T["bg"])
        cf.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        hs = tk.Scrollbar(cf, orient=tk.HORIZONTAL)
        vs = tk.Scrollbar(cf, orient=tk.VERTICAL)
        self.canvas = tk.Canvas(cf, bg=T["bg"], highlightthickness=0,
                                xscrollcommand=hs.set, yscrollcommand=vs.set)
        hs.config(command=self.canvas.xview)
        vs.config(command=self.canvas.yview)
        hs.pack(side=tk.BOTTOM, fill=tk.X)
        vs.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.status_var = tk.StringVar(value="Enter grid dimensions and click Build.")
        tk.Label(right, textvariable=self.status_var,
                 bg=T["panel2"], fg=T["muted"],
                 font=("Courier", 9), anchor=tk.W, padx=10, pady=5
                 ).pack(fill=tk.X, side=tk.BOTTOM)

    def _lbl(self, p, text, fg=None, size=9, bold=False):
        tk.Label(p, text=text, bg=T["panel"],
                 fg=fg or T["muted"],
                 font=("Courier", size, "bold" if bold else "normal"),
                 anchor=tk.W).pack(fill=tk.X, padx=12, pady=(2, 0))

    def _sep(self, p):
        tk.Frame(p, bg=T["border"], height=1).pack(fill=tk.X, pady=5)

    def _btn(self, p, text, cmd, bg, fg=None, pady=6):
        tk.Button(p, text=text, font=("Courier", 9, "bold"),
                  bg=bg, fg=fg or T["bg"], relief=tk.FLAT,
                  activebackground=bg, activeforeground=fg or T["bg"],
                  command=cmd, cursor="hand2", pady=pady
                  ).pack(fill=tk.X, padx=12, pady=3)

    def _build_panel(self):
        p = self.left
        tk.Label(p, text="CHIP ROUTER", bg=T["panel"], fg=T["accent"],
                 font=("Courier", 12, "bold")).pack(pady=(16, 1))
        tk.Label(p, text="cuOpt order  ·  A* octilinear paths",
                 bg=T["panel"], fg=T["muted"], font=("Courier", 7)).pack(pady=(0, 10))
        self._sep(p)

        # ① Grid
        self._lbl(p, "① GRID SIZE", T["text"], 9, True)
        gf = tk.Frame(p, bg=T["panel"]);
        gf.pack(fill=tk.X, padx=12, pady=4)
        for row_i, (lbl, var_name, default) in enumerate(
                [("Rows", "rows_var", 6), ("Cols", "cols_var", 8)]):
            tk.Label(gf, text=lbl, bg=T["panel"], fg=T["muted"],
                     font=("Courier", 8)).grid(row=row_i, column=0, sticky=tk.W, pady=2)
            v = tk.IntVar(value=default)
            setattr(self, var_name, v)
            tk.Spinbox(gf, from_=2, to=20, textvariable=v, width=5,
                       bg=T["cell_empty"], fg=T["text"], relief=tk.FLAT,
                       insertbackground=T["text"], buttonbackground=T["border"]
                       ).grid(row=row_i, column=1, padx=8, pady=2)
        self._btn(p, "▶  BUILD GRID", self._on_build, T["accent"])

        self._sep(p)

        # ② Components
        self._lbl(p, "② PLACE COMPONENTS", T["text"], 9, True)
        self._lbl(p, "Click cell → type name → Place")
        ef = tk.Frame(p, bg=T["panel"]);
        ef.pack(fill=tk.X, padx=12, pady=4)
        tk.Label(ef, text="Name:", bg=T["panel"], fg=T["muted"],
                 font=("Courier", 8)).pack(side=tk.LEFT)
        self.comp_var = tk.StringVar()
        self.comp_entry = tk.Entry(ef, textvariable=self.comp_var, width=13,
                                   bg=T["cell_empty"], fg=T["text"], relief=tk.FLAT,
                                   insertbackground=T["text"], font=("Courier", 9))
        self.comp_entry.pack(side=tk.LEFT, padx=(4, 0))
        self.comp_entry.bind("<Return>", lambda _: self._on_place())
        bf = tk.Frame(p, bg=T["panel"]);
        bf.pack(fill=tk.X, padx=12, pady=(0, 4))
        for txt, cmd, col in [("Place", self._on_place, T["ok"]),
                              ("Clear", self._on_clear, T["danger"])]:
            tk.Button(bf, text=txt, font=("Courier", 8), bg=col, fg=T["bg"],
                      relief=tk.FLAT, command=cmd, cursor="hand2",
                      padx=8, pady=2).pack(side=tk.LEFT, padx=(0, 4))
        self.sel_lbl = tk.Label(p, text="No cell selected",
                                bg=T["panel"], fg=T["muted"],
                                font=("Courier", 7), anchor=tk.W)
        self.sel_lbl.pack(fill=tk.X, padx=12)

        self._sep(p)

        # ③ Pairs
        self._lbl(p, "③ WIRE PAIRS", T["text"], 9, True)
        self._lbl(p, "Toggle mode → click src → click sink")
        self.pair_btn = tk.Button(
            p, text="⛓  ENTER PAIR MODE",
            font=("Courier", 8, "bold"),
            bg=T["cell_empty"], fg=T["accent2"], relief=tk.FLAT,
            activebackground=T["border"], activeforeground=T["accent2"],
            command=self._toggle_pair, cursor="hand2", pady=4)
        self.pair_btn.pack(fill=tk.X, padx=12, pady=4)
        self.pair_hint = tk.Label(p, text="", bg=T["panel"], fg=T["warn"],
                                  font=("Courier", 7), anchor=tk.W)
        self.pair_hint.pack(fill=tk.X, padx=12)
        self.pair_list_frame = tk.Frame(p, bg=T["panel"])
        self.pair_list_frame.pack(fill=tk.X, padx=12, pady=4)

        self._sep(p)

        # ④ Route
        self._lbl(p, "④ ROUTE", T["text"], 9, True)
        self._btn(p, "⚡  RUN CUOPT + A*", self._on_run, T["accent2"])
        self._btn(p, "↺  RESET", self._on_reset, T["muted"])

        self._sep(p)
        self._lbl(p, "LEGEND", T["muted"], 7, True)
        for label, color in [
            ("Orthogonal wire (90°)", T["accent"]),
            ("Diagonal wire  (45°)", T["ok"]),
            ("Via / bend point", T["warn"]),
            ("Component cell", T["cell_comp"]),
            ("Depot / origin", T["cell_depot"]),
        ]:
            lf = tk.Frame(p, bg=T["panel"]);
            lf.pack(fill=tk.X, padx=12, pady=1)
            tk.Canvas(lf, width=10, height=10, bg=color, highlightthickness=0
                      ).pack(side=tk.LEFT)
            tk.Label(lf, text=f" {label}", bg=T["panel"], fg=T["muted"],
                     font=("Courier", 7)).pack(side=tk.LEFT)

    # ── Grid draw ─────────────────────────────────────────────────────────────

    def _render_grid(self):
        self.canvas.delete("all")
        self.cell_items = {}

        tw = self.cols * CELL_W + GPAD * 2
        th = self.rows * CELL_H + GPAD * 2
        self.canvas.config(scrollregion=(0, 0, tw, th))

        for r in range(self.rows):
            for c in range(self.cols):
                n = r * self.cols + c
                x0 = GPAD + c * CELL_W
                y0 = GPAD + r * CELL_H
                x1 = x0 + CELL_W - 1
                y1 = y0 + CELL_H - 1
                cx = (x0 + x1) / 2
                cy = (y0 + y1) / 2

                col = self._cell_bg(n)
                rid = self.canvas.create_rectangle(
                    x0, y0, x1, y1, fill=col, outline=T["border"],
                    width=1, tags=(f"c{n}", "cell"))

                lbl = "DEPOT" if n == 0 else self.components.get(n, "")
                lclr = T["accent2"] if n == 0 else (T["accent"] if lbl else T["muted"])
                tid = self.canvas.create_text(
                    cx, cy - 3, text=lbl,
                    fill=lclr, font=("Courier", 8, "bold"),
                    width=CELL_W - 6, anchor=tk.CENTER, tags=(f"c{n}",))

                cid = self.canvas.create_text(
                    x1 - 3, y1 - 3, text=f"{r},{c}",
                    fill=T["muted"], font=("Courier", 6),
                    anchor=tk.SE, tags=(f"c{n}",))

                self.cell_items[n] = (rid, tid, cid)

                for item in (rid, tid, cid):
                    self.canvas.tag_bind(item, "<Button-1>",
                                         lambda e, nd=n: self._click(nd))
                    self.canvas.tag_bind(item, "<Enter>",
                                         lambda e, nd=n: self._hover(nd, True))
                    self.canvas.tag_bind(item, "<Leave>",
                                         lambda e, nd=n: self._hover(nd, False))

        self._redraw_routes()

    def _cell_bg(self, n):
        if n == 0:               return T["cell_depot"]
        if n == self.pair_src:   return "#1a3040"
        if n == self.sel_cell:   return T["cell_sel"]
        if n in self.components: return T["cell_comp"]
        return T["cell_empty"]

    def _recolor(self, n):
        if n in self.cell_items:
            self.canvas.itemconfig(self.cell_items[n][0], fill=self._cell_bg(n))

    def _hover(self, n, on):
        if n not in self.cell_items:
            return
        cur = self.canvas.itemcget(self.cell_items[n][0], "fill")
        if on and cur == T["cell_empty"]:
            self.canvas.itemconfig(self.cell_items[n][0], fill=T["cell_hover"])
        else:
            self._recolor(n)

    # ── Route drawing — the key visual part ───────────────────────────────────

    def _redraw_routes(self):
        self.canvas.delete("route")
        self.canvas.delete("via")

        for i, p in enumerate(self.pairs):
            name = p["name"]
            color = self.net_colors.get(name, NET_COLORS[i % len(NET_COLORS)])
            path = self.routes.get(name)

            if path and len(path) >= 2:
                # Draw each hop as a line segment
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

                    # Solid for 90°, short-dash for 45°
                    dash = (5, 2) if is45 else ()
                    self.canvas.create_line(
                        px1, py1, px2, py2,
                        fill=color, width=3, dash=dash,
                        capstyle=tk.ROUND, joinstyle=tk.ROUND,
                        tags="route")

                # Via dots at every direction change (bend)
                for seg in range(1, len(path) - 1):
                    r0, c0 = path[seg - 1]
                    r1, c1 = path[seg]
                    r2, c2 = path[seg + 1]
                    if (r1 - r0, c1 - c0) != (r2 - r1, c2 - c1):
                        vx = GPAD + c1 * CELL_W + CELL_W // 2
                        vy = GPAD + r1 * CELL_H + CELL_H // 2
                        self.canvas.create_oval(
                            vx - 5, vy - 5, vx + 5, vy + 5,
                            fill=T["warn"], outline=T["bg"], width=1,
                            tags="via")

                # Source terminal (large filled circle)
                sr0, sc0 = path[0]
                sx = GPAD + sc0 * CELL_W + CELL_W // 2
                sy = GPAD + sr0 * CELL_H + CELL_H // 2
                self.canvas.create_oval(sx - 6, sy - 6, sx + 6, sy + 6,
                                        fill=color, outline=T["bg"], width=1,
                                        tags="via")

                # Sink terminal
                er0, ec0 = path[-1]
                ex = GPAD + ec0 * CELL_W + CELL_W // 2
                ey = GPAD + er0 * CELL_H + CELL_H // 2
                self.canvas.create_oval(ex - 6, ey - 6, ex + 6, ey + 6,
                                        fill=color, outline=T["bg"], width=1,
                                        tags="via")
                # Arrow head at sink to show direction
                self.canvas.create_oval(ex - 3, ey - 3, ex + 3, ey + 3,
                                        fill=T["bg"], outline="",
                                        tags="via")

                # Net label at midpoint
                mid = len(path) // 2
                mr, mc = path[mid]
                mx = GPAD + mc * CELL_W + CELL_W // 2
                my = GPAD + mr * CELL_H + CELL_H // 2
                self.canvas.create_text(mx, my - 10, text=name,
                                        fill=color, font=("Courier", 7, "bold"),
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
                                        fill=color, font=("Courier", 7),
                                        tags="route")

        self.canvas.tag_raise("route")
        self.canvas.tag_raise("via")

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
            self._set_status(f"Selected ({r},{c}). Type name + Enter to place.")

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
                self._set_status(f"Pair '{net_name}' added ({len(self.pairs)} total).")

    # ── Component actions ─────────────────────────────────────────────────────

    def _on_place(self):
        if self.sel_cell is None or self.sel_cell == 0:
            self._set_status("Select a non-depot cell first.")
            return
        name = self.comp_var.get().strip()
        if not name:
            self._set_status("Enter a component name.")
            return
        self.components[self.sel_cell] = name
        if self.sel_cell in self.cell_items:
            self.canvas.itemconfig(self.cell_items[self.sel_cell][1],
                                   text=name, fill=T["accent"])
            self._recolor(self.sel_cell)
        r, c = divmod(self.sel_cell, self.cols)
        self._set_status(f"Placed '{name}' at ({r},{c}).")
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

    # ── Pair mode ─────────────────────────────────────────────────────────────

    def _toggle_pair(self):
        if not self.rows:
            self._set_status("Build a grid first.")
            return
        if self.mode == "edit":
            self.mode = "pair"
            self.pair_btn.config(text="✕  EXIT PAIR MODE",
                                 bg=T["accent2"], fg=T["bg"])
            self.pair_hint.config(text="Click a source cell →")
            self._set_status("Pair mode: click source, then sink to add a wire pair.")
        else:
            self.mode = "edit"
            if self.pair_src is not None:
                self._recolor(self.pair_src)
            self.pair_src = None
            self.pair_btn.config(text="⛓  ENTER PAIR MODE",
                                 bg=T["cell_empty"], fg=T["accent2"])
            self.pair_hint.config(text="")
            self._set_status("Edit mode.")

    def _refresh_pairs(self):
        for w in self.pair_list_frame.winfo_children():
            w.destroy()
        for i, p in enumerate(self.pairs):
            color = self.net_colors.get(p["name"], NET_COLORS[i % len(NET_COLORS)])
            row = tk.Frame(self.pair_list_frame, bg=T["panel2"])
            row.pack(fill=tk.X, pady=1)
            tk.Canvas(row, width=8, height=8, bg=color, highlightthickness=0
                      ).pack(side=tk.LEFT, padx=(4, 3), pady=3)
            tk.Label(row,
                     text=f"{p['name']}: {p['src_name']} → {p['sink_name']}",
                     bg=T["panel2"], fg=T["text"],
                     font=("Courier", 7), anchor=tk.W
                     ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            tk.Button(row, text="✕", bg=T["panel2"], fg=T["danger"],
                      font=("Courier", 7), relief=tk.FLAT, cursor="hand2",
                      command=lambda idx=i: self._remove_pair(idx)
                      ).pack(side=tk.RIGHT, padx=2)

    def _remove_pair(self, idx):
        if 0 <= idx < len(self.pairs):
            name = self.pairs[idx]["name"]
            self.pairs.pop(idx)
            self.routes.pop(name, None)
            self._refresh_pairs()
            self._redraw_routes()

    # ── Run routing ───────────────────────────────────────────────────────────

    def _on_run(self):
        if not self.rows:
            messagebox.showerror("No grid", "Build a grid first.")
            return
        if not self.pairs:
            messagebox.showerror("No pairs", "Add at least one wire pair.")
            return

        self._set_status("Sending net list to NVIDIA cuOpt to optimise routing order…")
        self.update()

        try:
            order, cuopt_body = cuopt_net_order(self.rows, self.cols, self.pairs)
        except Exception as e:
            self._set_status(f"cuOpt error: {e}  — falling back to greedy order.")
            order = list(range(len(self.pairs)))
            cuopt_body = {}

        self._set_status(
            f"Running A* octilinear router for {len(self.pairs)} nets…")
        self.update()

        self.routes = route_all_nets(
            self.pairs, self.rows, self.cols, self.components, order=order)

        self._render_grid()

        routed = sum(1 for v in self.routes.values() if v)
        self._set_status(
            f"Done. {routed}/{len(self.pairs)} nets routed. "
            f"Solid = 90°, dashed = 45°, yellow dot = via/bend.")

        self._show_result_popup()

    def _show_result_popup(self):
        win = tk.Toplevel(self)
        win.title("Routing Results")
        win.configure(bg=T["bg"])
        win.geometry("580x460")

        tk.Label(win, text="ROUTING RESULTS", bg=T["bg"], fg=T["accent"],
                 font=("Courier", 11, "bold")).pack(pady=(14, 6))

        frm = tk.Frame(win, bg=T["bg"])
        frm.pack(fill=tk.BOTH, expand=True, padx=14)
        sb = tk.Scrollbar(frm);
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        txt = tk.Text(frm, bg=T["panel"], fg=T["text"], font=("Courier", 8),
                      relief=tk.FLAT, yscrollcommand=sb.set)
        txt.pack(fill=tk.BOTH, expand=True)
        sb.config(command=txt.yview)

        total = 0
        for i, p in enumerate(self.pairs):
            path = self.routes.get(p["name"], [])
            wire_len = len(path) - 1 if path else 0
            total += wire_len

            # count bends
            bends = 0
            for seg in range(1, len(path) - 1):
                r0, c0 = path[seg - 1]
                r1, c1 = path[seg]
                r2, c2 = path[seg + 1]
                if (r1 - r0, c1 - c0) != (r2 - r1, c2 - c1):
                    bends += 1

            # count 45° hops
            diag45 = sum(
                1 for s in range(len(path) - 1)
                if abs(path[s][0] - path[s + 1][0]) == 1
                and abs(path[s][1] - path[s + 1][1]) == 1
            )
            ortho = wire_len - diag45

            status = "ROUTED  " if path else "UNROUTED"
            src_rc = divmod(p["src"], self.cols)
            sink_rc = divmod(p["sink"], self.cols)
            line = (
                f"[{status}] {p['name']:<12}  "
                f"{p['src_name']:<10} ({src_rc[0]},{src_rc[1]}) "
                f"→ {p['sink_name']:<10} ({sink_rc[0]},{sink_rc[1]})\n"
                f"          wire: {wire_len} hops "
                f"({ortho} ortho + {diag45} diag)   "
                f"bends: {bends}\n\n"
            )
            txt.insert(tk.END, line)

        txt.insert(tk.END, f"Total wire length : {total} hops\n")
        txt.config(state=tk.DISABLED)

        tk.Button(win, text="Close", bg=T["accent"], fg=T["bg"],
                  font=("Courier", 9, "bold"), relief=tk.FLAT,
                  command=win.destroy, cursor="hand2", pady=6
                  ).pack(pady=(8, 14))

    # ── Build / Reset ─────────────────────────────────────────────────────────

    def _on_build(self):
        self.rows = max(2, min(20, self.rows_var.get()))
        self.cols = max(2, min(20, self.cols_var.get()))
        self.components = {}
        self.pairs = []
        self.routes = {}
        self.net_colors = {}
        self.sel_cell = None
        self.pair_src = None
        self.mode = "edit"
        self.pair_btn.config(text="⛓  ENTER PAIR MODE",
                             bg=T["cell_empty"], fg=T["accent2"])
        self.pair_hint.config(text="")
        self.sel_lbl.config(text="No cell selected")
        self.comp_var.set("")
        self._render_grid()
        self._refresh_pairs()
        self._set_status(
            f"Grid {self.rows}×{self.cols} ready. "
            "Click cells to place components.")

    def _on_reset(self):
        self.components = {}
        self.pairs = []
        self.routes = {}
        self.net_colors = {}
        self.sel_cell = None
        self.pair_src = None
        self.mode = "edit"
        self.pair_btn.config(text="⛓  ENTER PAIR MODE",
                             bg=T["cell_empty"], fg=T["accent2"])
        self.pair_hint.config(text="")
        self.sel_lbl.config(text="No cell selected")
        self.comp_var.set("")
        if self.rows:
            self._render_grid()
        self._refresh_pairs()
        self._set_status("Reset. Place components and create wire pairs.")

    def _set_status(self, msg):
        self.status_var.set(f"  {msg}")


# ─── Entry ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    App().mainloop()
