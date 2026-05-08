#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@Author  : Mihir Mithani
@Date    : 08-05-2026 , 10:44
@File    : chip_routingv2.py
@Desc    : 
"""
"""
chip_routing_cuopt.py
─────────────────────────────────────────────────────────────────────────────
Interactive chip routing optimizer using NVIDIA cuOpt.

Features
────────
  • User enters grid dimensions (m × n) via GUI
  • Click cells to select them, then name the component placed there
  • Create connection pairs by clicking source → sink cells
  • Runs cuOpt VRP solver to find optimal routing
  • Visualises the routed result on the grid

Usage
─────
  pip install requests
  python chip_routing_cuopt.py

  Set your API key in the NVIDIA_API_KEY variable below (or via env var).
"""

import time
import tkinter as tk
from tkinter import messagebox, simpledialog

import requests

import API

# ─── API Configuration ────────────────────────────────────────────────────────

NVIDIA_API_KEY = API.API()
INVOKE_URL = "https://optimize.api.nvidia.com/v1/nvidia/cuopt"
FETCH_URL_FMT = "https://optimize.api.nvidia.com/v1/status/{}"
POLL_INTERVAL_S = 1.2
MAX_WAIT_S = 120

HEADERS = {
    "Authorization": f"Bearer {NVIDIA_API_KEY}",
    "Accept": "application/json",
    "Content-Type": "application/json",
}

# ─── Colors ───────────────────────────────────────────────────────────────────

CLR_BG = "#0f1117"
CLR_PANEL = "#1a1d27"
CLR_BORDER = "#2e3248"
CLR_ACCENT = "#5b6af0"
CLR_ACCENT2 = "#a78bfa"
CLR_TEXT = "#e8eaf6"
CLR_MUTED = "#6b7280"
CLR_CELL_EMPTY = "#1e2133"
CLR_CELL_COMP = "#1e2d50"
CLR_CELL_DEPOT = "#2d1e50"
CLR_CELL_SELA = "#1e3b2a"  # source (green tint)
CLR_CELL_SELB = "#3b2a1e"  # sink (amber tint)
CLR_CELL_ROUTED = "#1a3320"
CLR_CELL_HOVER = "#252840"
CLR_SUCCESS = "#34d399"
CLR_WARNING = "#fbbf24"
CLR_DANGER = "#f87171"
CLR_NET_COLORS = [
    "#5b6af0", "#a78bfa", "#34d399", "#fbbf24", "#f87171",
    "#38bdf8", "#fb7185", "#4ade80", "#facc15", "#818cf8",
]

CELL_W = 90
CELL_H = 60
PAD = 12


# ─── Matrix Builders ──────────────────────────────────────────────────────────

def node_to_rc(n, cols):
    return divmod(n, cols)


def manhattan(a, b, cols):
    r1, c1 = node_to_rc(a, cols)
    r2, c2 = node_to_rc(b, cols)
    return abs(r1 - r2) + abs(c1 - c2)


def build_cost_matrix(rows, cols, layer_id):
    n = rows * cols
    mat = []
    for a in range(n):
        row = []
        ra, ca = node_to_rc(a, cols)
        for b in range(n):
            if a == b:
                row.append(0)
                continue
            rb, cb = node_to_rc(b, cols)
            hd = abs(ca - cb)
            vd = abs(ra - rb)
            penalty = vd if layer_id == 1 else hd
            row.append(max(1, hd + vd + penalty))
        mat.append(row)
    return mat


def build_delay_matrix(rows, cols):
    n = rows * cols
    mat = []
    for a in range(n):
        row = []
        for b in range(n):
            if a == b:
                row.append(0)
            else:
                row.append(max(1, manhattan(a, b, cols)))
        mat.append(row)
    return mat


def build_payload(rows, cols, pairs):
    n_nets = len(pairs)
    max_t = rows * cols + 4
    cap = n_nets + 4

    cost_m1 = build_cost_matrix(rows, cols, 1)
    cost_m2 = build_cost_matrix(rows, cols, 2)
    delay = build_delay_matrix(rows, cols)

    return {
        "action": "cuOpt_OptimizedRouting",
        "data": {
            "cost_matrix_data": {"data": {"1": cost_m1, "2": cost_m2}},
            "travel_time_matrix_data": {"data": {"1": delay, "2": delay}},
            "fleet_data": {
                "vehicle_locations": [[0, 0], [0, 0]],
                "vehicle_ids": ["M1_router", "M2_router"],
                "capacities": [[cap, cap], [cap, cap]],
                "vehicle_time_windows": [[0, max_t], [0, max_t]],
                "vehicle_types": [1, 2],
                "vehicle_max_costs": [rows * cols * 6, rows * cols * 6],
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
        "client_version": "chip_router_interactive_v2",
    }


def call_cuopt(payload):
    session = requests.Session()
    response = session.post(INVOKE_URL, headers=HEADERS, json=payload, timeout=30)
    elapsed = 0
    while response.status_code == 202:
        req_id = response.headers.get("NVCF-REQID", "")
        fetch_url = FETCH_URL_FMT.format(req_id)
        time.sleep(POLL_INTERVAL_S)
        elapsed += POLL_INTERVAL_S
        if elapsed > MAX_WAIT_S:
            raise TimeoutError("cuOpt request timed out")
        response = session.get(fetch_url, headers=HEADERS, timeout=30)
    response.raise_for_status()
    return response.json()


# ─── Main Application ─────────────────────────────────────────────────────────

class ChipRoutingApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Chip Routing Optimizer · cuOpt")
        self.configure(bg=CLR_BG)
        self.resizable(True, True)
        self.geometry("1100x750")

        # State
        self.rows = 0
        self.cols = 0
        self.components = {}  # node_idx → component name
        self.pairs = []  # list of dicts {name, src, sink, src_name, sink_name}
        self.sel_cell = None  # currently selected cell (editing mode)
        self.pair_src = None  # first cell picked in pair mode
        self.mode = "edit"  # "edit" | "pair" | "result"
        self.cell_btns = {}  # node_idx → canvas item ids
        self.net_colors = {}  # pair name → color

        self._build_ui()

    # ── UI Construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # Left panel
        self.left = tk.Frame(self, bg=CLR_PANEL, width=260)
        self.left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 0))
        self.left.pack_propagate(False)
        self._build_left_panel()

        # Right canvas area
        self.right = tk.Frame(self, bg=CLR_BG)
        self.right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Canvas with scrollbars
        self.canvas_frame = tk.Frame(self.right, bg=CLR_BG)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True, padx=PAD, pady=PAD)

        self.hscroll = tk.Scrollbar(self.canvas_frame, orient=tk.HORIZONTAL)
        self.vscroll = tk.Scrollbar(self.canvas_frame, orient=tk.VERTICAL)
        self.canvas = tk.Canvas(
            self.canvas_frame,
            bg=CLR_BG,
            highlightthickness=0,
            xscrollcommand=self.hscroll.set,
            yscrollcommand=self.vscroll.set,
        )
        self.hscroll.config(command=self.canvas.xview)
        self.vscroll.config(command=self.canvas.yview)
        self.hscroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Status bar
        self.status_var = tk.StringVar(value="Enter grid dimensions and click Build.")
        self.status_bar = tk.Label(
            self.right, textvariable=self.status_var,
            bg=CLR_PANEL, fg=CLR_MUTED, font=("Consolas", 10),
            anchor=tk.W, padx=12, pady=6,
        )
        self.status_bar.pack(fill=tk.X, side=tk.BOTTOM)

    def _build_left_panel(self):
        lp = self.left
        tk.Label(lp, text="CHIP ROUTER", bg=CLR_PANEL, fg=CLR_ACCENT,
                 font=("Consolas", 13, "bold")).pack(pady=(18, 2))
        tk.Label(lp, text="cuOpt VRP Engine", bg=CLR_PANEL, fg=CLR_MUTED,
                 font=("Consolas", 9)).pack(pady=(0, 16))

        self._sep(lp)

        # ── Step 1: Grid dims ─────────────────────────────────────────────
        tk.Label(lp, text="① GRID DIMENSIONS", bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Consolas", 10, "bold")).pack(anchor=tk.W, padx=14, pady=(10, 4))

        df = tk.Frame(lp, bg=CLR_PANEL)
        df.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(df, text="Rows (m)", bg=CLR_PANEL, fg=CLR_MUTED,
                 font=("Consolas", 9)).grid(row=0, column=0, sticky=tk.W)
        self.rows_var = tk.IntVar(value=4)
        tk.Spinbox(df, from_=2, to=12, textvariable=self.rows_var, width=5,
                   bg=CLR_CELL_EMPTY, fg=CLR_TEXT, insertbackground=CLR_TEXT,
                   buttonbackground=CLR_BORDER, relief=tk.FLAT).grid(row=0, column=1, padx=6)
        tk.Label(df, text="Cols (n)", bg=CLR_PANEL, fg=CLR_MUTED,
                 font=("Consolas", 9)).grid(row=1, column=0, sticky=tk.W, pady=(4, 0))
        self.cols_var = tk.IntVar(value=5)
        tk.Spinbox(df, from_=2, to=12, textvariable=self.cols_var, width=5,
                   bg=CLR_CELL_EMPTY, fg=CLR_TEXT, insertbackground=CLR_TEXT,
                   buttonbackground=CLR_BORDER, relief=tk.FLAT).grid(row=1, column=1, padx=6, pady=(4, 0))

        self._btn(lp, "▶  BUILD GRID", self._on_build, CLR_ACCENT)

        self._sep(lp)

        # ── Step 2: Component placement ───────────────────────────────────
        tk.Label(lp, text="② PLACE COMPONENTS", bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Consolas", 10, "bold")).pack(anchor=tk.W, padx=14, pady=(10, 4))
        tk.Label(lp, text="Click any cell to select it,\nthen name the component below.",
                 bg=CLR_PANEL, fg=CLR_MUTED, font=("Consolas", 8),
                 justify=tk.LEFT).pack(anchor=tk.W, padx=14)

        cf = tk.Frame(lp, bg=CLR_PANEL)
        cf.pack(fill=tk.X, padx=14, pady=6)
        tk.Label(cf, text="Name", bg=CLR_PANEL, fg=CLR_MUTED,
                 font=("Consolas", 9)).pack(side=tk.LEFT)
        self.comp_var = tk.StringVar()
        self.comp_entry = tk.Entry(cf, textvariable=self.comp_var, width=14,
                                   bg=CLR_CELL_EMPTY, fg=CLR_TEXT, relief=tk.FLAT,
                                   insertbackground=CLR_TEXT, font=("Consolas", 10))
        self.comp_entry.pack(side=tk.LEFT, padx=(6, 0))
        self.comp_entry.bind("<Return>", lambda _: self._on_place_comp())

        bf = tk.Frame(lp, bg=CLR_PANEL)
        bf.pack(fill=tk.X, padx=14)
        self._btn_small(bf, "Place", self._on_place_comp, CLR_SUCCESS, side=tk.LEFT)
        self._btn_small(bf, "Clear", self._on_clear_comp, CLR_DANGER, side=tk.LEFT)

        self.sel_label = tk.Label(lp, text="No cell selected", bg=CLR_PANEL,
                                  fg=CLR_MUTED, font=("Consolas", 8))
        self.sel_label.pack(anchor=tk.W, padx=14, pady=(4, 0))

        self._sep(lp)

        # ── Step 3: Wire pairs ────────────────────────────────────────────
        tk.Label(lp, text="③ WIRE PAIRS", bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Consolas", 10, "bold")).pack(anchor=tk.W, padx=14, pady=(10, 4))
        tk.Label(lp, text="Click to toggle pair mode.",
                 bg=CLR_PANEL, fg=CLR_MUTED, font=("Consolas", 8),
                 justify=tk.LEFT).pack(anchor=tk.W, padx=14)

        self.pair_mode_btn = tk.Button(
            lp, text="⛓  ENTER PAIR MODE", font=("Consolas", 9, "bold"),
            bg=CLR_CELL_EMPTY, fg=CLR_ACCENT2, relief=tk.FLAT,
            activebackground=CLR_BORDER, activeforeground=CLR_ACCENT2,
            command=self._toggle_pair_mode, cursor="hand2",
        )
        self.pair_mode_btn.pack(fill=tk.X, padx=14, pady=(6, 4))

        self.pair_status_lbl = tk.Label(lp, text="", bg=CLR_PANEL,
                                        fg=CLR_WARNING, font=("Consolas", 8))
        self.pair_status_lbl.pack(anchor=tk.W, padx=14)

        # Pair list
        self.pair_list_frame = tk.Frame(lp, bg=CLR_PANEL)
        self.pair_list_frame.pack(fill=tk.X, padx=14, pady=4)

        self._sep(lp)

        # ── Step 4: Route ─────────────────────────────────────────────────
        tk.Label(lp, text="④ ROUTE", bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Consolas", 10, "bold")).pack(anchor=tk.W, padx=14, pady=(10, 4))
        self._btn(lp, "⚡  RUN CUOPT", self._on_run, CLR_ACCENT2)
        self._btn(lp, "↺  RESET ALL", self._on_reset, CLR_MUTED)

    def _sep(self, parent):
        tk.Frame(parent, bg=CLR_BORDER, height=1).pack(fill=tk.X, padx=0, pady=4)

    def _btn(self, parent, text, cmd, color):
        tk.Button(
            parent, text=text, font=("Consolas", 9, "bold"),
            bg=color, fg=CLR_BG, relief=tk.FLAT,
            activebackground=color, activeforeground=CLR_BG,
            command=cmd, cursor="hand2", padx=6, pady=6,
        ).pack(fill=tk.X, padx=14, pady=4)

    def _btn_small(self, parent, text, cmd, color, side=tk.LEFT):
        tk.Button(
            parent, text=text, font=("Consolas", 8),
            bg=color, fg=CLR_BG, relief=tk.FLAT,
            activebackground=color, activeforeground=CLR_BG,
            command=cmd, cursor="hand2", padx=6, pady=3,
        ).pack(side=side, padx=(0, 4))

    # ── Grid Drawing ──────────────────────────────────────────────────────────

    def _on_build(self):
        self.rows = self.rows_var.get()
        self.cols = self.cols_var.get()
        self.components = {}
        self.pairs = []
        self.sel_cell = None
        self.pair_src = None
        self.mode = "edit"
        self.net_colors = {}
        self.pair_mode_btn.config(text="⛓  ENTER PAIR MODE", bg=CLR_CELL_EMPTY, fg=CLR_ACCENT2)
        self._render_grid()
        self._refresh_pair_list()
        self._set_status(f"Grid {self.rows}×{self.cols} built. Click cells to name components.")

    def _render_grid(self, routed_nodes=None, route_assignments=None):
        self.canvas.delete("all")
        self.cell_items = {}  # node → (rect_id, text_id, sub_id)
        routed_nodes = routed_nodes or {}
        route_assignments = route_assignments or {}

        total_w = self.cols * CELL_W + PAD * 2
        total_h = self.rows * CELL_H + PAD * 2
        self.canvas.config(scrollregion=(0, 0, total_w, total_h))

        for r in range(self.rows):
            for c in range(self.cols):
                n = r * self.cols + c
                x0 = PAD + c * CELL_W
                y0 = PAD + r * CELL_H
                x1 = x0 + CELL_W - 2
                y1 = y0 + CELL_H - 2
                cx = (x0 + x1) / 2
                cy = (y0 + y1) / 2

                color = self._cell_color(n, routed_nodes)
                rid = self.canvas.create_rectangle(
                    x0, y0, x1, y1, fill=color, outline=CLR_BORDER,
                    width=1, tags=(f"cell_{n}", "cell"),
                )
                # node index label (bottom-right)
                self.canvas.create_text(
                    x1 - 3, y1 - 2, text=f"{r},{c}",
                    fill=CLR_MUTED, font=("Consolas", 7), anchor=tk.SE,
                    tags=(f"cell_{n}",),
                )
                # component name
                label = "DEPOT" if n == 0 else self.components.get(n, "")
                lcolor = CLR_ACCENT2 if n == 0 else CLR_TEXT
                tid = self.canvas.create_text(
                    cx, cy - 4, text=label,
                    fill=lcolor, font=("Consolas", 9, "bold"),
                    width=CELL_W - 8, anchor=tk.CENTER,
                    tags=(f"cell_{n}",),
                )
                # net assignment (result mode)
                sub = ""
                if n in route_assignments:
                    sub = route_assignments[n]
                sid = self.canvas.create_text(
                    cx, y1 - 10, text=sub,
                    fill=CLR_SUCCESS, font=("Consolas", 7),
                    width=CELL_W - 6, anchor=tk.CENTER,
                    tags=(f"cell_{n}",),
                )
                self.cell_items[n] = (rid, tid, sid)

                # bind clicks
                for tag_id in (rid, tid, sid):
                    self.canvas.tag_bind(tag_id, "<Button-1>",
                                         lambda e, nd=n: self._on_cell_click(nd))
                    self.canvas.tag_bind(tag_id, "<Enter>",
                                         lambda e, nd=n: self._on_cell_hover(nd, True))
                    self.canvas.tag_bind(tag_id, "<Leave>",
                                         lambda e, nd=n: self._on_cell_hover(nd, False))

        # Draw pair source lines (in pair mode, show pairs as arrows)
        if self.mode in ("pair", "edit"):
            self._draw_pair_arrows()

    def _draw_pair_arrows(self):
        self.canvas.delete("arrow")
        for i, p in enumerate(self.pairs):
            color = self.net_colors.get(p["name"], CLR_NET_COLORS[i % len(CLR_NET_COLORS)])
            sr, sc = node_to_rc(p["src"], self.cols)
            dr, dc = node_to_rc(p["sink"], self.cols)
            sx = PAD + sc * CELL_W + CELL_W // 2
            sy = PAD + sr * CELL_H + CELL_H // 2
            dx = PAD + dc * CELL_W + CELL_W // 2
            dy = PAD + dr * CELL_H + CELL_H // 2
            self.canvas.create_line(
                sx, sy, dx, dy, fill=color, width=2,
                arrow=tk.LAST, arrowshape=(8, 10, 4),
                dash=(4, 3), tags="arrow",
            )
            mx, my = (sx + dx) / 2, (sy + dy) / 2
            self.canvas.create_text(
                mx, my - 8, text=p["name"],
                fill=color, font=("Consolas", 7, "bold"),
                tags="arrow",
            )

    def _cell_color(self, n, routed_nodes):
        if n == 0:
            return CLR_CELL_DEPOT
        if n in routed_nodes:
            return CLR_CELL_ROUTED
        if n == self.pair_src:
            return CLR_CELL_SELB
        if n == self.sel_cell:
            return CLR_CELL_SELB
        if n in self.components:
            return CLR_CELL_COMP
        return CLR_CELL_EMPTY

    def _recolor_cell(self, n, color=None):
        if n not in self.cell_items:
            return
        rid = self.cell_items[n][0]
        c = color or self._cell_color(n, {})
        self.canvas.itemconfig(rid, fill=c)

    def _on_cell_hover(self, n, entering):
        if n not in self.cell_items:
            return
        if entering:
            cur = self.canvas.itemcget(self.cell_items[n][0], "fill")
            if cur == CLR_CELL_EMPTY:
                self.canvas.itemconfig(self.cell_items[n][0], fill=CLR_CELL_HOVER)
        else:
            self._recolor_cell(n)

    # ── Cell Click Logic ──────────────────────────────────────────────────────

    def _on_cell_click(self, n):
        if self.mode == "edit":
            # deselect previous
            if self.sel_cell is not None:
                self._recolor_cell(self.sel_cell)
            self.sel_cell = n
            self._recolor_cell(n, CLR_CELL_SELB)
            r, c = node_to_rc(n, self.cols)
            name = self.components.get(n, "")
            self.comp_var.set(name)
            self.comp_entry.focus_set()
            if n == 0:
                self.sel_label.config(text=f"Cell ({r},{c}) — depot/origin")
            else:
                self.sel_label.config(
                    text=f"Cell ({r},{c}){' · ' + name if name else ' — unnamed'}"
                )
            self._set_status(f"Selected ({r},{c}). Type a name and press Enter or Place.")

        elif self.mode == "pair":
            if self.pair_src is None:
                # pick source
                self.pair_src = n
                self._recolor_cell(n, CLR_CELL_SELB)
                r, c = node_to_rc(n, self.cols)
                name = self.components.get(n, "DEPOT" if n == 0 else f"node{n}")
                self.pair_status_lbl.config(
                    text=f"Source: {name} ({r},{c}). Now pick sink →"
                )
            else:
                if n == self.pair_src:
                    self._recolor_cell(n)
                    self.pair_src = None
                    self.pair_status_lbl.config(text="Source cleared. Pick again.")
                    return
                # ask for net name
                default_name = f"NET{len(self.pairs)}"
                net_name = simpledialog.askstring(
                    "Net name",
                    f"Name for this connection\n(src→sink):",
                    initialvalue=default_name,
                    parent=self,
                )
                if not net_name:
                    net_name = default_name
                net_name = net_name.strip().upper().replace(" ", "_")

                src_name = self.components.get(self.pair_src, "DEPOT" if self.pair_src == 0 else f"node{self.pair_src}")
                sink_name = self.components.get(n, "DEPOT" if n == 0 else f"node{n}")

                ci = len(self.pairs) % len(CLR_NET_COLORS)
                self.net_colors[net_name] = CLR_NET_COLORS[ci]

                self.pairs.append({
                    "name": net_name,
                    "src": self.pair_src,
                    "sink": n,
                    "src_name": src_name,
                    "sink_name": sink_name,
                })

                self._recolor_cell(self.pair_src)
                self.pair_src = None
                self.pair_status_lbl.config(text=f"Pair '{net_name}' added. Pick next source →")
                self._refresh_pair_list()
                self._draw_pair_arrows()
                self._set_status(f"Pair '{net_name}' added. {len(self.pairs)} pair(s) total.")

    # ── Component Editing ─────────────────────────────────────────────────────

    def _on_place_comp(self):
        if self.sel_cell is None or self.sel_cell == 0:
            self._set_status("Select a non-depot cell first.")
            return
        name = self.comp_var.get().strip()
        if not name:
            self._set_status("Enter a component name first.")
            return
        self.components[self.sel_cell] = name
        # update canvas text
        if self.sel_cell in self.cell_items:
            self.canvas.itemconfig(self.cell_items[self.sel_cell][1], text=name)
            self._recolor_cell(self.sel_cell, CLR_CELL_COMP)
        r, c = node_to_rc(self.sel_cell, self.cols)
        self._set_status(f"Placed '{name}' at ({r},{c}).")
        # also refresh any pairs that reference this node
        for p in self.pairs:
            if p["src"] == self.sel_cell:
                p["src_name"] = name
            if p["sink"] == self.sel_cell:
                p["sink_name"] = name
        self._refresh_pair_list()

    def _on_clear_comp(self):
        if self.sel_cell is None or self.sel_cell == 0:
            return
        self.components.pop(self.sel_cell, None)
        if self.sel_cell in self.cell_items:
            self.canvas.itemconfig(self.cell_items[self.sel_cell][1], text="")
            self._recolor_cell(self.sel_cell, CLR_CELL_SELB)
        self.comp_var.set("")
        self._set_status("Component cleared.")

    # ── Pair Mode ─────────────────────────────────────────────────────────────

    def _toggle_pair_mode(self):
        if self.rows == 0:
            self._set_status("Build a grid first.")
            return
        if self.mode == "edit":
            self.mode = "pair"
            self.pair_mode_btn.config(
                text="✕  EXIT PAIR MODE", bg=CLR_ACCENT2, fg=CLR_BG
            )
            self.pair_status_lbl.config(text="Click a source cell →")
            self._set_status("Pair mode: click source, then sink to create a connection.")
        else:
            self.mode = "edit"
            self.pair_src = None
            self.pair_mode_btn.config(
                text="⛓  ENTER PAIR MODE", bg=CLR_CELL_EMPTY, fg=CLR_ACCENT2
            )
            self.pair_status_lbl.config(text="")
            self._render_grid()
            self._set_status("Back to edit mode.")

    def _refresh_pair_list(self):
        for w in self.pair_list_frame.winfo_children():
            w.destroy()
        for i, p in enumerate(self.pairs):
            color = self.net_colors.get(p["name"], CLR_ACCENT)
            row = tk.Frame(self.pair_list_frame, bg=CLR_CELL_EMPTY)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text="●", bg=CLR_CELL_EMPTY, fg=color,
                     font=("Consolas", 9)).pack(side=tk.LEFT, padx=(4, 2))
            tk.Label(row, text=f"{p['name']}: {p['src_name']}→{p['sink_name']}",
                     bg=CLR_CELL_EMPTY, fg=CLR_TEXT,
                     font=("Consolas", 8), anchor=tk.W).pack(side=tk.LEFT, fill=tk.X, expand=True)
            tk.Button(row, text="✕", bg=CLR_CELL_EMPTY, fg=CLR_DANGER,
                      font=("Consolas", 8), relief=tk.FLAT, cursor="hand2",
                      command=lambda idx=i: self._remove_pair(idx)).pack(side=tk.RIGHT, padx=2)

    def _remove_pair(self, idx):
        if 0 <= idx < len(self.pairs):
            self.pairs.pop(idx)
            self._refresh_pair_list()
            self._draw_pair_arrows()

    # ── Run cuOpt ─────────────────────────────────────────────────────────────

    def _on_run(self):
        if self.rows == 0:
            messagebox.showerror("No grid", "Build a grid first.")
            return
        if not self.pairs:
            messagebox.showerror("No pairs", "Create at least one wire pair first.")
            return

        self._set_status("⏳ Building matrices and submitting to cuOpt…")
        self.update()

        try:
            payload = build_payload(self.rows, self.cols, self.pairs)
            body = call_cuopt(payload)
            self._show_results(body)
        except Exception as e:
            self._set_status(f"ERROR: {e}")
            messagebox.showerror("cuOpt error", str(e))

    def _show_results(self, body):
        routes = body.get("response", {}).get("solver_response", {})
        vehicle_data = routes.get("vehicle_data", {})
        obj = routes.get("solution_cost", routes.get("total_objective", "—"))

        routed_nodes = {}  # node → list of net names
        route_assignment = {}  # node → short label string

        result_lines = [f"Solver objective: {obj}\n"]

        for vid, data in vehicle_data.items():
            task_seq = data.get("task_id", [])
            route_nodes = data.get("route", [])
            arrivals = data.get("arrival_stamp", [])

            stops = []
            for i, t in enumerate(task_seq):
                if str(t) == "Depot":
                    continue
                node = route_nodes[i] if i < len(route_nodes) else None
                arr = arrivals[i] if i < len(arrivals) else "?"
                stops.append((str(t), int(node) if node is not None else None, arr))
                if node is not None:
                    n = int(node)
                    routed_nodes.setdefault(n, []).append(str(t))

            layer = "M1" if "M1" in str(vid) else "M2"
            result_lines.append(f"── {vid} [{layer}]  ({len(stops)} nets)")
            for net_name, node, arr in stops:
                pair = next((p for p in self.pairs if p["name"] == net_name), None)
                src = pair["src"] if pair else 0
                r1, c1 = node_to_rc(src, self.cols)
                r2, c2 = node_to_rc(node, self.cols) if node is not None else ("?", "?")
                slack = "?"
                if pair:
                    lat = pair.get("late", self.rows * self.cols + 4)
                    slack = f"{lat - float(arr):+.1f}" if arr != "?" else "?"
                result_lines.append(
                    f"  {net_name:<12} ({r1},{c1})→({r2},{c2})  "
                    f"arr={arr if arr != '?' else '?':<5}  slack={slack}"
                )
            result_lines.append("")

        # Build route_assignment label per node
        for node, nets in routed_nodes.items():
            route_assignment[node] = ",".join(nets)

        self.mode = "result"
        self._render_grid(routed_nodes=set(routed_nodes.keys()),
                          route_assignments=route_assignment)

        # Show result window
        self._show_result_window("\n".join(result_lines))
        self._set_status(f"Routing complete. Objective={obj}. Routed {len(self.pairs)} net(s).")

    def _show_result_window(self, text):
        win = tk.Toplevel(self)
        win.title("Routing Results")
        win.configure(bg=CLR_BG)
        win.geometry("540x420")

        tk.Label(win, text="ROUTING RESULTS", bg=CLR_BG, fg=CLR_ACCENT,
                 font=("Consolas", 12, "bold")).pack(pady=(14, 4))

        frame = tk.Frame(win, bg=CLR_BG)
        frame.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 14))

        sb = tk.Scrollbar(frame)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        txt = tk.Text(frame, bg=CLR_PANEL, fg=CLR_TEXT, font=("Consolas", 9),
                      relief=tk.FLAT, yscrollcommand=sb.set, wrap=tk.NONE)
        txt.pack(fill=tk.BOTH, expand=True)
        sb.config(command=txt.yview)
        txt.insert(tk.END, text)
        txt.config(state=tk.DISABLED)

        tk.Button(win, text="Close", bg=CLR_ACCENT, fg=CLR_BG,
                  font=("Consolas", 9, "bold"), relief=tk.FLAT,
                  command=win.destroy, cursor="hand2").pack(pady=(0, 12))

    # ── Reset ─────────────────────────────────────────────────────────────────

    def _on_reset(self):
        self.components = {}
        self.pairs = []
        self.sel_cell = None
        self.pair_src = None
        self.mode = "edit"
        self.net_colors = {}
        self.pair_mode_btn.config(text="⛓  ENTER PAIR MODE", bg=CLR_CELL_EMPTY, fg=CLR_ACCENT2)
        self.pair_status_lbl.config(text="")
        self.sel_label.config(text="No cell selected")
        self.comp_var.set("")
        if self.rows:
            self._render_grid()
        self._refresh_pair_list()
        self._set_status("Reset. Place components and re-wire pairs.")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_status(self, msg):
        self.status_var.set(f"  {msg}")


# ─── Entry Point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = ChipRoutingApp()
    app.mainloop()
