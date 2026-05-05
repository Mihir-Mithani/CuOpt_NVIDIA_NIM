"""
chip_routing_cuopt.py
─────────────────────────────────────────────────────────────────────────────
Real-world chip routing optimizer using NVIDIA cuOpt.

Context
───────
In chip/PCB routing, we model the problem as a Vehicle Routing Problem (VRP):
  • "Vehicles"  → routing agents (one per metal layer: M1, M2, …)
  • "Tasks"     → signal nets / vias that must be connected
  • "Locations" → grid nodes on the routing fabric
  • "Cost"      → wire length + layer-change penalty + congestion weight
  • "Capacity"  → track utilization budget per routing channel

Layers
  Layer 1 (M1) – horizontal preferred  (lower resistance for power rails)
  Layer 2 (M2) – vertical preferred    (signal routing)

The cost matrix encodes Manhattan distance + congestion surcharge between
every pair of grid nodes.  Travel-time matrix models signal delay (RC).

Usage
─────
  pip install requests
  python chip_routing_cuopt.py

Output
──────
  Prints the optimised route sequence per layer-agent with estimated
  wire-length and timing slack.
"""

import json
import sys
import time

import requests

import API

# ─── Configuration ───────────────────────────────────────────────────────────

API_KEY = API.API()
#print(API_KEY)
INVOKE_URL      = "https://optimize.api.nvidia.com/v1/nvidia/cuopt"
FETCH_URL_FMT   = "https://optimize.api.nvidia.com/v1/status/{}"
POLL_INTERVAL_S = 1.0   # seconds between status polls
MAX_WAIT_S      = 120   # give up after this many seconds

HEADERS = {
    "Authorization": f"Bearer {API_KEY}",
    "Accept":        "application/json",
    "Content-Type":  "application/json",
}

# ─── Chip / Grid Definition ──────────────────────────────────────────────────

# 5×5 routing grid  →  25 nodes, indexed row-major: node = row*5 + col
GRID_ROWS = 5
GRID_COLS = 5
N_NODES   = GRID_ROWS * GRID_COLS          # 25

# Metal layers modelled as separate "vehicle types" in cuOpt
LAYERS = {
    "M1": 1,   # horizontal preferred – lower unit cost on H-edges
    "M2": 2,   # vertical preferred   – lower unit cost on V-edges
}

# Congestion map: (node_index) → surcharge added to every edge touching it.
# Represents hot-spots from prior global routing passes.
CONGESTION = {
    6:  2,   # centre-left cluster
    7:  3,
    12: 4,   # middle – heavily congested
    13: 3,
    18: 2,
}

# Signal nets to route: each net needs a connection visit at its sink node.
# Format: (net_id, source_node, sink_node, demand_M1, demand_M2, earliest, latest)
NETS = [
    # net_id   src  sink  d_M1 d_M2  tw_early  tw_late
    ("CLK",     0,   24,   1,   1,      0,        8),
    ("VDD",     0,   20,   2,   1,      0,        9),
    ("DATA_A",  4,   16,   1,   1,      1,        7),
    ("DATA_B",  4,   21,   1,   1,      1,        8),
    ("RESET",   0,   14,   1,   1,      0,        6),
    ("OUT_X",  20,   24,   1,   1,      2,        9),
    ("OUT_Y",   5,   23,   1,   1,      2,        9),
]

# Router agents: one per layer.  They start / end at node 0 (top-left pad ring).
ROUTERS = [
    # router_id   layer   cap_M1  cap_M2  time_window  max_cost  max_time
    ("M1_router", "M1",    6,      4,     (0, 10),      30,       12),
    ("M2_router", "M2",    4,      6,     (0, 10),      30,       12),
]

# ─── Cost / Delay Matrix Builder ─────────────────────────────────────────────

def node_to_rc(n: int):
    return divmod(n, GRID_COLS)   # (row, col)

def manhattan(a: int, b: int) -> int:
    r1, c1 = node_to_rc(a)
    r2, c2 = node_to_rc(b)
    return abs(r1 - r2) + abs(c1 - c2)

def edge_congestion(a: int, b: int) -> int:
    """Sum congestion surcharges for endpoints."""
    return CONGESTION.get(a, 0) + CONGESTION.get(b, 0)

def build_cost_matrix(layer_id: int) -> list[list[int]]:
    """
    Cost = Manhattan distance + congestion penalty.
    M1 (layer 1) pays extra for vertical moves (prefers horizontal).
    M2 (layer 2) pays extra for horizontal moves (prefers vertical).
    """
    n = N_NODES
    mat = [[0] * n for _ in range(n)]
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            ra, ca = node_to_rc(a)
            rb, cb = node_to_rc(b)
            h_dist = abs(ca - cb)
            v_dist = abs(ra - rb)
            if layer_id == 1:                # M1 horizontal preferred
                layer_penalty = v_dist       # penalise vertical hops
            else:                            # M2 vertical preferred
                layer_penalty = h_dist       # penalise horizontal hops
            base = h_dist + v_dist + layer_penalty
            cong = edge_congestion(a, b)
            mat[a][b] = max(1, base + cong)
    return mat

def build_delay_matrix() -> list[list[int]]:
    """
    Signal delay ∝ wire length (RC).  Congestion adds extra delay (buffering).
    Layer-independent for timing analysis.
    """
    n = N_NODES
    mat = [[0] * n for _ in range(n)]
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            dist = manhattan(a, b)
            cong = edge_congestion(a, b) // 2   # partial delay impact
            mat[a][b] = max(1, dist + cong)
    return mat

# ─── cuOpt Payload Builder ────────────────────────────────────────────────────

def build_payload() -> dict:
    cost_M1 = build_cost_matrix(layer_id=1)
    cost_M2 = build_cost_matrix(layer_id=2)
    delay    = build_delay_matrix()

    # cuOpt expects matrices keyed by vehicle-type id (as string)
    cost_matrix_data = {
        "1": cost_M1,   # M1_router
        "2": cost_M2,   # M2_router
    }
    delay_matrix_data = {
        "1": delay,
        "2": delay,
    }

    n_routers = len(ROUTERS)
    n_nets    = len(NETS)

    # Fleet (routing agents / layers)
    # cuOpt capacities shape: [n_capacity_dims][n_vehicles]
    vehicle_locations  = [[0, 0]] * n_routers          # start + end at node 0
    vehicle_ids        = [r[0] for r in ROUTERS]
    capacities         = [
        [r[2] for r in ROUTERS],   # dim-0: M1 track budget per router
        [r[3] for r in ROUTERS],   # dim-1: M2 track budget per router
    ]
    vehicle_time_wins  = [list(r[4]) for r in ROUTERS]
    vehicle_max_costs  = [r[5] for r in ROUTERS]
    vehicle_max_times  = [r[6] for r in ROUTERS]
    vehicle_types      = [LAYERS[r[1]] for r in ROUTERS]

    # Tasks (nets to route)
    task_locations = [n[2] for n in NETS]          # sink nodes
    task_ids       = [n[0] for n in NETS]
    # cuOpt demand shape: [n_capacity_dims][n_tasks]
    demand         = [
        [net[3] for net in NETS],  # dim-0: M1 track demand per net  (index 3)
        [net[4] for net in NETS],  # dim-1: M2 track demand per net  (index 4)
    ]
    task_tw        = [[n[5], n[6]] for n in NETS]
    service_times  = [0] * n_nets  # routing is instantaneous post-opt

    payload = {
        "action": "cuOpt_OptimizedRouting",
        "data": {
            "cost_matrix_data":         {"data": cost_matrix_data},
            "travel_time_matrix_data":  {"data": delay_matrix_data},
            "fleet_data": {
                "vehicle_locations":    vehicle_locations,
                "vehicle_ids":          vehicle_ids,
                "capacities":           capacities,
                "vehicle_time_windows": vehicle_time_wins,
                "vehicle_types":        vehicle_types,
                "vehicle_max_costs":    vehicle_max_costs,
                "vehicle_max_times":    vehicle_max_times,
                "skip_first_trips":  [False] * n_routers,
                "drop_return_trips": [True]  * n_routers,  # no need to return to origin
                "min_vehicles":      1,
            },
            "task_data": {
                "task_locations":    task_locations,
                "task_ids":          task_ids,
                "demand":            demand,
                "task_time_windows": task_tw,
                "service_times":     service_times,
            },
            "solver_config": {
                "time_limit": 5,        # seconds – increase for larger chips
                "objectives": {
                    "cost":                    2,   # minimise wire length + congestion
                    "travel_time":             1,   # minimise signal delay
                    "variance_route_size":     1,   # balance load across layers
                    "variance_route_service_time": 0,
                    "prize":                   0,
                },
                "verbose_mode":  False,
                "error_logging": True,
            },
        },
        "client_version": "chip_router_v1.0",
    }
    return payload

# ─── Result Interpreter ───────────────────────────────────────────────────────

def interpret_results(response_body: dict):
    """Pretty-print the optimised routing schedule."""
    print("\n" + "═" * 60)
    print("  CHIP ROUTING OPTIMISATION RESULTS")
    print("═" * 60)

    if "error" in response_body:
        print(f"  ✗ Solver error: {response_body['error']}")
        return

    # Dump raw response if --debug flag passed
    if "--debug" in sys.argv:
        print(json.dumps(response_body, indent=2))

    routes = response_body.get("response", {}).get("solver_response", {})
    if not routes:
        print("  No routes returned.")
        print(json.dumps(response_body, indent=2))
        return

    vehicle_data   = routes.get("vehicle_data", {})
    # solution_cost holds the actual total objective value
    solution_cost  = routes.get("solution_cost", routes.get("total_objective", 0))
    router_by_name = {r[0]: r for r in ROUTERS}

    # Build task lookup: task string id → net name
    # cuOpt uses the task_ids we supplied, but also inserts "Depot" entries
    task_name_map  = {n[0]: n[0] for n in NETS}   # "CLK" -> "CLK", etc.

    # Compute real wire costs from the cost matrices we built
    cost_M1 = build_cost_matrix(layer_id=1)
    cost_M2 = build_cost_matrix(layer_id=2)
    layer_cost_matrix = {"M1": cost_M1, "M2": cost_M2}

    total_wirelength = 0

    for veh_id, data in vehicle_data.items():
        router      = router_by_name.get(str(veh_id))
        router_name = router[0] if router else str(veh_id)
        layer_name  = router[1] if router else "?"
        cmat        = layer_cost_matrix.get(layer_name, cost_M1)

        task_seq    = data.get("task_id",       [])
        route_nodes = data.get("route",         [])
        arrivals    = data.get("arrival_stamp", [])

        # Filter out depot stops (task id == "Depot" or node == 0 at start/end)
        stops = []
        for idx, t in enumerate(task_seq):
            node = route_nodes[idx] if idx < len(route_nodes) else None
            arr  = arrivals[idx]    if idx < len(arrivals)    else "?"
            if str(t) == "Depot":
                continue
            stops.append((t, node, arr))

        # Compute actual wire length for this router's path
        wire_cost = 0
        if len(stops) > 1:
            for i in range(len(stops) - 1):
                a = stops[i][1]
                b = stops[i+1][1]
                if a is not None and b is not None:
                    wire_cost += cmat[int(a)][int(b)]
        # Add depot→first and last→depot legs
        if stops:
            wire_cost += cmat[0][int(stops[0][1])]
            # drop_return_trips=True so no return leg

        total_wirelength += wire_cost

        print(f"\n  ┌─ {router_name}  [{layer_name}]")
        print(f"  │  Wire length (cost units) : {wire_cost}")
        print(f"  │  Nets routed : {len(stops)}")

        if not stops:
            print("  │  (no nets assigned)")
        else:
            print("  │  Net sequence:")
            for seq_i, (t, node, arr) in enumerate(stops):
                net_name = task_name_map.get(str(t), str(t))
                r, c     = node_to_rc(int(node)) if node is not None else ("?", "?")
                # Timing slack = latest_allowed - actual_arrival
                net_def  = next((n for n in NETS if n[0] == net_name), None)
                slack    = f"{net_def[6] - float(arr):+.1f}" if net_def and arr != "?" else "?"
                print(f"  │    [{seq_i+1}] {net_name:<10} → node {node:>2}  "
                      f"(grid {r},{c})  arrival={arr:<5}  slack={slack}")
        print("  └" + "─" * 50)

    print(f"\n  ► Total wire length  : {total_wirelength} cost-units")
    print(f"  ► Solver objective   : {solution_cost}")
    print(f"  ► Grid size          : {GRID_ROWS}×{GRID_COLS}")
    print(f"  ► Nets routed        : {len(NETS)}  |  Layers: {len(ROUTERS)}")
    print("═" * 60 + "\n")

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("Building cost matrices …")
    payload = build_payload()

    # Optionally dump the payload for inspection
    if "--dump" in sys.argv:
        print(json.dumps(payload, indent=2))
        return

    print("Submitting to NVIDIA cuOpt …")
    session  = requests.Session()
    response = session.post(INVOKE_URL, headers=HEADERS, json=payload, timeout=30)

    # Poll for async completion
    elapsed = 0
    while response.status_code == 202:
        req_id    = response.headers.get("NVCF-REQID")
        fetch_url = FETCH_URL_FMT.format(req_id)
        print(f"  Waiting for result (req={req_id[:8]}…) [{elapsed}s]")
        time.sleep(POLL_INTERVAL_S)
        elapsed  += POLL_INTERVAL_S
        response  = session.get(fetch_url, headers=HEADERS, timeout=30)
        if elapsed > MAX_WAIT_S:
            sys.exit("ERROR: Timed out waiting for cuOpt result.")

    try:
        response.raise_for_status()
    except requests.HTTPError as exc:
        sys.exit(f"HTTP error: {exc}\nBody: {response.text}")

    response_body = response.json()
    interpret_results(response_body)


if __name__ == "__main__":
    main()