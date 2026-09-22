# CuOpt Chip Routing Optimizer

An interactive chip/PCB routing optimizer that leverages **NVIDIA cuOpt** (Vehicle Routing Problem solver) combined with **octilinear A* pathfinding** to generate real-world PCB-style routing layouts.

---

## 🎯 Overview

This project models chip/PCB routing as a **Vehicle Routing Problem (VRP)**:

| VRP Concept | Chip Routing Equivalent |
|-------------|-------------------------|
| Vehicles    | Routing agents (one per metal layer: M1, M2, ...) |
| Tasks       | Signal nets / vias that must be connected |
| Locations   | Grid nodes on the routing fabric |
| Cost        | Wire length + layer-change penalty + congestion weight |
| Capacity    | Track utilization budget per routing channel |

### Key Features

- **🔧 cuOpt VRP Solver** — Uses NVIDIA's cuOpt API to find globally optimal net routing order
- **🎯 Octilinear A* Pathfinding** — Real PCB-style routing with 90° and 45° turns only
- **🚫 Non-crossing Wires** — Sequential routing with incremental blocking prevents wire overlaps
- **🔄 Via Visualization** — Automatic via dots at every layer transition/bend point
- **🎨 Interactive GUI** — Tkinter-based visual editor for grid creation, component placement, and net definition
- **📊 Real-time Results** — Live visualization of routed nets with wire length, bends, and layer statistics

---

## 📁 Project Structure

```
CuOpt-NVIDIA-NIM/
├── chip_routing.py      # V1: Baseline CLI version with fixed 5×5 grid
├── chip_routingv2.py    # V2: Interactive GUI with basic Manhattan routing
├── chip_routingv3.py    # V3: Full octilinear A* + cuOpt ordering + modern UI
├── client.py            # Basic cuOpt API client example
├── API.py               # API key loader (⚠️ replace with env var in production)
├── requirements.txt     # Python dependencies
├── improvements.md      # Roadmap & potential improvements
└── README.md            # This file
```

### Version Comparison

| Feature | V1 (`chip_routing.py`) | V2 (`chip_routingv2.py`) | V3 (`chip_routingv3.py`) |
|---------|------------------------|--------------------------|--------------------------|
| **Interface** | CLI only | Tkinter GUI | Tkinter GUI (modern) |
| **Grid** | Fixed 5×5 | User-defined (2–12) | User-defined (2–20) |
| **Routing** | cuOpt only | cuOpt only | cuOpt order + A* paths |
| **Path Style** | Abstract VRP routes | Manhattan grid lines | **Octilinear (90°/45°)** |
| **Wire Crossing** | Not prevented | Not prevented | **Blocked incrementally** |
| **Via Rendering** | No | No | **Yes (yellow dots)** |
| **Layer Preference** | M1=H, M2=V cost bias | Basic | Full layer-aware costs |
| **Accessibility** | N/A | Basic | **WCAG AA contrast, keyboard nav** |

---

## 🚀 Quick Start

### Prerequisites

- Python 3.10+
- NVIDIA API key for cuOpt (get from [NVIDIA NGC](https://ngc.nvidia.com/))

### Installation

```bash
# Clone or navigate to the project
cd CuOpt-NVIDIA-NIM

# Create virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Configuration

**⚠️ Security Note:** The current `API.py` contains a hardcoded API key. For production use, replace it with an environment variable:

```bash
# Option 1: Set environment variable (recommended)
export NVIDIA_API_KEY="your-api-key-here"

# Option 2: Edit API.py to read from env
# import os
# def API():
#     return os.environ.get("NVIDIA_API_KEY")
```

### Running the Application

```bash
# Run V3 (recommended — full featured)
python chip_routingv3.py

# Run V2 (simpler GUI)
python chip_routingv2.py

# Run V1 (CLI baseline)
python chip_routing.py
```

---

## 🖥️ GUI Workflow (V3)

### Step 1: Build Grid
- Enter **Rows** and **Cols** (2–20)
- Click **▶ BUILD GRID**
- Grid renders with depot at (0,0) highlighted in purple

### Step 2: Place Components
- Click any cell to select it (green highlight)
- Type component name in the entry field
- Press **Enter** or click **Place**
- Component name appears in the cell (blue text)

### Step 3: Create Wire Pairs (Nets)
- Click **⛓ ENTER PAIR MODE** (button turns purple)
- Click **source** cell → Click **sink** cell
- Enter net name (e.g., `CLK`, `VDD`, `DATA_A`)
- Repeat for all connections
- Dashed preview arrows show planned connections

### Step 4: Route
- Click **⚡ RUN CUOPT + A***
- cuOpt optimizes global net ordering
- A* routes each net sequentially with octilinear paths
- Results display in a popup with statistics

### Visual Legend
| Style | Meaning |
|-------|---------|
| **Solid line** | Orthogonal (90°) wire segment |
| **Dashed line** | Diagonal (45°) wire segment |
| **Yellow dot** | Via / bend point |
| **Colored circles** | Source (large) and sink (large) terminals |
| **Purple cell** | Depot / origin (0,0) |
| **Blue cell** | Component placement |
| **Green highlight** | Selected cell |

---

## 🔧 Algorithm Details

### cuOpt VRP Formulation

The problem is encoded as a VRP with:
- **2 vehicles** (M1_router, M2_router) representing metal layers
- **Cost matrices** per layer with directional preferences:
  - M1: Horizontal preferred (vertical moves penalized)
  - M2: Vertical preferred (horizontal moves penalized)
- **Travel-time matrix** for signal delay (RC) estimation
- **Capacity dimensions** for track budgets per layer
- **Time windows** for net scheduling constraints

### Octilinear A* Pathfinding

After cuOpt determines the optimal net order, each net is routed using A* with:

- **8-directional movement**: N, S, E, W, NE, NW, SE, SW
- **Cost**: 1.0 for orthogonal, √2 ≈ 1.414 for diagonal
- **Heuristic**: Chebyshev distance (admissible for octilinear)
- **Diagonal squeeze check**: Prevents cutting through blocked corners
- **Incremental blocking**: Previously routed wires reserve their interior cells

### Net Ordering Strategy

1. cuOpt solves VRP to minimize total wire length + balance layer load
2. Extract task sequence from cuOpt's `vehicle_data.task_id`
3. Route nets in that order using sequential A*
4. Fallback: greedy Manhattan order if cuOpt fails

---

## 📦 Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `requests` | ~2.33.1 | HTTP client for cuOpt API |
| `fastapi` | ~0.136.1 | (Optional) for future API server |
| `uvicorn[standard]` | - | (Optional) ASGI server |
| `numpy` | - | Numerical operations |
| `matplotlib` | - | (Optional) for plotting/export |

Install with:
```bash
pip install -r requirements.txt
```

---

## 🔐 Security Considerations

> **Current State:** The API key is stored in `API.py` as a hardcoded string. This is **not secure** for production or shared environments.

### Recommended Improvements (see `improvements.md`)

1. **Move API key to environment variable**:
   ```python
   # API.py
   import os
   def API():
       return os.environ.get("NVIDIA_API_KEY")
   ```

2. **Add `.env` support** with `python-dotenv`

3. **Enable certificate verification** (requests does this by default)

4. **Add secret scanning** to CI/CD pipeline

---

## 🧪 Testing & Development

### Run with Debug Output
```bash
# V1: Dump payload without calling API
python chip_routing.py --dump

# V1: Verbose solver response
python chip_routing.py --debug
```

### Code Quality Tools (Recommended)
```bash
# Add to requirements-dev.txt
pip install black ruff mypy pytest

# Format
black .

# Lint
ruff check .

# Type check
mypy .

# Test
pytest
```

---

## 🗺️ Roadmap

See [`improvements.md`](improvements.md) for detailed roadmap including:

### Priority P0 (Critical)
- [ ] API key via environment variable
- [ ] HTTPS certificate verification enforcement

### Priority P1 (High)
- [ ] Extract shared modules (`cuopt_client.py`, `routing_matrices.py`, `astar.py`)
- [ ] Full type hints across all versions
- [ ] Unit tests for matrix builders & A* pathfinding
- [ ] CI/CD pipeline with GitHub Actions

### Priority P2 (Routing Engine)
- [ ] Multi-layer support (M1–M8+)
- [ ] Obstacle/keep-out zones
- [ ] Differential pair routing
- [ ] Timing-driven routing
- [ ] Via optimization

### Priority P3 (Export & Integration)
- [ ] DEF/GDSII export for EDA toolchains
- [ ] Batch CLI for headless operation
- [ ] REST/gRPC API service mode

### Priority P4 (GUI/UX)
- [ ] Undo/redo stack
- [ ] Zoom/pan canvas
- [ ] Layer visibility toggles
- [ ] Net highlighting & statistics panel
- [ ] 3D stackup visualization

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Please read the [improvements.md](improvements.md) for suggested contribution areas.

---

## 📄 License

This project is for educational and research purposes. NVIDIA cuOpt API usage is subject to [NVIDIA's terms of service](https://www.nvidia.com/en-us/about-nvidia/terms-of-use/).

---

## 🙏 Acknowledgments

- **NVIDIA cuOpt** — GPU-accelerated VRP solver
- **Tkinter** — Built-in Python GUI framework
- **A* Pathfinding** — Classic algorithm with octilinear extension

---

## 📞 Support

For issues, questions, or contributions:
- Open a GitHub Issue
- Check `improvements.md` for known limitations
- Review cuOpt API documentation at [NVIDIA Developer](https://developer.nvidia.com/cuopt)

---

*Generated with [Claude Code](https://claude.com/claude-code)*