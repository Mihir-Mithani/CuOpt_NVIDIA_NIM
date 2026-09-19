# Potential Improvements for CuOpt-NVIDIA-NIM Chip Routing Optimizer

## 🔒 Security

- [ ] **API Key Management**: Move hardcoded API key from `API.py` to environment variable (e.g., `NVIDIA_API_KEY`) or secure secret management
- [ ] **Credential Rotation**: Add support for token refresh / API key rotation without code changes
- [ ] **HTTPS Verification**: Ensure certificate validation is enforced (currently uses default requests behavior)

## 🏗️ Architecture & Modularity

- [ ] **Extract Shared Modules**: Refactor duplicated code across V1/V2/V3 into shared modules:
  - `cuopt_client.py` — API communication, payload building, polling
  - `routing_matrices.py` — Cost/delay matrix builders, congestion modeling
  - `astar.py` — Octilinear A* pathfinding (V3)
  - `vrp_formulation.py` — VRP problem formulation helpers
  - `models.py` — Dataclasses for Grid, Net, Component, Route, Layer
- [ ] **Version Abstraction**: Create base `ChipRouter` class with V1/V2/V3 as subclasses or strategies
- [ ] **Plugin Architecture**: Allow custom cost functions, routing algorithms, visualizers

## 🧪 Testing

- [ ] **Unit Tests**: Add test coverage for:
  - Matrix builders (`build_cost_matrix`, `build_delay_matrix`)
  - A* pathfinding (octilinear moves, diagonal squeeze check, blocking)
  - Payload construction (fleet_data, task_data, solver_config)
  - Result interpretation
  - VRP formulation helpers
- [ ] **Integration Tests**: Mock cuOpt API responses for end-to-end testing
- [ ] **Property-Based Testing**: Use `hypothesis` for grid/routing invariants
- [ ] **Visual Regression Tests**: Screenshot comparison for GUI rendering
- [ ] **CI/CD Pipeline**: GitHub Actions for lint, type-check, test, build

## 📝 Code Quality

- [ ] **Type Hints**: Full type annotations across all modules (V3 has partial, V1/V2 minimal)
- [ ] **Docstrings**: Google/NumPy style docstrings for all public functions/classes
- [ ] **Linting**: Add `ruff` or `flake8` + `mypy` configuration
- [ ] **Code Formatting**: `black` or `ruff format` configuration
- [ ] **Pre-commit Hooks**: `pre-commit` for lint, format, type-check on commit

## ⚡ Performance & Reliability

- [ ] **Retry Logic**: Exponential backoff for cuOpt API calls (network resilience)
- [ ] **Connection Pooling**: Reuse `requests.Session` (partially done in V3)
- [ ] **Async Support**: `aiohttp` / `httpx` for non-blocking API calls
- [ ] **Caching**: Cache cost/delay matrices for same grid dimensions
- [ ] **Large Grid Optimization**: Sparse matrix representation for >20×20 grids
- [ ] **Parallel Routing**: Multi-threaded A* for independent net groups

## 🎯 Routing Engine Enhancements

- [ ] **Multi-Layer Support**: Extend beyond M1/M2 to M1-M8+ with via cost optimization
- [ ] **Obstacle Avoidance**: Keep-out zones, pre-placed macros, blockages
- [ ] **Differential Pair Routing**: Length matching, phase tuning, coupling control
- [ ] **Timing-Driven Routing**: Slack-based net prioritization, critical path optimization
- [ ] **Clock Tree Synthesis**: H-tree, balanced buffering, skew minimization
- [ ] **Power/Ground Routing**: Mesh/stripe planning, IR drop awareness
- [ ] **Via Optimization**: Via minimization, via stacking, layer transition rules
- [ ] **Congestion-Driven Routing**: Global routing feedback, iterative refinement
- [ ] **Rip-up & Reroute**: Smart rip-up strategies for blocked nets

## 📤 Export & Integration

- [ ] **DEF Export**: Standard Design Exchange Format for EDA toolchain
- [ ] **GDSII/OASIS Export**: Physical layout interchange
- [ ] **LEF/DEF Import**: Read existing designs for incremental routing
- [ ] **OpenAccess/PCell Integration**: Cadence/Synopsys DB connectivity
- [ ] **Batch Processing**: CLI for multiple designs, headless mode
- [ ] **REST/gRPC API**: Service mode for integration with other tools

## 🖥️ GUI/UX Improvements (V3)

- [ ] **Undo/Redo Stack**: Command pattern for component placement, pair creation
- [ ] **Zoom/Pan**: Mouse wheel zoom, middle-click pan on canvas
- [ ] **Layer Visibility Toggle**: Show/hide M1, M2, vias, components independently
- [ ] **Net Highlighting**: Click net in list to highlight route on canvas
- [ ] **Routing Statistics Panel**: Wire length, via count, bends, layer usage per net
- [ ] **DRC Visualization**: Design rule check violations (spacing, width, shorts)
- [ ] **3D View**: Side-view stackup visualization (pygame/OpenGL)
- [ ] **Keyboard Shortcuts**: Power-user accelerators
- [ ] **Theme Support**: Light/dark/system theme switching
- [ ] **Internationalization**: i18n support for UI strings

## 📦 Packaging & Distribution

- [ ] **PyPI Package**: `pip install cuopt-chip-router`
- [ ] **Docker Image**: Multi-stage build with cuOpt NGC container option
- [ ] **Desktop App**: PyInstaller / cx_Freeze / briefcase for standalone executable
- [ ] **VS Code Extension**: Integrated routing panel
- [ ] **Jupyter Notebook Widget**: Interactive routing in notebooks
- [ ] **Documentation Site**: Sphinx + ReadTheDocs with API reference

## 🔧 Developer Experience

- [ ] **Configuration File**: YAML/TOML for solver params, grid defaults, colors
- [ ] **Logging**: Structured logging (structlog) with levels, file rotation
- [ ] **Debug Mode**: Verbose solver logs, matrix dumps, path visualization
- [ ] **Profiling**: `py-spy` / `cProfile` integration for bottlenecks
- [ ] **Benchmark Suite**: Standard testcases (ISPD, custom) with metrics

## 📚 Documentation

- [ ] **Architecture Decision Records (ADRs)**: Document key design choices
- [ ] **API Reference**: Auto-generated from docstrings
- [ ] **Tutorials**: Step-by-step for common workflows
- [ ] **Algorithm Deep-Dives**: VRP formulation, A* details, cuOpt mapping
- [ ] **Migration Guides**: V1→V2→V3 upgrade paths
- [ ] **Contributing Guide**: Code style, PR process, issue templates

## 🌐 Cloud/Enterprise

- [ ] **Local cuOpt Deployment**: NGC container support (offline/air-gapped)
- [ ] **Multi-Tenant API Keys**: Project/workspace isolation
- [ ] **Usage Monitoring**: Token/call tracking, cost estimation
- [ ] **SSO/OIDC Integration**: Enterprise authentication
- [ ] **Audit Logging**: Compliance-ready operation logs

---

## Priority Matrix

| Priority | Category | Items |
|----------|----------|-------|
| **P0 (Critical)** | Security | API key env var, HTTPS verification |
| **P1 (High)** | Architecture | Shared modules, type hints, unit tests |
| **P2 (Medium)** | Routing Engine | Multi-layer, obstacles, diff pairs, timing-driven |
| **P3 (Medium)** | Export | DEF/GDSII, batch CLI |
| **P4 (Low)** | GUI | Zoom/pan, undo/redo, 3D view, themes |
| **P5 (Future)** | Cloud/Enterprise | Local cuOpt, SSO, audit logs |

---

## Quick Wins (Can Implement This Week)

1. Move API key to `os.environ.get("NVIDIA_API_KEY")`
2. Add `pyproject.toml` with `black`, `ruff`, `mypy` config
3. Extract `build_cost_matrix` / `build_delay_matrix` to `routing_matrices.py`
4. Add basic `pytest` tests for matrix builders
5. Add `logging` module with configurable level
6. Create `config.yaml` for solver parameters