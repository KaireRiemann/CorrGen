# Differentiable Collision-Free Parametric Corridors

A C++17 implementation of the paper's parametric corridor formulation, with ROS1 demos for interactive 3D path planning and corridor generation. This repository is an independent implementation and engineering extension; the authors' original implementation is linked below.

**Original paper:** Jon Arrizabalaga, Zachary Manchester, and Markus Ryll, *Differentiable Collision-Free Parametric Corridors*, 2024 (accepted at IROS 2024).

[Paper on arXiv](https://arxiv.org/abs/2407.12283) · [Full paper (PDF)](https://arxiv.org/pdf/2407.12283) · [Authors' original code](https://github.com/jonarriza96/corrgen) · [Map generator](https://github.com/KumarRobotics/kr_param_map)

Select a start and a goal in RViz to compute a 3D A* path, construct a smooth reference, and generate a differentiable corridor along the entire route. The current pipeline produces a geometric feasible region; trajectory optimization and execution control are outside its scope.

## Overview

```text
kr_param_map → occupancy voxels → interactive start/goal selection
            → 26-connected A* → piecewise C² cubic B-spline reference
            → cross-section intersections → global Chebyshev LP
            → corridor validation → RViz visualization
```

The implementation provides:

- One global set of polynomial corridor coefficients, with offset elliptical cross-sections and analytical first and second derivatives.
- HiGHS as the default LP backend, with ProxQP, GLPK, and optional real_planner SDLP alternatives.
- Constraint generation with verification against the complete original intersection constraints.
- Clearance-aware A*, reference smoothing, and conservative continuous checks against inflated occupied voxels and map boundaries.
- A cancellable worker process, explicit computation budgets, request snapshots, and diagnostics.
- Six interactive map presets, including sparse and dense structured environments.

## Build

The tested setup uses ROS Noetic in the existing `ros1_noetic` Docker container. The host workspace `/home/diffbot/ros1_ws/corr_gen` is mounted at `/root/ws/corr_gen` in that container. Adjust paths for your installation; the helper script assumes this existing container and mount.

The expected workspace layout is:

```text
corr_gen/
├── src/
│   ├── CorrGen/          # This repository
│   └── kr_param_map/     # Upstream map repository
├── .deps/               # Local solver dependencies
├── build/
└── devel/
```

For a new source checkout:

```bash
mkdir -p ~/corr_gen/src
cd ~/corr_gen/src
git clone https://github.com/KaireRiemann/CorrGen.git CorrGen
git clone https://github.com/KumarRobotics/kr_param_map.git kr_param_map
# Map revision used for the recorded regression tests:
git -C kr_param_map checkout 12dae849e05f28c9fbbcab2b3f18ba793fb536e2
```

Dependencies include Eigen3, GLPK, ROS Noetic, and nlohmann-json3-dev. The map generator also requires PCL, SDL 1.2, and SDL_image. The default demos use HiGHS 1.10.0; the ProxQP backend uses ProxSuite 0.6.3 C++ headers. `setup_solvers.sh` installs these solver dependencies under the workspace's `.deps`. Python/pip is used to obtain the headers, but corridor solving runs in C++.

From the host, using the tested container layout:

```bash
cd /home/diffbot/ros1_ws/corr_gen
bash src/CorrGen/scripts/build_ros1.sh ros1_noetic
```

Alternatively, inside a ROS Noetic environment with the dependencies installed:

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws/corr_gen  # Replace with your workspace path.
workspace="$PWD"
bash src/CorrGen/scripts/setup_solvers.sh "$workspace"
catkin_make -DCMAKE_BUILD_TYPE=Release -DCORRGEN_BUILD_ROS1=ON \
  -DCMAKE_PREFIX_PATH="$workspace/.deps/highs;/opt/ros/noetic" \
  -DPROXQP_INCLUDE_DIR="$workspace/.deps/proxsuite/cmeel.prefix/include" -j2
(cd build/CorrGen && ctest --output-on-failure)
```

For the standalone C++ core, with Eigen3 and GLPK installed:

```bash
cmake -S src/CorrGen -B src/CorrGen/build -DCMAKE_BUILD_TYPE=Release
cmake --build src/CorrGen/build -j2
ctest --test-dir src/CorrGen/build --output-on-failure
```

Add the HiGHS prefix and ProxSuite include path above to enable those optional backends. In the existing host setup, GLPK is unpacked under `.deps/usr`; host builds require that prefix and the corresponding library search path. Do not load host Ubuntu 24.04 libraries into the Ubuntu 20.04 ROS container.

## Interactive demos

Inside the configured ROS environment:

```bash
source /root/ws/corr_gen/devel/setup.bash
roslaunch corrgen_cpp medium.launch
```

1. Wait for the map and the `WAITING_FOR_START` status. No endpoints are selected automatically.
2. Use **2D Nav Goal** to select the start in free space. It appears as a green sphere.
3. Use **2D Nav Goal** again to select the goal, shown as a red sphere. This starts planning and corridor generation.
4. Inspect the orange A* path, cyan reference curve, and purple corridor. After `PASS`, select another pair of endpoints.
5. A new selection clears the previous output. Occupied or out-of-bounds inputs report `INVALID_POINT`; planning failures report their cause.

Use **2D Pose Estimate** to explicitly reset the start, followed by **2D Nav Goal** for the goal. **Publish Point** uses the actual message xyz coordinates; clicking an obstacle surface will usually produce an invalid point. Set the RViz Fixed Frame to `map`; the input interface does not transform between frames.

### Selection height

Structured maps default to `height_mode:=arrow`, following the local FIRI benchmark interaction: arrow rotation determines height as `z_min + inset + |q_z| * (z_size - 2 * inset)`, where `inset = ceil((inflation + robot_radius + safety_margin + reference_clearance) / resolution) * resolution + resolution / 2`.

Image-based maps default to a fixed height of 1.5 m. Every launch supports explicit fixed-height selection:

```bash
roslaunch corrgen_cpp medium.launch height_mode:=fixed height:=1.5
```

### Map presets

Run one launch at a time. Closing RViz also terminates its launch session.

| Launch | Environment |
|---|---|
| `sparse.launch` | Sparse structured 3D obstacles |
| `medium.launch` | Medium-density structured 3D obstacles |
| `dense.launch` | Dense structured 3D obstacles |
| `balls_demo.launch` | Image-based cylindrical obstacles |
| `multi_gap_demo.launch` | Image-based staggered openings |
| `maze_demo.launch` | Image-based maze |

Structured maps use a **50 × 50 × 5 m** volume, origin **(-25, -25, 0)**, and **0.25 m** resolution. RViz displays boxes colored by height. Image-based maps use a 20 × 10 × 3 m volume and 0.2 m resolution. The `sparse_demo.launch`, `medium_demo.launch`, and `dense_demo.launch` names remain as compatibility entry points.

```bash
roslaunch corrgen_cpp sparse.launch
roslaunch corrgen_cpp dense.launch
# Unified entry point with an explicit map seed:
roslaunch corrgen_cpp global_planning.launch density:=dense map_seed:=1024
```

| Density | cylinder_ratio | poly_ratio | circle_ratio | ellip_ratio |
|---|---:|---:|---:|---:|
| sparse | 0.03 | 0.003 | 0 | 0 |
| medium | 0.06 | 0.006 | 0 | 0 |
| dense | 0.10 | 0.010 | 0.01 | 0.01 |

All three presets use `gate_ratio=0` and `w1/w2/w3=0.10/0.50/2.00`. These are upstream generator parameters, not fixed obstacle counts. The upstream polyhedron generator uses an independent `random_device`, so a fixed `map_seed` does not guarantee identical point clouds. Actual maps are saved as `map_<version>.xyz` for replay.

`global_planning.launch` also accepts `map_size_x/y/z`, `map_x/y/z_origin`, `resolution`, and individual obstacle ratios. Update the RViz Map display's Size (m) when changing resolution. The generic `demo.launch` exposes `map_type`, `map`, `density`, `seed`, `size_x/y/z`, and `resolution`. Use `rviz:=false` for headless runs.

## Solvers, clearance, and budgets

The default is `solver:=highs`, with degree 6 and 35 global LP variables. Collision witnesses are added as constraints for re-optimization. The ROS pipeline does not uniformly shrink the corridor to obtain a passing result: `shrink_factor=1`. A validated output is labeled `FEASIBLE_BODY_CONSTRAINED`; it should not be interpreted as the optimum of the original finite-point LP.

```bash
roslaunch corrgen_cpp dense.launch
roslaunch corrgen_cpp dense.launch solver:=proxqp
```

A* uses `clearance_weight:=4` by default, and path simplification preserves this cost preference. Set it to 0 to compare with length-based search. `inflation` defaults to 0 and adds to `robot_radius` (0.2 m) and `safety_margin` (0.05 m) when constructing configuration-space obstacles. `reference_clearance` reserves additional room during search and smoothing: 0.1 m for structured maps and 0.05 m for image-based maps. Geometric cutting points use an inward normal margin of at most 1 cm.

The default `job_budget_ms:=5000` covers the entire worker; `solver_budget_ms:=1000` limits each inner LP call. Recorded forward/reverse regressions across six maps took approximately **1.6–3.7 seconds end to end**. These measurements are specific to the recorded setup, not a real-time guarantee. Earlier subsecond measurements used a different corridor pipeline and do not describe the current geometry checks.

Timeouts without a fully validated result report failure. Resetting, selecting a new start, or changing the map cancels the old worker and clears stale output. See the [corridor quality report](docs/quality_repair.md) and the [historical solver engineering report](docs/engineering_delivery.md) for measurements and limitations.

## Code structure

| Location | Responsibility |
|---|---|
| `include/corrgen/`, `src/corridor.*`, `src/generator.cpp` | Global corridor model, analytical derivatives, LP construction, and positive-definiteness checks |
| `src/corridor_check.cpp` | Continuous corridor geometry checks |
| `src/voxel_planner.cpp` | 3D A*, clearance cost, path simplification, and reference smoothing |
| `src/path.cpp` | Piecewise C² reference geometry, frames, and parameterization on u ∈ [0,1] |
| `src/solver.cpp`, `src/*adapter*` | Solver backends and working-set optimization |
| `apps/worker.cpp` | Snapshot loading, planning, projection, solving, checking, and result export |
| `ros/demo_node.cpp` | Map cache, interactive selection, worker lifecycle, and visualization |
| `launch/`, `rviz/` | Interactive demo configuration |
| `scripts/`, `tests/` | Dependency setup, replay, regressions, and core tests |

The reference preserves both selected endpoints and supports reverse travel, multiple turns, and different heights. Reference smoothing is geometric preparation, not time parameterization or dynamic trajectory optimization.

## Validation and recorded outputs

```bash
docker exec ros1_noetic bash -lc \
  'source /root/ws/corr_gen/devel/setup.bash; python3 /root/ws/corr_gen/src/CorrGen/scripts/test_ros1_demos.py'
```

The regression script uses a separate ROS master on port 11347 and publishes to the same selection topics as RViz. It checks waiting for selection, endpoint preservation, forward/reverse requests, repeated planning, invalid inputs, stale-output removal, explicit start selection, and reset behavior.

Manual sessions write to `results/live/<scene>/run_<time>_<pid>/request_<id>/`; automated sessions write under `results/interactive/<scene>/run_<time>_<pid>/`. Each session has a separate directory. `request.json` records the map snapshot, hash, endpoints, and configuration. Results contain the A* path, reference, Bézier control points, frame axis, corridor coefficients, and metrics. Failures, timeouts, and cancellations also produce `completion.json` or `failure.json`.

```bash
rosparam get /corrgen_demo/session_dir
rostopic pub -1 /corrgen_demo/reset std_msgs/Empty '{}'
```

Published topics under `/corrgen_demo/` include `map`, `astar_path`, `reference_path`, `corridor`, `selection`, `status`, and `diagnostics`.

Additional documentation (currently in Chinese): [coding conventions](coding.md), [mathematical formulation and differences from the paper](docs/method.md), [interactive validation](docs/interactive.md), [SDLP comparison](docs/solver.md), and [corridor quality repair](docs/quality_repair.md).

### Scope and limitations

Continuous acceptance checks conservatively bound the corridor against inflated occupied voxels and map boundaries. The implementation uses floating-point Bernstein bounds with rounding guards, not a formal interval-arithmetic proof. Safety depends on map coverage: unobserved voxels without points are treated as free space. The current output does not account for dynamics, localization uncertainty, or execution control. Solver-only timings from the paper or earlier experiments are not directly comparable to the complete ROS pipeline.

## Citation and acknowledgments

Please cite the original paper when using its corridor formulation. The following citation is provided by the [authors' repository](https://github.com/jonarriza96/corrgen):

```bibtex
@article{arrizabalaga2024differentiable,
  title={Differentiable Collision-Free Parametric Corridors},
  author={Arrizabalaga, Jon and Manchester, Zachary and Ryll, Markus},
  journal={arXiv preprint arXiv:2407.12283},
  year={2024}
}
```

- **Original research and implementation:** We thank Jon Arrizabalaga, Zachary Manchester, and Markus Ryll for the [paper](https://arxiv.org/abs/2407.12283) and the open-source [jonarriza96/corrgen](https://github.com/jonarriza96/corrgen) implementation, which provide the foundation for this C++ implementation.
- **Benchmark maps:** We thank **Kumar Robotics** and the contributors to [KumarRobotics/kr_param_map](https://github.com/KumarRobotics/kr_param_map) for the parameterized map generator used in the ROS demos and evaluations. The upstream map source is maintained separately and was not modified for the reported tests.
- **Numerical software:** We acknowledge HiGHS, ProxSuite/ProxQP, GLPK, and Eigen, as well as the real_planner SDLP implementation used for optional comparisons.
