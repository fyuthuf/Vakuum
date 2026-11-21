# Vakuum Physics Engine

**Vakuum** is an independent 3D physics engine written entirely in Kotlin/JVM.

It is **NOT** a Minecraft mod, a Valkyrien Skies addon, or a wrapper around an existing physics engine. Vakuum is a standalone physics library designed to be integrated into game engines, simulations, and other software through its public API.

## Architecture Reference

Vakuum's architecture is inspired by [Jolt Physics](https://github.com/jrouwe/JoltPhysics) (public documentation only). Jolt is an architectural and educational reference, **not** a runtime dependency.

## Current Status: V0.3.0 — Performance & Collision Expansion

V0.3.0 adds Dynamic AABB Tree broadphase, Continuous Collision Detection (CCD), Sphere/Capsule/ConvexHull shapes, collision filtering, sleeping system, contact caching, and a website integration adapter for Vakuum.xyz.

### V0.3.0 Features

- **Dynamic AABB Tree** — O(log n) broadphase replacing O(n²) brute force
- **CCD** — Continuous Collision Detection for fast-moving bodies (Box, Sphere, Capsule, ConvexHull vs static obstacles)
- **New Collision Shapes** — Sphere, Capsule, ConvexHull with full narrowphase and CCD support
- **Collision Filtering** — Category masks, collision groups, layer-based filtering
- **Sleeping System** — Automatic sleep/wake for bodies at rest
- **Contact Caching** — Persistent contacts with warm starting infrastructure
- **Sequential Impulse Solver** — Stable PGS solver with accumulated impulse clamping
- **Website Integration** — `vakuum-website` module for Vakuum.xyz with session management and grab constraints
- **Benchmark Suite** — 8 benchmark scenarios for performance measurement

### Implemented

- **Mathematical Foundation** (`vakuum-math`):
  - Vector2, Vector3, Vector4 with full arithmetic operations
  - Matrix3, Matrix4 with multiplication, transpose, determinant, inverse
  - Quaternion with SLERP, axis-angle, matrix conversion
  - Transform, Plane, Ray, AABB, BoundingSphere
  - Numerical stability: NaN/infinity rejection, zero-vector checks, singular matrix detection

- **Physics Core** (`vakuum-core`):
  - PhysicsWorld with lifecycle (initialize, step, pause, resume, shutdown)
  - PhysicsConfiguration with gravity, timestep, solver iterations, CCD settings
  - Fixed timestep simulation
  - State management (IDLE, RUNNING, PAUSED, STOPPED)

- **Rigid Body Dynamics** (`vakuum-dynamics`):
  - Body types: STATIC, DYNAMIC, KINEMATIC
  - Mass properties with inertia tensor
  - Semi-implicit Euler integration
  - Gravity, damping, force/torque application
  - Sleeping with configurable thresholds

- **Collision Detection** (`vakuum-collision`):
  - **Shapes**: Box, Sphere, Capsule, ConvexHull, Plane
  - **Broadphase**: Dynamic AABB Tree (O(log n)) + Brute Force (O(n²) reference)
  - **Narrowphase**: SAT for all shape pairs
  - **CCD**: Swept AABB + TOI for all shapes vs static obstacles
  - **Filtering**: Category masks, collision groups
  - Contact manifold generation

- **Collision Solver** (`vakuum-solver`):
  - Sequential Impulse (Projected Gauss-Seidel) solver
  - Normal impulse with accumulated clamping
  - Coulomb friction with friction cone
  - Restitution as velocity bias (first iteration only)
  - Penetration correction (Baumgarte stabilization with slop)

- **Simulation Runner** (`vakuum-runner`):
  - Full physics pipeline: integrate → broadphase → narrowphase → solve → correct
  - CCD substep processing with TOI
  - Contact caching and warm starting infrastructure
  - JSON export for frame-by-frame body transforms
  - CCD diagnostics

- **Website Integration** (`vakuum-website`):
  - Isolated simulation sessions
  - Body spawning (Box, Sphere)
  - Grab constraints with natural release
  - Boundary walls for ASCII Playground
  - State snapshots and diagnostics

- **Blender Bridge** (`vakuum-blender/`):
  - Python scripts to import JSON simulation data
  - Automatic keyframe animation
  - Camera and lighting setup

- **Benchmarks** (`vakuum-tests`):
  - 8 benchmark scenarios: 100/500/1000 bodies, dense collision, large stack, mixed shapes, CCD, long-running
  - P50/P95/P99 latency measurement
  - Deterministic with fixed random seeds

### Not Yet Implemented

- Joint constraints (hinge, fixed, distance) — V0.6
- Multithreading — V0.7
- Concave mesh collision — V0.8
- Dynamic vs dynamic CCD — Future
- Rotational CCD — Future
- Warm starting — Disabled (requires contact geometry validation)
- Body removal from pipeline — V0.4
- Serialization — V0.9
- WebSocket server for website — V0.9

## Build

Requires **Java 21** and **Maven**.

```bash
# Build
mvn clean compile

# Run tests
mvn test

# Full build with tests
mvn clean verify
```

## Running Simulations

```bash
# Run Experiment A (single falling cube)
mvn exec:java -pl vakuum-runner -Dexec.args="A output"

# Run Experiment B (10 cubes)
mvn exec:java -pl vakuum-runner -Dexec.args="B output"

# Run Experiment D (100 cubes stress test)
mvn exec:java -pl vakuum-runner -Dexec.args="D output"
```

## Blender Integration

1. Run a simulation to generate JSON:
   ```bash
   mvn exec:java -pl vakuum-runner -Dexec.args="A output"
   ```
2. Open Blender and run `vakuum-blender/import_simulation.py`
3. Edit the `json_path` variable to point to your JSON file
4. Run the script to import and animate the simulation

See `vakuum-blender/README.md` for details.

## Technical Specifications

| Property | Value |
|----------|-------|
| Language | Kotlin/JVM |
| JDK | Java 21 |
| Build System | Maven |
| Precision | Double (64-bit) |
| Coordinate System | Right-handed (X right, Y up, Z backward) |
| Quaternion Convention | Hamilton (ij=k) |
| Matrix Storage | Row-major |

## Module Structure

```
Vakuum/
├── vakuum-math/       # Mathematical foundation (independent)
├── vakuum-core/       # Physics world & lifecycle (depends on math)
├── vakuum-api/        # Public interfaces (depends on core, math)
├── vakuum-dynamics/   # Rigid body dynamics (depends on core, math)
├── vakuum-collision/  # Collision detection (depends on core, math, dynamics)
├── vakuum-constraints/# Future: joints & constraints
├── vakuum-solver/     # Impulse solver (depends on core, math, dynamics, collision)
├── vakuum-runtime/    # Future: execution & scheduling
├── vakuum-runner/     # Simulation runner, pipeline & JSON export
├── vakuum-website/    # Website integration adapter for Vakuum.xyz
├── vakuum-tests/      # Integration tests & benchmarks
└── vakuum-blender/    # Blender Python scripts
```

## Testing

229+ comprehensive tests covering all mathematical operations, physics core, rigid body dynamics, collision detection (Box, Sphere, Capsule, ConvexHull), CCD, collision solver, collision filtering, Dynamic AABB Tree, and full integration tests (falling cube, multi-cube, stacking, stress test, reproducibility, free fall, static body, long-running stability, CCD pipeline).

See also:
- [CCD Support Matrix](docs/CCD_SUPPORT.md)
- [Collision Shapes](docs/COLLISION_SHAPES.md)
- [Constraint Solver](docs/SOLVER.md)
- [Benchmarks](docs/BENCHMARKS.md)
- [Known Limitations](docs/LIMITATIONS.md)
- [Website Integration](docs/WEBSITE_INTEGRATION.md)

```bash
mvn test
```

## License

To be determined. See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.
