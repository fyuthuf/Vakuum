# Vakuum Physics Engine

**Vakuum** is an independent 3D physics engine written entirely in Kotlin/JVM.

It is **NOT** a Minecraft mod, a Valkyrien Skies addon, or a wrapper around an existing physics engine. Vakuum is a standalone physics library designed to be integrated into game engines, simulations, and other software through its public API.

## Architecture Reference

Vakuum's architecture is inspired by [Jolt Physics](https://github.com/jrouwe/JoltPhysics) (public documentation only). Jolt is an architectural and educational reference, **not** a runtime dependency.

## Current Status: V0.2.1 — Quality, Stability & Release Preparation

V0.2.1 is a quality improvement release that fixes critical bugs, adds integration tests, and prepares the repository for public GitHub publication.

### V0.2.1 Changes

- **Fixed**: Narrowphase SAT now correctly handles rotated boxes
- **Fixed**: Broadphase uses actual collision shapes instead of hardcoded defaults
- **Added**: 8 integration tests covering the full physics pipeline
- **Added**: GitHub Actions CI workflow
- **Added**: BUILDING.md, CONTRIBUTING.md, CHANGELOG.md
- **Removed**: Dead code and unused imports

### V0.2 Features

V0.2 adds rigid body dynamics, collision detection, an impulse-based collision solver, and a Blender visualization bridge.

### Implemented

- **Mathematical Foundation** (`vakuum-math`):
  - Vector2, Vector3, Vector4 with full arithmetic operations
  - Matrix3, Matrix4 with multiplication, transpose, determinant, inverse
  - Quaternion with SLERP, axis-angle, matrix conversion
  - Transform, Plane, Ray, AABB, BoundingSphere
  - Numerical stability: NaN/infinity rejection, zero-vector checks, singular matrix detection

- **Physics Core** (`vakuum-core`):
  - PhysicsWorld with lifecycle (initialize, step, pause, resume, shutdown)
  - PhysicsConfiguration with gravity, timestep, solver iterations
  - Fixed timestep simulation
  - State management (IDLE, RUNNING, PAUSED, STOPPED)

- **Rigid Body Dynamics** (`vakuum-dynamics`):
  - Body types: STATIC, DYNAMIC, KINEMATIC
  - Mass properties with inertia tensor
  - Semi-implicit Euler integration
  - Gravity, damping, force/torque application

- **Collision Detection** (`vakuum-collision`):
  - Box-box collision using SAT (Separating Axis Theorem)
  - Box-plane collision detection
  - Broadphase O(n²) pair detection
  - Contact manifold generation

- **Collision Solver** (`vakuum-solver`):
  - Impulse-based solver with Gauss-Seidel relaxation
  - Normal impulse with restitution
  - Friction using Coulomb's law
  - Penetration correction (Baumgarte stabilization)

- **Simulation Runner** (`vakuum-runner`):
  - Full physics pipeline: integrate → collide → solve → correct
  - JSON export for frame-by-frame body transforms
  - 5 experiments: single cube, multiple cubes, stacking, stress test, reproducibility

- **Blender Bridge** (`vakuum-blender/`):
  - Python scripts to import JSON simulation data
  - Automatic keyframe animation
  - Camera and lighting setup

### Not Yet Implemented

- Capsule, sphere, convex hull collision shapes (V0.3)
- Continuous collision detection (V0.6)
- Sleeping system (V0.6)
- Multithreading (V0.7)
- Joints beyond basic constraints (V0.8)

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
├── vakuum-runner/     # Simulation runner & JSON export
├── vakuum-tests/      # Integration tests
└── vakuum-blender/    # Blender Python scripts
```

## Testing

249+ comprehensive tests covering all mathematical operations, physics core, rigid body dynamics, collision detection, collision solver, and full integration tests (falling cube, multi-cube, stacking, stress test, reproducibility, free fall, static body, long-running stability).

```bash
mvn test
```

## License

To be determined. See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.
