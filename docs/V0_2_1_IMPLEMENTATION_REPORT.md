# Vakuum V0.2.1 Implementation Report

## Overview

V0.2.1 is a quality improvement release that audits, fixes, and prepares the Vakuum physics engine for its first public GitHub release.

## Bugs Fixed

1. **Narrowphase rotation bug**: `getLocalAxes()` returned identity axes (X, Y, Z) instead of transforming them by the body's rotation. This meant rotated boxes were tested as if they were axis-aligned, producing incorrect collision results for any non-identity orientation.

2. **Broadphase hardcoded extents**: The broadphase used a hardcoded `Vector3(1.0, 1.0, 1.0)` extent for all bodies instead of using actual collision shapes. This caused incorrect pair detection — bodies with small shapes were treated as large, and bodies with large shapes were treated as small.

3. **Dead code in ImpulseSolver**: The `solveVelocityConstraint()` method was a stub that did nothing. The `solvePositionConstraint()` method was also a no-op. These have been removed.

4. **Unused imports**: Removed unused `MathUtils` import from Narrowphase and `Matrix3` import from ImpulseSolver.

## Improvements

1. **Broadphase shape-aware**: Broadphase now accepts a `Map<Int, CollisionShape>` and computes proper AABBs from OBBs by projecting rotated axes onto world axes.

2. **Integration test suite**: 8 comprehensive integration tests covering the full physics pipeline:
   - Single cube falling and resting
   - Multiple cubes without NaN
   - Cube stack stability
   - 100-cube stress test
   - Reproducibility verification
   - Free fall analytical solution
   - Static body immobility
   - Long-running simulation stability (1000 steps)

3. **GitHub Actions CI**: Automated build and test pipeline.

4. **Documentation**: BUILDING.md, CONTRIBUTING.md, CHANGELOG.md.

5. **Repository hygiene**: Comprehensive .gitignore, Maven wrapper script.

## Files Changed

| File | Change |
|------|--------|
| `vakuum-collision/Narrowphase.kt` | Fixed rotation bug, removed unused import |
| `vakuum-collision/Broadphase.kt` | Shape-aware AABB computation |
| `vakuum-solver/ImpulseSolver.kt` | Removed dead code, removed unused import |
| `vakuum-runner/SimulationPipeline.kt` | Pass shapeMap to broadphase |
| `vakuum-tests/pom.xml` | Added runner dependency |
| `pom.xml` | Added runner to dependencyManagement |
| `.github/workflows/ci.yml` | New: CI workflow |
| `.gitignore` | New: comprehensive ignore rules |
| `mvnw.cmd` | New: Maven wrapper |
| `BUILDING.md` | New: build instructions |
| `CONTRIBUTING.md` | New: contribution guidelines |
| `CHANGELOG.md` | New: changelog |
| `vakuum-tests/.../IntegrationTest.kt` | New: 8 integration tests |

## Test Results

- **Total tests**: 249 (175 existing + 8 new integration + 66 from other modules)
- **All tests pass**: Yes
- **Build time**: ~14 seconds

## Known Limitations

1. No sleeping system — all bodies are simulated every step.
2. No continuous collision detection — fast-moving objects may tunnel.
3. Broadphase is O(n²) — suitable for <200 bodies.
4. Only box-box and box-plane collision shapes.
5. No joints/constraints beyond collision.
6. Single-threaded simulation.

## Next Steps

- V0.3: Additional collision shapes (capsule, sphere, convex hull)
- V0.4: Sweep and Prune broadphase
- V0.5: Sleeping system
- V0.6: Continuous collision detection
- V0.7: Multithreaded simulation
