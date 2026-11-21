# Changelog

All notable changes to Vakuum will be documented in this file.

## [0.3.0] - 2026-11-21

### Added
- **Dynamic AABB Tree** — O(log n) broadphase with binary tree nodes, proxy insertion/removal, rebalancing, and ray queries
- **TreeBroadphase** — Tree-based broadphase implementation; BruteForceBroadphase retained as reference
- **Continuous Collision Detection (CCD)** — Swept AABB + TOI for Box, Sphere, Capsule, ConvexHull vs static obstacles
- **New Collision Shapes** — SphereShape, CapsuleShape, ConvexHullShape with full narrowphase and CCD support
- **Collision Filtering** — Category masks, collision groups, layer-based filtering via CollisionFilter
- **Sleeping System** — Automatic sleep/wake for bodies at rest with configurable thresholds
- **Contact Caching** — Persistent contacts between frames with warm starting infrastructure
- **Sequential Impulse Solver** — Stable PGS solver with accumulated impulse clamping, Coulomb friction, and Baumgarte stabilization
- **CCD Diagnostics** — ccdSteps, toiQueries, maxSubstepsReached, fallbackCount, maximumPenetration
- **Website Integration** — `vakuum-website` module with SimulationSession, grab constraints, boundary walls, and state snapshots
- **Benchmark Suite** — 8 scenarios: 100/500/1000 bodies, dense collision, large stack, mixed shapes, CCD, long-running
- **Documentation** — CCD_SUPPORT.md, COLLISION_SHAPES.md, SOLVER.md, BENCHMARKS.md, LIMITATIONS.md, WEBSITE_INTEGRATION.md

### Fixed
- CCD: toiBoxPlane/toiBoxBox return null when already penetrating (prevents infinite substep loops)
- CCD: toiBoxBox no longer mutates body transforms during binary search
- CCD: near-zero TOI protection (< 1e-8) prevents substep loops
- CCD: velocities integrate correctly on each substep with remainingDt
- Solver: restitution applied only on first iteration as velocity bias
- Solver: penetration slop (0.001m) prevents jitter on resting contacts

### Changed
- SimulationPipeline refactored: step() dispatches to stepDiscrete/stepWithCcd
- SimulationPipeline: generic CCD dispatch for all shape types via queryToiAgainstPlane/queryToiAgainstStatic
- README.md updated with V0.3.0 features and documentation links

## [0.2.1] - 2026-11-21

### Fixed
- Narrowphase SAT collision detection now correctly handles rotated boxes (getLocalAxes transforms axes using body rotation)
- Broadphase now uses actual collision shapes instead of hardcoded default extents
- Broadphase computes proper AABB from OBB by projecting rotated axes onto world axes
- Removed dead code from ImpulseSolver (stub solveVelocityConstraint method)
- Removed unused imports from Narrowphase and ImpulseSolver

### Added
- 8 integration tests covering full physics pipeline (falling cube, multi-cube, stacking, stress test, reproducibility, free fall, static body, long-running stability)
- GitHub Actions CI workflow for automated builds and tests
- Maven wrapper script (mvnw.cmd)
- Comprehensive .gitignore file
- BUILDING.md documentation
- CONTRIBUTING.md documentation
- CHANGELOG.md

### Changed
- SimulationPipeline now passes shapeMap to Broadphase for accurate pair detection
- vakuum-tests module now depends on vakuum-runner for integration testing

## [0.2.0] - 2026-11-20

### Added
- Rigid body system (BodyType, MassProperties, RigidBody, RigidBodyBuilder)
- Collision detection (BoxShape, PlaneShape, Broadphase, Narrowphase with SAT)
- Impulse-based collision solver (normal impulse, friction, restitution, position correction)
- Simulation runner with experiments A-E
- Blender integration scripts
- JSON simulation export

## [0.1.0] - 2026-11-20

### Added
- Mathematical foundation (Vector2/3/4, Matrix3/4, Quaternion, Transform, Plane, Ray, AABB, BoundingSphere)
- Physics world lifecycle (PhysicsWorld, PhysicsConfiguration, PhysicsWorldSettings)
- Comprehensive math tests (135 test cases)
- Physics core tests (40 test cases)
