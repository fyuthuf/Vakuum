# Changelog

All notable changes to Vakuum will be documented in this file.

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
