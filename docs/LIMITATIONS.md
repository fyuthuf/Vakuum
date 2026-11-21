# Vakuum — Known Limitations

This document lists all known limitations of the Vakuum physics engine as of V0.3.0. These are honest limitations, not hidden bugs.

## Collision Detection

### Supported Shapes
- Only **Box, Sphere, Capsule, ConvexHull, and Plane** shapes are supported.
- **Concave meshes** (TriMesh, heightmaps) are not implemented.
- **Cylinder** and **Cone** shapes are not implemented.
- **Compound shapes** (multiple shapes per body) are not implemented.

### Contact Generation
- **Single contact point**: Each collision pair generates at most one contact point. Feature clipping for multiple contacts is not implemented.
- **No contact persistence**: Contact IDs are not stable across frames beyond the cache.

### Continuous Collision Detection
- **Static obstacles only**: CCD supports moving body vs static obstacle. Dynamic vs dynamic CCD is not implemented.
- **No rotational CCD**: Rotating bodies are not considered in TOI calculations. A rotating body may tunnel through geometry.
- **Substep exhaustion**: Complex scenarios with many repeated impacts may exhaust the CCD substep budget and fall back to discrete collision.

## Solver

### Impulse Solver
- **No warm starting**: Warm starting is disabled due to contact geometry validation requirements. This may reduce stacking stability.
- **Single iteration batch**: All constraints are solved in one batch. Per-contact iterative solving is not implemented.
- **No joint constraints**: The solver handles contact constraints only. Joint constraints (hinge, fixed, distance) are not implemented.

### Stability
- **Stacking**: Stacks of 50+ cubes may become unstable over long simulations.
- **Large mass ratios**: Collisions between bodies with very different masses (e.g., 0.01 kg vs 1000 kg) may produce unstable results.
- **Deep penetration**: Bodies that penetrate deeply (> 1m) may not be corrected properly.

## Performance

### Broadphase
- **Dynamic AABB Tree**: Implemented with O(log n) average query time, but worst case can degrade to O(n) for dense overlapping scenes.
- **No spatial partitioning**: Beyond the tree, there is no additional spatial hashing or grid-based culling.

### Multithreading
- **Single-threaded**: All simulation runs on a single thread. No parallel broadphase, narrowphase, or solver.

### Memory
- **No object pooling**: Each simulation step allocates new objects for contacts and constraints. This creates GC pressure for large scenes.
- **No memory benchmarks**: Allocation rates have not been measured.

## Sleeping System

- **Basic implementation**: Sleeping uses velocity thresholds and step counters. It does not use contact force thresholds.
- **No wake-up propagation**: When a sleeping body is woken, adjacent bodies are not automatically woken.

## Integration

### Rigid Body
- **No body removal**: `SimulationPipeline` does not support removing bodies after addition. Bodies can only be deactivated.
- **No compound bodies**: Each body has exactly one collision shape.

### Configuration
- **Fixed timestep only**: Variable timestep with interpolation is not implemented.
- **No sub-stepping**: The solver does not perform internal sub-steps beyond CCD.

## API

### Public API
- **No stable API guarantee**: The public API may change between releases.
- **No serialization**: Body and world state cannot be saved/loaded.

### Website Integration
- **WebSocket not implemented**: The `vakuum-website` module provides the Kotlin API only. The WebSocket protocol is planned but not implemented.
- **No server-side code**: There is no HTTP/WebSocket server. The adapter must be integrated with an existing server framework.

## Testing

### Coverage
- **No multithreading tests**: Since the engine is single-threaded, concurrent access is not tested.
- **No memory leak tests**: Long-running simulations are tested for NaN/Infinity, but not for memory leaks.

### Benchmarks
- **Single hardware**: Benchmarks have been run on one machine. Cross-hardware comparison is not available.
- **No GC measurement**: Garbage collection impact is not measured in benchmarks.

## Documentation

- **Incomplete KDoc**: Not all public APIs have KDoc comments.
- **No examples**: There are no standalone example projects demonstrating Vakuum usage.

## What's Next

The following limitations are planned for future milestones:

| Limitation | Planned Milestone |
|------------|-------------------|
| Joint constraints (hinge, fixed, distance) | V0.6 |
| Multithreading | V0.7 |
| Compound shapes | V0.6 |
| Concave mesh collision | V0.8 |
| Warm starting | V0.5 |
| Body removal | V0.4 |
| Serialization | V0.9 |
| WebSocket server | V0.9 |
| Object pooling | V0.5 |
