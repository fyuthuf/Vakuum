# Vakuum Architecture

## Module Dependency Graph

```
vakuum-math (independent)
    │
    ├── vakuum-core ───┬── vakuum-api ─── vakuum-runtime
    │                  │
    │                  ├── vakuum-dynamics
    │                  ├── vakuum-collision
    │                  └── vakuum-constraints ─── vakuum-solver
    │
    └── vakuum-tests ──┘
```

## Module Responsibilities

| Module | Description | Dependencies |
|--------|-------------|-------------|
| vakuum-math | Mathematical primitives | None |
| vakuum-core | Physics world, lifecycle | vakuum-math |
| vakuum-api | Public interfaces | vakuum-core, vakuum-math |
| vakuum-dynamics | Rigid body dynamics (future) | vakuum-core |
| vakuum-collision | Collision detection (future) | vakuum-core |
| vakuum-constraints | Joints & constraints (future) | vakuum-core |
| vakuum-solver | Constraint solver (future) | vakuum-core, vakuum-constraints |
| vakuum-runtime | Execution & scheduling (future) | vakuum-api |
| vakuum-tests | Integration tests | vakuum-math, vakuum-core, vakuum-api |

## Design Decisions

1. **Double precision** — All math uses `Double` for numerical stability.
2. **Row-major matrices** — Simpler indexing, matches Kotlin idioms.
3. **Right-handed coordinate system** — X right, Y up, Z backward.
4. **Hamilton quaternions** — ij=k convention, matches Jolt/Bullet/Unreal.
5. **Exception-based errors** — Invalid operations throw `IllegalArgumentException`.
6. **No circular dependencies** — Strict unidirectional module dependencies.
