# Vakuum — Constraint Solver

## Overview

Vakuum uses an **Iterative Sequential Impulse solver** (Projected Gauss-Seidel) for contact resolution. The solver processes contact constraints iteratively, applying impulses to separate colliding bodies and resolve friction.

## Architecture

```
ContactManifold (from Narrowphase)
    ↓
ContactConstraint (bodyA, bodyB, contact point, normal, restitution, friction)
    ↓
ImpulseSolver.solve()
    ├── Velocity iterations (Sequential Impulse)
    │   ├── Normal impulse (accumulated clamping)
    │   └── Friction impulse (Coulomb cone)
    └── Position correction (Baumgarte stabilization)
```

## Solver Settings

```kotlin
data class SolverSettings(
    val iterations: Int = 10,           // Velocity iterations per step
    val defaultRestitution: Double = 0.0,
    val defaultFriction: Double = 0.5,
    val penetrationPercentage: Double = 0.2,  // Baumgarte percentage
    val penetrationSlop: Double = 0.001       // Penetration tolerance
)
```

## Algorithm

### 1. Normal Impulse

For each contact, the solver calculates the relative velocity at the contact point:

```
v_rel = (vB + ωB × rB) - (vA + ωA × rA)
v_rel_normal = v_rel · n
```

The normal impulse delta is:

```
jN = -(1 + e) × v_rel_normal / effectiveMass
```

where `effectiveMass` includes inverse mass and inverse inertia contributions:

```
1/effectiveMass = 1/mA + 1/mB + (rA × n)²/IA + (rB × n)²/IB
```

The impulse is accumulated and clamped:

```
newImpulse = max(0, oldImpulse + deltaImpulse)
appliedImpulse = newImpulse - oldImpulse
```

This ensures no adhesive forces (impulse ≥ 0).

### 2. Friction Impulse

After the normal impulse, the tangential velocity is used to calculate friction:

```
v_rel_tangent = v_rel - (v_rel · n) × n
```

The desired friction impulse is clamped against the Coulomb cone:

```
|jT| ≤ μ × jN
```

where μ is the friction coefficient.

### 3. Position Correction (Baumgarte)

To correct penetration, a positional bias is applied:

```
correction = (penetration - slop) × penetrationPercentage / dt
```

Bodies are pushed apart proportionally to their inverse mass. Penetration below `slop` (0.001m) is ignored to prevent jitter.

## Restitution

Restitution is applied as a velocity bias on the first iteration only:

```
if (firstIteration && v_rel_normal < 0) {
    impulseMagnitude += restitution × jN
}
```

This ensures `restitution = 0` produces no bounce, and restitution is not applied repeatedly across iterations.

## Warm Starting

Warm starting is currently **disabled**. The contact cache stores impulses between frames, but warm starting requires careful contact geometry validation to avoid energy injection.

## Contact Caching

`SimulationPipeline` maintains a contact cache:

```kotlin
data class CachedContact(
    val normal: Vector3,
    val contactPoint: Vector3,
    val warmStartImpulse: Double
)
```

When bodies that were in contact last frame are still in contact, the cached normal and point are reused for stability.

## Known Limitations

- **Single contact point**: Each pair generates one contact point. Feature clipping for multiple contacts is not implemented.
- **No warm starting**: Disabled due to contact geometry validation requirements.
- **One-pass solving**: The solver processes all constraints in one batch, not per-contact.
- **No constraint mixing**: Contact constraints are solved separately from joint constraints.

## Performance

- **O(n × iterations)** per step, where n is the number of active contacts.
- Typical iteration count: 10.
- No allocations in the hot path (impulse calculation loop).
