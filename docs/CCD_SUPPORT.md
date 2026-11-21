# Vakuum — CCD Support Matrix

## Overview

Continuous Collision Detection (CCD) prevents fast-moving bodies from tunneling through collision geometry between discrete simulation steps.

## Supported CCD Shape Pairs

| Moving Body | Static Obstacle | TOI Method | Status |
|-------------|----------------|------------|--------|
| Box | Ground Plane | Analytical | ✅ Implemented |
| Box | Static Box | Binary search | ✅ Implemented |
| Sphere | Ground Plane | Analytical | ✅ Implemented |
| Sphere | Static Box | Binary search | ✅ Implemented |
| Sphere | Static Sphere | Quadratic | ✅ Implemented |
| Capsule | Ground Plane | Analytical | ✅ Implemented |
| Capsule | Static Box | Binary search | ✅ Implemented |
| Capsule | Static Sphere | Binary search | ✅ Implemented |
| ConvexHull | Ground Plane | Analytical | ✅ Implemented |
| ConvexHull | Static Box | Binary search | ✅ Implemented |
| ConvexHull | Static ConvexHull | Binary search | ✅ Implemented |

## Unsupported CCD Pairs

| Moving Body | Static Obstacle | Reason |
|-------------|----------------|--------|
| Any | Dynamic body | Dynamic vs dynamic CCD not implemented |
| Any | Rotating body | Rotational CCD not implemented |
| Plane | Any | Plane is always static |

## Configuration

CCD is controlled via `PhysicsConfiguration`:

```kotlin
val config = PhysicsConfiguration(
    ccdEnabled = true,              // Enable CCD
    ccdVelocityThreshold = 10.0,    // Min velocity for CCD (m/s)
    ccdMaxSubsteps = 10,            // Max CCD substeps per step
    ccdToiTolerance = 0.001         // Min TOI fraction
)
```

## Algorithm

1. **Fast-body detection**: Bodies with velocity > `ccdVelocityThreshold` use CCD.
2. **Swept AABB**: Quick broadphase check using the swept volume from start to end position.
3. **TOI calculation**: Precise time of impact using analytical or binary search methods.
4. **Conservative advancement**: Bodies are advanced to the TOI, collision is resolved, and the remaining time is processed.
5. **Substep limit**: If `ccdMaxSubsteps` is reached, the simulation falls back to discrete collision.

## Known Limitations

- **No dynamic vs dynamic CCD**: Only moving body vs static obstacle is supported.
- **No rotational CCD**: Rotating bodies are not considered in TOI calculations.
- **Binary search precision**: Binary search TOI methods use 20 iterations with 1e-6 tolerance.
- **Substep exhaustion**: Very complex collision scenarios may exhaust the substep budget.

## Diagnostics

CCD diagnostics are available via `SimulationPipeline.ccdDiagnostics`:

```kotlin
data class CcdDiagnostics(
    val ccdSteps: Int,              // CCD substeps in last step
    val toiQueries: Int,            // TOI queries performed
    val toiIterations: Int,         // Max binary search iterations
    val maxSubstepsReached: Boolean, // Whether budget was exhausted
    val ccdFallbackCount: Int,      // Fallback to discrete collision
    val maximumPenetration: Double,  // Max penetration during CCD
    val averageCCDSubsteps: Double   // Rolling average
)
```
