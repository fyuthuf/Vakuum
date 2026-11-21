# Vakuum — Collision Shapes

## Overview

Vakuum supports multiple collision shapes for rigid body collision detection. Each shape supports discrete collision detection and CCD (Continuous Collision Detection) against static obstacles.

## Supported Shapes

### BoxShape

Axis-aligned box defined by half-extents.

```kotlin
val shape = BoxShape(Vector3(0.5, 1.0, 0.5))  // 1×2×1 meter box
```

- **Collision pairs**: Box-Box, Box-Plane, Box-Sphere, Box-Capsule, Box-ConvexHull
- **CCD**: ✅ Against static plane and static box
- **AABB**: Direct from half-extents and transform

### SphereShape

Sphere defined by center offset and radius.

```kotlin
val shape = SphereShape(Vector3.ZERO, 0.5)  // 0.5m radius sphere
```

- **Collision pairs**: Sphere-Sphere, Sphere-Plane, Sphere-Box
- **CCD**: ✅ Against static plane, static box, static sphere
- **AABB**: Center ± radius

### CapsuleShape

Capsule defined by two endpoints and radius.

```kotlin
val shape = CapsuleShape(
    Vector3(0.0, -0.3, 0.0),  // End A (local space)
    Vector3(0.0, 0.3, 0.0),   // End B (local space)
    0.3                        // Radius
)
```

- **Collision pairs**: Capsule-Plane, Capsule-Sphere, Capsule-Box
- **CCD**: ✅ Against static plane, static box, static sphere
- **AABB**: Bounding sphere approximation

### ConvexHullShape

Convex hull defined by a list of vertices.

```kotlin
val shape = ConvexHullShape(
    listOf(
        Vector3(-0.5, -0.5, -0.5),
        Vector3(0.5, -0.5, -0.5),
        Vector3(0.5, -0.5, 0.5),
        Vector3(-0.5, -0.5, 0.5),
        Vector3(-0.5, 0.5, -0.5),
        Vector3(0.5, 0.5, -0.5),
        Vector3(0.5, 0.5, 0.5),
        Vector3(-0.5, 0.5, 0.5)
    )
)
```

- **Collision pairs**: ConvexHull-Plane, ConvexHull-Box, ConvexHull-ConvexHull
- **CCD**: ✅ Against static plane, static box, static convex hull
- **AABB**: Bounding sphere approximation
- **Validation**: Vertices must form a valid convex hull (coplanarity check)

### PlaneShape

Infinite plane defined by normal and distance.

```kotlin
val shape = PlaneShape(Vector3.UNIT_Y, 0.0)  // Ground plane
```

- **Collision pairs**: Plane-Box, Plane-Sphere, Plane-Capsule, Plane-ConvexHull
- **CCD**: ❌ Plane is always static; it serves as the obstacle, not the moving body
- **Note**: The ground plane (y=0) is handled implicitly by `SimulationPipeline`

## Collision Detection Algorithm

All narrowphase collision detection uses the **Separating Axis Theorem (SAT)**:

1. **Box-Box**: Tests face normals and cross-product axes (15 axes max)
2. **Box-Plane**: Tests plane normal against box vertices
3. **Sphere-Sphere**: Distance between centers vs sum of radii
4. **Sphere-Box**: Closest point on OBB to sphere center
5. **Capsule-Box**: Closest point on OBB to capsule segment
6. **ConvexHull-ConvexHull**: SAT with face normals and edge cross products

## Contact Manifold

Each collision produces a `ContactManifold`:

```kotlin
data class ContactManifold(
    val contactPoint: Vector3,      // World-space contact point
    val normal: Vector3,            // Push direction for bodyA (from bodyB toward bodyA)
    val penetrationDepth: Double,   // How deep the bodies overlap
    val bodyAId: Int,               // Body A identifier
    val bodyBId: Int,               // Body B identifier
    val warmStartImpulse: Double    // Cached impulse for warm starting
)
```

## Unsupported Shapes

The following shapes are **not yet implemented**:

- **Concave mesh**: TriMesh or heightmap collision
- **Cylinder**: Circular cylinder
- **Cone**: Circular cone
- **Compound**: Multiple shapes combined into one body

## Performance Considerations

- **BoxShape**: Fastest — direct SAT with fixed axes
- **SphereShape**: Very fast — simple distance check
- **CapsuleShape**: Moderate — segment sampling
- **ConvexHullShape**: Slowest — SAT with many axes, O(n²) edge cross products

For best performance, use BoxShape and SphereShape where possible. Reserve ConvexHullShape for complex geometry that cannot be approximated with simpler shapes.
