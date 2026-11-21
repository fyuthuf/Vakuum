# Vakuum V0.3.0 — M2 Narrowphase Audit Report

**Date:** 2026-11-21  
**Scope:** Collision detection algorithms, contact generation, solver integration, test coverage  
**Status:** Audit complete — no implementation changes made

---

## 1. Architecture Overview

The collision detection pipeline in Vakuum consists of the following components, executed in order during each simulation step:

```
SimulationPipeline.step()
  │
  ├─ 1. Integrate bodies (RigidBody.integrate())
  │
  ├─ 2. Broadphase (BroadphaseInterface.findPairs())
  │     ├─ BruteForceBroadphase — O(n²) AABB overlap, reference implementation
  │     └─ TreeBroadphase — Dynamic AABB Tree, O(n log n) average
  │
  ├─ 3. Box-Plane ground collisions (hardcoded in SimulationPipeline)
  │     └─ Narrowphase.testBoxPlane() → ImpulseSolver.solveContact()
  │
  └─ 4. Box-Box collisions (from broadphase pairs)
        └─ Narrowphase.testBoxBox() → ImpulseSolver.solveContact()
              └─ ImpulseSolver.solvePositionCorrection()
```

### Component Files

| Component | File | Lines | Purpose |
|-----------|------|-------|---------|
| Broadphase Interface | `BroadphaseInterface.kt` | 52 | Interface for broadphase implementations |
| Brute Force Broadphase | `Broadphase.kt` | 109 | O(n²) AABB pair detection |
| Tree Broadphase | `TreeBroadphase.kt` | 153 | Dynamic AABB Tree broadphase |
| Dynamic AABB Tree | `DynamicAABBTree.kt` | 513 | Binary tree with SAH insertion |
| Narrowphase | `Narrowphase.kt` | 172 | SAT for box-box and box-plane |
| ContactManifold | `ContactManifold.kt` | 27 | Collision result data class |
| CollisionShape | `CollisionShape.kt` | — | BoxShape and PlaneShape |
| ImpulseSolver | `ImpulseSolver.kt` | 208 | Normal impulse, friction, position correction |
| SimulationPipeline | `SimulationPipeline.kt` | 183 | Orchestrates the full pipeline |
| RigidBody | `RigidBody.kt` | 207 | Body state, integration, sleep |
| MassProperties | `MassProperties.kt` | 97 | Mass, inertia tensor |

### Data Flow

1. **SimulationPipeline** maintains a list of `RigidBody` instances and a `Map<Int, CollisionShape>`
2. **Broadphase** computes world-space AABBs from body transforms + shapes, then finds overlapping pairs
3. **Narrowphase** receives two bodies + their shapes and returns a `ContactManifold?`
4. **ImpulseSolver** applies the contact impulse and position correction directly to body velocities and positions
5. **Contact cache** stores the previous frame's contact normal, point, and warm start impulse

---

## 2. Confirmed Bugs

### BUG-1: Contact point calculation is incorrect in `testBoxBox()`

**File:** `Narrowphase.kt`, lines 66–67

**Issue:** The contact point formula is mathematically wrong:
```kotlin
val contactPoint = bodyA.transform.position +
    (normal * (projectOntoAxis(translation, normal) - minPenetration / 2.0))
```

This computes a point along the collision normal from body A's position, but does not account for the actual geometry of the colliding faces. The result is a point on the line between the two body centers, offset by half the penetration depth. This is **not** the actual contact point on the surface of either body.

**Impact:** The solver uses this contact point to compute `rA` and `rB` (vectors from body center to contact point), which affect the angular impulse. An incorrect contact point means incorrect rotational response. For axis-aligned boxes with centers near the contact, the error is small. For rotated boxes or off-center contacts, the angular impulse can be significantly wrong.

**Severity:** Medium — affects rotational dynamics; linear impulse is correct.

---

### BUG-2: `ImpulseSolver.solve()` is empty

**File:** `ImpulseSolver.kt`, lines 20–23

**Issue:** The batch `solve()` method exists but does nothing:
```kotlin
fun solve(bodies: List<RigidBody>, contacts: List<ContactManifold>, dt: Double) {
    // Batch solve is not yet fully implemented.
    // Use solveContact() and solvePositionCorrection() for individual contacts.
}
```

**Impact:** No current code calls this method, so it's dead code rather than a runtime bug. However, it creates a false impression that batch solving is available.

**Severity:** Low — dead code, no runtime impact.

---

### BUG-3: Ground plane is hardcoded in `SimulationPipeline`

**File:** `SimulationPipeline.kt`, lines 96–112

**Issue:** Box-plane collisions are handled separately from the broadphase/narrowphase pipeline. Every awake body with a BoxShape is tested against a hardcoded ground plane at Y=0:
```kotlin
val contact = narrowphase.testBoxPlane(body, shape, Vector3.UNIT_Y, 0.0)
```

Additionally, a temporary static ground body is created on every step:
```kotlin
val ground = io.vakuum.dynamics.rigidBody()
    .id(-1)
    .bodyType(BodyType.STATIC)
    .position(Vector3.ZERO)
    .mass(0.0)
    .build()
```

**Impact:** 
- Only one ground plane exists; no support for arbitrary planes
- A new RigidBody is allocated every step for every body that collides with the ground — this is a GC pressure issue
- The ground body has `id=-1` and `bodyBId=-1` in the contact manifold, which is inconsistent
- PlaneShape is defined but never used in the pipeline

**Severity:** Medium — limits functionality, causes unnecessary allocations.

---

### BUG-4: Contact normal direction may be inconsistent

**File:** `Narrowphase.kt`, lines 61–63

**Issue:** The normal orientation logic:
```kotlin
val direction = bestNormal.dot(translation)
val normal = if (direction < 0) bestNormal else -bestNormal
```

`translation = bodyB.position - bodyA.position`. The code attempts to ensure the normal points from A to B, but the logic is inverted: when `bestNormal.dot(translation) < 0`, the bestNormal points **away** from B, so it should be negated. The current code keeps it as-is. This means the normal sometimes points from B to A instead of A to B.

**Impact:** The solver checks `approachVelocity = -velAlongNormal` and skips resolution when bodies are separating. If the normal is inverted, the solver may resolve separating contacts or skip approaching contacts. In practice, the solver still works because the impulse magnitude is computed from the absolute approach velocity, but the direction of the applied impulse is wrong, pushing bodies toward each other instead of apart.

However, looking at the solver more carefully:
```kotlin
val impulse = normal * jN
bodyA.linearVelocity += impulse * invMassA
bodyB.linearVelocity -= impulse * invMassB
```

The solver pushes bodyA along +normal and bodyB along -normal. If the normal points from A to B, bodyA is pushed away from B and bodyB is pushed away from A — correct behavior. If the normal is inverted, both bodies are pushed toward each other — **this would cause bodies to accelerate into each other**.

**Severity:** High — if triggered, causes energy injection and instability.

**Mitigation:** The existing tests pass, which suggests the normal direction happens to be correct for the tested configurations (axis-aligned boxes). Rotated boxes may trigger this bug.

---

### BUG-5: Warm starting applies impulse without iteration

**File:** `ImpulseSolver.kt`, lines 46–58

**Issue:** Warm starting applies the previous frame's impulse directly to velocities before the main impulse calculation. However, the warm start impulse value stored in `ContactManifold.warmStartImpulse` is never set by the solver — it's always 0.0 by default. The `SimulationPipeline` stores `contact.warmStartImpulse` in the cache, but since the solver never writes the impulse magnitude back to the contact, the cached value is always 0.0.

**Impact:** Warm starting is effectively a no-op. The code structure is correct, but the data flow is broken: the solver computes `jN` locally but never returns it to be cached.

**Severity:** Low — warm starting was intended but never functional. No instability caused.

---

## 3. Potential Weaknesses

### W-1: Single contact point per manifold

`ContactManifold` holds exactly one contact point. Real physics engines generate multiple contact points (up to 3 for box-box) to provide stable resting contacts. A single point cannot prevent rotation around the contact, leading to potential jitter in stacked configurations.

**Current test coverage:** The stack test passes, but relies on friction and position correction rather than multiple contact points.

### W-2: No contact point clipping

The SAT algorithm finds the axis of minimum penetration but does not clip the colliding features to find the actual contact point(s). The contact point is approximated (see BUG-1).

### W-3: SAT does not handle degenerate boxes

If a box has zero extent on one axis (effectively a 2D plane), the cross product axes may be zero vectors. While `BoxShape` validates positive extents, a body could theoretically have a shape with very small extents, leading to numerical instability in the cross product axes.

### W-4: No penetration depth tolerance

`testBoxBox()` returns a contact for any positive overlap, no matter how small. A tolerance (e.g., 1e-6) would prevent spurious contacts from floating-point noise.

### W-5: `testBoxPlane()` contact point is the closest vertex projected onto the plane

The contact point calculation finds the vertex of the box closest to the plane, then projects it onto the plane. This is a reasonable approximation but doesn't account for the actual contact area. For a flat box resting on a plane, the contact point is one corner rather than the center of the contact face.

### W-6: Solver processes contacts sequentially without ordering

Contacts are solved in the order they appear in the broadphase result. The order affects the final velocities (Gauss-Seidel relaxation). While the broadphase results are sorted by body ID for determinism, the solver does not prioritize contacts by penetration depth or any other heuristic.

### W-7: No iterative solving in the pipeline

`SimulationPipeline` calls `solveContact()` once per contact. The `SolverSettings.iterations` field exists but is never used — there's no loop over all contacts multiple times. True iterative solving would loop over all contacts `iterations` times, converging to a more stable solution.

### W-8: PlaneShape is defined but unused

`PlaneShape` exists as a collision shape but is never used in the pipeline. Ground collisions use hardcoded parameters. This limits the ability to have arbitrary planes (walls, ramps, etc.).

### W-9: Broadphase AABB computation duplicates code

`BruteForceBroadphase.getAABB()` and `TreeBroadphase.getAABB()` are nearly identical. This code is duplicated across two files.

### W-10: No NaN/infinity validation in Narrowphase

If a body has NaN position or rotation, the SAT algorithm will propagate NaN through all calculations and return a ContactManifold with NaN values. The solver then applies NaN impulses.

---

## 4. Missing Test Coverage

### Critical gaps

| Missing Test | Why It Matters |
|--------------|----------------|
| **Rotated box-box collision** | SAT with rotated boxes is the most complex case; only one test exists for rotated box-plane |
| **Contact point accuracy** | No test verifies the contact point is on the surface of either body |
| **Normal direction consistency** | No test verifies the normal always points from A to B across various configurations |
| **Corner-to-corner contact** | Two boxes touching at a corner is a degenerate SAT case |
| **Edge-to-edge contact** | Two boxes touching along parallel edges |
| **Nearly parallel faces** | When two face normals are almost parallel, cross products are near-zero |
| **Very small penetration** | Floating-point precision near the collision boundary |
| **Large mass ratio** | e.g., 1000:1 mass ratio between colliding bodies |
| **High-speed collision** | Bodies moving fast relative to timestep |
| **Solver iterative convergence** | No test verifies that multiple iterations improve the solution |
| **Warm starting data flow** | No test verifies the impulse is actually cached and reused |
| **Multiple contacts on one body** | No test for a body colliding with multiple bodies simultaneously |
| **Position correction stability** | No test verifies repeated position correction doesn't cause oscillation |

### Existing test summary

| Test File | Test Count | Coverage |
|-----------|-----------|----------|
| `NarrowphaseTest.kt` | 10 | Basic box-box and box-plane, one rotated box-plane |
| `CollisionShapeTest.kt` | 5 | Shape validation |
| `ImpulseSolverTest.kt` | 10 | Normal impulse, restitution, friction, position correction |
| `IntegrationTest.kt` | 8 | Full pipeline: falling cube, multi-cube, stack, stress, reproducibility |

**Total collision-related tests:** 33

---

## 5. Integration Analysis

### Broadphase → Narrowphase

- **Pair format:** `CollisionPair(bodyAId, bodyBId)` — consistent across both broadphase implementations
- **Body lookup:** `SimulationPipeline` uses `bodies.find { it.id == pair.bodyAId }` — O(n) lookup per pair. For 100 bodies with many pairs, this is inefficient but correct.
- **Shape lookup:** `shapes[pair.bodyAId]` — O(1) map lookup, correct.

### Narrowphase → Solver

- **ContactManifold:** Contains `bodyAId` and `bodyBId` (added in V0.3.0), but the solver doesn't use them — it receives body references directly.
- **Warm start:** `warmStartImpulse` field exists but is always 0.0 (see BUG-5).

### Solver → RigidBody

- **Direct mutation:** The solver modifies `body.linearVelocity`, `body.angularVelocity`, and `body.transform` directly. This works but bypasses any encapsulation.
- **Static body handling:** The solver checks `invMassA + invMassB < 1e-10` and returns early. This correctly prevents moving static bodies.

---

## 6. Performance Considerations

### Current bottlenecks

1. **Ground plane allocation:** A new `RigidBody` is created every step for every body colliding with the ground. For 100 bodies, this is 100 allocations per step.
2. **Body lookup:** `bodies.find { it.id == ... }` is O(n) per pair. With O(n²) pairs in the worst case, this is O(n³).
3. **No iterative solving:** Single-pass contact solving is less stable than iterative solving, requiring more position correction.
4. **Contact cache:** Uses `MutableMap<Pair<Int, Int>, CachedContact>` with `clear()` every step. This allocates a new map entry for each contact every frame.

### Dynamic AABB Tree performance

The tree implementation is correct and tested. Key observations:
- SAH-based insertion provides good tree quality
- The `findBestSubtree` method has a depth limit of 64 iterations, preventing infinite loops
- Tree resizing (doubling capacity) is implemented with proper used-node tracking

---

## 7. Proposed Implementation Plan

### Priority 1: Bug Fixes

| Priority | Bug | Fix |
|----------|-----|-----|
| P0 | BUG-4: Normal direction | Fix the normal orientation logic in `testBoxBox()`; add tests with rotated boxes |
| P1 | BUG-1: Contact point | Implement proper contact point calculation using feature clipping or closest-point-on-face |
| P1 | BUG-3: Hardcoded ground | Refactor to use PlaneShape; remove temporary body allocation |
| P2 | BUG-5: Warm starting | Have the solver return the impulse magnitude; cache it in the pipeline |

### Priority 2: Narrowphase Improvements

| Task | Description |
|------|-------------|
| Multiple contact points | Extend `ContactManifold` to hold a list of contact points (up to 3 for box-box) |
| Contact point clipping | Implement feature clipping for box-box to find actual contact points |
| Penetration tolerance | Add a small tolerance (1e-6) to prevent spurious contacts |
| NaN validation | Add NaN checks in Narrowphase methods |

### Priority 3: Solver Improvements

| Task | Description |
|------|-------------|
| Iterative solving | Implement the loop over all contacts for `iterations` passes |
| Return impulse magnitude | Have `solveContact()` return the normal impulse for caching |
| Remove dead code | Remove or implement the `solve()` batch method |

### Priority 4: Test Expansion

| Test Category | Count |
|--------------|-------|
| Rotated box-box collisions | ~5 |
| Contact point accuracy | ~3 |
| Normal direction consistency | ~3 |
| Degenerate cases (corner, edge, nearly parallel) | ~5 |
| Large mass ratio | ~2 |
| Iterative solver convergence | ~2 |
| Warm starting verification | ~2 |
| **Total new tests** | **~22** |

---

## 8. Numerical Stability Assessment

| Aspect | Status | Notes |
|--------|--------|-------|
| Fixed timestep | ✅ | 1/60s fixed, no variable dt |
| NaN propagation | ⚠️ | RigidBody.hasNaN() exists but Narrowphase doesn't check |
| Infinity handling | ⚠️ | No explicit infinity checks in collision detection |
| Zero-mass bodies | ✅ | Static bodies have inverseMass=0; solver checks for this |
| Singular inertia | ⚠️ | Diagonal inertia tensor avoids inversion issues, but no validation |
| Quaternion normalization | ⚠️ | No re-normalization after integration; drift possible over long simulations |
| Baumgarte stabilization | ✅ | Penetration correction uses 0.2 percentage with 0.001m slop |

---

## 9. Summary

### What Works Well

- The SAT algorithm correctly detects box-box and box-plane overlaps
- The impulse solver produces physically plausible results for simple scenarios
- The Dynamic AABB Tree is correct and well-tested
- Integration tests confirm the full pipeline works for falling cubes, stacks, and stress tests
- Determinism is verified — identical inputs produce identical outputs

### What Needs Attention

- **Contact point accuracy** is the largest gap: the current approximation works for axis-aligned boxes but degrades with rotation
- **Normal direction** may be inverted for some configurations, potentially causing energy injection
- **Iterative solving** is not implemented despite the `iterations` setting
- **Warm starting** infrastructure exists but doesn't function due to a data flow gap
- **Test coverage** for rotated boxes and edge cases is minimal

### Risk Assessment

The current implementation is **stable for the tested scenarios** (axis-aligned falling cubes, simple stacks) but has **untested edge cases** that could cause instability:
- Rotated box collisions may produce incorrect normals or contact points
- Long-running simulations may accumulate quaternion drift
- High-speed collisions are not tested and may tunnel

### Recommended Next Steps

1. Fix BUG-4 (normal direction) — highest risk
2. Add rotated box-box tests to verify BUG-4 and assess BUG-1 impact
3. Implement proper contact point calculation
4. Enable iterative solving in the pipeline
5. Fix warm starting data flow
6. Add the ~22 missing tests identified above
