package io.vakuum.collision

import io.vakuum.math.Vector3

/**
 * Result of a narrowphase collision test.
 */
data class ContactManifold(
    /** Contact point in world space. */
    val contactPoint: Vector3,
    /** Collision normal pointing from bodyA to bodyB. */
    val normal: Vector3,
    /** How deep the bodies penetrate each other. */
    val penetrationDepth: Double
)

/**
 * A pair of body IDs that may be colliding.
 */
data class CollisionPair(val bodyAId: Int, val bodyBId: Int)
