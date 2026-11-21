package io.vakuum.collision

import io.vakuum.math.Vector3

/**
 * Result of a narrowphase collision test.
 */
data class ContactManifold(
    /** Contact point in world space. */
    val contactPoint: Vector3,
    /**
     * Collision normal pointing in the push direction for bodyA (from bodyB toward bodyA).
     * The solver uses this to push bodyA along +normal and bodyB along -normal,
     * separating the two bodies.
     */
    val normal: Vector3,
    /** How deep the bodies penetrate each other. */
    val penetrationDepth: Double,
    /** Body A ID. Default -1 for backward compatibility. */
    val bodyAId: Int = -1,
    /** Body B ID. Default -1 for backward compatibility. */
    val bodyBId: Int = -1,
    /** Warm start impulse from the previous step. */
    val warmStartImpulse: Double = 0.0
)

/**
 * A pair of body IDs that may be colliding.
 */
data class CollisionPair(val bodyAId: Int, val bodyBId: Int)
