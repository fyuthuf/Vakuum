package io.vakuum.collision

import io.vakuum.math.Vector3

/**
 * Sealed interface for collision shapes.
 */
sealed interface CollisionShape

/**
 * Box collision shape defined by half-extents.
 */
data class BoxShape(val halfExtents: Vector3) : CollisionShape {
    init {
        require(halfExtents.x > 0.0 && halfExtents.y > 0.0 && halfExtents.z > 0.0) {
            "Box half-extents must be positive"
        }
    }
}

/**
 * Plane collision shape defined by normal and distance from origin.
 * The plane equation is: n · x = d
 */
data class PlaneShape(val normal: Vector3, val distance: Double) : CollisionShape {
    init {
        require(!normal.isNearZero()) { "Plane normal must not be zero" }
        require(!distance.isNaN() && !distance.isInfinite()) { "Plane distance must be finite" }
    }
}
