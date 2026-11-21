package io.vakuum.math

/**
 * Bounding sphere defined by a center and radius.
 */
data class BoundingSphere(val center: Vector3, val radius: Double) {

    init {
        require(radius >= 0.0) { "Radius must be non-negative"}
        MathUtils.validateFinite(radius, "radius")
    }

    companion object {
        /**
         * Create a bounding sphere from an AABB.
         */
        fun fromAABB(aabb: AABB): BoundingSphere {
            return BoundingSphere(aabb.center, aabb.extents.length)
        }

        /**
         * Create a bounding sphere from a point (zero radius).
         */
        fun fromPoint(point: Vector3): BoundingSphere {
            return BoundingSphere(point, 0.0)
        }
    }

    /**
     * Check if this sphere contains a point.
     */
    fun contains(point: Vector3): Boolean {
        val diff = point - center
        return diff.dot(diff) <= radius * radius
    }

    /**
     * Check if this sphere intersects another sphere.
     */
    fun intersects(other: BoundingSphere): Boolean {
        val diff = center - other.center
        val distSq = diff.dot(diff)
        val r = radius + other.radius
        return distSq <= r * r
    }

    /**
     * Check if this sphere fully contains another sphere.
     */
    fun contains(other: BoundingSphere): Boolean {
        val diff = center - other.center
        return diff.length <= radius - other.radius
    }

    /**
     * Create a union of this sphere and another.
     */
    fun union(other: BoundingSphere): BoundingSphere {
        val diff = other.center - center
        val dist = diff.length
        if (dist <= MathUtils.EPSILON) {
            // Same center
            return BoundingSphere(center, kotlin.math.max(radius, other.radius))
        }
        val mid = (center + other.center) * 0.5
        val newRadius = dist * 0.5 + kotlin.math.max(radius, other.radius)
        return BoundingSphere(mid, newRadius)
    }

    /**
     * Create a union of this sphere and a point.
     */
    fun union(point: Vector3): BoundingSphere {
        val diff = point - center
        val dist = diff.length
        if (dist <= radius + MathUtils.EPSILON) {
            return this // Point is already inside
        }
        val newCenter = (center + point) * 0.5
        val newRadius = (dist + radius) * 0.5
        return BoundingSphere(newCenter, newRadius)
    }

    /**
     * Expand this sphere by a scalar amount.
     */
    fun expand(amount: Double): BoundingSphere {
        return BoundingSphere(center, radius + amount)
    }

    override fun toString(): String = "BoundingSphere(center=$center, radius=$radius)"
}
