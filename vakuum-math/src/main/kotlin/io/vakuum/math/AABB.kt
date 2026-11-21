package io.vakuum.math

/**
 * Axis-Aligned Bounding Box defined by minimum and maximum corners.
 */
data class AABB(val min: Vector3, val max: Vector3) {

    init {
        require(min.x <= max.x + MathUtils.EPSILON) { "min.x must be <= max.x"}
        require(min.y <= max.y + MathUtils.EPSILON) { "min.y must be <= max.y"}
        require(min.z <= max.z + MathUtils.EPSILON) { "min.z must be <= max.z"}
    }

    companion object {
        /**
         * Create an AABB from a center and extents.
         */
        fun fromCenterExtents(center: Vector3, extents: Vector3): AABB {
            return AABB(center - extents, center + extents)
        }

        /**
         * Create an AABB from a single point.
         */
        fun fromPoint(point: Vector3): AABB {
            return AABB(point, point)
        }
    }

    /** Center of the AABB. */
    val center: Vector3 get() = (min + max) * 0.5

    /** Extents (half-size) of the AABB. */
    val extents: Vector3 get() = (max - min) * 0.5

    /** Size (full dimensions) of the AABB. */
    val size: Vector3 get() = max - min

    /**
     * Check if this AABB contains a point.
     */
    fun contains(point: Vector3): Boolean {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z
    }

    /**
     * Check if this AABB intersects another AABB.
     */
    fun intersects(other: AABB): Boolean {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y &&
               min.z <= other.max.z && max.z >= other.min.z
    }

    /**
     * Check if this AABB fully contains another AABB.
     */
    fun contains(other: AABB): Boolean {
        return min.x <= other.min.x && max.x >= other.max.x &&
               min.y <= other.min.y && max.y >= other.max.y &&
               min.z <= other.min.z && max.z >= other.max.z
    }

    /**
     * Create a union of this AABB and another.
     */
    fun union(other: AABB): AABB {
        return AABB(
            this.min.min(other.min),
            this.max.max(other.max)
        )
    }

    /**
     * Create a union of this AABB and a point.
     */
    fun union(point: Vector3): AABB {
        return AABB(
            this.min.min(point),
            this.max.max(point)
        )
    }

    /**
     * Expand this AABB by a scalar amount in all directions.
     */
    fun expand(amount: Double): AABB {
        return AABB(min - Vector3.ONE * amount, max + Vector3.ONE * amount)
    }

    /**
     * Expand this AABB by vector amounts.
     */
    fun expand(amount: Vector3): AABB {
        return AABB(min - amount, max + amount)
    }

    /**
     * Check if this AABB intersects a bounding sphere.
     */
    fun intersects(sphere: BoundingSphere): Boolean {
        // Find the closest point on the AABB to the sphere center
        val closest = Vector3(
            kotlin.math.max(min.x, kotlin.math.min(sphere.center.x, max.x)),
            kotlin.math.max(min.y, kotlin.math.min(sphere.center.y, max.y)),
            kotlin.math.max(min.z, kotlin.math.min(sphere.center.z, max.z))
        )
        val diff = closest - sphere.center
        return diff.dot(diff) <= sphere.radius * sphere.radius
    }

    override fun toString(): String = "AABB(min=$min, max=$max)"
}
