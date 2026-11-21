package io.vakuum.math

/**
 * Ray defined by an origin and a direction.
 *
 * A point on the ray is given by: origin + direction * t, where t >= 0.
 */
data class Ray(val origin: Vector3, val direction: Vector3) {

    init {
        require(!direction.isNearZero()) { "Ray direction must not be a zero vector"}
    }

    /**
     * Get the normalized direction.
     */
    val normalizedDirection: Vector3 get() = direction.normalize()

    /**
     * Get a point on the ray at parameter t.
     */
    fun pointAt(t: Double): Vector3 = origin + direction * t

    /**
     * Test intersection with a plane.
     * Returns the parameter t if the ray intersects the plane, or null if parallel.
     */
    fun intersectPlane(plane: Plane): Double? {
        val denom = plane.normal.dot(normalizedDirection)
        if (MathUtils.isNearZero(denom)) {
            return null // Ray is parallel to the plane
        }
        val t = -(plane.normal.dot(origin) + plane.distance) / denom
        return if (t >= 0.0) t else null
    }

    /**
     * Test intersection with an AABB.
     * Returns the closest intersection distance, or null if no intersection.
     */
    fun intersectAABB(aabb: AABB): Double? {
        val dir = normalizedDirection
        val invDir = Vector3(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z)

        var tmin = 0.0
        var tmax = Double.POSITIVE_INFINITY

        // X axis
        if (MathUtils.isNearZero(invDir.x)) {
            if (origin.x < aabb.min.x || origin.x > aabb.max.x) return null
        } else {
            var t1 = (aabb.min.x - origin.x) * invDir.x
            var t2 = (aabb.max.x - origin.x) * invDir.x
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
            }
            tmin = kotlin.math.max(tmin, t1)
            tmax = kotlin.math.min(tmax, t2)
            if (tmin > tmax) return null
        }

        // Y axis
        if (MathUtils.isNearZero(invDir.y)) {
            if (origin.y < aabb.min.y || origin.y > aabb.max.y) return null
        } else {
            var t1 = (aabb.min.y - origin.y) * invDir.y
            var t2 = (aabb.max.y - origin.y) * invDir.y
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
            }
            tmin = kotlin.math.max(tmin, t1)
            tmax = kotlin.math.min(tmax, t2)
            if (tmin > tmax) return null
        }

        // Z axis
        if (MathUtils.isNearZero(invDir.z)) {
            if (origin.z < aabb.min.z || origin.z > aabb.max.z) return null
        } else {
            var t1 = (aabb.min.z - origin.z) * invDir.z
            var t2 = (aabb.max.z - origin.z) * invDir.z
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
            }
            tmin = kotlin.math.max(tmin, t1)
            tmax = kotlin.math.min(tmax, t2)
            if (tmin > tmax) return null
        }

        return if (tmax >= 0.0) tmin else null
    }

    /**
     * Test intersection with a bounding sphere.
     * Returns the closest intersection distance, or null if no intersection.
     */
    fun intersectSphere(sphere: BoundingSphere): Double? {
        val dir = normalizedDirection
        val oc = origin - sphere.center
        val b = oc.dot(dir)
        val c = oc.dot(oc) - sphere.radius * sphere.radius

        if (c > 0.0 && b > 0.0) return null

        val disc = b * b - c
        if (disc < 0.0) return null

        val t = -b - kotlin.math.sqrt(disc)
        return if (t >= 0.0) t else {
            val t2 = -b + kotlin.math.sqrt(disc)
            if (t2 >= 0.0) t2 else null
        }
    }

    override fun toString(): String = "Ray(origin=$origin, direction=$direction)"
}
