package io.vakuum.math

/**
 * Plane defined by a normal vector and distance from origin.
 *
 * The plane equation is: normal · x + distance = 0
 * where normal is a unit vector.
 */
data class Plane(val normal: Vector3, val distance: Double) {

    init {
        MathUtils.validateFinite(distance, "distance")
        require(!normal.isNearZero()) { "Plane normal must not be a zero vector"}
    }

    companion object {
        /**
         * Create a plane from a point and a normal.
         */
        fun fromPointNormal(point: Vector3, normal: Vector3): Plane {
            require(!normal.isNearZero()) { "Plane normal must not be a zero vector"}
            val n = normal.normalize()
            return Plane(n, -n.dot(point))
        }

        /**
         * Create a plane from three points.
         */
        fun fromThreePoints(p1: Vector3, p2: Vector3, p3: Vector3): Plane {
            val n = (p2 - p1).cross(p3 - p1)
            require(!n.isNearZero()) { "Three points are collinear, cannot form a plane"}
            return fromPointNormal(p1, n)
        }

        /**
         * XY plane (Z = 0), normal pointing in +Z direction.
         */
        val XY_PLANE = Plane(Vector3.UNIT_Z, 0.0)

        /**
         * XZ plane (Y = 0), normal pointing in +Y direction.
         */
        val XZ_PLANE = Plane(Vector3.UNIT_Y, 0.0)

        /**
         * YZ plane (X = 0), normal pointing in +X direction.
         */
        val YZ_PLANE = Plane(Vector3.UNIT_X, 0.0)
    }

    /**
     * Classify a point relative to this plane.
     * Returns positive if the point is on the positive side,
     * negative if on the negative side, zero if on the plane.
     */
    fun classifyPoint(point: Vector3): Double {
        return normal.dot(point) + distance
    }

    /**
     * Check if a point is on the positive side of the plane.
     */
    fun isPointAbove(point: Vector3): Boolean {
        return classifyPoint(point) > 0.0
    }

    /**
     * Check if a point is on the negative side of the plane.
     */
    fun isPointBelow(point: Vector3): Boolean {
        return classifyPoint(point) < 0.0
    }

    /**
     * Check if a point is on the plane (within epsilon).
     */
    fun isPointOnPlane(point: Vector3): Boolean {
        return kotlin.math.abs(classifyPoint(point)) < MathUtils.EPSILON
    }

    /**
     * Project a point onto the plane.
     */
    fun projectPoint(point: Vector3): Vector3 {
        val d = classifyPoint(point)
        return point - normal * d
    }

    override fun toString(): String = "Plane(normal=$normal, distance=$distance)"
}
