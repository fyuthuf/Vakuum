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

/**
 * Sphere collision shape defined by center (local space) and radius.
 */
data class SphereShape(val center: Vector3, val radius: Double) : CollisionShape {
    init {
        require(radius > 0.0) { "Sphere radius must be positive" }
        require(!radius.isNaN() && !radius.isInfinite()) { "Sphere radius must be finite" }
    }

    companion object {
        /** Create a sphere centered at the origin. */
        fun fromRadius(radius: Double): SphereShape = SphereShape(Vector3.ZERO, radius)
    }
}

/**
 * Capsule collision shape defined by two endpoints (local space) and radius.
 * The capsule is the volume swept by a sphere of the given radius along the segment p1→p2.
 */
data class CapsuleShape(val p1: Vector3, val p2: Vector3, val radius: Double) : CollisionShape {
    init {
        require(radius > 0.0) { "Capsule radius must be positive" }
        require(!radius.isNaN() && !radius.isInfinite()) { "Capsule radius must be finite" }
        require((p2 - p1).lengthSquared > 1e-10) { "Capsule endpoints must be distinct" }
    }

    /** Length of the capsule segment. */
    val length: Double get() = (p2 - p1).length

    /** Direction from p1 to p2. */
    val direction: Vector3 get() = (p2 - p1) / length

    /** Bounding sphere of the capsule (approximate). */
    val boundingSphere: io.vakuum.math.BoundingSphere by lazy {
        val center = (p1 + p2) * 0.5
        val halfLen = length * 0.5
        io.vakuum.math.BoundingSphere(center, halfLen + radius)
    }

    companion object {
        /** Create a capsule along the Y axis, centered at the origin. */
        fun fromHalfHeight(halfHeight: Double, radius: Double): CapsuleShape {
            require(halfHeight > 0.0) { "Capsule half-height must be positive" }
            return CapsuleShape(Vector3(0.0, -halfHeight, 0.0), Vector3(0.0, halfHeight, 0.0), radius)
        }
    }
}

/**
 * Convex hull collision shape defined by a list of vertices in local space.
 * The vertices define a convex polyhedron. The faces and edges are computed lazily.
 */
data class ConvexHullShape(val vertices: List<Vector3>) : CollisionShape {
    init {
        require(vertices.size >= 4) { "Convex hull must have at least 4 vertices" }
        require(vertices.none { it.x.isNaN() || it.y.isNaN() || it.z.isNaN() }) { "Convex hull vertices must not contain NaN" }
        // Check that not all vertices are coplanar
        val v0 = vertices[0]
        val v1 = vertices[1]
        val v2 = vertices[2]
        val v3 = vertices[3]
        val normal = (v1 - v0).cross(v2 - v0)
        require(!normal.isNearZero()) { "First 3 vertices must not be collinear" }
        val dist = normal.dot(v3 - v0)
        require(kotlin.math.abs(dist) > 1e-10) { "Vertices must not be coplanar" }
    }

    /** Bounding sphere of the convex hull (approximate). */
    val boundingSphere: io.vakuum.math.BoundingSphere by lazy {
        val center = vertices.fold(Vector3.ZERO) { acc, v -> acc + v } / vertices.size.toDouble()
        val maxDist = vertices.maxOf { (it - center).length }
        io.vakuum.math.BoundingSphere(center, maxDist)
    }

    companion object {
        /** Create a convex hull from a box with the given half-extents. */
        fun fromBox(halfExtents: Vector3): ConvexHullShape {
            val x = halfExtents.x
            val y = halfExtents.y
            val z = halfExtents.z
            // Order vertices so the first 4 are NOT coplanar
            return ConvexHullShape(listOf(
                Vector3(-x, -y, -z),  // v0: back-bottom-left
                Vector3(x, -y, -z),   // v1: back-bottom-right
                Vector3(x, y, -z),    // v2: back-top-right
                Vector3(-x, -y, z),   // v3: front-bottom-left (different Z)
                Vector3(-x, y, -z),   // v4: back-top-left
                Vector3(-x, y, z),    // v5: front-top-left
                Vector3(x, -y, z),    // v6: front-bottom-right
                Vector3(x, y, z)      // v7: front-top-right
            ))
        }
    }
}
