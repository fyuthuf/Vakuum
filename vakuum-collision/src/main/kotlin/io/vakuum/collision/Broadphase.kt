package io.vakuum.collision

import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Vector3

/**
 * Broadphase collision detection using simple O(n²) AABB pair detection.
 * Suitable for small simulations (<200 bodies).
 */
class Broadphase {

    /**
     * Find all potential collision pairs from a list of bodies with their associated shapes.
     * Uses AABB overlap test for broadphase filtering.
     * @param bodies List of rigid bodies to check.
     * @param shapeMap Map from body ID to collision shape.
     */
    fun findPairs(bodies: List<RigidBody>, shapeMap: Map<Int, CollisionShape>): List<CollisionPair> {
        val pairs = mutableListOf<CollisionPair>()
        val n = bodies.size

        for (i in 0 until n) {
            for (j in i + 1 until n) {
                val a = bodies[i]
                val b = bodies[j]

                // Get AABBs for both bodies using their shapes
                val aAABB = getAABB(a, shapeMap[a.id])
                val bAABB = getAABB(b, shapeMap[b.id])

                if (aAABB != null && bAABB != null && aAABB.intersects(bAABB)) {
                    pairs.add(CollisionPair(a.id, b.id))
                }
            }
        }

        return pairs
    }

    /**
     * Get the AABB of a body based on its collision shape.
     * For box shapes, computes the oriented bounding box projected onto world axes.
     * Falls back to a default extent if no shape is provided.
     */
    private fun getAABB(body: RigidBody, shape: CollisionShape?): io.vakuum.math.AABB? {
        val center = body.transform.position

        val extent = when (shape) {
            is BoxShape -> {
                // For rotated boxes, compute the AABB by projecting the OBB onto world axes
                val worldAxes = listOf(
                    body.transform.rotation.rotateVector(Vector3.UNIT_X),
                    body.transform.rotation.rotateVector(Vector3.UNIT_Y),
                    body.transform.rotation.rotateVector(Vector3.UNIT_Z)
                )
                val halfExtents = listOf(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z)
                
                // Project each world axis scaled by half-extent onto world axes
                var ax = 0.0
                var ay = 0.0
                var az = 0.0
                for (i in worldAxes.indices) {
                    ax += kotlin.math.abs(worldAxes[i].x) * halfExtents[i]
                    ay += kotlin.math.abs(worldAxes[i].y) * halfExtents[i]
                    az += kotlin.math.abs(worldAxes[i].z) * halfExtents[i]
                }
                Vector3(ax, ay, az)
            }
            is PlaneShape -> {
                // Planes are typically large; use a large default extent
                Vector3(100.0, 100.0, 100.0)
            }
            null -> {
                // Fallback: default extent
                Vector3(0.5, 0.5, 0.5)
            }
        }

        return io.vakuum.math.AABB(center - extent, center + extent)
    }
}
