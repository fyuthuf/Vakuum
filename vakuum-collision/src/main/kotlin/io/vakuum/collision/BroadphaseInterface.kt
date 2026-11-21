package io.vakuum.collision

import io.vakuum.dynamics.RigidBody
import io.vakuum.math.AABB
import io.vakuum.math.Vector3

/**
 * Interface for broadphase collision detection.
 *
 * A broadphase quickly identifies potential collision pairs, reducing the O(n²)
 * problem to a manageable set of candidates for the narrowphase.
 */
interface BroadphaseInterface {

    /**
     * Find all potential collision pairs from a list of bodies with their associated shapes.
     * @param bodies List of rigid bodies to check.
     * @param shapeMap Map from body ID to collision shape.
     * @param filterMap Optional collision filter map for filtering pairs.
     * @return List of collision pairs that may be overlapping.
     */
    fun findPairs(
        bodies: List<RigidBody>,
        shapeMap: Map<Int, CollisionShape>,
        filterMap: Map<Int, CollisionFilter>? = null
    ): List<CollisionPair>

    /**
     * Query all proxies that overlap with the given AABB.
     * @param aabb Query AABB.
     * @return List of body IDs whose AABBs overlap with the query AABB.
     */
    fun query(aabb: AABB): List<Int>

    /**
     * Query all proxies that the given ray intersects.
     * @param origin Ray origin.
     * @param direction Ray direction.
     * @param maxDistance Maximum ray distance.
     * @return List of body IDs whose AABBs the ray intersects.
     */
    fun queryRay(origin: Vector3, direction: Vector3, maxDistance: Double): List<Int>
}

/**
 * Result of a broadphase overlap query.
 */
data class BroadphaseQueryResult(
    /** Body IDs that overlap with the query. */
    val overlappingBodyIds: List<Int>
)
