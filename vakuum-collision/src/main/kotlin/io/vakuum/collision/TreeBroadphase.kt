package io.vakuum.collision

import io.vakuum.dynamics.RigidBody
import io.vakuum.math.AABB
import io.vakuum.math.Vector3

/**
 * Broadphase implementation using a Dynamic AABB Tree.
 *
 * Much more efficient than brute-force for large numbers of bodies.
 * Typical performance: O(n log n) vs O(n²) for brute-force.
 */
class TreeBroadphase(initialCapacity: Int = 256) : BroadphaseInterface {

    private val tree = DynamicAABBTree(initialCapacity)
    
    /** Maps body ID to proxy ID in the tree. */
    private val proxyMap = mutableMapOf<Int, Int>()

    /**
     * Find all potential collision pairs from a list of bodies with their associated shapes.
     * @param bodies List of rigid bodies to check.
     * @param shapeMap Map from body ID to collision shape.
     * @param filterMap Optional collision filter map for filtering pairs.
     */
    override fun findPairs(
        bodies: List<RigidBody>,
        shapeMap: Map<Int, CollisionShape>,
        filterMap: Map<Int, CollisionFilter>?
    ): List<CollisionPair> {
        // Update tree with current body positions
        val currentBodyIds = bodies.map { it.id }.toSet()
        
        // Remove bodies no longer in the simulation
        val toRemove = proxyMap.keys - currentBodyIds
        for (bodyId in toRemove) {
            val proxyId = proxyMap.remove(bodyId)
            if (proxyId != null) {
                tree.remove(proxyId)
            }
        }

        // Insert or update bodies
        for (body in bodies) {
            val shape = shapeMap[body.id]
            val aabb = getAABB(body, shape) ?: continue
            
            val existingProxy = proxyMap[body.id]
            if (existingProxy != null) {
                tree.update(existingProxy, aabb)
            } else {
                val proxyId = tree.insert(aabb, body.id)
                proxyMap[body.id] = proxyId
            }
        }

        // Find all overlapping pairs
        val pairs = mutableListOf<CollisionPair>()
        val seenPairs = mutableSetOf<Pair<Int, Int>>()
        
        for (body in bodies) {
            val shape = shapeMap[body.id]
            val aabb = getAABB(body, shape) ?: continue
            
            val overlapping = tree.query(aabb)
            for (otherId in overlapping) {
                if (otherId <= body.id) continue // avoid duplicates and self
                
                // Check collision filters
                if (filterMap != null) {
                    val filterA = filterMap[body.id] ?: CollisionFilter.DEFAULT
                    val filterB = filterMap[otherId] ?: CollisionFilter.DEFAULT
                    if (!CollisionFilter.shouldCollide(filterA, filterB)) {
                        continue
                    }
                }
                
                val pair = Pair(body.id, otherId)
                if (seenPairs.add(pair)) {
                    pairs.add(CollisionPair(body.id, otherId))
                }
            }
        }

        // Sort for deterministic ordering
        pairs.sortWith(compareBy({ it.bodyAId }, { it.bodyBId }))

        return pairs
    }

    override fun query(aabb: AABB): List<Int> {
        return tree.query(aabb)
    }

    override fun queryRay(origin: Vector3, direction: Vector3, maxDistance: Double): List<Int> {
        return tree.queryRay(origin, direction, maxDistance)
    }

    /**
     * Get the AABB of a body based on its collision shape.
     */
    fun getAABB(body: RigidBody, shape: CollisionShape?): AABB? {
        val center = body.transform.position

        val extent = when (shape) {
            is BoxShape -> {
                val worldAxes = listOf(
                    body.transform.rotation.rotateVector(Vector3.UNIT_X),
                    body.transform.rotation.rotateVector(Vector3.UNIT_Y),
                    body.transform.rotation.rotateVector(Vector3.UNIT_Z)
                )
                val halfExtents = listOf(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z)
                
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
                Vector3(100.0, 100.0, 100.0)
            }
            is SphereShape -> {
                Vector3(shape.radius, shape.radius, shape.radius)
            }
            is CapsuleShape -> {
                val localP1 = shape.p1
                val localP2 = shape.p2
                val dx = kotlin.math.abs(localP2.x - localP1.x)
                val dy = kotlin.math.abs(localP2.y - localP1.y)
                val dz = kotlin.math.abs(localP2.z - localP1.z)
                Vector3(dx + shape.radius, dy + shape.radius, dz + shape.radius)
            }
            is ConvexHullShape -> {
                shape.boundingSphere.radius.let { r -> Vector3(r, r, r) }
            }
            null -> {
                Vector3(0.5, 0.5, 0.5)
            }
        }

        return AABB(center - extent, center + extent)
    }

    /**
     * Clear all proxies from the tree.
     */
    fun clear() {
        tree.clear()
        proxyMap.clear()
    }

    /**
     * Get the tree height for diagnostics.
     */
    fun getTreeHeight(): Int = tree.getTreeHeight()

    /**
     * Get the number of proxies in the tree.
     */
    fun getProxyCount(): Int = tree.proxyCount
}
