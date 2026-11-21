package io.vakuum.collision

import io.vakuum.math.AABB
import io.vakuum.math.Vector3

/**
 * Dynamic AABB Tree for efficient broadphase collision detection.
 *
 * Uses a binary tree structure where each internal node contains an AABB
 * that encloses its children. Leaf nodes contain proxies (body references).
 *
 * Based on the approach described in Jolt Physics and Bullet Physics,
 * but independently implemented.
 */
class DynamicAABBTree {

    /** Maximum number of proxies in the tree. */
    private var capacity: Int

    /** Tree nodes. */
    private var nodes: Array<TreeNode>

    /** Free list for O(1) allocation. */
    private var freeList: IntArray

    /** Number of free slots (top of stack). */
    private var freeListCount: Int

    /** Root node index, or -1 if the tree is empty. */
    private var root: Int

    /** Number of active proxies. */
    var proxyCount: Int = 0
        private set

    /**
     * Create a new Dynamic AABB Tree.
     * @param initialCapacity Expected maximum number of proxies.
     */
    constructor(initialCapacity: Int = 256) {
        require(initialCapacity > 0) { "Initial capacity must be positive" }
        this.capacity = initialCapacity
        // Binary tree needs up to 4*n-3 nodes for n leaves
        val nodeCount = 4 * initialCapacity
        this.nodes = Array(nodeCount) { TreeNode() }
        this.freeList = IntArray(nodeCount)
        this.root = -1  // -1 means empty tree
        this.proxyCount = 0
        // All nodes are free initially
        this.freeListCount = nodeCount
        for (i in 0 until nodeCount) {
            freeList[i] = i
        }
    }

    /**
     * Insert a new proxy with the given AABB.
     * @param aabb Axis-aligned bounding box.
     * @param bodyId Body ID associated with this proxy.
     * @return Proxy ID for later update/removal.
     */
    fun insert(aabb: AABB, bodyId: Int): Int {
        val leafIndex = allocateNode()
        nodes[leafIndex] = TreeNode(
            aabb = aabb,
            isLeaf = true,
            bodyId = bodyId,
            parent = -1,
            leftChild = -1,
            rightChild = -1,
            subIndex = -1
        )

        proxyCount++

        // If tree is empty, the new leaf becomes the root
        if (root == -1) {
            root = leafIndex
            return leafIndex
        }

        // Find the best subtree to insert into
        val foundIndex = findBestSubtree(leafIndex)
        val foundParent = nodes[foundIndex].parent

        // Create a new parent for the found index and the new leaf
        val newParent = allocateNode()
        nodes[newParent].parent = foundParent
        nodes[newParent].leftChild = foundIndex
        nodes[newParent].rightChild = leafIndex
        nodes[foundIndex].parent = newParent
        nodes[leafIndex].parent = newParent

        // Calculate the bounding box of the new parent
        nodes[newParent].aabb = nodes[nodes[newParent].leftChild].aabb.union(
            nodes[nodes[newParent].rightChild].aabb
        )
        nodes[newParent].isLeaf = false
        nodes[newParent].bodyId = -1
        nodes[newParent].subIndex = -1

        if (foundParent == -1) {
            // The found node was the root; newParent becomes the root
            root = newParent
        } else {
            // Replace foundIndex with newParent in the foundParent's children
            if (nodes[foundParent].leftChild == foundIndex) {
                nodes[foundParent].leftChild = newParent
            } else {
                nodes[foundParent].rightChild = newParent
            }

            // Walk up from foundParent, updating AABBs only
            var currentIndex = foundParent
            while (currentIndex != -1) {
                val p = nodes[currentIndex].parent
                if (p == -1) break
                nodes[p].aabb = nodes[nodes[p].leftChild].aabb.union(
                    nodes[nodes[p].rightChild].aabb
                )
                currentIndex = p
            }
        }

        return leafIndex
    }

    /**
     * Remove a proxy from the tree.
     * @param proxyId Proxy ID returned by insert().
     * @return True if the proxy was found and removed.
     */
    fun remove(proxyId: Int): Boolean {
        if (proxyId < 0 || proxyId >= nodes.size) return false
        if (!nodes[proxyId].isLeaf) return false

        val parent = nodes[proxyId].parent
        if (parent == -1) {
            // This was the root — tree becomes empty
            root = -1
        } else {
            val sibling = if (nodes[parent].leftChild == proxyId) {
                nodes[parent].rightChild
            } else {
                nodes[parent].leftChild
            }

            // Replace parent with sibling
            val grandParent = nodes[parent].parent
            nodes[sibling].parent = grandParent

            if (grandParent != -1) {
                if (nodes[grandParent].leftChild == parent) {
                    nodes[grandParent].leftChild = sibling
                } else {
                    nodes[grandParent].rightChild = sibling
                }

                // Update ancestors up to the root
                var index = grandParent
                do {
                    val p = nodes[index].parent
                    if (p == -1) break
                    nodes[p].aabb = nodes[nodes[p].leftChild].aabb.union(
                        nodes[nodes[p].rightChild].aabb
                    )
                    index = p
                } while (true)
            } else {
                root = sibling
            }

            // Free the parent node
            freeNode(parent)
        }

        // Free the leaf node
        freeNode(proxyId)
        proxyCount--

        return true
    }

    /**
     * Update the AABB of an existing proxy.
     * @param proxyId Proxy ID returned by insert().
     * @param aabb New AABB.
     * @return True if the proxy was found and updated.
     */
    fun update(proxyId: Int, aabb: AABB): Boolean {
        if (proxyId < 0 || proxyId >= nodes.size) return false
        if (!nodes[proxyId].isLeaf) return false

        nodes[proxyId].aabb = aabb

        // Update ancestors up to the root
        var index = nodes[proxyId].parent
        while (index != -1) {
            val node = nodes[index]
            val newAABB = nodes[node.leftChild].aabb.union(nodes[node.rightChild].aabb)
            if (newAABB == node.aabb) {
                break
            }
            nodes[index].aabb = newAABB
            index = node.parent
        }

        return true
    }

    /**
     * Remove and re-insert a proxy with a new AABB (for moving bodies).
     * @param proxyId Proxy ID returned by insert().
     * @param aabb New AABB.
     * @return New proxy ID.
     */
    fun removeAndInsert(proxyId: Int, aabb: AABB): Int {
        // Save the bodyId before removal since the node will be freed
        val bodyId = nodes[proxyId].bodyId
        remove(proxyId)
        return insert(aabb, bodyId)
    }

    /**
     * Query all proxies that overlap with the given AABB.
     * @param aabb Query AABB.
     * @return List of body IDs whose AABBs overlap with the query AABB.
     */
    fun query(aabb: AABB): List<Int> {
        if (root == -1) return emptyList()
        val result = mutableListOf<Int>()
        queryRecursive(root, aabb, result)
        return result
    }

    /**
     * Query all proxies that the given ray intersects.
     * @param origin Ray origin.
     * @param direction Ray direction.
     * @param maxDistance Maximum ray distance.
     * @return List of body IDs whose AABBs the ray intersects.
     */
    fun queryRay(origin: Vector3, direction: Vector3, maxDistance: Double): List<Int> {
        if (root == -1) return emptyList()
        val result = mutableListOf<Int>()
        queryRayRecursive(root, origin, direction, maxDistance, result)
        return result
    }

    /**
     * Get the AABB of a proxy.
     */
    fun getAABB(proxyId: Int): AABB? {
        if (proxyId < 0 || proxyId >= nodes.size) return null
        if (!nodes[proxyId].isLeaf) return null
        return nodes[proxyId].aabb
    }

    /**
     * Get the body ID of a proxy.
     */
    fun getBodyId(proxyId: Int): Int {
        if (proxyId < 0 || proxyId >= nodes.size) return -1
        return nodes[proxyId].bodyId
    }

    /**
     * Get the tree height (depth).
     */
    fun getTreeHeight(): Int {
        if (root == -1) return 0
        return getTreeHeightRecursive(root)
    }

    /**
     * Clear all proxies from the tree.
     */
    fun clear() {
        proxyCount = 0
        root = -1
        // Reset free list: all nodes are free
        freeListCount = nodes.size
        for (i in 0 until nodes.size) {
            freeList[i] = i
            nodes[i] = TreeNode()
        }
    }

    private fun queryRecursive(nodeIndex: Int, aabb: AABB, result: MutableList<Int>) {
        val node = nodes[nodeIndex]
        if (!node.aabb.intersects(aabb)) return

        if (node.isLeaf) {
            result.add(node.bodyId)
        } else {
            queryRecursive(node.leftChild, aabb, result)
            queryRecursive(node.rightChild, aabb, result)
        }
    }

    private fun queryRayRecursive(
        nodeIndex: Int,
        origin: Vector3,
        direction: Vector3,
        maxDistance: Double,
        result: MutableList<Int>
    ) {
        val node = nodes[nodeIndex]
        if (!rayIntersectsAABB(origin, direction, maxDistance, node.aabb)) return

        if (node.isLeaf) {
            result.add(node.bodyId)
        } else {
            queryRayRecursive(node.leftChild, origin, direction, maxDistance, result)
            queryRayRecursive(node.rightChild, origin, direction, maxDistance, result)
        }
    }

    private fun rayIntersectsAABB(
        origin: Vector3,
        direction: Vector3,
        maxDistance: Double,
        aabb: AABB
    ): Boolean {
        var tmin = 0.0
        var tmax = maxDistance

        // X axis
        if (kotlin.math.abs(direction.x) > 1e-10) {
            val invDirX = 1.0 / direction.x
            var t1 = (aabb.min.x - origin.x) * invDirX
            var t2 = (aabb.max.x - origin.x) * invDirX
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
            }
            if (t1 > tmin) tmin = t1
            if (t2 < tmax) tmax = t2
            if (tmin > tmax) return false
        } else {
            if (origin.x < aabb.min.x || origin.x > aabb.max.x) return false
        }

        // Y axis
        if (kotlin.math.abs(direction.y) > 1e-10) {
            val invDirY = 1.0 / direction.y
            var t1 = (aabb.min.y - origin.y) * invDirY
            var t2 = (aabb.max.y - origin.y) * invDirY
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
            }
            if (t1 > tmin) tmin = t1
            if (t2 < tmax) tmax = t2
            if (tmin > tmax) return false
        } else {
            if (origin.y < aabb.min.y || origin.y > aabb.max.y) return false
        }

        // Z axis
        if (kotlin.math.abs(direction.z) > 1e-10) {
            val invDirZ = 1.0 / direction.z
            var t1 = (aabb.min.z - origin.z) * invDirZ
            var t2 = (aabb.max.z - origin.z) * invDirZ
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
            }
            if (t1 > tmin) tmin = t1
            if (t2 < tmax) tmax = t2
            if (tmin > tmax) return false
        } else {
            if (origin.z < aabb.min.z || origin.z > aabb.max.z) return false
        }

        return tmin >= 0.0 && tmax <= maxDistance
    }

    private fun findBestSubtree(newLeaf: Int): Int {
        // Use the surface area heuristic to find the best insertion point
        var insertIndex = root
        var t = 0

        while (!nodes[insertIndex].isLeaf) {
            val left = nodes[insertIndex].leftChild
            val right = nodes[insertIndex].rightChild

            val combinedAABB = nodes[insertIndex].aabb.union(nodes[newLeaf].aabb)
            val oldCost = 2.0 * (nodes[insertIndex].aabb.surfaceArea() +
                    combinedAABB.surfaceArea() +
                    nodes[left].aabb.surfaceArea() +
                    nodes[right].aabb.surfaceArea())

            val newCost = 2.0 * (combinedAABB.surfaceArea() +
                    nodes[insertIndex].aabb.surfaceArea() +
                    nodes[left].aabb.surfaceArea() +
                    nodes[right].aabb.surfaceArea() +
                    nodes[newLeaf].aabb.surfaceArea())

            if (t > 64) {
                // Limit search depth
                break
            }

            if (newCost > oldCost) {
                // Descend to the child with the least cost increase
                val leftCost = nodes[left].aabb.surfaceArea() +
                        nodes[left].aabb.union(nodes[newLeaf].aabb).surfaceArea()
                val rightCost = nodes[right].aabb.surfaceArea() +
                        nodes[right].aabb.union(nodes[newLeaf].aabb).surfaceArea()

                if (leftCost < rightCost) {
                    insertIndex = left
                } else {
                    insertIndex = right
                }
                t++
            } else {
                break
            }
        }

        return insertIndex
    }

    private fun getTreeHeightRecursive(nodeIndex: Int): Int {
        val node = nodes[nodeIndex]
        if (node.isLeaf) return 1
        val leftHeight = getTreeHeightRecursive(node.leftChild)
        val rightHeight = getTreeHeightRecursive(node.rightChild)
        return 1 + kotlin.math.max(leftHeight, rightHeight)
    }

    private fun allocateNode(): Int {
        if (freeListCount == 0) {
            grow()
            if (freeListCount == 0) {
                throw IllegalStateException("Tree capacity exceeded after growth")
            }
        }
        freeListCount--
        return freeList[freeListCount]
    }

    private fun grow() {
        val newCapacity = capacity * 2
        val newNodeCount = 4 * newCapacity

        // Mark used nodes in the old tree
        val used = BooleanArray(nodes.size)
        if (root != -1) {
            markUsed(root, used)
        }

        // Create new arrays
        val newNodes = Array(newNodeCount) { TreeNode() }
        val newFreeList = IntArray(newNodeCount)

        // Copy existing nodes
        for (i in nodes.indices) {
            newNodes[i] = nodes[i].copy()
        }

        // Build new free list from unused old nodes + all new nodes
        var newFreeCount = 0
        for (i in 0 until nodes.size) {
            if (!used[i]) {
                newFreeList[newFreeCount++] = i
            }
        }
        for (i in nodes.size until newNodeCount) {
            newFreeList[newFreeCount++] = i
        }

        this.nodes = newNodes
        this.freeList = newFreeList
        this.freeListCount = newFreeCount
        this.capacity = newCapacity
    }

    private fun markUsed(nodeIndex: Int, used: BooleanArray) {
        if (nodeIndex < 0 || nodeIndex >= used.size) return
        if (used[nodeIndex]) return
        used[nodeIndex] = true
        val node = nodes[nodeIndex]
        if (!node.isLeaf) {
            markUsed(node.leftChild, used)
            markUsed(node.rightChild, used)
        }
    }

    private fun freeNode(index: Int) {
        freeList[freeListCount] = index
        freeListCount++
    }

    /**
     * Tree node for the Dynamic AABB Tree.
     */
    private data class TreeNode(
        var aabb: AABB = AABB(Vector3.ZERO, Vector3.ZERO),
        var isLeaf: Boolean = false,
        var bodyId: Int = -1,
        var parent: Int = -1,
        var leftChild: Int = -1,
        var rightChild: Int = -1,
        var subIndex: Int = -1
    )
}
