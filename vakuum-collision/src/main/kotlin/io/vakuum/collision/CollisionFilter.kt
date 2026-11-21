package io.vakuum.collision

/**
 * Collision filter for controlling which bodies collide with each other.
 *
 * Uses a combination of category masks and collision masks (layers) as well as
 * group-based filtering.
 *
 * Two bodies collide if:
 * - Their group filtering allows it (same negative group = no collision)
 * - AND (A.categoryMask & B.collideMask) != 0
 * - AND (B.categoryMask & A.collideMask) != 0
 */
data class CollisionFilter(
    /**
     * Group ID for group-based filtering.
     * - 0: no group filtering
     * - Positive: bodies in the same group collide normally
     * - Negative: bodies in the same group never collide with each other
     */
    val group: Int = 0,

    /**
     * Bitmask indicating which collision categories this body belongs to.
     * Each bit represents a category (0-63).
     */
    val categoryMask: Long = -1L,

    /**
     * Bitmask indicating which collision categories this body collides with.
     * Each bit represents a category (0-63).
     */
    val collideMask: Long = -1L
) {

    companion object {
        /** Default filter that allows all collisions. */
        val DEFAULT = CollisionFilter()

        /** Filter that prevents all collisions. */
        val NO_COLLISION = CollisionFilter(
            group = 0,
            categoryMask = 0L,
            collideMask = 0L
        )

        /**
         * Check if two bodies should collide based on their filters.
         */
        fun shouldCollide(filterA: CollisionFilter, filterB: CollisionFilter): Boolean {
            // Group filtering: same negative group means no collision
            if (filterA.group < 0 && filterA.group == filterB.group) {
                return false
            }

            // Category/collide mask filtering
            if ((filterA.categoryMask and filterB.collideMask) == 0L) {
                return false
            }
            if ((filterB.categoryMask and filterA.collideMask) == 0L) {
                return false
            }

            return true
        }
    }
}

/**
 * Create a default collision filter that allows all collisions.
 */
fun defaultFilter(): CollisionFilter = CollisionFilter.DEFAULT

/**
 * Create a collision filter that prevents all collisions.
 */
fun noCollision(): CollisionFilter = CollisionFilter.NO_COLLISION

/**
 * Create a collision filter with specific category and collide masks.
 */
fun collisionFilter(
    group: Int = 0,
    categoryMask: Long = -1L,
    collideMask: Long = -1L
): CollisionFilter = CollisionFilter(group, categoryMask, collideMask)
