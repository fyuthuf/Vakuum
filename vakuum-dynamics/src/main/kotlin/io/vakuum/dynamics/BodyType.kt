package io.vakuum.dynamics

/**
 * Type of rigid body determining how it behaves in the simulation.
 */
enum class BodyType {
    /**
     * Static body: infinite mass, never moves.
     * Used for ground planes and immovable objects.
     */
    STATIC,

    /**
     * Dynamic body: finite mass, affected by forces and collisions.
     * The standard movable object.
     */
    DYNAMIC,

    /**
     * Kinematic body: zero mass, moves by setting velocity directly.
     * Collides with dynamic bodies but is not affected by forces.
     */
    KINEMATIC
}
