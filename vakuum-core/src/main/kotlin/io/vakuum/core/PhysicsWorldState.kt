package io.vakuum.core

/**
 * Simulation state of the physics world.
 */
enum class PhysicsWorldState {
    /** World is created but not yet initialized. */
    IDLE,

    /** World is running and processing simulation steps. */
    RUNNING,

    /** World is paused; simulation is temporarily halted. */
    PAUSED,

    /** World has been shut down and resources released. */
    STOPPED,
}
