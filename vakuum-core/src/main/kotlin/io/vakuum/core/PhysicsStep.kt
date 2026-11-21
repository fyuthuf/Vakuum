package io.vakuum.core

/**
 * Result of a single simulation step.
 */
data class PhysicsStep(
    /** Delta time simulated in this step (seconds). */
    val deltaTime: Double,

    /** Step index (0-based). */
    val stepIndex: Long,

    /** Number of solver iterations performed. */
    val solverIterations: Int,
)
