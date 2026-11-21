package io.vakuum.solver

import io.vakuum.math.Vector3

/**
 * Settings for the impulse solver.
 */
data class SolverSettings(
    /** Number of solver iterations per step. Default: 10. */
    val iterations: Int = 10,
    /** Default restitution coefficient. Default: 0.0. */
    val defaultRestitution: Double = 0.0,
    /** Default friction coefficient. Default: 0.5. */
    val defaultFriction: Double = 0.5,
    /** Penetration correction percentage (Baumgarte). Default: 0.2. */
    val penetrationPercentage: Double = 0.2
) {
    init {
        require(iterations > 0) { "Iterations must be positive" }
        require(defaultRestitution in 0.0..1.0) { "Restitution must be in [0, 1]" }
        require(defaultFriction >= 0.0) { "Friction must be non-negative" }
        require(penetrationPercentage in 0.0..1.0) { "Penetration percentage must be in [0, 1]" }
    }
}
