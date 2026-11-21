package io.vakuum.core

import io.vakuum.math.Vector3

/**
 * Immutable physics configuration for a simulation world.
 */
data class PhysicsConfiguration(
    /** Gravity vector (m/s²). Default: (0, -9.81, 0). */
    val gravity: Vector3 = Vector3(0.0, -9.81, 0.0),

    /** Fixed timestep in seconds. Default: 1/60. */
    val fixedTimestep: Double = 1.0 / 60.0,

    /** Maximum number of solver iterations per step. Default: 10. */
    val maxSolverIterations: Int = 10,

    /** Linear damping factor (0.0 = no damping, 1.0 = full damping). Default: 0.0. */
    val linearDamping: Double = 0.0,

    /** Angular damping factor (0.0 = no damping, 1.0 = full damping). Default: 0.0. */
    val angularDamping: Double = 0.0,

    /** Allow sleeping for bodies at rest. Default: true. */
    val allowSleeping: Boolean = true,

    /** Velocity threshold for sleeping (m/s). Default: 0.03. */
    val sleepThreshold: Double = 0.03,

    /** Enable Continuous Collision Detection. Default: false. */
    val ccdEnabled: Boolean = false,

    /**
     * Minimum velocity magnitude (m/s) for CCD activation.
     * Bodies moving slower than this threshold use discrete collision detection.
     * Default: 10.0 m/s.
     */
    val ccdVelocityThreshold: Double = 10.0,

    /**
     * Maximum number of CCD substeps per simulation step.
     * Prevents infinite loops in cases of repeated impacts.
     * Default: 10.
     */
    val ccdMaxSubsteps: Int = 10,

    /**
     * TOI (Time of Impact) tolerance.
     * Minimum fraction of remaining timestep to consider a valid TOI.
     * Prevents infinite loops from near-zero TOI values.
     * Default: 0.001 (0.1% of remaining time).
     */
    val ccdToiTolerance: Double = 0.001,
) {
    init {
        require(!gravity.x.isNaN() && !gravity.y.isNaN() && !gravity.z.isNaN()) {
            "Gravity must not contain NaN values"
        }
        require(fixedTimestep > 0.0) {
            "Fixed timestep must be positive"
        }
        require(!fixedTimestep.isInfinite()) {
            "Fixed timestep must not be infinite"
        }
        require(maxSolverIterations > 0) {
            "Max solver iterations must be positive"
        }
        require(linearDamping in 0.0..1.0) {
            "Linear damping must be between 0.0 and 1.0"
        }
        require(angularDamping in 0.0..1.0) {
            "Angular damping must be between 0.0 and 1.0"
        }
        require(sleepThreshold >= 0.0) {
            "Sleep threshold must be non-negative"
        }
        require(ccdVelocityThreshold > 0.0) {
            "CCD velocity threshold must be positive"
        }
        require(ccdMaxSubsteps > 0) {
            "CCD max substeps must be positive"
        }
        require(ccdToiTolerance in 0.0..1.0) {
            "CCD TOI tolerance must be between 0.0 and 1.0"
        }
    }

    companion object {
        /** Default configuration with standard gravity and 60Hz timestep. */
        val DEFAULT = PhysicsConfiguration()
    }
}
