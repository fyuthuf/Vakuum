package io.vakuum.website

import io.vakuum.math.Vector3
import io.vakuum.math.Quaternion

/**
 * Immutable snapshot of a rigid body's current state.
 *
 * Used for transmitting body state to the website client.
 */
data class BodyState(
    val id: Int,
    val position: Vector3,
    val rotation: Quaternion,
    val linearVelocity: Vector3,
    val angularVelocity: Vector3,
    val mass: Double,
    val isAwake: Boolean
)

/**
 * Simulation diagnostics snapshot.
 */
data class SimulationDiagnostics(
    val bodyCount: Int,
    val stepCount: Long,
    val contactCount: Int,
    val ccdSteps: Int,
    val toiQueries: Int,
    val hasNaN: Boolean
)
