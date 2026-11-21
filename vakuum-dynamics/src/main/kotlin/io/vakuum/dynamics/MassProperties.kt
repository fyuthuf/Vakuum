package io.vakuum.dynamics

import io.vakuum.math.Matrix3
import io.vakuum.math.Vector3

/**
 * Mass properties of a rigid body.
 *
 * For dynamic bodies, mass > 0 and inverseMass = 1/mass.
 * For static/kinematic bodies, mass = 0 and inverseMass = 0.
 */
data class MassProperties(
    /** Total mass in kg. */
    val mass: Double,

    /** Inverse mass (1/mass). Zero for static bodies. */
    val inverseMass: Double,

    /** Inertia tensor in local space (3x3 matrix, stored as 9 elements row-major). */
    val inertiaTensor: Matrix3,

    /** Inverse inertia tensor in local space. Identity for static bodies. */
    val inverseInertiaTensor: Matrix3,

    /** Center of mass in local space. */
    val centerOfMass: Vector3 = Vector3.ZERO
) {
    init {
        require(mass >= 0.0) { "Mass must be non-negative" }
        require(inverseMass >= 0.0) { "Inverse mass must be non-negative" }
        require(!inverseMass.isNaN() && !inverseMass.isInfinite()) {
            "Inverse mass must be finite"
        }
        require(centerOfMass.x.isNaN().not() && centerOfMass.y.isNaN().not() && centerOfMass.z.isNaN().not()) {
            "Center of mass must not contain NaN"
        }
    }

    /** True if this represents a body with no mass (static/kinematic). */
    val isMassless: Boolean
        get() = inverseMass == 0.0

    companion object {
        /**
         * Calculate mass properties for a uniform box.
         * @param halfExtents Half-width, half-height, half-depth of the box.
         * @param mass Total mass in kg. If 0, returns static mass properties.
         */
        fun box(halfExtents: Vector3, mass: Double): MassProperties {
            require(halfExtents.x > 0.0 && halfExtents.y > 0.0 && halfExtents.z > 0.0) {
                "Box half-extents must be positive"
            }
            require(mass >= 0.0) { "Mass must be non-negative" }

            return if (mass <= 0.0) {
                // Static body
                MassProperties(
                    mass = 0.0,
                    inverseMass = 0.0,
                    inertiaTensor = Matrix3.IDENTITY,
                    inverseInertiaTensor = Matrix3.IDENTITY,
                    centerOfMass = Vector3.ZERO
                )
            } else {
                val invMass = 1.0 / mass
                // Inertia tensor for a uniform box about its center
                // Ixx = (1/12) * m * (2*hy^2 + 2*hz^2) = (1/6) * m * (hy^2 + hz^2)
                val ixx = (1.0 / 6.0) * mass * (halfExtents.y * halfExtents.y + halfExtents.z * halfExtents.z)
                val iyy = (1.0 / 6.0) * mass * (halfExtents.x * halfExtents.x + halfExtents.z * halfExtents.z)
                val izz = (1.0 / 6.0) * mass * (halfExtents.x * halfExtents.x + halfExtents.y * halfExtents.y)

                val inertia = Matrix3.scaling(ixx, iyy, izz)
                val invInertia = Matrix3.scaling(1.0 / ixx, 1.0 / iyy, 1.0 / izz)

                MassProperties(
                    mass = mass,
                    inverseMass = invMass,
                    inertiaTensor = inertia,
                    inverseInertiaTensor = invInertia,
                    centerOfMass = Vector3.ZERO
                )
            }
        }

        /**
         * Create static mass properties (zero mass, zero inverse mass).
         */
        fun static(): MassProperties = MassProperties(
            mass = 0.0,
            inverseMass = 0.0,
            inertiaTensor = Matrix3.IDENTITY,
            inverseInertiaTensor = Matrix3.IDENTITY,
            centerOfMass = Vector3.ZERO
        )
    }
}
