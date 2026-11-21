package io.vakuum.dynamics

import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3

/**
 * Rigid body representing a physical object in the simulation.
 *
 * Contains position, orientation, velocity, mass properties, and force/torque accumulators.
 * Supports STATIC, DYNAMIC, and KINEMATIC body types.
 */
class RigidBody {
    /** Unique identifier for this body. */
    val id: Int

    /** Body type determines simulation behavior. */
    var bodyType: BodyType

    /** Current transform (position + orientation). */
    var transform: Transform

    /** Linear velocity in world space (m/s). */
    var linearVelocity: Vector3

    /** Angular velocity in local space (rad/s). */
    var angularVelocity: Vector3

    /** Mass properties (mass, inertia, center of mass). */
    var massProperties: MassProperties

    /** Accumulated force in world space (N). Reset each step. */
    var force: Vector3 = Vector3.ZERO

    /** Accumulated torque in local space (N·m). Reset each step. */
    var torque: Vector3 = Vector3.ZERO

    /** Restitution coefficient (0.0 = no bounce, 1.0 = full bounce). */
    var restitution: Double = 0.0

    /** Friction coefficient (0.0 = frictionless, 1.0 = high friction). */
    var friction: Double = 0.5

    /** Whether the body is awake and participating in simulation. */
    var awake: Boolean = true

    /** Linear damping factor (0.0 = no damping). */
    var linearDamping: Double = 0.0

    /** Angular damping factor (0.0 = no damping). */
    var angularDamping: Double = 0.0

    /**
     * Create a rigid body.
     */
    constructor(
        id: Int,
        bodyType: BodyType,
        transform: Transform,
        linearVelocity: Vector3,
        angularVelocity: Vector3,
        massProperties: MassProperties
    ) {
        this.id = id
        this.bodyType = bodyType
        this.transform = transform
        this.linearVelocity = linearVelocity
        this.angularVelocity = angularVelocity
        this.massProperties = massProperties
    }

    /**
     * Reset force and torque accumulators.
     */
    fun clearForces() {
        force = Vector3.ZERO
        torque = Vector3.ZERO
    }

    /**
     * Apply a force at a specific point in world space.
     * Adds both linear force and torque from the offset.
     */
    fun applyForce(atWorldPoint: Vector3, worldForce: Vector3) {
        if (bodyType == BodyType.STATIC) return
        force += worldForce
        // Torque = r × F
        val r = atWorldPoint - transform.position
        torque += transform.rotation.conjugate().rotateVector(r.cross(worldForce))
    }

    /**
     * Apply a force at the center of mass.
     */
    fun applyForceAtCenter(worldForce: Vector3) {
        if (bodyType == BodyType.STATIC) return
        force += worldForce
    }

    /**
     * Apply an impulse at a specific point in world space.
     */
    fun applyImpulse(atWorldPoint: Vector3, worldImpulse: Vector3) {
        if (bodyType == BodyType.STATIC) return
        val r = atWorldPoint - transform.position
        val localR = transform.rotation.conjugate().rotateVector(r)

        // Linear impulse
        linearVelocity += worldImpulse * massProperties.inverseMass

        // Angular impulse: τ = r × J
        val angularImpulse = localR.cross(transform.rotation.conjugate().rotateVector(worldImpulse))
        angularVelocity += massProperties.inverseInertiaTensor.multiplyVector(angularImpulse)
    }

    /**
     * Integrate this body for one timestep using semi-implicit Euler.
     * @param dt Timestep in seconds.
     * @param gravity Gravity vector (m/s²).
     * @param linearDamping Global linear damping.
     * @param angularDamping Global angular damping.
     */
    fun integrate(dt: Double, gravity: Vector3, linearDamping: Double, angularDamping: Double) {
        if (bodyType == BodyType.STATIC) return

        // Apply gravity and forces to update linear velocity
        if (bodyType == BodyType.DYNAMIC) {
            val acceleration = force * massProperties.inverseMass + gravity
            linearVelocity += acceleration * dt
        }

        // Apply torque to update angular velocity
        if (bodyType == BodyType.DYNAMIC) {
            val angularAcceleration = massProperties.inverseInertiaTensor.multiplyVector(torque)
            angularVelocity += angularAcceleration * dt
        }

        // Apply damping
        val linDamp = 1.0 - (linearDamping + this.linearDamping).coerceIn(0.0, 1.0) * dt
        val angDamp = 1.0 - (angularDamping + this.angularDamping).coerceIn(0.0, 1.0) * dt
        linearVelocity *= linDamp
        angularVelocity *= angDamp

        // Update position from velocity (semi-implicit Euler)
        transform = Transform(
            position = transform.position + linearVelocity * dt,
            rotation = transform.rotation
        )

        // Update orientation from angular velocity
        if (angularVelocity.lengthSquared > 1e-10) {
            val angSpeed = angularVelocity.length
            val angAxis = angularVelocity / angSpeed
            val deltaRotation = Quaternion.fromAxisAngle(angAxis, angSpeed * dt)
            transform = Transform(
                position = transform.position,
                rotation = deltaRotation * transform.rotation
            )
        }
    }

    /**
     * Check if any values are NaN or infinite.
     */
    fun hasNaN(): Boolean {
        return transform.position.x.isNaN() || transform.position.y.isNaN() || transform.position.z.isNaN() ||
                transform.rotation.w.isNaN() || transform.rotation.x.isNaN() ||
                transform.rotation.y.isNaN() || transform.rotation.z.isNaN() ||
                linearVelocity.x.isNaN() || linearVelocity.y.isNaN() || linearVelocity.z.isNaN() ||
                angularVelocity.x.isNaN() || angularVelocity.y.isNaN() || angularVelocity.z.isNaN()
    }

    override fun toString(): String =
        "RigidBody(id=$id, type=$bodyType, pos=${transform.position}, vel=$linearVelocity)"
}
