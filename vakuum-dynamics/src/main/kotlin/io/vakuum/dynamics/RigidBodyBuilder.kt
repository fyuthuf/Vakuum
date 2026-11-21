package io.vakuum.dynamics

import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3

/**
 * Fluent builder for creating RigidBody instances.
 */
class RigidBodyBuilder {
    private var id: Int = 0
    private var bodyType: BodyType = BodyType.DYNAMIC
    private var position: Vector3 = Vector3.ZERO
    private var rotation: Quaternion = Quaternion.IDENTITY
    private var linearVelocity: Vector3 = Vector3.ZERO
    private var angularVelocity: Vector3 = Vector3.ZERO
    private var mass: Double = 1.0
    private var halfExtents: Vector3 = Vector3(0.5, 0.5, 0.5)
    private var restitution: Double = 0.0
    private var friction: Double = 0.5
    private var linearDamping: Double = 0.0
    private var angularDamping: Double = 0.0

    fun id(id: Int) = apply { this.id = id }

    fun bodyType(bodyType: BodyType) = apply { this.bodyType = bodyType }

    fun position(x: Double, y: Double, z: Double) = apply { this.position = Vector3(x, y, z) }

    fun position(position: Vector3) = apply { this.position = position }

    fun rotation(rotation: Quaternion) = apply { this.rotation = rotation }

    fun linearVelocity(velocity: Vector3) = apply { this.linearVelocity = velocity }

    fun angularVelocity(velocity: Vector3) = apply { this.angularVelocity = velocity }

    fun mass(mass: Double) = apply { this.mass = mass }

    /** Set the half-extents of the box collision shape. */
    fun halfExtents(x: Double, y: Double, z: Double) = apply { this.halfExtents = Vector3(x, y, z) }

    fun halfExtents(halfExtents: Vector3) = apply { this.halfExtents = halfExtents }

    /** Set cube size (full dimensions, not half-extents). */
    fun cubeSize(size: Double) = apply {
        val half = size / 2.0
        this.halfExtents = Vector3(half, half, half)
    }

    fun restitution(restitution: Double) = apply { this.restitution = restitution }

    fun friction(friction: Double) = apply { this.friction = friction }

    fun linearDamping(damping: Double) = apply { this.linearDamping = damping }

    fun angularDamping(damping: Double) = apply { this.angularDamping = damping }

    /**
     * Build the rigid body.
     */
    fun build(): RigidBody {
        val transform = Transform(position, rotation.normalize())

        val massProperties = when (bodyType) {
            BodyType.STATIC -> MassProperties.static()
            BodyType.KINEMATIC -> MassProperties.static()
            BodyType.DYNAMIC -> MassProperties.box(halfExtents, mass)
        }

        val body = RigidBody(
            id = id,
            bodyType = bodyType,
            transform = transform,
            linearVelocity = linearVelocity,
            angularVelocity = angularVelocity,
            massProperties = massProperties
        )

        body.restitution = restitution
        body.friction = friction
        body.linearDamping = linearDamping
        body.angularDamping = angularDamping

        return body
    }

    /**
     * Create a static ground plane at y=0.
     */
    fun staticGround(id: Int = 0): RigidBody = apply {
        this.id = id
        this.bodyType = BodyType.STATIC
        this.position = Vector3.ZERO
        this.rotation = Quaternion.IDENTITY
        this.linearVelocity = Vector3.ZERO
        this.angularVelocity = Vector3.ZERO
        this.mass = 0.0
    }.build()

    /**
     * Create a dynamic cube at the given height.
     */
    fun dynamicCube(id: Int, size: Double, x: Double, y: Double, z: Double, mass: Double = 1.0): RigidBody =
        apply {
            this.id = id
            this.bodyType = BodyType.DYNAMIC
            this.position = Vector3(x, y, z)
            this.rotation = Quaternion.IDENTITY
            this.linearVelocity = Vector3.ZERO
            this.angularVelocity = Vector3.ZERO
            this.mass = mass
            this.halfExtents = Vector3(size / 2.0, size / 2.0, size / 2.0)
        }.build()
}

/**
 * Create a new RigidBodyBuilder.
 */
fun rigidBody(): RigidBodyBuilder = RigidBodyBuilder()
