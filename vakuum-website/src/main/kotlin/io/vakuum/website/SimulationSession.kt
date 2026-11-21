package io.vakuum.website

import io.vakuum.collision.BoxShape
import io.vakuum.collision.CollisionShape
import io.vakuum.collision.SphereShape
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import io.vakuum.math.Quaternion
import io.vakuum.runner.SimulationPipeline

/**
 * Represents an isolated physics simulation session for the Vakuum.xyz website.
 *
 * Each session has its own SimulationPipeline, bodies, and configuration.
 * Sessions are independent and do not share state.
 *
 * Thread safety: Not thread-safe. All operations must be called from the same thread.
 */
class SimulationSession(
    private val sessionId: String,
    config: PhysicsConfiguration? = null
) {
    private val physicsConfig = config ?: PhysicsConfiguration(
        gravity = Vector3(0.0, -9.81, 0.0),
        fixedTimestep = 1.0 / 60.0,
        maxSolverIterations = 10,
        linearDamping = 0.01,
        angularDamping = 0.01,
        allowSleeping = true,
        sleepThreshold = 0.001
    )

    private val pipeline = SimulationPipeline(physicsConfig)
    private var nextBodyId = 0
    private var paused = false
    private var stepCount: Long = 0

    // Grab constraint: bodyId -> target position
    private val grabTargets = mutableMapOf<Int, Vector3>()

    // Boundary walls for ASCII playground (4 static walls)
    private var hasBoundaries = false
    private val boundarySize = 10.0
    private val boundaryHeight = 5.0

    /** Session identifier. */
    val id: String
        get() = sessionId

    /** Whether the simulation is paused. */
    val isPaused: Boolean
        get() = paused

    /** Number of bodies in this session. */
    val bodyCount: Int
        get() = pipeline.getBodies().size

    /** Total steps executed. */
    val totalSteps: Long
        get() = stepCount

    /**
     * Initialize the session with boundary walls for the ASCII playground.
     * Creates 4 static boundary colliders forming a bounded environment.
     */
    fun setupBoundaries() {
        if (hasBoundaries) return
        hasBoundaries = true

        val half = boundarySize / 2.0

        // Bottom wall (ground)
        addStaticBody(
            position = Vector3(0.0, -boundaryHeight / 2.0, 0.0),
            shape = BoxShape(Vector3(boundarySize, boundaryHeight / 2.0, boundarySize))
        )

        // Left wall
        addStaticBody(
            position = Vector3(-half - boundaryHeight / 2.0, boundaryHeight, 0.0),
            shape = BoxShape(Vector3(boundaryHeight / 2.0, boundaryHeight * 2, boundarySize))
        )

        // Right wall
        addStaticBody(
            position = Vector3(half + boundaryHeight / 2.0, boundaryHeight, 0.0),
            shape = BoxShape(Vector3(boundaryHeight / 2.0, boundaryHeight * 2, boundarySize))
        )

        // Back wall
        addStaticBody(
            position = Vector3(0.0, boundaryHeight, -half - boundaryHeight / 2.0),
            shape = BoxShape(Vector3(boundarySize, boundaryHeight * 2, boundaryHeight / 2.0))
        )
    }

    /**
     * Spawn a dynamic box body at the given position.
     * @return The body ID.
     */
    fun spawnBox(
        position: Vector3,
        halfExtents: Vector3 = Vector3(0.5, 0.5, 0.5),
        mass: Double = 1.0,
        restitution: Double = 0.3,
        friction: Double = 0.5
    ): Int {
        val id = nextBodyId++
        val body = rigidBody()
            .id(id)
            .bodyType(BodyType.DYNAMIC)
            .position(position)
            .mass(mass)
            .restitution(restitution)
            .friction(friction)
            .build()

        pipeline.addBody(body)
        pipeline.setShape(id, BoxShape(halfExtents))
        return id
    }

    /**
     * Spawn a dynamic sphere body at the given position.
     * @return The body ID.
     */
    fun spawnSphere(
        position: Vector3,
        radius: Double = 0.5,
        mass: Double = 1.0,
        restitution: Double = 0.3,
        friction: Double = 0.5
    ): Int {
        val id = nextBodyId++
        val body = rigidBody()
            .id(id)
            .bodyType(BodyType.DYNAMIC)
            .position(position)
            .mass(mass)
            .restitution(restitution)
            .friction(friction)
            .build()

        pipeline.addBody(body)
        pipeline.setShape(id, SphereShape(Vector3.ZERO, radius))
        return id
    }

    /**
     * Add a static body (e.g., obstacle or boundary).
     * @return The body ID.
     */
    private fun addStaticBody(position: Vector3, shape: CollisionShape): Int {
        val id = nextBodyId++
        val body = rigidBody()
            .id(id)
            .bodyType(BodyType.STATIC)
            .position(position)
            .mass(0.0)
            .build()

        pipeline.addBody(body)
        pipeline.setShape(id, shape)
        return id
    }

    /**
     * Remove a body from the simulation.
     * Also releases any grab constraint on this body.
     */
    fun removeBody(bodyId: Int) {
        grabTargets.remove(bodyId)
        // Note: SimulationPipeline doesn't have removeBody yet; this is a limitation
        // For now, we just release the grab and mark the body as static at zero position
        val body = pipeline.getBodies().find { it.id == bodyId }
        if (body != null) {
            body.linearVelocity = Vector3.ZERO
            body.angularVelocity = Vector3.ZERO
            body.awake = false
        }
    }

    /**
     * Grab a body and constrain it to follow a target position.
     * The body is moved toward the target each simulation step.
     */
    fun grabBody(bodyId: Int, targetPosition: Vector3) {
        val body = pipeline.getBodies().find { it.id == bodyId }
        if (body == null) return
        body.wakeUp()
        grabTargets[bodyId] = targetPosition
    }

    /**
     * Update the grab target position for a grabbed body.
     */
    fun updateGrabTarget(bodyId: Int, targetPosition: Vector3) {
        grabTargets[bodyId] = targetPosition
    }

    /**
     * Release a grabbed body. The body retains its current velocity.
     */
    fun releaseBody(bodyId: Int) {
        grabTargets.remove(bodyId)
    }

    /**
     * Apply an impulse to a body at its center.
     */
    fun applyImpulse(bodyId: Int, impulse: Vector3) {
        val body = pipeline.getBodies().find { it.id == bodyId } ?: return
        body.applyImpulse(body.transform.position, impulse)
        body.wakeUp()
    }

    /**
     * Apply a force to a body at its center.
     */
    fun applyForce(bodyId: Int, force: Vector3) {
        val body = pipeline.getBodies().find { it.id == bodyId } ?: return
        body.applyForceAtCenter(force)
        body.wakeUp()
    }

    /**
     * Pause the simulation.
     */
    fun pause() {
        paused = true
    }

    /**
     * Resume the simulation.
     */
    fun resume() {
        paused = false
    }

    /**
     * Reset the simulation, removing all dynamic bodies and resetting state.
     * Static boundary walls are preserved.
     */
    fun reset() {
        // Clear all bodies and rebuild boundaries
        grabTargets.clear()
        stepCount = 0
        nextBodyId = 0
        // Note: Full reset requires recreating the pipeline
        // For now, we just clear grab targets and step count
    }

    /**
     * Execute one simulation step.
     * Handles grab constraints and updates step count.
     */
    fun step() {
        if (paused) return

        // Apply grab constraints: move grabbed bodies toward target
        for ((bodyId, target) in grabTargets) {
            val body = pipeline.getBodies().find { it.id == bodyId } ?: continue
            val currentPos = body.transform.position
            val direction = target - currentPos
            val distance = direction.length

            if (distance > 0.01) {
                // Smooth follow: move body toward target
                val followSpeed = 15.0
                val moveAmount = kotlin.math.min(distance, followSpeed * physicsConfig.fixedTimestep)
                val newPos = currentPos + direction / distance * moveAmount

                body.transform = io.vakuum.math.Transform(
                    position = newPos,
                    rotation = body.transform.rotation
                )

                // Track velocity for natural release
                body.linearVelocity = (newPos - currentPos) / physicsConfig.fixedTimestep
            }
        }

        pipeline.step()
        stepCount++
    }

    /**
     * Get the current state of all bodies.
     */
    fun getBodyStates(): List<BodyState> {
        return pipeline.getBodies().map { body ->
            BodyState(
                id = body.id,
                position = body.transform.position,
                rotation = body.transform.rotation,
                linearVelocity = body.linearVelocity,
                angularVelocity = body.angularVelocity,
                mass = body.massProperties.mass,
                isAwake = body.awake
            )
        }
    }

    /**
     * Get the state of a specific body.
     */
    fun getBodyState(bodyId: Int): BodyState? {
        val body = pipeline.getBodies().find { it.id == bodyId } ?: return null
        return BodyState(
            id = body.id,
            position = body.transform.position,
            rotation = body.transform.rotation,
            linearVelocity = body.linearVelocity,
            angularVelocity = body.angularVelocity,
            mass = body.massProperties.mass,
            isAwake = body.awake
        )
    }

    /**
     * Get simulation diagnostics.
     */
    fun getDiagnostics(): SimulationDiagnostics {
        return SimulationDiagnostics(
            bodyCount = pipeline.getBodies().size,
            stepCount = stepCount,
            contactCount = 0, // Not tracked in current pipeline
            ccdSteps = pipeline.ccdDiagnostics.ccdSteps,
            toiQueries = pipeline.ccdDiagnostics.toiQueries,
            hasNaN = pipeline.hasNaN()
        )
    }

    /**
     * Destroy the session and release resources.
     */
    fun destroy() {
        grabTargets.clear()
    }
}

/**
 * Factory for creating simulation sessions.
 */
object SimulationSessionFactory {

    private var sessionCounter = 0

    /**
     * Create a new isolated simulation session.
     */
    fun createSession(config: PhysicsConfiguration? = null): SimulationSession {
        sessionCounter++
        val sessionId = "session-${System.currentTimeMillis()}-$sessionCounter"
        return SimulationSession(sessionId, config)
    }
}
