package io.vakuum.core

import io.vakuum.math.Vector3

/**
 * Main physics simulation world.
 *
 * Manages the simulation lifecycle: initialization, stepping, pausing, and shutdown.
 * Uses a fixed timestep for deterministic simulation.
 *
 * Thread safety: Not thread-safe. All operations must be called from the same thread.
 */
class PhysicsWorld(private val settings: PhysicsWorldSettings) {

    private var state: PhysicsWorldState = PhysicsWorldState.IDLE
    private var stepCount: Long = 0
    private var accumulator: Double = 0.0

    /** Current simulation state. */
    val currentState: PhysicsWorldState
        get() = state

    /** World name. */
    val name: String
        get() = settings.worldName

    /** Current configuration. */
    val configuration: PhysicsConfiguration
        get() = settings.configuration

    /** Total number of steps executed. */
    val totalSteps: Long
        get() = stepCount

    /** Gravity vector. */
    val gravity: Vector3
        get() = settings.configuration.gravity

    /**
     * Initialize the world for simulation.
     * Must be called before stepping.
     */
    fun initialize() {
        require(state == PhysicsWorldState.IDLE) {
            "World must be in IDLE state to initialize. Current state: $state"
        }
        state = PhysicsWorldState.RUNNING
        stepCount = 0
        accumulator = 0.0
    }

    /**
     * Execute a single simulation step with the fixed timestep.
     * @return The result of the simulation step.
     */
    fun step(): PhysicsStep {
        require(state == PhysicsWorldState.RUNNING) {
            "World must be in RUNNING state to step. Current state: $state"
        }

        val dt = settings.configuration.fixedTimestep
        stepCount++

        return PhysicsStep(
            deltaTime = dt,
            stepIndex = stepCount - 1,
            solverIterations = settings.configuration.maxSolverIterations
        )
    }

    /**
     * Execute simulation steps for the given elapsed time using fixed timestep.
     * @param elapsedTime The real elapsed time since the last call.
     * @return List of step results.
     */
    fun step(elapsedTime: Double): List<PhysicsStep> {
        require(state == PhysicsWorldState.RUNNING) {
            "World must be in RUNNING state to step. Current state: $state"
        }
        require(elapsedTime >= 0.0) {
            "Elapsed time must be non-negative"
        }

        val results = mutableListOf<PhysicsStep>()
        accumulator += elapsedTime

        val fixedDt = settings.configuration.fixedTimestep
        var steps = 0
        val maxSteps = 6 // Prevent spiral of death

        while (accumulator >= fixedDt && steps < maxSteps) {
            results.add(step())
            accumulator -= fixedDt
            steps++
        }

        return results
    }

    /**
     * Pause the simulation.
     */
    fun pause() {
        require(state == PhysicsWorldState.RUNNING) {
            "World must be in RUNNING state to pause. Current state: $state"
        }
        state = PhysicsWorldState.PAUSED
    }

    /**
     * Resume a paused simulation.
     */
    fun resume() {
        require(state == PhysicsWorldState.PAUSED) {
            "World must be in PAUSED state to resume. Current state: $state"
        }
        state = PhysicsWorldState.RUNNING
    }

    /**
     * Shut down the world and release resources.
     */
    fun shutdown() {
        require(state == PhysicsWorldState.RUNNING || state == PhysicsWorldState.PAUSED) {
            "World must be in RUNNING or PAUSED state to shutdown. Current state: $state"
        }
        state = PhysicsWorldState.STOPPED
        stepCount = 0
        accumulator = 0.0
    }

    /**
     * Check if the world is initialized and ready for simulation.
     */
    fun isInitialized(): Boolean = state != PhysicsWorldState.IDLE && state != PhysicsWorldState.STOPPED

    /**
     * Check if the world is currently running.
     */
    fun isRunning(): Boolean = state == PhysicsWorldState.RUNNING

    /**
     * Check if the world is paused.
     */
    fun isPaused(): Boolean = state == PhysicsWorldState.PAUSED

    /**
     * Check if the world has been shut down.
     */
    fun isStopped(): Boolean = state == PhysicsWorldState.STOPPED

    override fun toString(): String = "PhysicsWorld(name='$name', state=$state, steps=$stepCount)"
}
