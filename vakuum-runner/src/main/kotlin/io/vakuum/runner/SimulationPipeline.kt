package io.vakuum.runner

import io.vakuum.collision.*
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Vector3
import io.vakuum.solver.ImpulseSolver
import io.vakuum.solver.SolverSettings

/**
 * Orchestrates the full physics simulation pipeline.
 *
 * Manages bodies, shapes, and runs the simulation step:
 * clear forces → integrate → broadphase → narrowphase → solve → position correction.
 */
class SimulationPipeline(private val config: PhysicsConfiguration) {

    private val bodies = mutableListOf<RigidBody>()
    private val shapes = mutableMapOf<Int, CollisionShape>()

    private val solver = ImpulseSolver(SolverSettings(
        iterations = config.maxSolverIterations,
        defaultRestitution = 0.0,
        defaultFriction = 0.5,
        penetrationPercentage = 0.2
    ))
    private val broadphase = Broadphase()
    private val narrowphase = Narrowphase()

    /**
     * Add a body to the simulation.
     */
    fun addBody(body: RigidBody) {
        require(!bodies.any { it.id == body.id }) {
            "Body with id ${body.id} already exists"
        }
        bodies.add(body)
    }

    /**
     * Set the collision shape for a body.
     */
    fun setShape(bodyId: Int, shape: CollisionShape) {
        shapes[bodyId] = shape
    }

    /**
     * Get all bodies.
     */
    fun getBodies(): List<RigidBody> = bodies.toList()

    /**
     * Execute one simulation step.
     */
    fun step() {
        val dt = config.fixedTimestep

        // 1. Clear forces
        for (body in bodies) {
            body.clearForces()
        }

        // 2. Integrate
        for (body in bodies) {
            body.integrate(dt, config.gravity, config.linearDamping, config.angularDamping)
        }

        // 3. Broadphase
        val pairs = broadphase.findPairs(bodies, shapes)

        // 4. Box-plane (ground) collisions
        for (body in bodies) {
            val shape = shapes[body.id]
            if (shape is BoxShape) {
                val contact = narrowphase.testBoxPlane(body, shape, Vector3.UNIT_Y, 0.0)
                if (contact != null) {
                    val ground = io.vakuum.dynamics.rigidBody()
                        .id(-1)
                        .bodyType(BodyType.STATIC)
                        .position(Vector3.ZERO)
                        .mass(0.0)
                        .build()
                    solver.solveContact(body, ground, contact, dt)
                    solver.solvePositionCorrection(body, ground, contact, dt)
                }
            }
        }

        // 5. Box-box collisions
        for (pair in pairs) {
            val bodyA = bodies.find { it.id == pair.bodyAId } ?: continue
            val bodyB = bodies.find { it.id == pair.bodyBId } ?: continue
            val shapeA = shapes[pair.bodyAId]
            val shapeB = shapes[pair.bodyBId]

            if (shapeA is BoxShape && shapeB is BoxShape) {
                val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)
                if (contact != null) {
                    solver.solveContact(bodyA, bodyB, contact, dt)
                    solver.solvePositionCorrection(bodyA, bodyB, contact, dt)
                }
            }
        }
    }

    /**
     * Check if any body has NaN values.
     */
    fun hasNaN(): Boolean = bodies.any { it.hasNaN() }
}
