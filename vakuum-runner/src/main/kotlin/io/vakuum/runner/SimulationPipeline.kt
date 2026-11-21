package io.vakuum.runner

import io.vakuum.collision.*
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Vector3
import io.vakuum.solver.ContactConstraint
import io.vakuum.solver.ImpulseSolver
import io.vakuum.solver.SolverSettings

/**
 * Orchestrates the full physics simulation pipeline.
 *
 * Manages bodies, shapes, and runs the simulation step:
 * clear forces → integrate → broadphase → narrowphase → solve → position correction.
 */
class SimulationPipeline(
    private val config: PhysicsConfiguration,
    broadphase: BroadphaseInterface? = null
) {

    private val bodies = mutableListOf<RigidBody>()
    private val shapes = mutableMapOf<Int, CollisionShape>()
    private val filters = mutableMapOf<Int, CollisionFilter>()

    private val solver = ImpulseSolver(SolverSettings(
        iterations = config.maxSolverIterations,
        defaultRestitution = 0.0,
        defaultFriction = 0.5,
        penetrationPercentage = 0.2
    ))
    private val broadphase: BroadphaseInterface = broadphase ?: TreeBroadphase(256)
    private val narrowphase = Narrowphase()

    // Contact cache for warm starting
    private val contactCache = mutableMapOf<Pair<Int, Int>, CachedContact>()

    // CCD diagnostics (optional, low overhead when disabled)
    var ccdDiagnostics = CcdDiagnostics()
        private set

    data class CachedContact(
        val normal: Vector3,
        val contactPoint: Vector3,
        val warmStartImpulse: Double
    )

    /**
     * CCD diagnostics metrics.
     * Reset at the start of each CCD step and populated during execution.
     */
    data class CcdDiagnostics(
        /** Number of CCD substeps executed in the last step. */
        val ccdSteps: Int = 0,
        /** Number of TOI queries performed. */
        val toiQueries: Int = 0,
        /** Maximum number of binary search iterations for TOI. */
        val toiIterations: Int = 0,
        /** Whether maxSubsteps was reached (budget exhausted). */
        val maxSubstepsReached: Boolean = false,
        /** Number of times CCD fell back to discrete collision. */
        val ccdFallbackCount: Int = 0,
        /** Maximum penetration depth observed during CCD resolution. */
        val maximumPenetration: Double = 0.0,
        /** Average CCD substeps per step (rolling). */
        val averageCCDSubsteps: Double = 0.0
    )

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
     * Set the collision filter for a body.
     */
    fun setFilter(bodyId: Int, filter: CollisionFilter) {
        filters[bodyId] = filter
    }

    /**
     * Get all bodies.
     */
    fun getBodies(): List<RigidBody> = bodies.toList()

    /**
     * Execute one simulation step.
     *
     * If CCD is enabled, uses substep-based TOI processing to prevent tunneling.
     * Otherwise, uses the standard discrete collision detection pipeline.
     */
    fun step() {
        val dt = config.fixedTimestep

        if (config.ccdEnabled) {
            stepWithCcd(dt)
        } else {
            stepDiscrete(dt)
        }
    }

    /**
     * Standard discrete collision detection step.
     */
    private fun stepDiscrete(dt: Double) {
        // 1. Clear forces
        for (body in bodies) {
            body.clearForces()
        }

        // 2. Integrate (only awake bodies)
        for (body in bodies) {
            if (body.awake) {
                body.integrate(dt, config.gravity, config.linearDamping, config.angularDamping)
            }
        }

        // 3. Collision detection and resolution
        processCollisions(dt)

        // 4. Sleep check
        checkSleep()
    }

    /**
     * CCD-enabled simulation step using TOI substeps.
     *
     * Algorithm:
     * 1. Clear forces.
     * 2. For each CCD substep:
     *    a. Integrate velocities for the remaining time.
     *    b. Find the earliest TOI among all CCD-enabled bodies.
     *    c. Advance all bodies to the TOI time.
     *    d. Resolve the collision at TOI.
     *    e. Reduce remaining time and repeat.
     * 3. If no TOI found, advance all bodies to the end of the timestep.
     * 4. If max substeps reached, fall back to discrete collision.
     */
    private fun stepWithCcd(dt: Double) {
        // Reset diagnostics
        var diagSteps = 0
        var diagToiQueries = 0
        var diagMaxSubstepsReached = false
        var diagFallbackCount = 0
        var diagMaxPenetration = 0.0

        // 1. Clear forces
        for (body in bodies) {
            body.clearForces()
        }

        // 2. CCD substep loop
        var remainingDt = dt
        var substepCount = 0

        while (remainingDt > 1e-10 && substepCount < config.ccdMaxSubsteps) {
            substepCount++
            diagSteps++

            // Integrate velocities for the remaining time
            // This must happen each substep so velocities reflect current state after collisions
            for (body in bodies) {
                if (body.awake && body.bodyType == io.vakuum.dynamics.BodyType.DYNAMIC) {
                    val acceleration = body.force * body.massProperties.inverseMass + config.gravity
                    body.linearVelocity += acceleration * remainingDt

                    val angularAcceleration = body.massProperties.inverseInertiaTensor.multiplyVector(body.torque)
                    body.angularVelocity += angularAcceleration * remainingDt

                    val linDamp = 1.0 - (config.linearDamping + body.linearDamping).coerceIn(0.0, 1.0) * remainingDt
                    val angDamp = 1.0 - (config.angularDamping + body.angularDamping).coerceIn(0.0, 1.0) * remainingDt
                    body.linearVelocity *= linDamp
                    body.angularVelocity *= angDamp
                }
            }

            // Find the earliest TOI among all awake dynamic bodies
            var earliestToi: Ccd.ToiResult? = null
            var earliestToiFraction = 1.0

            for (body in bodies) {
                if (!body.awake || body.bodyType != io.vakuum.dynamics.BodyType.DYNAMIC) continue
                if (!Ccd.shouldUseCcd(body, config.ccdVelocityThreshold)) continue

                val shape = shapes[body.id] ?: continue

                // Check TOI against ground plane (y=0)
                diagToiQueries++
                val toiPlane = queryToiAgainstPlane(body, shape, remainingDt)
                if (toiPlane != null) {
                    val toiFrac = toiPlane.toi
                    if (toiFrac < earliestToiFraction) {
                        earliestToiFraction = toiFrac
                        earliestToi = toiPlane
                    }
                }

                // Check TOI against other static bodies
                for (other in bodies) {
                    if (other.id == body.id) continue
                    if (other.bodyType != io.vakuum.dynamics.BodyType.STATIC) continue
                    val otherShape = shapes[other.id] ?: continue

                    diagToiQueries++
                    val toi = queryToiAgainstStatic(body, shape, other, otherShape, remainingDt)
                    if (toi != null) {
                        val toiFrac = toi.toi
                        if (toiFrac < earliestToiFraction) {
                            earliestToiFraction = toiFrac
                            earliestToi = toi
                        }
                    }
                }
            }

            if (earliestToi != null && earliestToiFraction >= config.ccdToiTolerance) {
                // Advance all bodies to the TOI time
                val advanceDt = earliestToiFraction * remainingDt
                for (body in bodies) {
                    if (body.awake && body.bodyType == io.vakuum.dynamics.BodyType.DYNAMIC) {
                        Ccd.advanceBodyToToi(body, earliestToiFraction, remainingDt)

                        // Update orientation
                        if (body.angularVelocity.lengthSquared > 1e-10) {
                            val angSpeed = body.angularVelocity.length
                            val angAxis = body.angularVelocity / angSpeed
                            val deltaRotation = io.vakuum.math.Quaternion.fromAxisAngle(angAxis, angSpeed * advanceDt)
                            body.transform = io.vakuum.math.Transform(
                                position = body.transform.position,
                                rotation = deltaRotation * body.transform.rotation
                            )
                        }
                    }
                }

                // Resolve the collision at TOI
                resolveToiCollision(earliestToi, advanceDt)

                // Track max penetration
                val movingBody = bodies.find { it.id == earliestToi.movingBodyId }
                if (movingBody != null) {
                    val shape = shapes[movingBody.id]
                    if (shape is BoxShape) {
                        val contact = narrowphase.testBoxPlane(movingBody, shape, Vector3.UNIT_Y, 0.0)
                        if (contact != null && contact.penetrationDepth > diagMaxPenetration) {
                            diagMaxPenetration = contact.penetrationDepth
                        }
                    }
                }

                // Reduce remaining time
                remainingDt *= (1.0 - earliestToiFraction)
            } else {
                // No TOI found — advance all bodies to the end of the timestep
                for (body in bodies) {
                    if (body.awake && body.bodyType == io.vakuum.dynamics.BodyType.DYNAMIC) {
                        body.transform = io.vakuum.math.Transform(
                            position = body.transform.position + body.linearVelocity * remainingDt,
                            rotation = body.transform.rotation
                        )

                        // Update orientation
                        if (body.angularVelocity.lengthSquared > 1e-10) {
                            val angSpeed = body.angularVelocity.length
                            val angAxis = body.angularVelocity / angSpeed
                            val deltaRotation = io.vakuum.math.Quaternion.fromAxisAngle(angAxis, angSpeed * remainingDt)
                            body.transform = io.vakuum.math.Transform(
                                position = body.transform.position,
                                rotation = deltaRotation * body.transform.rotation
                            )
                        }
                    }
                }

                // Process any remaining collisions at the final position
                processCollisions(remainingDt)
                diagFallbackCount++
                remainingDt = 0.0
            }
        }

        // If we hit the max substep limit, fall back to discrete collision
        if (substepCount >= config.ccdMaxSubsteps) {
            diagMaxSubstepsReached = true
            diagFallbackCount++
            processCollisions(remainingDt)
        }

        // Update diagnostics
        ccdDiagnostics = CcdDiagnostics(
            ccdSteps = diagSteps,
            toiQueries = diagToiQueries,
            toiIterations = 20, // max binary search iterations per TOI
            maxSubstepsReached = diagMaxSubstepsReached,
            ccdFallbackCount = diagFallbackCount,
            maximumPenetration = diagMaxPenetration,
            averageCCDSubsteps = diagSteps.toDouble()
        )

        // 3. Sleep check
        checkSleep()
    }

    /**
     * Resolve a single TOI collision using the impulse solver.
     * Supports all collision shape types: Box, Sphere, Capsule, ConvexHull.
     */
    private fun resolveToiCollision(toi: Ccd.ToiResult, subDt: Double) {
        val movingBody = bodies.find { it.id == toi.movingBodyId } ?: return
        val constraints = mutableListOf<ContactConstraint>()

        if (toi.hitBodyId == -1) {
            // Ground plane collision
            val shape = shapes[movingBody.id] ?: return
            val contact = queryContactAgainstPlane(movingBody, shape)
            if (contact != null) {
                movingBody.wakeUp()
                val localGround = io.vakuum.dynamics.rigidBody()
                    .id(-1)
                    .bodyType(BodyType.STATIC)
                    .position(Vector3.ZERO)
                    .mass(0.0)
                    .restitution(movingBody.restitution)
                    .friction(movingBody.friction)
                    .build()

                constraints.add(ContactConstraint(
                    bodyA = movingBody,
                    bodyB = localGround,
                    manifold = contact,
                    restitution = movingBody.restitution,
                    friction = movingBody.friction
                ))
            }
        } else {
            // Body-body collision
            val staticBody = bodies.find { it.id == toi.hitBodyId } ?: return
            val shapeA = shapes[movingBody.id] ?: return
            val shapeB = shapes[staticBody.id] ?: return
            val contact = queryContact(movingBody, shapeA, staticBody, shapeB)
            if (contact != null) {
                movingBody.wakeUp()
                constraints.add(ContactConstraint(
                    bodyA = movingBody,
                    bodyB = staticBody,
                    manifold = contact,
                    restitution = kotlin.math.max(movingBody.restitution, staticBody.restitution),
                    friction = kotlin.math.sqrt(movingBody.friction * staticBody.friction)
                ))
            }
        }

        if (constraints.isNotEmpty()) {
            solver.solve(constraints, subDt)
        }
    }

    /**
     * Query TOI against the ground plane (y=0) for any shape type.
     */
    private fun queryToiAgainstPlane(body: RigidBody, shape: CollisionShape, dt: Double): Ccd.ToiResult? {
        return when (shape) {
            is BoxShape -> Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)
            is SphereShape -> Ccd.toiSpherePlane(body, shape, Vector3.UNIT_Y, 0.0, dt)
            is CapsuleShape -> Ccd.toiCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0, dt)
            is ConvexHullShape -> Ccd.toiConvexPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)
            is PlaneShape -> null // Plane vs plane not supported
        }
    }

    /**
     * Query TOI against a static body for any shape pair.
     */
    private fun queryToiAgainstStatic(
        movingBody: RigidBody, movingShape: CollisionShape,
        staticBody: RigidBody, staticShape: CollisionShape,
        dt: Double
    ): Ccd.ToiResult? {
        return when {
            // Box vs static Box
            movingShape is BoxShape && staticShape is BoxShape ->
                Ccd.toiBoxBox(movingBody, movingShape, staticBody, staticShape, dt)
            // Sphere vs static Box
            movingShape is SphereShape && staticShape is BoxShape ->
                Ccd.toiSphereBox(movingBody, movingShape, staticBody, staticShape, dt)
            // Sphere vs static Sphere
            movingShape is SphereShape && staticShape is SphereShape ->
                Ccd.toiSphereSphere(movingBody, movingShape, staticBody, staticShape, dt)
            // Capsule vs static Box
            movingShape is CapsuleShape && staticShape is BoxShape ->
                Ccd.toiCapsuleBox(movingBody, movingShape, staticBody, staticShape, dt)
            // Capsule vs static Sphere
            movingShape is CapsuleShape && staticShape is SphereShape ->
                Ccd.toiCapsuleSphere(movingBody, movingShape, staticBody, staticShape, dt)
            // ConvexHull vs static Box
            movingShape is ConvexHullShape && staticShape is BoxShape ->
                Ccd.toiConvexBox(movingBody, movingShape, staticBody, staticShape, dt)
            // ConvexHull vs static ConvexHull
            movingShape is ConvexHullShape && staticShape is ConvexHullShape ->
                Ccd.toiConvexConvex(movingBody, movingShape, staticBody, staticShape, dt)
            else -> null
        }
    }

    /**
     * Query contact against the ground plane (y=0) for any shape type.
     */
    private fun queryContactAgainstPlane(body: RigidBody, shape: CollisionShape): ContactManifold? {
        return when (shape) {
            is BoxShape -> narrowphase.testBoxPlane(body, shape, Vector3.UNIT_Y, 0.0)
            is SphereShape -> narrowphase.testSpherePlane(body, shape, Vector3.UNIT_Y, 0.0)
            is CapsuleShape -> narrowphase.testCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0)
            is ConvexHullShape -> narrowphase.testConvexPlane(body, shape, Vector3.UNIT_Y, 0.0)
            is PlaneShape -> null
        }
    }

    /**
     * Query contact between two bodies for any shape pair.
     */
    private fun queryContact(
        bodyA: RigidBody, shapeA: CollisionShape,
        bodyB: RigidBody, shapeB: CollisionShape
    ): ContactManifold? {
        return when {
            shapeA is BoxShape && shapeB is BoxShape ->
                narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)
            shapeA is SphereShape && shapeB is BoxShape ->
                narrowphase.testSphereBox(bodyA, shapeA, bodyB, shapeB)
            shapeA is BoxShape && shapeB is SphereShape ->
                narrowphase.testSphereBox(bodyB, shapeB, bodyA, shapeA)
            shapeA is SphereShape && shapeB is SphereShape ->
                narrowphase.testSphereSphere(bodyA, bodyB, shapeA, shapeB)
            shapeA is CapsuleShape && shapeB is BoxShape ->
                narrowphase.testCapsuleBox(bodyA, shapeA, bodyB, shapeB)
            shapeA is BoxShape && shapeB is CapsuleShape ->
                narrowphase.testCapsuleBox(bodyB, shapeB, bodyA, shapeA)
            shapeA is CapsuleShape && shapeB is SphereShape ->
                narrowphase.testCapsuleSphere(bodyA, shapeA, bodyB, shapeB)
            shapeA is SphereShape && shapeB is CapsuleShape ->
                narrowphase.testCapsuleSphere(bodyB, shapeB, bodyA, shapeA)
            shapeA is ConvexHullShape && shapeB is BoxShape ->
                narrowphase.testConvexBox(bodyA, shapeA, bodyB, shapeB)
            shapeA is BoxShape && shapeB is ConvexHullShape ->
                narrowphase.testConvexBox(bodyB, shapeB, bodyA, shapeA)
            shapeA is ConvexHullShape && shapeB is ConvexHullShape ->
                narrowphase.testConvexConvex(bodyA, shapeA, bodyB, shapeB)
            else -> null
        }
    }

    /**
     * Process collision detection and resolution for all awake bodies.
     */
    private fun processCollisions(dt: Double) {
        val awakeBodies = bodies.filter { it.awake }
        val pairs = broadphase.findPairs(awakeBodies, shapes, filters.ifEmpty { null })

        // Collect all contact constraints
        val constraints = mutableListOf<ContactConstraint>()

        // Ground plane collisions for all shape types
        for (body in awakeBodies) {
            val shape = shapes[body.id] ?: continue
            val contact = queryContactAgainstPlane(body, shape)
            if (contact != null) {
                body.wakeUp()

                // Check contact cache for warm starting
                val cacheKey = body.id to -1
                val cached = contactCache[cacheKey]
                val contactWithWarmStart = if (cached != null && cached.normal.dot(contact.normal) > 0.1) {
                    contact.copy(warmStartImpulse = cached.warmStartImpulse)
                } else {
                    contact
                }

                // Create a per-contact ground body to avoid shared mutable state in parallel tests
                val localGround = io.vakuum.dynamics.rigidBody()
                    .id(-1)
                    .bodyType(BodyType.STATIC)
                    .position(Vector3.ZERO)
                    .mass(0.0)
                    .restitution(body.restitution)
                    .friction(body.friction)
                    .build()

                constraints.add(ContactConstraint(
                    bodyA = body,
                    bodyB = localGround,
                    manifold = contactWithWarmStart,
                    restitution = body.restitution,
                    friction = body.friction
                ))
            }
        }

        // Body-body collisions with contact caching (all shape types)
        val newContacts = mutableMapOf<Pair<Int, Int>, ContactManifold>()
        for (pair in pairs) {
            val bodyA = bodies.find { it.id == pair.bodyAId } ?: continue
            val bodyB = bodies.find { it.id == pair.bodyBId } ?: continue
            val shapeA = shapes[pair.bodyAId] ?: continue
            val shapeB = shapes[pair.bodyBId] ?: continue

            val contact = queryContact(bodyA, shapeA, bodyB, shapeB)
            if (contact != null) {
                bodyA.wakeUp()
                bodyB.wakeUp()

                // Check contact cache for warm starting
                val cacheKey = pair.bodyAId to pair.bodyBId
                val cached = contactCache[cacheKey]
                val contactWithWarmStart = if (cached != null) {
                    if (cached.normal.dot(contact.normal) > 0.1) {
                        contact.copy(warmStartImpulse = cached.warmStartImpulse)
                    } else {
                        contact
                    }
                } else {
                    contact
                }

                constraints.add(ContactConstraint(
                    bodyA = bodyA,
                    bodyB = bodyB,
                    manifold = contactWithWarmStart,
                    restitution = kotlin.math.max(bodyA.restitution, bodyB.restitution),
                    friction = kotlin.math.sqrt(bodyA.friction * bodyB.friction)
                ))
                newContacts[cacheKey] = contactWithWarmStart
            }
        }

        // Iterative solver: solve all constraints together
        if (constraints.isNotEmpty()) {
            solver.solve(constraints, dt)

            // Update contact cache with accumulated impulses from constraints
            for (constraint in constraints) {
                val bodyAId = constraint.bodyA.id
                val bodyBId = constraint.bodyB.id
                val cacheKey = if (bodyAId < bodyBId) bodyAId to bodyBId else bodyBId to bodyAId
                contactCache[cacheKey] = CachedContact(
                    normal = constraint.normal,
                    contactPoint = constraint.contactPoint,
                    warmStartImpulse = constraint.totalNormalImpulse.coerceIn(-1000.0, 1000.0)
                )
            }
        }

        // Remove cached contacts for pairs that no longer collide
        val activeKeys = newContacts.keys.toSet()
        // Keep ground contacts in cache
        val toRemove = contactCache.keys.filter {
            it !in activeKeys && it.first != -1 && it.second != -1
        }
        for (key in toRemove) {
            contactCache.remove(key)
        }
    }

    /**
     * Check sleeping conditions for all bodies.
     */
    private fun checkSleep() {
        if (!config.allowSleeping) return

        for (body in bodies) {
            if (body.bodyType == BodyType.STATIC) continue
            if (!body.awake) continue

            if (body.isAtRest(config.sleepThreshold)) {
                body.sleepTimer++
                if (body.sleepTimer >= body.sleepTimeLimit) {
                    body.awake = false
                }
            } else {
                body.sleepTimer = 0
            }
        }
    }

    /**
     * Check if any body has NaN values.
     */
    fun hasNaN(): Boolean = bodies.any { it.hasNaN() }
}
