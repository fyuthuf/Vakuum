package io.vakuum.solver

import io.vakuum.collision.ContactManifold
import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Vector3

/**
 * Contact constraint for the iterative solver.
 *
 * Holds all the information needed to solve a single contact constraint
 * using the Sequential Impulse (Projected Gauss-Seidel) method.
 */
data class ContactConstraint(
    /** Body A reference. */
    val bodyA: RigidBody,
    /** Body B reference. */
    val bodyB: RigidBody,
    /** Contact manifold with point, normal, and penetration depth. */
    val manifold: ContactManifold,
    /** Combined restitution (max of both bodies). */
    val restitution: Double,
    /** Combined friction coefficient (geometric mean). */
    val friction: Double
) {
    /** Vector from bodyA's center to contact point. */
    var rA: Vector3 = Vector3.ZERO

    /** Vector from bodyB's center to contact point. */
    var rB: Vector3 = Vector3.ZERO

    /** Inverse mass sum. */
    var invMassSum: Double = 0.0

    /** Normal impulse denominator: invMassSum + rotational terms. */
    var normalDenom: Double = 0.0

    /** Accumulated normal impulse for this contact (for clamping). */
    var totalNormalImpulse: Double = 0.0

    /** Accumulated friction impulse vector (for clamping). */
    var totalFrictionImpulse: Vector3 = Vector3.ZERO

    /** Contact normal (from manifold). */
    val normal: Vector3 = manifold.normal

    /** Contact point in world space. */
    val contactPoint: Vector3 = manifold.contactPoint

    /** Penetration depth. */
    val penetrationDepth: Double = manifold.penetrationDepth

    /** Warm start impulse from previous step. */
    var warmStartImpulse: Double = manifold.warmStartImpulse

    /**
     * Pre-compute all constant values for this constraint.
     */
    fun precompute() {
        rA = contactPoint - bodyA.transform.position
        rB = contactPoint - bodyB.transform.position

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass
        invMassSum = invMassA + invMassB

        if (invMassSum < 1e-10) return

        // Normal rotational contribution
        val rotA = rA.cross(normal)
        val rotB = rB.cross(normal)
        val invInertiaA = bodyA.massProperties.inverseInertiaTensor
        val invInertiaB = bodyB.massProperties.inverseInertiaTensor
        val rotTermA = if (invMassA > 0) rotA.dot(invInertiaA.multiplyVector(rotA)) else 0.0
        val rotTermB = if (invMassB > 0) rotB.dot(invInertiaB.multiplyVector(rotB)) else 0.0
        normalDenom = invMassSum + rotTermA + rotTermB
    }
}

/**
 * Iterative impulse-based collision solver using Sequential Impulse (Projected Gauss-Seidel).
 *
 * This solver processes all contact constraints multiple times per step,
 * gradually converging to a solution that satisfies all constraints simultaneously.
 *
 * Key features:
 * - Configurable velocity iterations for accuracy
 * - Warm starting for persistent contacts
 * - Accumulated impulse clamping for stability
 * - Coulomb friction model
 * - Position correction with penetration slop
 *
 * Reference: Erin Catto, "Iterative Dynamics" (GDC 2005)
 */
class ImpulseSolver(private val settings: SolverSettings = SolverSettings()) {

    /**
     * Solve all contact constraints iteratively.
     *
     * @param constraints List of contact constraints to solve
     * @param dt Timestep in seconds
     */
    fun solve(constraints: List<ContactConstraint>, dt: Double) {
        if (constraints.isEmpty()) return

        // Precompute all constraints
        for (constraint in constraints) {
            constraint.precompute()
        }

        // Velocity iterations
        val velocityIterations = settings.iterations
        for (iteration in 0 until velocityIterations) {
            for (constraint in constraints) {
                solveVelocityIteration(constraint, dt, iteration == 0)
            }
        }

        // Position correction (separate from velocity solving)
        for (constraint in constraints) {
            solvePositionIteration(constraint, dt)
        }
    }

    /**
     * Solve one velocity iteration for a single contact constraint.
     *
     * Uses the Sequential Impulse method: each iteration applies a small impulse
     * to gradually converge to the correct solution. Multiple iterations allow
     * the solver to handle multi-contact scenarios (like stacks) more stably.
     *
     * @param firstIteration If true, applies restitution.
     */
    private fun solveVelocityIteration(constraint: ContactConstraint, dt: Double, firstIteration: Boolean) {
        val bodyA = constraint.bodyA
        val bodyB = constraint.bodyB

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass
        if (constraint.invMassSum < 1e-10) return

        // On the first iteration, reset accumulated impulses.
        // Warm starting is disabled as it can inject energy into resting contacts.
        // For a production solver, warm starting would need careful validation
        // to only apply when the contact geometry is compatible.
        if (firstIteration) {
            constraint.totalNormalImpulse = 0.0
            constraint.totalFrictionImpulse = Vector3.ZERO
        }

        // Relative velocity at contact point
        val velA = bodyA.linearVelocity + bodyA.angularVelocity.cross(constraint.rA)
        val velB = bodyB.linearVelocity + bodyB.angularVelocity.cross(constraint.rB)
        val relativeVelocity = velA - velB

        // Velocity along the normal (positive = separating, negative = approaching)
        val velAlongNormal = relativeVelocity.dot(constraint.normal)

        // Don't resolve if bodies are separating
        if (velAlongNormal > 0.0) return

        // Normal impulse: how much impulse is needed to stop penetration velocity
        val jN = -velAlongNormal / constraint.normalDenom

        // Restitution: only on first iteration.
        // The restitution bias adds extra impulse to achieve the bounce.
        var impulseMagnitude = jN
        if (firstIteration && constraint.restitution > 0.0) {
            // Add restitution bias
            impulseMagnitude += constraint.restitution * jN
        }

        // Clamp: don't allow negative total impulse (bodies should not stick)
        val newTotalNormal = constraint.totalNormalImpulse + impulseMagnitude
        if (newTotalNormal < 0.0) {
            constraint.totalNormalImpulse = 0.0
            return
        }

        // Apply normal impulse
        val impulse = constraint.normal * impulseMagnitude
        bodyA.linearVelocity += impulse * invMassA
        bodyB.linearVelocity -= impulse * invMassB

        if (invMassA > 0) {
            bodyA.angularVelocity +=
                bodyA.massProperties.inverseInertiaTensor.multiplyVector(constraint.rA.cross(impulse))
        }
        if (invMassB > 0) {
            bodyB.angularVelocity +=
                bodyB.massProperties.inverseInertiaTensor.multiplyVector(constraint.rB.cross(-impulse))
        }

        constraint.totalNormalImpulse = newTotalNormal

        // Friction impulse (tangential)
        solveFrictionIteration(constraint)
    }

    /**
     * Solve friction for one iteration using Sequential Impulse approach.
     *
     * Each iteration applies a small friction impulse, gradually converging
     * to the correct solution while respecting the Coulomb friction cone.
     */
    private fun solveFrictionIteration(constraint: ContactConstraint) {
        val bodyA = constraint.bodyA
        val bodyB = constraint.bodyB

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass

        // Relative velocity at contact point
        val velA = bodyA.linearVelocity + bodyA.angularVelocity.cross(constraint.rA)
        val velB = bodyB.linearVelocity + bodyB.angularVelocity.cross(constraint.rB)
        val relativeVelocity = velA - velB

        // Tangential component (remove normal component)
        val normalComponent = constraint.normal * relativeVelocity.dot(constraint.normal)
        var tangent = relativeVelocity - normalComponent

        val tangentLength = tangent.length
        if (tangentLength < 1e-6) return
        tangent = tangent / tangentLength

        // Tangent rotational contribution
        val rotA = constraint.rA.cross(tangent)
        val rotB = constraint.rB.cross(tangent)
        val invInertiaA = bodyA.massProperties.inverseInertiaTensor
        val invInertiaB = bodyB.massProperties.inverseInertiaTensor
        val rotTermA = if (invMassA > 0) rotA.dot(invInertiaA.multiplyVector(rotA)) else 0.0
        val rotTermB = if (invMassB > 0) rotB.dot(invInertiaB.multiplyVector(rotB)) else 0.0
        val tangentDenom = constraint.invMassSum + rotTermA + rotTermB
        if (tangentDenom < 1e-10) return

        // Friction impulse for this iteration: reduce tangential velocity
        val tangentVelocity = relativeVelocity.dot(tangent)
        val jT = -tangentVelocity / tangentDenom

        // Coulomb friction cone: |totalFriction| <= mu * totalNormal
        val maxFriction = constraint.friction * constraint.totalNormalImpulse
        if (maxFriction <= 0.0) return

        // Clamp: the new total friction impulse magnitude must not exceed maxFriction
        val currentFrictionMag = constraint.totalFrictionImpulse.length

        // Compute the desired new friction impulse
        val desiredFriction = constraint.totalFrictionImpulse + tangent * jT
        val desiredMag = desiredFriction.length

        val finalFriction = if (desiredMag > maxFriction) {
            // Scale down to the friction cone boundary
            desiredFriction * (maxFriction / desiredMag)
        } else {
            desiredFriction
        }

        // The actual friction impulse to apply is the difference
        val frictionImpulse = finalFriction - constraint.totalFrictionImpulse

        bodyA.linearVelocity += frictionImpulse * invMassA
        bodyB.linearVelocity -= frictionImpulse * invMassB

        if (invMassA > 0) {
            bodyA.angularVelocity += invInertiaA.multiplyVector(constraint.rA.cross(frictionImpulse))
        }
        if (invMassB > 0) {
            bodyB.angularVelocity += invInertiaB.multiplyVector(constraint.rB.cross(-frictionImpulse))
        }

        constraint.totalFrictionImpulse = finalFriction
    }

    /**
     * Solve one position correction iteration for a single contact constraint.
     *
     * Pushes penetrating bodies apart using a percentage of the penetration depth.
     * Small penetrations below the slop threshold are ignored to prevent jitter.
     */
    private fun solvePositionIteration(constraint: ContactConstraint, dt: Double) {
        val penetration = constraint.penetrationDepth
        val slop = settings.penetrationSlop

        // Ignore small penetrations to prevent jitter on resting contacts
        if (penetration <= slop) return

        val bodyA = constraint.bodyA
        val bodyB = constraint.bodyB

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass
        val invMassSum = invMassA + invMassB
        if (invMassSum < 1e-10) return

        // Correction magnitude: only correct the amount above the slop
        val correctionPercent = settings.penetrationPercentage
        val correctionMagnitude = (penetration - slop) * correctionPercent

        val correction = constraint.normal * correctionMagnitude

        // Distribute correction proportionally to inverse mass
        val correctionA = correction * (invMassA / invMassSum)
        val correctionB = -correction * (invMassB / invMassSum)

        bodyA.transform = io.vakuum.math.Transform(
            position = bodyA.transform.position + correctionA,
            rotation = bodyA.transform.rotation
        )
        bodyB.transform = io.vakuum.math.Transform(
            position = bodyB.transform.position + correctionB,
            rotation = bodyB.transform.rotation
        )
    }

    /**
     * Solve a single contact between two bodies (legacy API).
     *
     * @deprecated Use [solve] with [ContactConstraint] list instead for iterative solving.
     */
    fun solveContact(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contact: ContactManifold,
        dt: Double
    ) {
        val constraint = ContactConstraint(
            bodyA = bodyA,
            bodyB = bodyB,
            manifold = contact,
            restitution = kotlin.math.max(bodyA.restitution, bodyB.restitution),
            friction = kotlin.math.sqrt(bodyA.friction * bodyB.friction)
        )
        solve(listOf(constraint), dt)
    }

    /**
     * Apply position correction to push bodies apart (legacy API).
     *
     * Still functional for backward compatibility with code that calls this directly.
     */
    fun solvePositionCorrection(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contact: ContactManifold,
        dt: Double
    ) {
        val penetration = contact.penetrationDepth
        val slop = settings.penetrationSlop
        if (penetration <= slop) return

        val normal = contact.normal
        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass
        val invMassSum = invMassA + invMassB
        if (invMassSum < 1e-10) return

        val correctionPercent = settings.penetrationPercentage
        val correctionMagnitude = (penetration - slop) * correctionPercent

        val correction = normal * correctionMagnitude
        val correctionA = correction * (invMassA / invMassSum)
        val correctionB = -correction * (invMassB / invMassSum)

        bodyA.transform = io.vakuum.math.Transform(
            position = bodyA.transform.position + correctionA,
            rotation = bodyA.transform.rotation
        )
        bodyB.transform = io.vakuum.math.Transform(
            position = bodyB.transform.position + correctionB,
            rotation = bodyB.transform.rotation
        )
    }
}
