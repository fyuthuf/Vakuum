package io.vakuum.solver

import io.vakuum.collision.ContactManifold
import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Vector3

/**
 * Impulse-based collision solver.
 *
 * Uses iterative Gauss-Seidel relaxation to solve contact constraints.
 * Supports normal impulse, friction, and penetration correction (Baumgarte stabilization).
 */
class ImpulseSolver(private val settings: SolverSettings = SolverSettings()) {

    /**
     * Solve all contact constraints for the given bodies and contacts.
     * This method is provided for batch solving but requires body references to be linked to contacts.
     * For direct contact solving, use [solveContact] and [solvePositionCorrection] instead.
     */
    fun solve(bodies: List<RigidBody>, contacts: List<ContactManifold>, dt: Double) {
        // Batch solve is not yet fully implemented.
        // Use solveContact() and solvePositionCorrection() for individual contacts.
    }

    /**
     * Solve a single contact between two bodies.
     */
    fun solveContact(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contact: ContactManifold,
        dt: Double
    ) {
        val normal = contact.normal
        val contactPoint = contact.contactPoint

        val rA = contactPoint - bodyA.transform.position
        val rB = contactPoint - bodyB.transform.position

        // Velocity at contact point for each body
        val velA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA)
        val velB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB)

        // Relative velocity: how fast A is moving relative to B at the contact
        val relativeVelocity = velA - velB

        // Velocity along the normal
        val velAlongNormal = relativeVelocity.dot(normal)

        // Don't resolve if bodies are separating along the normal
        // The normal points in the push direction for bodyA.
        // If velAlongNormal < 0, bodyA moves toward bodyB (approaching) — resolve.
        // If velAlongNormal > 0, bodyA moves away from bodyB (separating) — skip.
        val approachVelocity = -velAlongNormal

        // Don't resolve if bodies are separating
        if (approachVelocity <= 0.0) return

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass
        if (invMassA + invMassB < 1e-10) return

        // Rotational contribution
        val rotA = rA.cross(normal)
        val rotB = rB.cross(normal)
        val invInertiaA = bodyA.massProperties.inverseInertiaTensor
        val invInertiaB = bodyB.massProperties.inverseInertiaTensor
        val rotTermA = if (invMassA > 0) rotA.dot(invInertiaA.multiplyVector(rotA)) else 0.0
        val rotTermB = if (invMassB > 0) rotB.dot(invInertiaB.multiplyVector(rotB)) else 0.0

        val denom = invMassA + invMassB + rotTermA + rotTermB
        if (denom < 1e-10) return

        val restitution = kotlin.math.max(bodyA.restitution, bodyB.restitution)

        // Normal impulse magnitude: j = (1+e) * approachVelocity / denom
        // approachVelocity > 0 means bodies are approaching
        val jN = (1.0 + restitution) * approachVelocity / denom

        // Apply impulse: push bodyA along +normal (away from bodyB)
        val impulse = normal * jN
        bodyA.linearVelocity += impulse * invMassA
        bodyB.linearVelocity -= impulse * invMassB

        // Angular impulse
        if (invMassA > 0) {
            bodyA.angularVelocity += invInertiaA.multiplyVector(rA.cross(impulse))
        }
        if (invMassB > 0) {
            bodyB.angularVelocity += invInertiaB.multiplyVector(rB.cross(-impulse))
        }

        // Friction
        if (jN > 0.0) {
            solveFriction(bodyA, bodyB, contact, rA, rB, jN)
        }
    }

    private fun solveFriction(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contact: ContactManifold,
        rA: Vector3,
        rB: Vector3,
        normalImpulse: Double
    ) {
        val normal = contact.normal

        val velA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA)
        val velB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB)
        val relativeVelocity = velA - velB

        // Tangential component
        val normalComponent = normal * relativeVelocity.dot(normal)
        var tangent = relativeVelocity - normalComponent

        val tangentLength = tangent.length
        if (tangentLength < 1e-6) return
        tangent = tangent / tangentLength

        val friction = kotlin.math.sqrt(bodyA.friction * bodyB.friction)

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass

        val rotA = rA.cross(tangent)
        val rotB = rB.cross(tangent)
        val invInertiaA = bodyA.massProperties.inverseInertiaTensor
        val invInertiaB = bodyB.massProperties.inverseInertiaTensor
        val rotTermA = if (invMassA > 0) rotA.dot(invInertiaA.multiplyVector(rotA)) else 0.0
        val rotTermB = if (invMassB > 0) rotB.dot(invInertiaB.multiplyVector(rotB)) else 0.0

        val denom = invMassA + invMassB + rotTermA + rotTermB
        if (denom < 1e-10) return

        val tangentVelocity = relativeVelocity.dot(tangent)
        var jT = -tangentVelocity / denom
        val maxFriction = friction * normalImpulse
        jT = kotlin.math.max(-maxFriction, kotlin.math.min(maxFriction, jT))

        val frictionImpulse = tangent * jT
        bodyA.linearVelocity += frictionImpulse * invMassA
        bodyB.linearVelocity -= frictionImpulse * invMassB
        if (invMassA > 0) {
            bodyA.angularVelocity += invInertiaA.multiplyVector(rA.cross(frictionImpulse))
        }
        if (invMassB > 0) {
            bodyB.angularVelocity += invInertiaB.multiplyVector(rB.cross(-frictionImpulse))
        }
    }

    /**
     * Apply position correction to push bodies apart.
     */
    fun solvePositionCorrection(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contact: ContactManifold,
        dt: Double
    ) {
        val penetration = contact.penetrationDepth
        if (penetration < 1e-6) return

        val normal = contact.normal

        // Push bodies apart along the normal
        // Normal points in the push direction for bodyA
        val correctionPercent = settings.penetrationPercentage
        val slop = 0.001 // Penetration allowance in meters
        val correctionMagnitude = kotlin.math.max(penetration - slop, 0.0) * correctionPercent

        val invMassA = bodyA.massProperties.inverseMass
        val invMassB = bodyB.massProperties.inverseMass
        val invMassSum = invMassA + invMassB
        if (invMassSum < 1e-10) return

        // Correction displacement for each body, proportional to inverse mass
        val correctionA = normal * (correctionMagnitude * invMassA / invMassSum)
        val correctionB = -normal * (correctionMagnitude * invMassB / invMassSum)

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
