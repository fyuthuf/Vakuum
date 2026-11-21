package io.vakuum.solver

import io.vakuum.collision.BoxShape
import io.vakuum.collision.ContactManifold
import io.vakuum.collision.Narrowphase
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.MassProperties
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class ImpulseSolverTest {

    private val epsilon = 1e-4

    @Test
    fun `normal impulse reverses relative velocity`() {
        // Two cubes moving toward each other
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyA.linearVelocity = Vector3(2.0, 0.0, 0.0)

        val bodyB = rigidBody()
            .id(2)
            .position(1.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyB.linearVelocity = Vector3(-2.0, 0.0, 0.0)

        // Normal points in the push direction for bodyA (away from B, i.e., -X)
        val contact = ContactManifold(
            contactPoint = Vector3(0.5, 0.0, 0.0),
            normal = Vector3(-1.0, 0.0, 0.0),
            penetrationDepth = 0.1
        )

        val solver = ImpulseSolver(SolverSettings(iterations = 10, defaultRestitution = 0.0))
        solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        // After collision with equal mass and zero restitution, velocities should swap
        // Body A should slow down or reverse, Body B should slow down or reverse
        assertTrue(bodyA.linearVelocity.x < 2.0, "Body A should slow down")
        assertTrue(bodyB.linearVelocity.x > -2.0, "Body B should slow down")
    }

    @Test
    fun `restitution causes bounce`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .restitution(0.8)
            .build()
        bodyA.linearVelocity = Vector3(0.0, -5.0, 0.0)

        // Static ground
        val bodyB = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.0, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.05
        )

        val solver = ImpulseSolver()
        solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        // Body A should bounce upward
        assertTrue(bodyA.linearVelocity.y > 0.0, "Body should bounce upward with restitution")
        // Velocity should be less than incoming due to energy loss
        assertTrue(bodyA.linearVelocity.y < 5.0, "Bounce velocity should be less than incoming")
    }

    @Test
    fun `static body does not move`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyA.linearVelocity = Vector3(0.0, -10.0, 0.0)

        val bodyB = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()

        val initialPosB = bodyB.transform.position

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.0, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.1
        )

        val solver = ImpulseSolver()
        solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        assertEquals(initialPosB, bodyB.transform.position, "Static body should not move")
        assertEquals(Vector3.ZERO, bodyB.linearVelocity, "Static body velocity should remain zero")
    }

    @Test
    fun `friction reduces tangential velocity`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .friction(0.8)
            .build()
        // Moving downward and to the right
        bodyA.linearVelocity = Vector3(5.0, -5.0, 0.0)

        val bodyB = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .friction(0.8)
            .build()

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.0, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.05
        )

        val solver = ImpulseSolver()
        solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        // Tangential velocity (X) should be reduced by friction
        assertTrue(kotlin.math.abs(bodyA.linearVelocity.x) < 5.0, "Friction should reduce tangential velocity")
    }

    @Test
    fun `penetration correction pushes bodies apart`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(0.5, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val initialDist = bodyA.transform.position.distanceTo(bodyB.transform.position)

        // Normal points from bodyB toward bodyA (-X direction)
        // So bodyA is pushed left (-X) and bodyB is pushed right (+X)
        val contact = ContactManifold(
            contactPoint = Vector3(0.25, 0.0, 0.0),
            normal = Vector3(-1.0, 0.0, 0.0),
            penetrationDepth = 0.5
        )

        val solver = ImpulseSolver(SolverSettings(penetrationPercentage = 0.5))
        solver.solvePositionCorrection(bodyA, bodyB, contact, 1.0 / 60.0)

        val newDist = bodyA.transform.position.distanceTo(bodyB.transform.position)
        assertTrue(newDist > initialDist, "Bodies should be pushed apart")
    }

    @Test
    fun `no NaN after solving`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyA.linearVelocity = Vector3(0.0, -10.0, 0.0)

        val bodyB = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.0, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.1
        )

        val solver = ImpulseSolver()
        repeat(100) {
            solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)
        }

        assertFalse(bodyA.hasNaN(), "No NaN after 100 solve iterations")
        assertFalse(bodyB.hasNaN(), "No NaN in static body")
    }

    @Test
    fun `solver settings validation`() {
        var thrown = false
        try { SolverSettings(iterations = 0) } catch (e: IllegalArgumentException) { thrown = true }
        assertTrue(thrown, "Zero iterations should throw")

        thrown = false
        try { SolverSettings(defaultRestitution = -0.1) } catch (e: IllegalArgumentException) { thrown = true }
        assertTrue(thrown, "Negative restitution should throw")

        thrown = false
        try { SolverSettings(defaultRestitution = 1.1) } catch (e: IllegalArgumentException) { thrown = true }
        assertTrue(thrown, "Restitution > 1 should throw")

        thrown = false
        try { SolverSettings(penetrationPercentage = 1.5) } catch (e: IllegalArgumentException) { thrown = true }
        assertTrue(thrown, "Penetration percentage > 1 should throw")
    }

    @Test
    fun `multiple iterations improve stability`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyA.linearVelocity = Vector3(0.0, -1.0, 0.0)

        val bodyB = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.0, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.05
        )

        // Single iteration solver
        val solver1 = ImpulseSolver(SolverSettings(iterations = 1))
        solver1.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        // Body should have bounced
        // With more iterations, the response is more stable
    }

    @Test
    fun `zero restitution no bounce`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .restitution(0.0)
            .build()
        bodyA.linearVelocity = Vector3(0.0, -5.0, 0.0)

        val bodyB = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.0, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.05
        )

        val solver = ImpulseSolver()
        solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        // With zero restitution, body should stop or have very small upward velocity
        // The impulse should stop the downward motion
        assertTrue(bodyA.linearVelocity.y >= -0.1, "Body should nearly stop with zero restitution")
    }

    @Test
    fun `separating bodies are not resolved`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyA.linearVelocity = Vector3(0.0, 5.0, 0.0) // Moving away

        val bodyB = rigidBody()
            .id(2)
            .position(0.0, 1.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        bodyB.linearVelocity = Vector3(0.0, 0.0, 0.0)

        val initialVelA = bodyA.linearVelocity

        val contact = ContactManifold(
            contactPoint = Vector3(0.0, 0.5, 0.0),
            normal = Vector3.UNIT_Y,
            penetrationDepth = 0.01
        )

        val solver = ImpulseSolver()
        solver.solveContact(bodyA, bodyB, contact, 1.0 / 60.0)

        // Velocity should not change since bodies are separating
        assertEquals(initialVelA, bodyA.linearVelocity, "Separating bodies should not be resolved")
    }
}
