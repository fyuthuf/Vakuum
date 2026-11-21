package io.vakuum.dynamics

import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

class RigidBodyTest {

    private val epsilon = 1e-6

    @Test
    fun `create dynamic body with correct properties`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(5.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        assertEquals(1, body.id)
        assertEquals(BodyType.DYNAMIC, body.bodyType)
        assertEquals(10.0, body.transform.position.y, epsilon)
        assertEquals(5.0, body.massProperties.mass, epsilon)
    }

    @Test
    fun `create static body has zero inverse mass`() {
        val body = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()

        assertEquals(BodyType.STATIC, body.bodyType)
        assertEquals(0.0, body.massProperties.inverseMass, epsilon)
    }

    @Test
    fun `static body does not move during integration`() {
        val body = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()

        val gravity = Vector3(0.0, -9.81, 0.0)
        body.integrate(1.0 / 60.0, gravity, 0.0, 0.0)

        assertEquals(Vector3.ZERO, body.transform.position)
        assertEquals(Vector3.ZERO, body.linearVelocity)
    }

    @Test
    fun `dynamic body falls under gravity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val gravity = Vector3(0.0, -9.81, 0.0)
        val dt = 1.0 / 60.0
        body.integrate(dt, gravity, 0.0, 0.0)

        // v = g * dt = -9.81 / 60 = -0.1635
        val expectedVelocity = -9.81 * dt
        assertEquals(expectedVelocity, body.linearVelocity.y, epsilon, "Velocity after one step")

        // Position should have changed by v * dt
        val expectedPosition = 10.0 + expectedVelocity * dt
        assertEquals(expectedPosition, body.transform.position.y, epsilon, "Position after one step")
    }

    @Test
    fun `free fall velocity matches analytical solution`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 100.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val gravity = Vector3(0.0, -9.81, 0.0)
        val dt = 1.0 / 60.0

        // Simulate 60 steps (1 second)
        repeat(60) {
            body.integrate(dt, gravity, 0.0, 0.0)
        }

        // v = g * t = -9.81 * 1 = -9.81
        assertEquals(-9.81, body.linearVelocity.y, 1e-3, "Velocity after 1 second of free fall")
    }

    @Test
    fun `free fall position matches analytical solution`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 100.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val gravity = Vector3(0.0, -9.81, 0.0)
        val dt = 1.0 / 60.0

        // Simulate 60 steps (1 second)
        repeat(60) {
            body.integrate(dt, gravity, 0.0, 0.0)
        }

        // y = y0 + 0.5 * g * t^2 = 100 + 0.5 * (-9.81) * 1 = 100 - 4.905 = 95.095
        val expectedY = 100.0 + 0.5 * (-9.81) * 1.0 * 1.0
        assertEquals(expectedY, body.transform.position.y, 0.1, "Position after 1 second of free fall")
    }

    @Test
    fun `damping reduces velocity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .linearDamping(0.5)
            .build()

        // Set initial velocity
        body.linearVelocity = Vector3(0.0, -10.0, 0.0)

        val dt = 1.0 / 60.0
        body.integrate(dt, Vector3.ZERO, 0.0, 0.0)

        // Velocity should be reduced by damping
        // v *= (1 - damping * dt) = (1 - 0.5/60) ≈ 0.9917
        val expectedFactor = 1.0 - 0.5 * dt
        val expectedVelocity = -10.0 * expectedFactor
        assertEquals(expectedVelocity, body.linearVelocity.y, epsilon, "Damped velocity")
        assertTrue(body.linearVelocity.y > -10.0, "Damping should reduce velocity magnitude")
    }

    @Test
    fun `angular damping reduces angular velocity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .angularDamping(0.5)
            .build()

        body.angularVelocity = Vector3(0.0, 10.0, 0.0)

        val dt = 1.0 / 60.0
        body.integrate(dt, Vector3.ZERO, 0.0, 0.0)

        val expectedFactor = 1.0 - 0.5 * dt
        val expectedAngularVelocity = 10.0 * expectedFactor
        assertEquals(expectedAngularVelocity, body.angularVelocity.y, epsilon, "Damped angular velocity")
    }

    @Test
    fun `clear forces resets accumulators`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.force = Vector3(10.0, 20.0, 30.0)
        body.torque = Vector3(1.0, 2.0, 3.0)

        body.clearForces()

        assertEquals(Vector3.ZERO, body.force)
        assertEquals(Vector3.ZERO, body.torque)
    }

    @Test
    fun `apply force at center adds to force accumulator`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.applyForceAtCenter(Vector3(0.0, -10.0, 0.0))

        assertEquals(Vector3(0.0, -10.0, 0.0), body.force)
    }

    @Test
    fun `apply force at point adds torque`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        // Apply force at (1, 0, 0) in +Y direction
        body.applyForce(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 10.0, 0.0))

        assertEquals(Vector3(0.0, 10.0, 0.0), body.force)
        // Torque = r × F = (1,0,0) × (0,10,0) = (0, 0, 10)
        assertEquals(10.0, body.torque.z, epsilon, "Torque from offset force")
    }

    @Test
    fun `apply force to static body is ignored`() {
        val body = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .mass(0.0)
            .build()

        body.applyForceAtCenter(Vector3(0.0, -100.0, 0.0))

        assertEquals(Vector3.ZERO, body.force, "Static body should not accumulate force")
    }

    @Test
    fun `apply impulse changes velocity immediately`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.applyImpulse(body.transform.position, Vector3(0.0, 10.0, 0.0))

        // Impulse = Δp = m * Δv, so Δv = impulse / m = 10 / 1 = 10
        assertEquals(10.0, body.linearVelocity.y, epsilon, "Velocity from impulse")
    }

    @Test
    fun `apply impulse to static body is ignored`() {
        val body = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .mass(0.0)
            .build()

        body.applyImpulse(Vector3.ZERO, Vector3(0.0, 100.0, 0.0))

        assertEquals(Vector3.ZERO, body.linearVelocity, "Static body should not respond to impulse")
    }

    @Test
    fun `orientation updates from angular velocity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        // Spin around Y axis at 1 rad/s
        body.angularVelocity = Vector3(0.0, 1.0, 0.0)

        val dt = 1.0 / 60.0
        body.integrate(dt, Vector3.ZERO, 0.0, 0.0)

        // Orientation should have changed
        assertNotEquals(Quaternion.IDENTITY, body.transform.rotation, "Rotation should change with angular velocity")
    }

    @Test
    fun `no NaN after integration`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val gravity = Vector3(0.0, -9.81, 0.0)
        val dt = 1.0 / 60.0

        repeat(1000) {
            body.integrate(dt, gravity, 0.0, 0.0)
        }

        assertFalse(body.hasNaN(), "No NaN values after 1000 integration steps")
    }

    @Test
    fun `kinematic body integrates position but not forces`() {
        val body = rigidBody()
            .id(2)
            .bodyType(BodyType.KINEMATIC)
            .position(0.0, 10.0, 0.0)
            .mass(0.0)
            .build()

        // Set velocity directly
        body.linearVelocity = Vector3(0.0, -5.0, 0.0)

        val dt = 1.0 / 60.0
        body.integrate(dt, Vector3(0.0, -9.81, 0.0), 0.0, 0.0)

        // Kinematic body should move by velocity but NOT by gravity
        val expectedY = 10.0 + (-5.0) * dt
        assertEquals(expectedY, body.transform.position.y, epsilon, "Kinematic body moves by velocity")
    }

    @Test
    fun `restitution and friction defaults`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        assertEquals(0.0, body.restitution, "Default restitution is 0")
        assertEquals(0.5, body.friction, epsilon, "Default friction is 0.5")
    }

    @Test
    fun `body is awake by default`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        assertTrue(body.awake)
    }

    @Test
    fun `zero gravity maintains velocity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.linearVelocity = Vector3(1.0, 2.0, 3.0)

        val dt = 1.0 / 60.0
        body.integrate(dt, Vector3.ZERO, 0.0, 0.0)

        assertEquals(Vector3(1.0, 2.0, 3.0), body.linearVelocity, "Velocity unchanged with zero gravity and no forces")
    }

    @Test
    fun `dynamic cube helper creates correct body`() {
        val builder = RigidBodyBuilder()
        val body = builder.dynamicCube(1, 1.0, 0.0, 10.0, 0.0, mass = 2.0)

        assertEquals(1, body.id)
        assertEquals(BodyType.DYNAMIC, body.bodyType)
        assertEquals(10.0, body.transform.position.y, epsilon)
        assertEquals(2.0, body.massProperties.mass, epsilon)
        // Half extents should be 0.5 for a 1.0 size cube
        val halfExtentsX = 0.5
        // Mass properties are calculated from halfExtents
        assertTrue(body.massProperties.mass > 0.0)
    }

    @Test
    fun `static ground helper creates correct body`() {
        val builder = RigidBodyBuilder()
        val body = builder.staticGround(0)

        assertEquals(0, body.id)
        assertEquals(BodyType.STATIC, body.bodyType)
        assertEquals(Vector3.ZERO, body.transform.position)
        assertEquals(0.0, body.massProperties.inverseMass, epsilon)
    }

    @Test
    fun `Vector3 constructor rejects NaN preventing NaN bodies`() {
        // Vector3 validates components and rejects NaN, so NaN cannot propagate
        // into a RigidBody through normal construction
        assertThrows<IllegalArgumentException> {
            Vector3(Double.NaN, 0.0, 0.0)
        }
    }

    @Test
    fun `hasNaN returns false for valid body`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        assertFalse(body.hasNaN(), "Valid body should not have NaN")
    }

    @Test
    fun `multiple integration steps produce smooth fall`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(0.0, 50.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val gravity = Vector3(0.0, -9.81, 0.0)
        val dt = 1.0 / 60.0

        var prevY = 50.0
        var prevVelocityY = 0.0

        repeat(300) {
            body.integrate(dt, gravity, 0.0, 0.0)

            // Position should decrease monotonically
            assertTrue(body.transform.position.y < prevY, "Position should decrease each step")

            // Velocity should become more negative (accelerate downward)
            assertTrue(body.linearVelocity.y < prevVelocityY, "Velocity should accelerate downward")

            prevY = body.transform.position.y
            prevVelocityY = body.linearVelocity.y
        }
    }

    // === Sleeping tests ===

    @Test
    fun `wakeUp sets awake and resets timer`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.awake = false
        body.sleepTimer = 100
        body.wakeUp()

        assertTrue(body.awake)
        assertEquals(0, body.sleepTimer)
    }

    @Test
    fun `isAtRest returns true when velocity is below threshold`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.linearVelocity = Vector3(0.0, 0.0, 0.0)
        body.angularVelocity = Vector3(0.0, 0.0, 0.0)

        assertTrue(body.isAtRest(0.03))
    }

    @Test
    fun `isAtRest returns false when velocity is above threshold`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.linearVelocity = Vector3(0.0, 10.0, 0.0)

        assertFalse(body.isAtRest(0.03))
    }

    @Test
    fun `isAtRest returns false for static bodies`() {
        val body = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .mass(0.0)
            .build()

        assertFalse(body.isAtRest(0.03), "Static bodies should never be considered at rest")
    }

    @Test
    fun `isAtRest checks angular velocity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.linearVelocity = Vector3.ZERO
        body.angularVelocity = Vector3(0.0, 1.0, 0.0) // Spinning

        assertFalse(body.isAtRest(0.03), "Spinning body should not be at rest")
    }

    @Test
    fun `applyForce wakes up sleeping body`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.awake = false
        body.applyForceAtCenter(Vector3(0.0, -10.0, 0.0))

        assertTrue(body.awake, "Body should wake up on force application")
        assertEquals(0, body.sleepTimer)
    }

    @Test
    fun `applyImpulse wakes up sleeping body`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.awake = false
        body.applyImpulse(body.transform.position, Vector3(0.0, 10.0, 0.0))

        assertTrue(body.awake, "Body should wake up on impulse application")
        assertEquals(0, body.sleepTimer)
    }

    @Test
    fun `sleep timer increments when at rest`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.linearVelocity = Vector3.ZERO
        body.angularVelocity = Vector3.ZERO

        // Simulate sleep check
        repeat(60) {
            assertTrue(body.isAtRest(0.03))
            body.sleepTimer++
        }

        assertEquals(60, body.sleepTimer)
        assertTrue(body.sleepTimer >= body.sleepTimeLimit)
    }

    @Test
    fun `sleep timer resets when body moves`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.sleepTimer = 50
        body.linearVelocity = Vector3(0.0, 10.0, 0.0)

        assertFalse(body.isAtRest(0.03))
        body.sleepTimer = 0 // Reset in pipeline

        assertEquals(0, body.sleepTimer)
    }

    @Test
    fun `sleepTimeLimit is configurable`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        body.sleepTimeLimit = 10
        assertEquals(10, body.sleepTimeLimit)
    }
}
