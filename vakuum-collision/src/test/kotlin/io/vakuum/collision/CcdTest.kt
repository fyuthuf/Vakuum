package io.vakuum.collision

import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

/**
 * Unit tests for Continuous Collision Detection (CCD).
 *
 * Validates swept AABB, TOI calculation, and CCD helper functions.
 * Integration tests with SimulationPipeline are in vakuum-tests.
 */
class CcdTest {

    // ============================================================
    // Swept AABB tests
    // ============================================================

    @Test
    fun `swept AABB covers start and end position`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val swept = Ccd.sweptAABB(body, shape, dt)

        assertTrue(swept.contains(Vector3(0.0, 5.0, 0.0)), "Should contain start position")
        assertTrue(swept.contains(Vector3(0.0, 4.0, 0.0)), "Should contain end position")
        assertTrue(swept.contains(Vector3(0.0, 4.5, 0.0)), "Should contain midpoint")
    }

    @Test
    fun `swept AABB for stationary body equals regular AABB`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3.ZERO)
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val swept = Ccd.sweptAABB(body, shape, dt)

        assertEquals(-0.5, swept.min.x, 1e-10)
        assertEquals(4.5, swept.min.y, 1e-10)
        assertEquals(-0.5, swept.min.z, 1e-10)
        assertEquals(0.5, swept.max.x, 1e-10)
        assertEquals(5.5, swept.max.y, 1e-10)
        assertEquals(0.5, swept.max.z, 1e-10)
    }

    @Test
    fun `swept AABB expands in velocity direction`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(10.0, 0.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val swept = Ccd.sweptAABB(body, shape, dt)

        assertEquals(-0.5, swept.min.x, 1e-10)
        assertEquals(1.5, swept.max.x, 1e-10)
    }

    // ============================================================
    // shouldUseCcd tests
    // ============================================================

    @Test
    fun `shouldUseCcd returns true for fast body`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3.ZERO)
            .linearVelocity(Vector3(0.0, -20.0, 0.0))
            .mass(1.0)
            .build()

        assertTrue(Ccd.shouldUseCcd(body, 10.0))
    }

    @Test
    fun `shouldUseCcd returns false for slow body`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3.ZERO)
            .linearVelocity(Vector3(0.0, -1.0, 0.0))
            .mass(1.0)
            .build()

        assertFalse(Ccd.shouldUseCcd(body, 10.0))
    }

    @Test
    fun `shouldUseCcd returns false for stationary body`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3.ZERO)
            .linearVelocity(Vector3.ZERO)
            .mass(1.0)
            .build()

        assertFalse(Ccd.shouldUseCcd(body, 10.0))
    }

    // ============================================================
    // TOI Box-Plane tests
    // ============================================================

    @Test
    fun `TOI box-plane falling cube hits ground`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -20.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect TOI with ground plane")
        assertEquals(0.475, result!!.toi, 0.01, "TOI fraction should be ~0.475")
        assertEquals(Vector3.UNIT_Y, result.normal, "Normal should be up")
        assertEquals(-1, result.hitBodyId, "Hit body should be ground plane")
    }

    @Test
    fun `TOI box-plane  body moving away from plane returns null`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.0, 0.0))
            .linearVelocity(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNull(result, "Should not detect TOI when moving away from plane")
    }

    @Test
    fun `TOI box-plane  body too far to hit within timestep`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 100.0, 0.0))
            .linearVelocity(Vector3(0.0, -1.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.016

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNull(result, "Should not detect TOI when body won't reach plane within timestep")
    }

    @Test
    fun `TOI box-plane  already penetrating returns null`() {
        // When already penetrating, CCD returns null to avoid infinite substep loops.
        // Discrete collision detection handles existing penetration.
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.1, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNull(result, "Should return null when already penetrating to avoid infinite substep loops")
    }

    @Test
    fun `TOI box-plane  zero timestep returns null`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, 0.0)

        assertNull(result, "Zero timestep should return null")
    }

    // ============================================================
    // TOI Box-Box tests
    // ============================================================

    @Test
    fun `TOI box-box  moving cube hits static cube`() {
        // Moving cube just above static cube, falling slowly
        // Moving cube at (0, 1.6, 0), static at (0, 0.5, 0)
        // Both have halfExtents (0.5, 0.5, 0.5)
        // Moving cube bottom: 1.6 - 0.5 = 1.1, static cube top: 0.5 + 0.5 = 1.0
        // Distance to contact: 0.1m at 2 m/s → TOI = 0.05s
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.6, 0.0))
            .linearVelocity(Vector3(0.0, -2.0, 0.0))
            .mass(1.0)
            .build()
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .mass(0.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        assertNotNull(result, "Should detect TOI between moving and static box")
        assertTrue(result!!.toi >= 0.0 && result.toi <= 1.0, "TOI should be in [0, 1]")
        assertEquals(1, result.movingBodyId)
        assertEquals(2, result.hitBodyId)
    }

    @Test
    fun `TOI box-box  no collision returns null`() {
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(10.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -20.0, 0.0))
            .mass(1.0)
            .build()
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .mass(0.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        assertNull(result, "Should not detect TOI when bodies won't collide")
    }

    @Test
    fun `TOI box-box  stationary body returns null`() {
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3.ZERO)
            .mass(1.0)
            .build()
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .mass(0.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        assertNull(result, "Stationary body should not have TOI")
    }

    @Test
    fun `TOI box-box  already colliding returns null`() {
        // When already colliding, CCD returns null to avoid infinite substep loops.
        // Discrete collision detection handles existing penetration.
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.0, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .mass(0.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        assertNull(result, "Should return null when already colliding to avoid infinite substep loops")
    }

    // ============================================================
    // Edge case tests
    // ============================================================

    @Test
    fun `CCD with rotated box`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -20.0, 0.0))
            .mass(1.0)
            .build()
        val rotation = Quaternion.fromAxisAngle(Vector3.UNIT_Z, Math.PI / 4.0)
        body.transform = Transform(body.transform.position, rotation)

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect TOI for rotated box")
        assertTrue(result!!.toi >= 0.0 && result.toi <= 1.0, "TOI should be in [0, 1]")
    }

    @Test
    fun `CCD with very small timestep`() {
        // Body at y=0.5 with halfExtents.y=0.5 → bottom at y=0.0, just touching the plane.
        // Moving down at 1 m/s with dt=0.001 → will penetrate slightly.
        // Since the body is already touching, CCD returns null (discrete collision handles it).
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .linearVelocity(Vector3(0.0, -1.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.001

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        // Body is already touching the plane, so CCD returns null
        assertNull(result, "Touching body should return null for CCD")
    }

    @Test
    fun `CCD with very small timestep and body above plane`() {
        // Body at y=0.6 with halfExtents.y=0.5 → bottom at y=0.1, above the plane.
        // Moving down at 100 m/s with dt=0.001 → travels 0.1m, just reaching the plane.
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.6, 0.0))
            .linearVelocity(Vector3(0.0, -100.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.001

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect TOI with small timestep when body will reach plane")
        assertTrue(result!!.toi >= 0.0 && result.toi <= 1.0, "TOI should be in [0, 1]")
    }

    @Test
    fun `CCD with very large timestep`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 100.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 10.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect TOI with large timestep")
        assertEquals(0.995, result!!.toi, 0.01, "TOI fraction should be ~0.995")
    }

    @Test
    fun `advanceBodyToToi moves body correctly`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -20.0, 0.0))
            .mass(1.0)
            .build()
        val dt = 1.0

        Ccd.advanceBodyToToi(body, 0.5, dt)

        assertEquals(0.0, body.transform.position.x, 1e-10)
        assertEquals(0.0, body.transform.position.y, 1e-10)
        assertEquals(0.0, body.transform.position.z, 1e-10)
    }
}
