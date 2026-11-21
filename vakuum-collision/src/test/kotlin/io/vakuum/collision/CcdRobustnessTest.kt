package io.vakuum.collision

import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

/**
 * M3.3 — CCD Robustness & Edge Case Tests.
 *
 * Tests TOI edge cases, numerical stability, and robustness of the CCD subsystem.
 */
class CcdRobustnessTest {

    // ============================================================
    // TOI Edge Cases
    // ============================================================

    @Test
    fun `TOI box-plane  grazing contact at shallow angle`() {
        // Body moving mostly horizontally with slight downward component
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.0, 0.0))
            .linearVelocity(Vector3(50.0, -0.5, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect grazing contact")
        assertTrue(result!!.toi >= 0.0 && result.toi <= 1.0)
    }

    @Test
    fun `TOI box-plane  parallel movement returns null`() {
        // Body moving parallel to the plane (no vertical velocity)
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(100.0, 0.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNull(result, "Parallel movement should not produce TOI")
    }

    @Test
    fun `TOI box-plane  nearly parallel movement`() {
        // Body at y=0.6, bottom at y=0.1, moving down at 0.1 m/s with dt=1.0
        // Distance to plane: 0.1m, TOI = 0.1/0.1 = 1.0
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.6, 0.0))
            .linearVelocity(Vector3(100.0, -0.1, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Nearly parallel movement should detect TOI")
        assertFalse(result!!.toi.isNaN(), "TOI should not be NaN")
        assertFalse(result.toi.isInfinite(), "TOI should not be infinite")
    }

    @Test
    fun `TOI box-plane  extremely high velocity`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1000.0, 0.0))
            .linearVelocity(Vector3(0.0, -10000.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect TOI at extreme velocity")
        assertFalse(result!!.toi.isNaN(), "TOI should not be NaN")
        assertFalse(result.toi.isInfinite(), "TOI should not be infinite")
        assertTrue(result.toi >= 0.0 && result.toi <= 1.0)
    }

    @Test
    fun `TOI box-plane  velocity near CCD threshold`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0001, 0.0)) // Just above threshold
            .mass(1.0)
            .build()

        assertTrue(Ccd.shouldUseCcd(body, 10.0), "Should use CCD just above threshold")

        val body2 = rigidBody()
            .id(2)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.0, 0.0))
            .linearVelocity(Vector3(0.0, -9.9999, 0.0)) // Just below threshold
            .mass(1.0)
            .build()

        assertFalse(Ccd.shouldUseCcd(body2, 10.0), "Should not use CCD just below threshold")
    }

    @Test
    fun `TOI box-box  very large static body`() {
        // Huge ground-like box at y=-0.5 (top at y=0.5). Small body falls from y=2.0.
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 2.0, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, -0.5, 0.0))
            .mass(0.0)
            .build()
        val smallShape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val hugeShape = BoxShape(Vector3(100.0, 1.0, 100.0))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, smallShape, staticBody, hugeShape, dt)

        assertNotNull(result, "Should detect TOI with very large static body")
        assertFalse(result!!.toi.isNaN())
    }

    @Test
    fun `TOI box-plane  TOI at end of timestep`() {
        // Body exactly reaches the plane at the end of the timestep
        // y=1.5, halfExtents.y=0.5, bottom=1.0, velocity=-10, dt=0.1
        // distance=1.0, toi=1.0/10=0.1, fraction=0.1/0.1=1.0
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.5, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNotNull(result, "Should detect TOI at end of timestep")
        assertEquals(1.0, result!!.toi, 0.01, "TOI should be ~1.0")
    }

    @Test
    fun `TOI box-plane  TOI slightly outside timestep returns null`() {
        // Body won't quite reach the plane within the timestep
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 2.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1 // In 0.1s, travels 1.0m. Bottom at 1.5, needs 1.5m. Won't reach.

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        assertNull(result, "Should return null when TOI is outside timestep")
    }

    @Test
    fun `TOI box-box  rotated static box`() {
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 2.0, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val rotation = Quaternion.fromAxisAngle(Vector3.UNIT_Z, Math.PI / 6.0)
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .mass(0.0)
            .build()
        staticBody.transform = Transform(staticBody.transform.position, rotation)

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        // May or may not detect depending on rotation, but should not crash or produce NaN
        if (result != null) {
            assertFalse(result.toi.isNaN(), "TOI should not be NaN for rotated box")
            assertFalse(result.toi.isInfinite(), "TOI should not be infinite")
        }
    }

    @Test
    fun `TOI box-plane  infinity velocity handled`() {
        // Vector3 rejects NaN at construction, so test with extreme values instead
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(1e15, -1e15, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxPlane(body, shape, Vector3.UNIT_Y, 0.0, dt)

        // With extreme velocity, TOI should still be valid
        if (result != null) {
            assertFalse(result.toi.isNaN(), "TOI should not be NaN with extreme velocity")
            assertFalse(result.toi.isInfinite(), "TOI should not be infinite")
        }
    }

    @Test
    fun `swept AABB  high velocity covers large distance`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 100.0, 0.0))
            .linearVelocity(Vector3(0.0, -1000.0, 0.0))
            .mass(1.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 0.1

        val swept = Ccd.sweptAABB(body, shape, dt)

        // Should cover from y=99.5 to y=0.5 (100 - 0.5 start, 100 - 100 - 0.5 end)
        assertTrue(swept.min.y <= 1.0, "Swept AABB should extend to near ground")
        assertTrue(swept.max.y >= 99.0, "Swept AABB should include start position")
    }

    @Test
    fun `TOI box-box  no mutation of moving body transform`() {
        // Verify that toiBoxBox does not mutate the body's transform
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val originalPos = movingBody.transform.position
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .mass(0.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        assertEquals(originalPos.x, movingBody.transform.position.x, 1e-10, "X should not change")
        assertEquals(originalPos.y, movingBody.transform.position.y, 1e-10, "Y should not change")
        assertEquals(originalPos.z, movingBody.transform.position.z, 1e-10, "Z should not change")
    }

    @Test
    fun `TOI box-box  touching contact returns null`() {
        // Bodies just touching (no penetration, no gap)
        val movingBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 1.5, 0.0)) // bottom at 1.0
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val staticBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.5, 0.0)) // top at 1.0
            .mass(0.0)
            .build()
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val dt = 1.0

        val result = Ccd.toiBoxBox(movingBody, shape, staticBody, shape, dt)

        // Touching = already colliding, should return null
        assertNull(result, "Touching bodies should return null for CCD")
    }
}
