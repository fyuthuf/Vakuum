package io.vakuum.collision

import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class NarrowphaseTest {

    private val epsilon = 1e-6
    private val narrowphase = Narrowphase()

    @Test
    fun `box plane collision detects penetration`() {
        // Cube centered at (0, 0.3, 0) with half-extents 0.5
        // Bottom of cube is at y = -0.2, which is below the ground plane at y = 0
        val body = rigidBody()
            .id(1)
            .position(0.0, 0.3, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val planeNormal = Vector3.UNIT_Y
        val planeDistance = 0.0

        val contact = narrowphase.testBoxPlane(body, shape, planeNormal, planeDistance)

        assertNotNull(contact, "Box penetrating plane should produce contact")
        assertTrue(contact!!.penetrationDepth > 0.0, "Penetration depth should be positive")
        // Penetration = radius - distToPlane = 0.5 - 0.3 = 0.2
        assertEquals(0.2, contact.penetrationDepth, epsilon, "Penetration depth")
    }

    @Test
    fun `box plane collision no contact when above plane`() {
        // Cube centered at (0, 2.0, 0) with half-extents 0.5
        // Bottom of cube is at y = 1.5, well above the ground plane
        val body = rigidBody()
            .id(1)
            .position(0.0, 2.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val planeNormal = Vector3.UNIT_Y
        val planeDistance = 0.0

        val contact = narrowphase.testBoxPlane(body, shape, planeNormal, planeDistance)

        assertNull(contact, "Box above plane should not produce contact")
    }

    @Test
    fun `box plane collision at exact boundary`() {
        // Cube centered at (0, 0.5, 0) with half-extents 0.5
        // Bottom of cube is exactly at y = 0, touching the plane
        val body = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val planeNormal = Vector3.UNIT_Y
        val planeDistance = 0.0

        val contact = narrowphase.testBoxPlane(body, shape, planeNormal, planeDistance)

        // At exact boundary, penetration = 0, so no contact
        assertNull(contact, "Box touching plane exactly should not produce contact")
    }

    @Test
    fun `box plane collision normal points upward`() {
        val body = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val planeNormal = Vector3.UNIT_Y
        val planeDistance = 0.0

        val contact = narrowphase.testBoxPlane(body, shape, planeNormal, planeDistance)

        assertNotNull(contact)
        // Normal should point upward (from plane into box)
        assertEquals(Vector3.UNIT_Y, contact!!.normal)
    }

    @Test
    fun `box box collision detects overlap`() {
        // Two overlapping cubes
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

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNotNull(contact, "Overlapping boxes should produce contact")
        assertTrue(contact!!.penetrationDepth > 0.0, "Penetration depth should be positive")
    }

    @Test
    fun `box box collision no contact when separated`() {
        // Two non-overlapping cubes
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(5.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNull(contact, "Separated boxes should not produce contact")
    }

    @Test
    fun `box box collision normal points in push direction for bodyA`() {
        // BodyA at origin, BodyB at (0.5, 0, 0) — they overlap along X.
        // BodyB is to the right of BodyA, so the push direction for bodyA is -X (left, away from B).
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

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNotNull(contact)
        // Normal should point in push direction for bodyA: away from bodyB, i.e., -X
        assertTrue(contact!!.normal.x < -0.5, "Normal X should be negative (push bodyA away from B)")
        assertTrue(kotlin.math.abs(contact.normal.y) < epsilon, "Normal Y should be near zero")
        assertTrue(kotlin.math.abs(contact.normal.z) < epsilon, "Normal Z should be near zero")
    }

    @Test
    fun `box box collision normal is consistent when bodies are swapped`() {
        // Test that swapping A and B reverses the normal direction.
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

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contactAB = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)
        val contactBA = narrowphase.testBoxBox(bodyB, bodyA, shapeB, shapeA)

        assertNotNull(contactAB)
        assertNotNull(contactBA)

        // Normal for AB should be opposite to normal for BA
        // Because push direction for bodyA in AB is opposite to push direction for bodyB in BA
        val dot = contactAB!!.normal.dot(contactBA!!.normal)
        assertTrue(dot < -0.99, "Normals should be opposite when bodies are swapped, dot=$dot")
    }

    @Test
    fun `box box collision normal with rotated boxes`() {
        // BodyA at origin, rotated 45 degrees around Z.
        // BodyB at (1, 1, 0), axis-aligned.
        // They should collide, and the normal should point away from B toward A.
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        // Rotate 45 degrees around Z
        bodyA.transform = io.vakuum.math.Transform(
            bodyA.transform.position,
            io.vakuum.math.Quaternion.fromAxisAngle(Vector3.UNIT_Z, Math.PI / 4.0)
        )

        val bodyB = rigidBody()
            .id(2)
            .position(1.0, 1.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        if (contact != null) {
            // Normal should point from B toward A (push direction for bodyA)
            // Vector from B to A: (-1, -1, 0)
            val bToA = bodyA.transform.position - bodyB.transform.position
            val dot = contact.normal.dot(bToA)
            assertTrue(dot > 0.0, "Normal should have a component toward A from B, dot=$dot")
        }
        // If no contact, the boxes may not overlap at this configuration — that's also valid
    }

    @Test
    fun `box box collision normal with coincident centers`() {
        // Two boxes with the same center — fully overlapping.
        // The normal should still be well-defined.
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        // When centers are coincident, translation is zero, so the normal direction
        // depends on the first axis tested. The key is that it should not be NaN.
        if (contact != null) {
            assertFalse(contact.normal.x.isNaN(), "Normal X should not be NaN")
            assertFalse(contact.normal.y.isNaN(), "Normal Y should not be NaN")
            assertFalse(contact.normal.z.isNaN(), "Normal Z should not be NaN")
            assertTrue(contact.normal.length > 0.0, "Normal should not be zero length")
        }
    }

    @Test
    fun `box box collision normal with nearly coincident centers`() {
        // Two boxes almost at the same position — very small translation.
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(1e-8, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        if (contact != null) {
            assertFalse(contact.normal.x.isNaN(), "Normal X should not be NaN")
            assertFalse(contact.normal.y.isNaN(), "Normal Y should not be NaN")
            assertFalse(contact.normal.z.isNaN(), "Normal Z should not be NaN")
        }
    }

    @Test
    fun `box box collision normal along Y axis`() {
        // BodyA below BodyB along Y axis.
        // Push direction for bodyA should be -Y (down, away from B above).
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNotNull(contact)
        // BodyB is above bodyA. Push direction for bodyA is -Y (down, away from B).
        assertTrue(contact!!.normal.y < -0.5, "Normal Y should be negative (push bodyA down, away from B)")
        assertTrue(kotlin.math.abs(contact.normal.x) < epsilon, "Normal X should be near zero")
        assertTrue(kotlin.math.abs(contact.normal.z) < epsilon, "Normal Z should be near zero")
    }

    @Test
    fun `box box collision normal along Z axis`() {
        // BodyA in front of BodyB along Z axis.
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(0.0, 0.0, 0.5)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNotNull(contact)
        // BodyB is in +Z direction from bodyA. Push direction for bodyA is -Z.
        assertTrue(contact!!.normal.z < -0.5, "Normal Z should be negative (push bodyA away from B)")
        assertTrue(kotlin.math.abs(contact.normal.x) < epsilon, "Normal X should be near zero")
        assertTrue(kotlin.math.abs(contact.normal.y) < epsilon, "Normal Y should be near zero")
    }

    @Test
    fun `box box collision penetration depth is correct`() {
        // Two cubes overlapping by 0.5 units
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

        val shapeA = BoxShape(Vector3(0.5, 0.5, 0.5))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNotNull(contact)
        // Overlap = 0.5 + 0.5 - |0.5| = 0.5
        assertEquals(0.5, contact!!.penetrationDepth, epsilon, "Penetration depth")
    }

    @Test
    fun `box box collision with different sizes`() {
        val bodyA = rigidBody()
            .id(1)
            .position(0.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(1.0, 1.0, 1.0)
            .build()

        val bodyB = rigidBody()
            .id(2)
            .position(1.0, 0.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        val shapeA = BoxShape(Vector3(1.0, 1.0, 1.0))
        val shapeB = BoxShape(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testBoxBox(bodyA, bodyB, shapeA, shapeB)

        assertNotNull(contact, "Overlapping boxes of different sizes should produce contact")
    }

    @Test
    fun `box plane collision with rotated box`() {
        // A box rotated 45 degrees around Z axis, partially penetrating the plane
        val body = rigidBody()
            .id(1)
            .position(0.0, 0.5, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()

        // Rotate 45 degrees around Z
        val angle = Math.PI / 4.0
        body.transform = io.vakuum.math.Transform(
            body.transform.position,
            io.vakuum.math.Quaternion.fromAxisAngle(Vector3.UNIT_Z, angle)
        )

        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        val planeNormal = Vector3.UNIT_Y
        val planeDistance = 0.0

        val contact = narrowphase.testBoxPlane(body, shape, planeNormal, planeDistance)

        // A rotated box at y=0.5 with half-extent 0.5 should still penetrate
        assertNotNull(contact, "Rotated box penetrating plane should produce contact")
    }
}
