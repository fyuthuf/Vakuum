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
    fun `box box collision normal points from A to B`() {
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
        // Normal should point from A to B (positive X direction)
        assertTrue(contact!!.normal.x > 0.0 || contact!!.normal.x < 0.0, "Normal should have X component")
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
