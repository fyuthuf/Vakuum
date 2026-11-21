package io.vakuum.collision

import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class CcdNewShapesTest {

    // ============================================================
    // Sphere-Plane CCD Tests
    // ============================================================

    @Test
    fun `TOI sphere-plane detects impact`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = SphereShape.fromRadius(0.5)
        val dt = 1.0

        val result = Ccd.toiSpherePlane(body, shape, Vector3.UNIT_Y, 0.0, dt)
        assertNotNull(result, "Fast sphere should hit plane")
        assertTrue(result!!.toi > 0.0 && result.toi <= 1.0)
    }

    @Test
    fun `TOI sphere-plane no impact when moving away`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.3, 0.0))
            .linearVelocity(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = SphereShape.fromRadius(0.5)

        val result = Ccd.toiSpherePlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNull(result, "Sphere moving away from plane should not hit")
    }

    @Test
    fun `TOI sphere-plane already penetrating returns null`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.1, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val shape = SphereShape.fromRadius(0.5)

        val result = Ccd.toiSpherePlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNull(result, "Already penetrating sphere should return null")
    }

    @Test
    fun `TOI sphere-plane accurate TOI fraction`() {
        // Sphere at y=5, radius=0.5, velocity=-10 m/s
        // Lowest point at y=4.5, distance to plane = 4.5
        // TOI = 4.5 / 10 = 0.45s, fraction = 0.45
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = SphereShape.fromRadius(0.5)

        val result = Ccd.toiSpherePlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNotNull(result)
        assertEquals(0.45, result!!.toi, 0.01, "TOI fraction should be ~0.45")
    }

    // ============================================================
    // Sphere-Sphere CCD Tests
    // ============================================================

    @Test
    fun `TOI sphere-sphere already penetrating returns null`() {
        val a = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .linearVelocity(Vector3(1.0, 0.0, 0.0))
            .mass(1.0)
            .build()
        val b = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.5, 0.0, 0.0))
            .mass(0.0)
            .build()
        val sa = SphereShape.fromRadius(0.5)
        val sb = SphereShape.fromRadius(0.5)

        val result = Ccd.toiSphereSphere(a, sa, b, sb, 1.0)
        assertNull(result, "Already penetrating spheres should return null")
    }

    // ============================================================
    // Capsule-Plane CCD Tests
    // ============================================================

    @Test
    fun `TOI capsule-plane detects impact`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)
        val dt = 1.0

        val result = Ccd.toiCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0, dt)
        assertNotNull(result, "Fast capsule should hit plane")
    }

    @Test
    fun `TOI capsule-plane no impact when moving away`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .linearVelocity(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val result = Ccd.toiCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNull(result, "Capsule moving away should not hit")
    }

    @Test
    fun `TOI capsule-plane already penetrating returns null`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.1, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val result = Ccd.toiCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNull(result, "Already penetrating capsule should return null")
    }

    @Test
    fun `TOI capsule-plane accurate TOI fraction`() {
        // Capsule at y=5, halfHeight=0.5, radius=0.3
        // Lowest endpoint at y=4.5, lowest point = 4.5 - 0.3 = 4.2
        // TOI = 4.2 / 10 = 0.42s
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val result = Ccd.toiCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNotNull(result)
        assertEquals(0.42, result!!.toi, 0.01, "TOI fraction should be ~0.42")
    }

    // ============================================================
    // Capsule-Sphere CCD Tests
    // ============================================================

    @Test
    fun `TOI capsule-sphere no impact when far`() {
        val capsuleBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(100.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val sphereBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .mass(0.0)
            .build()
        val capsuleShape = CapsuleShape.fromHalfHeight(0.3, 0.2)
        val sphereShape = SphereShape.fromRadius(1.0)

        val result = Ccd.toiCapsuleSphere(capsuleBody, capsuleShape, sphereBody, sphereShape, 1.0)
        assertNull(result, "Capsule far from sphere should not hit")
    }

    // ============================================================
    // Convex-Plane CCD Tests
    // ============================================================

    @Test
    fun `TOI convex-plane detects impact`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val result = Ccd.toiConvexPlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNotNull(result, "Fast convex hull should hit plane")
    }

    @Test
    fun `TOI convex-plane no impact when moving away`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.5, 0.0))
            .linearVelocity(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val result = Ccd.toiConvexPlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNull(result, "Convex hull moving away should not hit")
    }

    @Test
    fun `TOI convex-plane already penetrating returns null`() {
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.1, 0.0))
            .linearVelocity(Vector3(0.0, -5.0, 0.0))
            .mass(1.0)
            .build()
        val shape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val result = Ccd.toiConvexPlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNull(result, "Already penetrating convex hull should return null")
    }

    @Test
    fun `TOI convex-plane accurate TOI fraction`() {
        // Convex hull (box) at y=5, halfExtents.y=0.5
        // Lowest vertex at y=4.5
        // TOI = 4.5 / 10 = 0.45s
        val body = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val shape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val result = Ccd.toiConvexPlane(body, shape, Vector3.UNIT_Y, 0.0, 1.0)
        assertNotNull(result)
        assertEquals(0.45, result!!.toi, 0.01, "TOI fraction should be ~0.45")
    }

    // ============================================================
    // Convex-Box CCD Tests
    // ============================================================

    @Test
    fun `TOI convex-box detects impact`() {
        val convexBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val boxBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .mass(0.0)
            .build()
        val convexShape = ConvexHullShape.fromBox(Vector3(0.3, 0.3, 0.3))
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val result = Ccd.toiConvexBox(convexBody, convexShape, boxBody, boxShape, 1.0)
        assertNotNull(result, "Fast convex hull should hit box")
    }

    @Test
    fun `TOI convex-box no impact when far`() {
        val convexBody = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(100.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .build()
        val boxBody = rigidBody()
            .id(2)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .mass(0.0)
            .build()
        val convexShape = ConvexHullShape.fromBox(Vector3(0.3, 0.3, 0.3))
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val result = Ccd.toiConvexBox(convexBody, convexShape, boxBody, boxShape, 1.0)
        assertNull(result, "Convex hull far from box should not hit")
    }

    // ============================================================
    // Edge Cases
    // ============================================================

    @Test
    fun `TOI sphere-plane zero dt returns null`() {
        val body = rigidBody()
            .id(1).bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0).build()
        val shape = SphereShape.fromRadius(0.5)

        val result = Ccd.toiSpherePlane(body, shape, Vector3.UNIT_Y, 0.0, 0.0)
        assertNull(result, "Zero dt should return null")
    }

    @Test
    fun `TOI capsule-plane zero dt returns null`() {
        val body = rigidBody()
            .id(1).bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0).build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val result = Ccd.toiCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0, 0.0)
        assertNull(result, "Zero dt should return null")
    }

    @Test
    fun `TOI convex-plane zero dt returns null`() {
        val body = rigidBody()
            .id(1).bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0).build()
        val shape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val result = Ccd.toiConvexPlane(body, shape, Vector3.UNIT_Y, 0.0, 0.0)
        assertNull(result, "Zero dt should return null")
    }

    @Test
    fun `TOI sphere-sphere zero velocity returns null`() {
        val a = rigidBody()
            .id(1).bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .linearVelocity(Vector3.ZERO)
            .mass(1.0).build()
        val b = rigidBody()
            .id(2).bodyType(BodyType.STATIC)
            .position(Vector3(5.0, 0.0, 0.0))
            .mass(0.0).build()
        val sa = SphereShape.fromRadius(0.5)
        val sb = SphereShape.fromRadius(0.5)

        val result = Ccd.toiSphereSphere(a, sa, b, sb, 1.0)
        assertNull(result, "Zero velocity should return null")
    }
}
