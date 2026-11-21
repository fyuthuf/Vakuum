package io.vakuum.collision

import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Quaternion
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class NarrowphaseNewShapesTest {

    val narrowphase = Narrowphase()

    // ============================================================
    // Sphere-Sphere Tests
    // ============================================================

    @Test
    fun `sphere-sphere overlapping detects collision`() {
        val a = rigidBody().id(1).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val b = rigidBody().id(2).position(Vector3(1.0, 0.0, 0.0)).mass(1.0).build()
        val sa = SphereShape.fromRadius(1.0)
        val sb = SphereShape.fromRadius(1.0)

        val contact = narrowphase.testSphereSphere(a, b, sa, sb)
        assertNotNull(contact, "Overlapping spheres should collide")
        assertTrue(contact!!.penetrationDepth > 0.0)
    }

    @Test
    fun `sphere-sphere not overlapping returns null`() {
        val a = rigidBody().id(1).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val b = rigidBody().id(2).position(Vector3(5.0, 0.0, 0.0)).mass(1.0).build()
        val sa = SphereShape.fromRadius(1.0)
        val sb = SphereShape.fromRadius(1.0)

        val contact = narrowphase.testSphereSphere(a, b, sa, sb)
        assertNull(contact, "Non-overlapping spheres should not collide")
    }

    @Test
    fun `sphere-sphere touching detects collision`() {
        val a = rigidBody().id(1).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val b = rigidBody().id(2).position(Vector3(2.0, 0.0, 0.0)).mass(1.0).build()
        val sa = SphereShape.fromRadius(1.0)
        val sb = SphereShape.fromRadius(1.0)

        val contact = narrowphase.testSphereSphere(a, b, sa, sb)
        // Touching spheres: penetration should be ~0
        if (contact != null) {
            assertEquals(0.0, contact.penetrationDepth, 1e-6, "Touching spheres have zero penetration")
        }
    }

    @Test
    fun `sphere-sphere normal points in push direction for bodyA`() {
        val a = rigidBody().id(1).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val b = rigidBody().id(2).position(Vector3(1.5, 0.0, 0.0)).mass(1.0).build()
        val sa = SphereShape.fromRadius(1.0)
        val sb = SphereShape.fromRadius(1.0)

        val contact = narrowphase.testSphereSphere(a, b, sa, sb)
        assertNotNull(contact)
        // Normal should point from B toward A (negative X direction)
        assertTrue(contact!!.normal.x < 0, "Normal should push bodyA away from bodyB")
    }

    // ============================================================
    // Sphere-Plane Tests
    // ============================================================

    @Test
    fun `sphere-plane penetrating detects collision`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 0.3, 0.0)).mass(1.0).build()
        val shape = SphereShape.fromRadius(0.5)

        val contact = narrowphase.testSpherePlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNotNull(contact, "Sphere penetrating plane should collide")
        assertTrue(contact!!.penetrationDepth > 0.0)
    }

    @Test
    fun `sphere-plane above returns null`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 5.0, 0.0)).mass(1.0).build()
        val shape = SphereShape.fromRadius(0.5)

        val contact = narrowphase.testSpherePlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNull(contact, "Sphere above plane should not collide")
    }

    @Test
    fun `sphere-plane normal points along plane normal`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 0.3, 0.0)).mass(1.0).build()
        val shape = SphereShape.fromRadius(0.5)

        val contact = narrowphase.testSpherePlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNotNull(contact)
        assertEquals(Vector3.UNIT_Y, contact!!.normal, "Normal should be the plane normal")
    }

    // ============================================================
    // Sphere-Box Tests
    // ============================================================

    @Test
    fun `sphere-box overlapping detects collision`() {
        val sphereBody = rigidBody().id(1).position(Vector3(0.0, 0.5, 0.0)).mass(1.0).build()
        val boxBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(0.0).build()
        val sphereShape = SphereShape.fromRadius(0.5)
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val contact = narrowphase.testSphereBox(sphereBody, sphereShape, boxBody, boxShape)
        assertNotNull(contact, "Sphere overlapping box should collide")
    }

    @Test
    fun `sphere-box not overlapping returns null`() {
        val sphereBody = rigidBody().id(1).position(Vector3(0.0, 10.0, 0.0)).mass(1.0).build()
        val boxBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(0.0).build()
        val sphereShape = SphereShape.fromRadius(0.5)
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val contact = narrowphase.testSphereBox(sphereBody, sphereShape, boxBody, boxShape)
        assertNull(contact, "Sphere far from box should not collide")
    }

    // ============================================================
    // Capsule-Plane Tests
    // ============================================================

    @Test
    fun `capsule-plane penetrating detects collision`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 0.5, 0.0)).mass(1.0).build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val contact = narrowphase.testCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNotNull(contact, "Capsule penetrating plane should collide")
        assertTrue(contact!!.penetrationDepth > 0.0)
    }

    @Test
    fun `capsule-plane above returns null`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 10.0, 0.0)).mass(1.0).build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val contact = narrowphase.testCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNull(contact, "Capsule above plane should not collide")
    }

    @Test
    fun `capsule-plane normal points along plane normal`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 0.5, 0.0)).mass(1.0).build()
        val shape = CapsuleShape.fromHalfHeight(0.5, 0.3)

        val contact = narrowphase.testCapsulePlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNotNull(contact)
        assertEquals(Vector3.UNIT_Y, contact!!.normal)
    }

    // ============================================================
    // Capsule-Sphere Tests
    // ============================================================

    @Test
    fun `capsule-sphere overlapping detects collision`() {
        val capsuleBody = rigidBody().id(1).position(Vector3(0.0, 1.0, 0.0)).mass(1.0).build()
        val sphereBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val capsuleShape = CapsuleShape.fromHalfHeight(0.5, 0.3)
        val sphereShape = SphereShape.fromRadius(1.0)

        val contact = narrowphase.testCapsuleSphere(capsuleBody, capsuleShape, sphereBody, sphereShape)
        assertNotNull(contact, "Capsule overlapping sphere should collide")
    }

    @Test
    fun `capsule-sphere not overlapping returns null`() {
        val capsuleBody = rigidBody().id(1).position(Vector3(0.0, 10.0, 0.0)).mass(1.0).build()
        val sphereBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val capsuleShape = CapsuleShape.fromHalfHeight(0.5, 0.3)
        val sphereShape = SphereShape.fromRadius(1.0)

        val contact = narrowphase.testCapsuleSphere(capsuleBody, capsuleShape, sphereBody, sphereShape)
        assertNull(contact, "Capsule far from sphere should not collide")
    }

    // ============================================================
    // Capsule-Box Tests
    // ============================================================

    @Test
    fun `capsule-box overlapping detects collision`() {
        val capsuleBody = rigidBody().id(1).position(Vector3(0.0, 0.5, 0.0)).mass(1.0).build()
        val boxBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(0.0).build()
        val capsuleShape = CapsuleShape.fromHalfHeight(0.3, 0.2)
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val contact = narrowphase.testCapsuleBox(capsuleBody, capsuleShape, boxBody, boxShape)
        assertNotNull(contact, "Capsule overlapping box should collide")
    }

    @Test
    fun `capsule-box not overlapping returns null`() {
        val capsuleBody = rigidBody().id(1).position(Vector3(0.0, 10.0, 0.0)).mass(1.0).build()
        val boxBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(0.0).build()
        val capsuleShape = CapsuleShape.fromHalfHeight(0.3, 0.2)
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val contact = narrowphase.testCapsuleBox(capsuleBody, capsuleShape, boxBody, boxShape)
        assertNull(contact, "Capsule far from box should not collide")
    }

    // ============================================================
    // Convex-Plane Tests
    // ============================================================

    @Test
    fun `convex-plane penetrating detects collision`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 0.1, 0.0)).mass(1.0).build()
        val shape = ConvexHullShape.fromBox(Vector3(0.3, 0.3, 0.3))

        val contact = narrowphase.testConvexPlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNotNull(contact, "Convex hull penetrating plane should collide")
        assertTrue(contact!!.penetrationDepth > 0.0)
    }

    @Test
    fun `convex-plane above returns null`() {
        val body = rigidBody().id(1).position(Vector3(0.0, 5.0, 0.0)).mass(1.0).build()
        val shape = ConvexHullShape.fromBox(Vector3(0.3, 0.3, 0.3))

        val contact = narrowphase.testConvexPlane(body, shape, Vector3.UNIT_Y, 0.0)
        assertNull(contact, "Convex hull above plane should not collide")
    }

    // ============================================================
    // Convex-Box Tests
    // ============================================================

    @Test
    fun `convex-box overlapping detects collision`() {
        val convexBody = rigidBody().id(1).position(Vector3(0.0, 0.5, 0.0)).mass(1.0).build()
        val boxBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(0.0).build()
        val convexShape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val contact = narrowphase.testConvexBox(convexBody, convexShape, boxBody, boxShape)
        assertNotNull(contact, "Convex hull overlapping box should collide")
    }

    @Test
    fun `convex-box not overlapping returns null`() {
        val convexBody = rigidBody().id(1).position(Vector3(0.0, 10.0, 0.0)).mass(1.0).build()
        val boxBody = rigidBody().id(2).position(Vector3(0.0, 0.0, 0.0)).mass(0.0).build()
        val convexShape = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))
        val boxShape = BoxShape(Vector3(1.0, 0.5, 1.0))

        val contact = narrowphase.testConvexBox(convexBody, convexShape, boxBody, boxShape)
        assertNull(contact, "Convex hull far from box should not collide")
    }

    // ============================================================
    // Convex-Convex Tests
    // ============================================================

    @Test
    fun `convex-convex overlapping detects collision`() {
        val a = rigidBody().id(1).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val b = rigidBody().id(2).position(Vector3(0.5, 0.0, 0.0)).mass(1.0).build()
        val sa = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))
        val sb = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testConvexConvex(a, sa, b, sb)
        assertNotNull(contact, "Overlapping convex hulls should collide")
    }

    @Test
    fun `convex-convex not overlapping returns null`() {
        val a = rigidBody().id(1).position(Vector3(0.0, 0.0, 0.0)).mass(1.0).build()
        val b = rigidBody().id(2).position(Vector3(5.0, 0.0, 0.0)).mass(1.0).build()
        val sa = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))
        val sb = ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5))

        val contact = narrowphase.testConvexConvex(a, sa, b, sb)
        assertNull(contact, "Non-overlapping convex hulls should not collide")
    }

    // ============================================================
    // Shape Validation Tests
    // ============================================================

    @Test
    fun `sphere shape rejects zero radius`() {
        assertThrows(IllegalArgumentException::class.java) {
            SphereShape.fromRadius(0.0)
        }
    }

    @Test
    fun `sphere shape rejects negative radius`() {
        assertThrows(IllegalArgumentException::class.java) {
            SphereShape.fromRadius(-1.0)
        }
    }

    @Test
    fun `capsule shape rejects zero half-height`() {
        assertThrows(IllegalArgumentException::class.java) {
            CapsuleShape.fromHalfHeight(0.0, 0.5)
        }
    }

    @Test
    fun `convex hull rejects coplanar vertices`() {
        assertThrows(IllegalArgumentException::class.java) {
            ConvexHullShape(listOf(
                Vector3(0.0, 0.0, 0.0),
                Vector3(1.0, 0.0, 0.0),
                Vector3(0.0, 1.0, 0.0),
                Vector3(1.0, 1.0, 0.0) // All on Z=0 plane
            ))
        }
    }

    @Test
    fun `convex hull rejects too few vertices`() {
        assertThrows(IllegalArgumentException::class.java) {
            ConvexHullShape(listOf(
                Vector3(0.0, 0.0, 0.0),
                Vector3(1.0, 0.0, 0.0),
                Vector3(0.0, 1.0, 0.0)
            ))
        }
    }

    @Test
    fun `convex hull from box creates valid shape`() {
        val shape = ConvexHullShape.fromBox(Vector3(1.0, 1.0, 1.0))
        assertEquals(8, shape.vertices.size)
    }
}
