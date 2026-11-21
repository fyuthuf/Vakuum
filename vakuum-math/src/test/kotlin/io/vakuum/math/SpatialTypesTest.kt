package io.vakuum.math

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class Matrix4Test {

    @Test
    fun `identity matrix`() {
        val m = Matrix4.IDENTITY
        for (i in 0..3) {
            for (j in 0..3) {
                assertEquals(if (i == j) 1.0 else 0.0, m.get(i, j), 1e-15)
            }
        }
    }

    @Test
    fun `identity determinant is 1`() {
        assertEquals(1.0, Matrix4.IDENTITY.determinant, 1e-15)
    }

    @Test
    fun `translation matrix`() {
        val m = Matrix4.translation(1.0, 2.0, 3.0)
        val v = Vector3.ZERO
        val result = m.multiplyPoint(v)
        assertEquals(1.0, result.x, 1e-15)
        assertEquals(2.0, result.y, 1e-15)
        assertEquals(3.0, result.z, 1e-15)
    }

    @Test
    fun `translation extraction`() {
        val m = Matrix4.translation(1.0, 2.0, 3.0)
        val t = m.getTranslation()
        assertEquals(1.0, t.x, 1e-15)
        assertEquals(2.0, t.y, 1e-15)
        assertEquals(3.0, t.z, 1e-15)
    }

    @Test
    fun `scaling matrix`() {
        val m = Matrix4.scaling(2.0, 3.0, 4.0)
        val v = Vector3(1.0, 1.0, 1.0)
        val result = m.multiplyPoint(v)
        assertEquals(2.0, result.x, 1e-15)
        assertEquals(3.0, result.y, 1e-15)
        assertEquals(4.0, result.z, 1e-15)
    }

    @Test
    fun `rotation from quaternion`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 2.0)
        val m = Matrix4.rotation(q)
        val v = Vector3.UNIT_X
        val result = m.multiplyDirection(v)
        assertEquals(0.0, result.x, 1e-15)
        assertEquals(0.0, result.y, 1e-15)
        assertEquals(-1.0, result.z, 1e-15)
    }

    @Test
    fun `multiply point with translation`() {
        val m = Matrix4.translation(5.0, 0.0, 0.0)
        val v = Vector3(1.0, 0.0, 0.0)
        val result = m.multiplyPoint(v)
        assertEquals(6.0, result.x, 1e-15)
    }

    @Test
    fun `multiply direction ignores translation`() {
        val m = Matrix4.translation(5.0, 0.0, 0.0)
        val v = Vector3(1.0, 0.0, 0.0)
        val result = m.multiplyDirection(v)
        assertEquals(1.0, result.x, 1e-15)
    }

    @Test
    fun `inverse of translation`() {
        val m = Matrix4.translation(1.0, 2.0, 3.0)
        val inv = m.inverse()
        val identity = m * inv
        assertEquals(Matrix4.IDENTITY, identity)
    }

    @Test
    fun `singular matrix inverse throws`() {
        assertThrows<IllegalArgumentException> { Matrix4.ZERO.inverse() }
    }

    @Test
    fun `transpose`() {
        val m = Matrix4.translation(1.0, 2.0, 3.0)
        val t = m.transpose()
        // Translation matrix has translation in column 3
        // After transpose, translation moves to row 3
        assertEquals(1.0, t.get(3, 0), 1e-15)
        assertEquals(2.0, t.get(3, 1), 1e-15)
        assertEquals(3.0, t.get(3, 2), 1e-15)
    }

    @Test
    fun `from translation rotation scale`() {
        val pos = Vector3(1.0, 0.0, 0.0)
        val rot = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 2.0)
        val scale = Vector3(2.0, 2.0, 2.0)
        val m = Matrix4.fromTranslationRotationScale(pos, rot, scale)
        val v = Vector3.ZERO
        val result = m.multiplyPoint(v)
        assertEquals(1.0, result.x, 1e-15)
    }
}

class SpatialTypesTest {

    @Test
    fun `AABB contains point inside`() {
        val aabb = AABB(Vector3(-1.0, -1.0, -1.0), Vector3(1.0, 1.0, 1.0))
        assertTrue(aabb.contains(Vector3.ZERO))
    }

    @Test
    fun `AABB does not contain point outside`() {
        val aabb = AABB(Vector3(-1.0, -1.0, -1.0), Vector3(1.0, 1.0, 1.0))
        assertFalse(aabb.contains(Vector3(2.0, 0.0, 0.0)))
    }

    @Test
    fun `AABB intersection overlapping`() {
        val a = AABB(Vector3(0.0, 0.0, 0.0), Vector3(2.0, 2.0, 2.0))
        val b = AABB(Vector3(1.0, 1.0, 1.0), Vector3(3.0, 3.0, 3.0))
        assertTrue(a.intersects(b))
    }

    @Test
    fun `AABB intersection disjoint`() {
        val a = AABB(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 1.0))
        val b = AABB(Vector3(2.0, 0.0, 0.0), Vector3(3.0, 1.0, 1.0))
        assertFalse(a.intersects(b))
    }

    @Test
    fun `AABB center and extents`() {
        val aabb = AABB(Vector3(-2.0, -2.0, -2.0), Vector3(2.0, 2.0, 2.0))
        assertEquals(0.0, aabb.center.x, 1e-15)
        assertEquals(2.0, aabb.extents.x, 1e-15)
    }

    @Test
    fun `AABB union`() {
        val a = AABB(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 1.0))
        val b = AABB(Vector3(2.0, 0.0, 0.0), Vector3(3.0, 1.0, 1.0))
        val u = a.union(b)
        assertEquals(0.0, u.min.x, 1e-15)
        assertEquals(3.0, u.max.x, 1e-15)
    }

    @Test
    fun `AABB expand`() {
        val aabb = AABB(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 1.0))
        val expanded = aabb.expand(0.5)
        assertEquals(-0.5, expanded.min.x, 1e-15)
        assertEquals(1.5, expanded.max.x, 1e-15)
    }

    @Test
    fun `AABB from center extents`() {
        val aabb = AABB.fromCenterExtents(Vector3.ZERO, Vector3(1.0, 1.0, 1.0))
        assertEquals(-1.0, aabb.min.x, 1e-15)
        assertEquals(1.0, aabb.max.x, 1e-15)
    }

    @Test
    fun `BoundingSphere contains point`() {
        val sphere = BoundingSphere(Vector3.ZERO, 1.0)
        assertTrue(sphere.contains(Vector3(0.5, 0.0, 0.0)))
        assertFalse(sphere.contains(Vector3(2.0, 0.0, 0.0)))
    }

    @Test
    fun `BoundingSphere intersection`() {
        val a = BoundingSphere(Vector3(0.0, 0.0, 0.0), 1.0)
        val b = BoundingSphere(Vector3(1.5, 0.0, 0.0), 1.0)
        assertTrue(a.intersects(b))
    }

    @Test
    fun `BoundingSphere no intersection`() {
        val a = BoundingSphere(Vector3(0.0, 0.0, 0.0), 1.0)
        val b = BoundingSphere(Vector3(5.0, 0.0, 0.0), 1.0)
        assertFalse(a.intersects(b))
    }

    @Test
    fun `BoundingSphere from AABB`() {
        val aabb = AABB(Vector3(-1.0, -1.0, -1.0), Vector3(1.0, 1.0, 1.0))
        val sphere = BoundingSphere.fromAABB(aabb)
        assertEquals(Vector3.ZERO, sphere.center)
        assertTrue(sphere.radius > 0.0)
    }

    @Test
    fun `Plane classify point`() {
        val plane = Plane(Vector3.UNIT_Y, 0.0)
        assertTrue(plane.isPointAbove(Vector3(0.0, 1.0, 0.0)))
        assertTrue(plane.isPointBelow(Vector3(0.0, -1.0, 0.0)))
        assertTrue(plane.isPointOnPlane(Vector3(0.0, 0.0, 0.0)))
    }

    @Test
    fun `Plane from point normal`() {
        val plane = Plane.fromPointNormal(Vector3(0.0, 5.0, 0.0), Vector3.UNIT_Y)
        assertTrue(plane.isPointOnPlane(Vector3(0.0, 5.0, 0.0)))
    }

    @Test
    fun `Plane project point`() {
        val plane = Plane(Vector3.UNIT_Y, 0.0)
        val projected = plane.projectPoint(Vector3(1.0, 5.0, 3.0))
        assertEquals(1.0, projected.x, 1e-15)
        assertEquals(0.0, projected.y, 1e-15)
        assertEquals(3.0, projected.z, 1e-15)
    }

    @Test
    fun `Ray point at parameter`() {
        val ray = Ray(Vector3.ZERO, Vector3.UNIT_X)
        val p = ray.pointAt(5.0)
        assertEquals(5.0, p.x, 1e-15)
    }

    @Test
    fun `Ray intersect plane`() {
        val ray = Ray(Vector3(0.0, -1.0, 0.0), Vector3.UNIT_Y)
        val plane = Plane(Vector3.UNIT_Y, 0.0)
        val t = ray.intersectPlane(plane)
        assertNotNull(t)
        assertEquals(1.0, t, 1e-15)
    }

    @Test
    fun `Ray parallel to plane`() {
        val ray = Ray(Vector3.ZERO, Vector3.UNIT_X)
        val plane = Plane(Vector3.UNIT_Y, 0.0)
        val t = ray.intersectPlane(plane)
        assertNull(t)
    }

    @Test
    fun `Transform identity`() {
        val t = Transform.IDENTITY
        val p = t.transformPoint(Vector3(1.0, 2.0, 3.0))
        assertEquals(1.0, p.x, 1e-15)
        assertEquals(2.0, p.y, 1e-15)
        assertEquals(3.0, p.z, 1e-15)
    }

    @Test
    fun `Transform with translation`() {
        val t = Transform(Vector3(5.0, 0.0, 0.0), Quaternion.IDENTITY)
        val p = t.transformPoint(Vector3.ZERO)
        assertEquals(5.0, p.x, 1e-15)
    }

    @Test
    fun `Transform with rotation`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 2.0)
        val t = Transform(Vector3.ZERO, q)
        val p = t.transformPoint(Vector3.UNIT_X)
        assertEquals(0.0, p.x, 1e-15)
        assertEquals(0.0, p.y, 1e-15)
        assertEquals(-1.0, p.z, 1e-15)
    }

    @Test
    fun `Transform inverse`() {
        val pos = Vector3(5.0, 0.0, 0.0)
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        val t = Transform(pos, q)
        val inv = t.inverse()
        val original = Vector3(1.0, 2.0, 3.0)
        val transformed = t.transformPoint(original)
        val recovered = inv.transformPoint(transformed)
        assertEquals(original.x, recovered.x, 1e-15)
        assertEquals(original.y, recovered.y, 1e-15)
        assertEquals(original.z, recovered.z, 1e-15)
    }

    @Test
    fun `Transform composition`() {
        val t1 = Transform(Vector3(1.0, 0.0, 0.0), Quaternion.IDENTITY)
        val t2 = Transform(Vector3(2.0, 0.0, 0.0), Quaternion.IDENTITY)
        val composed = t1 * t2
        val p = composed.transformPoint(Vector3.ZERO)
        assertEquals(3.0, p.x, 1e-15)
    }

    @Test
    fun `AABB invalid min max throws`() {
        assertThrows<IllegalArgumentException> {
            AABB(Vector3(2.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0))
        }
    }

    @Test
    fun `BoundingSphere negative radius throws`() {
        assertThrows<IllegalArgumentException> {
            BoundingSphere(Vector3.ZERO, -1.0)
        }
    }

    @Test
    fun `Ray zero direction throws`() {
        assertThrows<IllegalArgumentException> {
            Ray(Vector3.ZERO, Vector3.ZERO)
        }
    }

    @Test
    fun `Transform non-unit quaternion throws`() {
        assertThrows<IllegalArgumentException> {
            Transform(Vector3.ZERO, Quaternion(1.0, 1.0, 0.0, 0.0))
        }
    }
}
