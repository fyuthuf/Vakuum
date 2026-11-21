package io.vakuum.math

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class QuaternionTest {

    @Test
    fun `identity quaternion`() {
        assertEquals(1.0, Quaternion.IDENTITY.w, 1e-15)
        assertEquals(0.0, Quaternion.IDENTITY.x, 1e-15)
        assertEquals(0.0, Quaternion.IDENTITY.y, 1e-15)
        assertEquals(0.0, Quaternion.IDENTITY.z, 1e-15)
    }

    @Test
    fun `identity rotates vector unchanged`() {
        val v = Vector3(1.0, 2.0, 3.0)
        val rotated = Quaternion.IDENTITY.rotateVector(v)
        assertEquals(v.x, rotated.x, 1e-15)
        assertEquals(v.y, rotated.y, 1e-15)
        assertEquals(v.z, rotated.z, 1e-15)
    }

    @Test
    fun `from axis angle 90 degrees X`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_X, MathUtils.PI / 2.0)
        val v = Vector3.UNIT_Y
        val rotated = q.rotateVector(v)
        assertEquals(0.0, rotated.x, 1e-15)
        assertEquals(0.0, rotated.y, 1e-15)
        assertEquals(1.0, rotated.z, 1e-15)
    }

    @Test
    fun `from axis angle 180 degrees Y`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI)
        val v = Vector3.UNIT_X
        val rotated = q.rotateVector(v)
        assertEquals(-1.0, rotated.x, 1e-15)
        assertEquals(0.0, rotated.y, 1e-15)
        assertEquals(0.0, rotated.z, 1e-15)
    }

    @Test
    fun `normalization`() {
        val q = Quaternion(1.0, 2.0, 2.0, 0.0)
        val n = q.normalize()
        assertEquals(1.0, n.length, 1e-15)
    }

    @Test
    fun `normalize zero throws`() {
        val q = Quaternion(0.0, 0.0, 0.0, 0.0)
        assertThrows<IllegalArgumentException> { q.normalize() }
    }

    @Test
    fun `conjugate`() {
        val q = Quaternion(1.0, 2.0, 3.0, 4.0)
        val c = q.conjugate()
        assertEquals(1.0, c.w, 1e-15)
        assertEquals(-2.0, c.x, 1e-15)
        assertEquals(-3.0, c.y, 1e-15)
        assertEquals(-4.0, c.z, 1e-15)
    }

    @Test
    fun `inverse of unit quaternion equals conjugate`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        val inv = q.inverse()
        val conj = q.conjugate()
        assertEquals(conj.w, inv.w, 1e-15)
        assertEquals(conj.x, inv.x, 1e-15)
        assertEquals(conj.y, inv.y, 1e-15)
        assertEquals(conj.z, inv.z, 1e-15)
    }

    @Test
    fun `multiplication is non-commutative`() {
        val a = Quaternion.fromAxisAngle(Vector3.UNIT_X, MathUtils.PI / 2.0)
        val b = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 2.0)
        val ab = a * b
        val ba = b * a
        assertNotEquals(ab, ba)
    }

    @Test
    fun `SLERP at t=0 returns self`() {
        val a = Quaternion.fromAxisAngle(Vector3.UNIT_X, MathUtils.PI / 4.0)
        val b = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        val result = a.slerp(b, 0.0)
        assertEquals(a.w, result.w, 1e-15)
        assertEquals(a.x, result.x, 1e-15)
        assertEquals(a.y, result.y, 1e-15)
        assertEquals(a.z, result.z, 1e-15)
    }

    @Test
    fun `SLERP at t=1 returns other`() {
        val a = Quaternion.fromAxisAngle(Vector3.UNIT_X, MathUtils.PI / 4.0)
        val b = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        val result = a.slerp(b, 1.0)
        assertEquals(b.w, result.w, 1e-15)
        assertEquals(b.x, result.x, 1e-15)
        assertEquals(b.y, result.y, 1e-15)
        assertEquals(b.z, result.z, 1e-15)
    }

    @Test
    fun `SLERP produces unit quaternion`() {
        val a = Quaternion.fromAxisAngle(Vector3.UNIT_X, MathUtils.PI / 4.0)
        val b = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        val result = a.slerp(b, 0.5)
        assertEquals(1.0, result.length, 1e-15)
    }

    @Test
    fun `to matrix3 and back`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 3.0)
        val m = q.toMatrix3()
        val q2 = Quaternion.fromMatrix3(m)
        val v = Vector3(1.0, 2.0, 3.0)
        val r1 = q.rotateVector(v)
        val r2 = q2.rotateVector(v)
        assertEquals(r1.x, r2.x, 1e-12)
        assertEquals(r1.y, r2.y, 1e-12)
        assertEquals(r1.z, r2.z, 1e-12)
    }

    @Test
    fun `get axis and angle`() {
        val axis = Vector3.UNIT_Y
        val angle = MathUtils.PI / 3.0
        val q = Quaternion.fromAxisAngle(axis, angle)
        val recoveredAxis = q.getAxis()
        val recoveredAngle = q.getAngle()
        assertEquals(axis.x, recoveredAxis.x, 1e-15)
        assertEquals(axis.y, recoveredAxis.y, 1e-15)
        assertEquals(axis.z, recoveredAxis.z, 1e-15)
        assertEquals(angle, recoveredAngle, 1e-15)
    }

    @Test
    fun `is unit quaternion`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        assertTrue(q.isUnit())
    }

    @Test
    fun `NaN rejection`() {
        assertThrows<IllegalArgumentException> { Quaternion(Double.NaN, 0.0, 0.0, 0.0) }
    }

    @Test
    fun `from directions identical vectors`() {
        val v = Vector3.UNIT_X
        val q = Quaternion.fromDirections(v, v)
        assertEquals(Quaternion.IDENTITY.w, q.w, 1e-15)
    }

    @Test
    fun `from directions opposite vectors`() {
        val a = Vector3.UNIT_X
        val b = Vector3(-1.0, 0.0, 0.0)
        val q = Quaternion.fromDirections(a, b)
        val rotated = q.rotateVector(a)
        assertEquals(-1.0, rotated.x, 1e-15)
    }
}
