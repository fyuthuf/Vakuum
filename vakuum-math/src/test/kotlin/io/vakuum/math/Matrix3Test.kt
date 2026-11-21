package io.vakuum.math

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class Matrix3Test {

    @Test
    fun `identity matrix`() {
        val m = Matrix3.IDENTITY
        for (i in 0..2) {
            for (j in 0..2) {
                assertEquals(if (i == j) 1.0 else 0.0, m.get(i, j), 1e-15)
            }
        }
    }

    @Test
    fun `identity determinant is 1`() {
        assertEquals(1.0, Matrix3.IDENTITY.determinant, 1e-15)
    }

    @Test
    fun `identity inverse is identity`() {
        val inv = Matrix3.IDENTITY.inverse()
        assertEquals(Matrix3.IDENTITY, inv)
    }

    @Test
    fun `matrix multiplication identity preservation`() {
        val m = Matrix3.rotationX(0.5)
        val result = m * Matrix3.IDENTITY
        assertEquals(m, result)
    }

    @Test
    fun `matrix multiplication associativity`() {
        val a = Matrix3.rotationX(0.3)
        val b = Matrix3.rotationY(0.4)
        val c = Matrix3.rotationZ(0.5)
        val left = (a * b) * c
        val right = a * (b * c)
        assertEquals(left, right)
    }

    @Test
    fun `transpose of identity`() {
        val t = Matrix3.IDENTITY.transpose()
        assertEquals(Matrix3.IDENTITY, t)
    }

    @Test
    fun `transpose of symmetric matrix`() {
        val m = Matrix3.scaling(2.0, 3.0, 4.0)
        val t = m.transpose()
        assertEquals(m, t)
    }

    @Test
    fun `inverse times original equals identity`() {
        val m = Matrix3.rotationX(MathUtils.PI / 4.0)
        val inv = m.inverse()
        val product = m * inv
        assertEquals(Matrix3.IDENTITY, product)
    }

    @Test
    fun `singular matrix inverse throws`() {
        val m = Matrix3.ZERO
        assertThrows<IllegalArgumentException> { m.inverse() }
    }

    @Test
    fun `rotation X matrix`() {
        val m = Matrix3.rotationX(MathUtils.PI / 2.0)
        val v = Vector3.UNIT_Y
        val result = m.multiplyVector(v)
        assertEquals(0.0, result.x, 1e-15)
        assertEquals(0.0, result.y, 1e-15)
        assertEquals(1.0, result.z, 1e-15)
    }

    @Test
    fun `rotation Y matrix`() {
        val m = Matrix3.rotationY(MathUtils.PI / 2.0)
        val v = Vector3.UNIT_X
        val result = m.multiplyVector(v)
        assertEquals(0.0, result.x, 1e-15)
        assertEquals(0.0, result.y, 1e-15)
        assertEquals(-1.0, result.z, 1e-15)
    }

    @Test
    fun `scaling matrix`() {
        val m = Matrix3.scaling(2.0, 3.0, 4.0)
        val v = Vector3(1.0, 1.0, 1.0)
        val result = m.multiplyVector(v)
        assertEquals(2.0, result.x, 1e-15)
        assertEquals(3.0, result.y, 1e-15)
        assertEquals(4.0, result.z, 1e-15)
    }

    @Test
    fun `uniform scaling`() {
        val m = Matrix3.scaling(2.0)
        assertEquals(2.0, m.get(0, 0), 1e-15)
        assertEquals(2.0, m.get(1, 1), 1e-15)
        assertEquals(2.0, m.get(2, 2), 1e-15)
    }

    @Test
    fun `from quaternion and back`() {
        val q = Quaternion.fromAxisAngle(Vector3.UNIT_Y, MathUtils.PI / 4.0)
        val m = Matrix3.fromQuaternion(q)
        val v = Vector3.UNIT_X
        val rotated = m.multiplyVector(v)
        val rotatedQ = q.rotateVector(v)
        assertEquals(rotatedQ.x, rotated.x, 1e-15)
        assertEquals(rotatedQ.y, rotated.y, 1e-15)
        assertEquals(rotatedQ.z, rotated.z, 1e-15)
    }

    @Test
    fun `NaN rejection`() {
        assertThrows<IllegalArgumentException> {
            Matrix3.fromRowMajor(Double.NaN, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        }
    }

    @Test
    fun `zero matrix determinant is zero`() {
        assertEquals(0.0, Matrix3.ZERO.determinant, 1e-15)
    }
}
