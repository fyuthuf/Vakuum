package io.vakuum.math

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class Vector3Test {

    @Test
    fun `addition`() {
        val a = Vector3(1.0, 2.0, 3.0)
        val b = Vector3(4.0, 5.0, 6.0)
        val c = a + b
        assertEquals(5.0, c.x, 1e-15)
        assertEquals(7.0, c.y, 1e-15)
        assertEquals(9.0, c.z, 1e-15)
    }

    @Test
    fun `subtraction`() {
        val a = Vector3(4.0, 5.0, 6.0)
        val b = Vector3(1.0, 2.0, 3.0)
        val c = a - b
        assertEquals(3.0, c.x, 1e-15)
        assertEquals(3.0, c.y, 1e-15)
        assertEquals(3.0, c.z, 1e-15)
    }

    @Test
    fun `scalar multiplication`() {
        val a = Vector3(1.0, 2.0, 3.0)
        val b = a * 2.0
        assertEquals(2.0, b.x, 1e-15)
        assertEquals(4.0, b.y, 1e-15)
        assertEquals(6.0, b.z, 1e-15)
    }

    @Test
    fun `dot product`() {
        val a = Vector3(1.0, 2.0, 3.0)
        val b = Vector3(4.0, 5.0, 6.0)
        assertEquals(32.0, a.dot(b), 1e-15)
    }

    @Test
    fun `dot product commutativity`() {
        val a = Vector3(1.0, 2.0, 3.0)
        val b = Vector3(4.0, 5.0, 6.0)
        assertEquals(a.dot(b), b.dot(a), 1e-15)
    }

    @Test
    fun `cross product`() {
        val a = Vector3.UNIT_X
        val b = Vector3.UNIT_Y
        val c = a.cross(b)
        assertEquals(0.0, c.x, 1e-15)
        assertEquals(0.0, c.y, 1e-15)
        assertEquals(1.0, c.z, 1e-15)
    }

    @Test
    fun `cross product anti-commutativity`() {
        val a = Vector3.UNIT_X
        val b = Vector3.UNIT_Y
        val c = a.cross(b)
        val d = b.cross(a)
        assertEquals(-c.x, d.x, 1e-15)
        assertEquals(-c.y, d.y, 1e-15)
        assertEquals(-c.z, d.z, 1e-15)
    }

    @Test
    fun `cross product perpendicularity`() {
        val a = Vector3(1.0, 2.0, 3.0)
        val b = Vector3(4.0, 5.0, 6.0)
        val c = a.cross(b)
        assertEquals(0.0, a.dot(c), 1e-15)
        assertEquals(0.0, b.dot(c), 1e-15)
    }

    @Test
    fun `length`() {
        val a = Vector3(1.0, 2.0, 2.0)
        assertEquals(3.0, a.length, 1e-15)
    }

    @Test
    fun `normalize`() {
        val a = Vector3(1.0, 2.0, 2.0)
        val n = a.normalize()
        assertEquals(1.0, n.length, 1e-15)
    }

    @Test
    fun `normalize zero vector throws`() {
        assertThrows<IllegalArgumentException> { Vector3.ZERO.normalize() }
    }

    @Test
    fun `distance`() {
        val a = Vector3(0.0, 0.0, 0.0)
        val b = Vector3(1.0, 2.0, 2.0)
        assertEquals(3.0, a.distanceTo(b), 1e-15)
    }

    @Test
    fun `lerp`() {
        val a = Vector3(0.0, 0.0, 0.0)
        val b = Vector3(10.0, 10.0, 10.0)
        val c = a.lerp(b, 0.5)
        assertEquals(5.0, c.x, 1e-15)
        assertEquals(5.0, c.y, 1e-15)
        assertEquals(5.0, c.z, 1e-15)
    }

    @Test
    fun `project onto`() {
        val a = Vector3(3.0, 4.0, 0.0)
        val b = Vector3.UNIT_X
        val p = a.projectOnto(b)
        assertEquals(3.0, p.x, 1e-15)
        assertEquals(0.0, p.y, 1e-15)
        assertEquals(0.0, p.z, 1e-15)
    }

    @Test
    fun `isNearZero`() {
        assertTrue(Vector3.ZERO.isNearZero())
        assertFalse(Vector3(1e-20, 1e-20, 1e-20).isNearZero().not())
        assertFalse(Vector3(1.0, 0.0, 0.0).isNearZero())
    }

    @Test
    fun `NaN rejection`() {
        assertThrows<IllegalArgumentException> { Vector3(Double.NaN, 0.0, 0.0) }
    }

    @Test
    fun `infinity rejection`() {
        assertThrows<IllegalArgumentException> { Vector3(Double.POSITIVE_INFINITY, 0.0, 0.0) }
    }

    @Test
    fun `component-wise operations`() {
        val a = Vector3(-1.0, 2.0, -3.0)
        val abs = a.absolute()
        assertEquals(1.0, abs.x, 1e-15)
        assertEquals(2.0, abs.y, 1e-15)
        assertEquals(3.0, abs.z, 1e-15)
    }

    @Test
    fun `companion constants`() {
        assertEquals(0.0, Vector3.ZERO.x, 1e-15)
        assertEquals(1.0, Vector3.UNIT_X.x, 1e-15)
        assertEquals(0.0, Vector3.UNIT_X.y, 1e-15)
        assertEquals(0.0, Vector3.UNIT_X.z, 1e-15)
    }
}
