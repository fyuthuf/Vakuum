package io.vakuum.math

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class Vector2Test {

    @Test
    fun `addition`() {
        val a = Vector2(1.0, 2.0)
        val b = Vector2(3.0, 4.0)
        val c = a + b
        assertEquals(4.0, c.x, 1e-15)
        assertEquals(6.0, c.y, 1e-15)
    }

    @Test
    fun `subtraction`() {
        val a = Vector2(3.0, 4.0)
        val b = Vector2(1.0, 2.0)
        val c = a - b
        assertEquals(2.0, c.x, 1e-15)
        assertEquals(2.0, c.y, 1e-15)
    }

    @Test
    fun `scalar multiplication`() {
        val a = Vector2(2.0, 3.0)
        val b = a * 2.0
        assertEquals(4.0, b.x, 1e-15)
        assertEquals(6.0, b.y, 1e-15)
    }

    @Test
    fun `scalar division`() {
        val a = Vector2(4.0, 6.0)
        val b = a / 2.0
        assertEquals(2.0, b.x, 1e-15)
        assertEquals(3.0, b.y, 1e-15)
    }

    @Test
    fun `division by zero throws`() {
        val a = Vector2(1.0, 2.0)
        assertThrows<IllegalArgumentException> { a / 0.0 }
    }

    @Test
    fun `dot product`() {
        val a = Vector2(1.0, 2.0)
        val b = Vector2(3.0, 4.0)
        assertEquals(11.0, a.dot(b), 1e-15)
    }

    @Test
    fun `dot product orthogonality`() {
        val a = Vector2(1.0, 0.0)
        val b = Vector2(0.0, 1.0)
        assertEquals(0.0, a.dot(b), 1e-15)
    }

    @Test
    fun `length`() {
        val a = Vector2(3.0, 4.0)
        assertEquals(5.0, a.length, 1e-15)
    }

    @Test
    fun `length squared`() {
        val a = Vector2(3.0, 4.0)
        assertEquals(25.0, a.lengthSquared, 1e-15)
    }

    @Test
    fun `normalize`() {
        val a = Vector2(3.0, 4.0)
        val n = a.normalize()
        assertEquals(1.0, n.length, 1e-15)
        assertEquals(0.6, n.x, 1e-15)
        assertEquals(0.8, n.y, 1e-15)
    }

    @Test
    fun `normalize zero vector throws`() {
        val a = Vector2.ZERO
        assertThrows<IllegalArgumentException> { a.normalize() }
    }

    @Test
    fun `distance`() {
        val a = Vector2(0.0, 0.0)
        val b = Vector2(3.0, 4.0)
        assertEquals(5.0, a.distanceTo(b), 1e-15)
    }

    @Test
    fun `distance symmetry`() {
        val a = Vector2(1.0, 2.0)
        val b = Vector2(4.0, 6.0)
        assertEquals(a.distanceTo(b), b.distanceTo(a), 1e-15)
    }

    @Test
    fun `lerp`() {
        val a = Vector2(0.0, 0.0)
        val b = Vector2(10.0, 10.0)
        val c = a.lerp(b, 0.5)
        assertEquals(5.0, c.x, 1e-15)
        assertEquals(5.0, c.y, 1e-15)
    }

    @Test
    fun `lerp t=0 returns a`() {
        val a = Vector2(1.0, 2.0)
        val b = Vector2(3.0, 4.0)
        val c = a.lerp(b, 0.0)
        assertEquals(a.x, c.x, 1e-15)
        assertEquals(a.y, c.y, 1e-15)
    }

    @Test
    fun `lerp t=1 returns b`() {
        val a = Vector2(1.0, 2.0)
        val b = Vector2(3.0, 4.0)
        val c = a.lerp(b, 1.0)
        assertEquals(b.x, c.x, 1e-15)
        assertEquals(b.y, c.y, 1e-15)
    }

    @Test
    fun `negate`() {
        val a = Vector2(1.0, -2.0)
        val b = a.negate()
        assertEquals(-1.0, b.x, 1e-15)
        assertEquals(2.0, b.y, 1e-15)
    }

    @Test
    fun `unary minus`() {
        val a = Vector2(1.0, -2.0)
        val b = -a
        assertEquals(-1.0, b.x, 1e-15)
        assertEquals(2.0, b.y, 1e-15)
    }

    @Test
    fun `NaN rejection`() {
        assertThrows<IllegalArgumentException> { Vector2(Double.NaN, 1.0) }
        assertThrows<IllegalArgumentException> { Vector2(1.0, Double.NaN) }
    }

    @Test
    fun `infinity rejection`() {
        assertThrows<IllegalArgumentException> { Vector2(Double.POSITIVE_INFINITY, 1.0) }
        assertThrows<IllegalArgumentException> { Vector2(1.0, Double.NEGATIVE_INFINITY) }
    }

    @Test
    fun `component-wise min`() {
        val a = Vector2(1.0, 5.0)
        val b = Vector2(3.0, 2.0)
        val c = a.min(b)
        assertEquals(1.0, c.x, 1e-15)
        assertEquals(2.0, c.y, 1e-15)
    }

    @Test
    fun `component-wise max`() {
        val a = Vector2(1.0, 5.0)
        val b = Vector2(3.0, 2.0)
        val c = a.max(b)
        assertEquals(3.0, c.x, 1e-15)
        assertEquals(5.0, c.y, 1e-15)
    }

    @Test
    fun `absolute`() {
        val a = Vector2(-3.0, 4.0)
        val b = a.absolute()
        assertEquals(3.0, b.x, 1e-15)
        assertEquals(4.0, b.y, 1e-15)
    }

    @Test
    fun `companion constants`() {
        assertEquals(0.0, Vector2.ZERO.x, 1e-15)
        assertEquals(0.0, Vector2.ZERO.y, 1e-15)
        assertEquals(1.0, Vector2.UNIT_X.x, 1e-15)
        assertEquals(0.0, Vector2.UNIT_X.y, 1e-15)
    }
}
