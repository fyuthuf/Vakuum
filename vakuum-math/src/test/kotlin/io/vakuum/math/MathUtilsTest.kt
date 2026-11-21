package io.vakuum.math

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class MathUtilsTest {

    @Test
    fun `EPSILON is small`() {
        assertTrue(MathUtils.EPSILON > 0.0)
        assertTrue(MathUtils.EPSILON < 1e-5)
    }

    @Test
    fun `isNearZero for zero`() {
        assertTrue(MathUtils.isNearZero(0.0))
    }

    @Test
    fun `isNearZero for small value`() {
        assertTrue(MathUtils.isNearZero(1e-15))
    }

    @Test
    fun `isNearZero for non-zero`() {
        assertFalse(MathUtils.isNearZero(1.0))
    }

    @Test
    fun `approximatelyEqual for equal values`() {
        assertTrue(MathUtils.approximatelyEqual(1.0, 1.0))
    }

    @Test
    fun `approximatelyEqual for close values`() {
        assertTrue(MathUtils.approximatelyEqual(1.0, 1.0 + 1e-15))
    }

    @Test
    fun `approximatelyEqual for different values`() {
        assertFalse(MathUtils.approximatelyEqual(1.0, 2.0))
    }

    @Test
    fun `clamp within range`() {
        assertEquals(5.0, MathUtils.clamp(5.0, 0.0, 10.0), 1e-15)
    }

    @Test
    fun `clamp below range`() {
        assertEquals(0.0, MathUtils.clamp(-5.0, 0.0, 10.0), 1e-15)
    }

    @Test
    fun `clamp above range`() {
        assertEquals(10.0, MathUtils.clamp(15.0, 0.0, 10.0), 1e-15)
    }

    @Test
    fun `lerp at 0`() {
        assertEquals(0.0, MathUtils.lerp(0.0, 10.0, 0.0), 1e-15)
    }

    @Test
    fun `lerp at 1`() {
        assertEquals(10.0, MathUtils.lerp(0.0, 10.0, 1.0), 1e-15)
    }

    @Test
    fun `lerp at half`() {
        assertEquals(5.0, MathUtils.lerp(0.0, 10.0, 0.5), 1e-15)
    }

    @Test
    fun `validateFinite for valid value`() {
        MathUtils.validateFinite(1.0) // Should not throw
    }

    @Test
    fun `validateFinite for NaN throws`() {
        assertThrows<IllegalArgumentException> { MathUtils.validateFinite(Double.NaN) }
    }

    @Test
    fun `validateFinite for infinity throws`() {
        assertThrows<IllegalArgumentException> { MathUtils.validateFinite(Double.POSITIVE_INFINITY) }
        assertThrows<IllegalArgumentException> { MathUtils.validateFinite(Double.NEGATIVE_INFINITY) }
    }

    @Test
    fun `PI constant`() {
        assertEquals(3.141592653589793, MathUtils.PI, 1e-15)
    }

    @Test
    fun `DEG_TO_RAD conversion`() {
        assertEquals(MathUtils.PI, 180.0 * MathUtils.DEG_TO_RAD, 1e-15)
    }

    @Test
    fun `RAD_TO_DEG conversion`() {
        assertEquals(180.0, MathUtils.PI * MathUtils.RAD_TO_DEG, 1e-15)
    }
}
