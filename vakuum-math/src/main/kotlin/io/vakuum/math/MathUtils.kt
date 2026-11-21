package io.vakuum.math

/**
 * Mathematical constants and utility functions for Vakuum.
 *
 * Uses Double precision (64-bit floating point) throughout.
 * EPSILON is the default tolerance for near-zero comparisons.
 */
object MathUtils {

    /** Default epsilon for near-zero comparisons. */
    const val EPSILON: Double = 1e-10

    /** PI constant. */
    const val PI: Double = 3.141592653589793

    /** PI / 2. */
    const val HALF_PI: Double = 1.5707963267948966

    /** PI * 2. */
    const val TWO_PI: Double = 6.283185307179586

    /** Degrees to radians conversion factor. */
    const val DEG_TO_RAD: Double = PI / 180.0

    /** Radians to degrees conversion factor. */
    const val RAD_TO_DEG: Double = 180.0 / PI

    /**
     * Check if a value is near zero within EPSILON tolerance.
     */
    fun isNearZero(value: Double): Boolean {
        return kotlin.math.abs(value) < EPSILON
    }

    /**
     * Check if two values are approximately equal within the given tolerance.
     */
    fun approximatelyEqual(a: Double, b: Double, tolerance: Double = EPSILON): Boolean {
        return kotlin.math.abs(a - b) < tolerance
    }

    /**
     * Clamp a value between [min] and [max].
     */
    fun clamp(value: Double, min: Double, max: Double): Double {
        return when {
            value < min -> min
            value > max -> max
            else -> value
        }
    }

    /**
     * Linear interpolation between [a] and [b] by [t].
     * [t] is clamped to [0.0, 1.0].
     */
    fun lerp(a: Double, b: Double, t: Double): Double {
        val clampedT = clamp(t, 0.0, 1.0)
        return a + (b - a) * clampedT
    }

    /**
     * Validate that a value is not NaN or infinite.
     * @throws IllegalArgumentException if the value is NaN or infinite.
     */
    fun validateFinite(value: Double, name: String = "value") {
        if (value.isNaN()) {
            throw IllegalArgumentException("$name must not be NaN")
        }
        if (value.isInfinite()) {
            throw IllegalArgumentException("$name must not be infinite")
        }
    }
}
