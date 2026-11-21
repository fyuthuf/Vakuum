package io.vakuum.math

/**
 * 2D vector with x and y components.
 *
 * Uses Double precision for numerical stability.
 */
data class Vector2(val x: Double, val y: Double) {

    init {
        MathUtils.validateFinite(x, "x")
        MathUtils.validateFinite(y, "y")
    }

    companion object {
        val ZERO = Vector2(0.0, 0.0)
        val ONE = Vector2(1.0, 1.0)
        val UNIT_X = Vector2(1.0, 0.0)
        val UNIT_Y = Vector2(0.0, 1.0)
    }

    /** Negate this vector. */
    fun negate(): Vector2 = Vector2(-x, -y)

    /** Add another vector. */
    operator fun plus(other: Vector2): Vector2 = Vector2(x + other.x, y + other.y)

    /** Subtract another vector. */
    operator fun minus(other: Vector2): Vector2 = Vector2(x - other.x, y - other.y)

    /** Multiply by a scalar. */
    operator fun times(scalar: Double): Vector2 = Vector2(x * scalar, y * scalar)

    /** Divide by a scalar. */
    operator fun div(scalar: Double): Vector2 {
        require(!MathUtils.isNearZero(scalar)) { "Cannot divide vector by zero"}
        return Vector2(x / scalar, y / scalar)
    }

    /** Unary minus. */
    operator fun unaryMinus(): Vector2 = Vector2(-x, -y)

    /** Dot product with another vector. */
    fun dot(other: Vector2): Double = x * other.x + y * other.y

    /** Squared length of this vector. */
    val lengthSquared: Double get() = x * x + y * y

    /** Length (magnitude) of this vector. */
    val length: Double get() = kotlin.math.sqrt(lengthSquared)

    /**
     * Normalize this vector to unit length.
     * @throws IllegalArgumentException if the vector has zero length.
     */
    fun normalize(): Vector2 {
        val len = length
        require(!MathUtils.isNearZero(len)) { "Cannot normalize a zero-length vector"}
        return this / len
    }

    /** Distance to another vector. */
    fun distanceTo(other: Vector2): Double = (this - other).length

    /** Squared distance to another vector. */
    fun distanceSquaredTo(other: Vector2): Double = (this - other).lengthSquared

    /** Linear interpolation to another vector. */
    fun lerp(other: Vector2, t: Double): Vector2 =
        Vector2(MathUtils.lerp(x, other.x, t), MathUtils.lerp(y, other.y, t))

    /** Component-wise minimum. */
    fun min(other: Vector2): Vector2 =
        Vector2(kotlin.math.min(x, other.x), kotlin.math.min(y, other.y))

    /** Component-wise maximum. */
    fun max(other: Vector2): Vector2 =
        Vector2(kotlin.math.max(x, other.x), kotlin.math.max(y, other.y))

    /** Component-wise absolute value. */
    fun absolute(): Vector2 = Vector2(kotlin.math.abs(x), kotlin.math.abs(y))

    /** Component-wise multiplication. */
    fun multiplyComponentWise(other: Vector2): Vector2 =
        Vector2(x * other.x, y * other.y)

    override fun toString(): String = "Vector2(x=$x, y=$y)"
}
