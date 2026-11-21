package io.vakuum.math

/**
 * 3D vector with x, y, z components.
 *
 * Uses Double precision for numerical stability.
 * Right-handed coordinate system: X right, Y up, Z backward.
 */
data class Vector3(val x: Double, val y: Double, val z: Double) {

    init {
        MathUtils.validateFinite(x, "x")
        MathUtils.validateFinite(y, "y")
        MathUtils.validateFinite(z, "z")
    }

    companion object {
        val ZERO = Vector3(0.0, 0.0, 0.0)
        val ONE = Vector3(1.0, 1.0, 1.0)
        val UNIT_X = Vector3(1.0, 0.0, 0.0)
        val UNIT_Y = Vector3(0.0, 1.0, 0.0)
        val UNIT_Z = Vector3(0.0, 0.0, 1.0)
        val UP = UNIT_Y
        val RIGHT = UNIT_X
        val BACK = UNIT_Z
    }

    fun negate(): Vector3 = Vector3(-x, -y, -z)

    operator fun plus(other: Vector3): Vector3 = Vector3(x + other.x, y + other.y, z + other.z)

    operator fun minus(other: Vector3): Vector3 = Vector3(x - other.x, y - other.y, z - other.z)

    operator fun times(scalar: Double): Vector3 = Vector3(x * scalar, y * scalar, z * scalar)

    operator fun div(scalar: Double): Vector3 {
        require(!MathUtils.isNearZero(scalar)) { "Cannot divide vector by zero"}
        return Vector3(x / scalar, y / scalar, z / scalar)
    }

    operator fun unaryMinus(): Vector3 = Vector3(-x, -y, -z)

    /** Dot product with another vector. */
    fun dot(other: Vector3): Double = x * other.x + y * other.y + z * other.z

    /**
     * Cross product with another vector.
     * In a right-handed system: cross(A, B) produces a vector perpendicular to both A and B.
     */
    fun cross(other: Vector3): Vector3 = Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    )

    val lengthSquared: Double get() = x * x + y * y + z * z

    val length: Double get() = kotlin.math.sqrt(lengthSquared)

    /**
     * Normalize this vector to unit length.
     * @throws IllegalArgumentException if the vector has zero length.
     */
    fun normalize(): Vector3 {
        val len = length
        require(!MathUtils.isNearZero(len)) { "Cannot normalize a zero-length vector"}
        return this / len
    }

    fun distanceTo(other: Vector3): Double = (this - other).length

    fun distanceSquaredTo(other: Vector3): Double = (this - other).lengthSquared

    fun lerp(other: Vector3, t: Double): Vector3 =
        Vector3(
            MathUtils.lerp(x, other.x, t),
            MathUtils.lerp(y, other.y, t),
            MathUtils.lerp(z, other.z, t)
        )

    /**
     * Project this vector onto [other].
     */
    fun projectOnto(other: Vector3): Vector3 {
        val otherLenSq = other.lengthSquared
        require(!MathUtils.isNearZero(otherLenSq)) { "Cannot project onto a zero-length vector"}
        val t = this.dot(other) / otherLenSq
        return other * t
    }

    fun min(other: Vector3): Vector3 =
        Vector3(
            kotlin.math.min(x, other.x),
            kotlin.math.min(y, other.y),
            kotlin.math.min(z, other.z)
        )

    fun max(other: Vector3): Vector3 =
        Vector3(
            kotlin.math.max(x, other.x),
            kotlin.math.max(y, other.y),
            kotlin.math.max(z, other.z)
        )

    fun absolute(): Vector3 = Vector3(kotlin.math.abs(x), kotlin.math.abs(y), kotlin.math.abs(z))

    fun multiplyComponentWise(other: Vector3): Vector3 =
        Vector3(x * other.x, y * other.y, z * other.z)

    /**
     * Check if this vector is approximately zero.
     */
    fun isNearZero(): Boolean = lengthSquared < MathUtils.EPSILON * MathUtils.EPSILON

    override fun toString(): String = "Vector3(x=$x, y=$y, z=$z)"
}
