package io.vakuum.math

/**
 * 4D vector with x, y, z, w components.
 *
 * Used for homogeneous coordinates and quaternion storage.
 */
data class Vector4(val x: Double, val y: Double, val z: Double, val w: Double) {

    init {
        MathUtils.validateFinite(x, "x")
        MathUtils.validateFinite(y, "y")
        MathUtils.validateFinite(z, "z")
        MathUtils.validateFinite(w, "w")
    }

    companion object {
        val ZERO = Vector4(0.0, 0.0, 0.0, 0.0)
        val ONE = Vector4(1.0, 1.0, 1.0, 1.0)
        val UNIT_X = Vector4(1.0, 0.0, 0.0, 0.0)
        val UNIT_Y = Vector4(0.0, 1.0, 0.0, 0.0)
        val UNIT_Z = Vector4(0.0, 0.0, 1.0, 0.0)
        val UNIT_W = Vector4(0.0, 0.0, 0.0, 1.0)
    }

    fun negate(): Vector4 = Vector4(-x, -y, -z, -w)

    operator fun plus(other: Vector4): Vector4 = Vector4(x + other.x, y + other.y, z + other.z, w + other.w)

    operator fun minus(other: Vector4): Vector4 = Vector4(x - other.x, y - other.y, z - other.z, w - other.w)

    operator fun times(scalar: Double): Vector4 = Vector4(x * scalar, y * scalar, z * scalar, w * scalar)

    operator fun div(scalar: Double): Vector4 {
        require(!MathUtils.isNearZero(scalar)) { "Cannot divide vector by zero"}
        return Vector4(x / scalar, y / scalar, z / scalar, w / scalar)
    }

    operator fun unaryMinus(): Vector4 = Vector4(-x, -y, -z, -w)

    fun dot(other: Vector4): Double = x * other.x + y * other.y + z * other.z + w * other.w

    val lengthSquared: Double get() = x * x + y * y + z * z + w * w

    val length: Double get() = kotlin.math.sqrt(lengthSquared)

    fun normalize(): Vector4 {
        val len = length
        require(!MathUtils.isNearZero(len)) { "Cannot normalize a zero-length vector"}
        return this / len
    }

    fun distanceTo(other: Vector4): Double = (this - other).length

    fun lerp(other: Vector4, t: Double): Vector4 =
        Vector4(
            MathUtils.lerp(x, other.x, t),
            MathUtils.lerp(y, other.y, t),
            MathUtils.lerp(z, other.z, t),
            MathUtils.lerp(w, other.w, t)
        )

    fun multiplyComponentWise(other: Vector4): Vector4 =
        Vector4(x * other.x, y * other.y, z * other.z, w * other.w)

    override fun toString(): String = "Vector4(x=$x, y=$y, z=$z, w=$w)"
}
