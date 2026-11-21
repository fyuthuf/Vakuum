package io.vakuum.math

/**
 * Quaternion for 3D rotation representation.
 *
 * Uses Hamilton convention (ij = k) with a right-handed coordinate system.
 * A quaternion q = w + xi + yj + zk represents a rotation where:
 * - w is the scalar (cosine of half-angle)
 * - (x, y, z) is the vector part (axis * sine of half-angle)
 *
 * Multiplication order: q * v * q^-1 rotates vector v by quaternion q.
 * Composition: q1 * q2 means "apply q2 first, then q1".
 */
data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {

    init {
        MathUtils.validateFinite(w, "w")
        MathUtils.validateFinite(x, "x")
        MathUtils.validateFinite(y, "y")
        MathUtils.validateFinite(z, "z")
    }

    companion object {
        val IDENTITY = Quaternion(1.0, 0.0, 0.0, 0.0)

        /**
         * Create a quaternion from an axis and angle.
         * The axis must be normalized.
         * Angle is in radians.
         */
        fun fromAxisAngle(axis: Vector3, angle: Double): Quaternion {
            require(!axis.isNearZero()) { "Axis must not be a zero vector"}
            val halfAngle = angle / 2.0
            val s = kotlin.math.sin(halfAngle)
            return Quaternion(
                kotlin.math.cos(halfAngle),
                axis.x * s,
                axis.y * s,
                axis.z * s
            )
        }

        /**
         * Create a quaternion from two directions.
         * Returns the shortest rotation from [from] to [to].
         */
        fun fromDirections(from: Vector3, to: Vector3): Quaternion {
            val f = from.normalize()
            val t = to.normalize()
            val cosAngle = f.dot(t)

            // If vectors are nearly identical
            if (MathUtils.approximatelyEqual(cosAngle, 1.0)) {
                return IDENTITY
            }

            // If vectors are nearly opposite
            if (MathUtils.approximatelyEqual(cosAngle, -1.0)) {
                // Find an orthogonal axis
                val axis = if (kotlin.math.abs(f.x) < kotlin.math.abs(f.z)) {
                    f.cross(Vector3.UNIT_X).normalize()
                } else {
                    f.cross(Vector3.UNIT_Z).normalize()
                }
                return fromAxisAngle(axis, MathUtils.PI)
            }

            val axis = f.cross(t).normalize()
            val angle = kotlin.math.acos(cosAngle)
            return fromAxisAngle(axis, angle)
        }

        /**
         * Create a quaternion from a rotation matrix.
         */
        fun fromMatrix3(m: Matrix3): Quaternion {
            val trace = m.get(0, 0) + m.get(1, 1) + m.get(2, 2)

            return if (trace > 0) {
                val s = 0.5 / kotlin.math.sqrt(trace + 1.0)
                Quaternion(
                    0.25 / s,
                    (m.get(2, 1) - m.get(1, 2)) * s,
                    (m.get(0, 2) - m.get(2, 0)) * s,
                    (m.get(1, 0) - m.get(0, 1)) * s
                )
            } else if (m.get(0, 0) > m.get(1, 1) && m.get(0, 0) > m.get(2, 2)) {
                val s = 2.0 * kotlin.math.sqrt(1.0 + m.get(0, 0) - m.get(1, 1) - m.get(2, 2))
                Quaternion(
                    (m.get(2, 1) - m.get(1, 2)) / s,
                    0.25 * s,
                    (m.get(0, 1) + m.get(1, 0)) / s,
                    (m.get(0, 2) + m.get(2, 0)) / s
                )
            } else if (m.get(1, 1) > m.get(2, 2)) {
                val s = 2.0 * kotlin.math.sqrt(1.0 + m.get(1, 1) - m.get(0, 0) - m.get(2, 2))
                Quaternion(
                    (m.get(0, 2) - m.get(2, 0)) / s,
                    (m.get(0, 1) + m.get(1, 0)) / s,
                    0.25 * s,
                    (m.get(1, 2) + m.get(2, 1)) / s
                )
            } else {
                val s = 2.0 * kotlin.math.sqrt(1.0 + m.get(2, 2) - m.get(0, 0) - m.get(1, 1))
                Quaternion(
                    (m.get(1, 0) - m.get(0, 1)) / s,
                    (m.get(0, 2) + m.get(2, 0)) / s,
                    (m.get(1, 2) + m.get(2, 1)) / s,
                    0.25 * s
                )
            }
        }
    }

    /** Squared norm (w² + x² + y² + z²). */
    val lengthSquared: Double get() = w * w + x * x + y * y + z * z

    /** Norm of the quaternion. */
    val length: Double get() = kotlin.math.sqrt(lengthSquared)

    /**
     * Normalize this quaternion to unit length.
     * @throws IllegalArgumentException if the quaternion has zero length.
     */
    fun normalize(): Quaternion {
        val len = length
        require(!MathUtils.isNearZero(len)) { "Cannot normalize a zero-length quaternion"}
        return Quaternion(w / len, x / len, y / len, z / len)
    }

    /**
     * Conjugate of this quaternion (w, -x, -y, -z).
     */
    fun conjugate(): Quaternion = Quaternion(w, -x, -y, -z)

    /**
     * Inverse of this quaternion.
     * For unit quaternions, the inverse equals the conjugate.
     * @throws IllegalArgumentException if the quaternion has zero length.
     */
    fun inverse(): Quaternion {
        val lenSq = lengthSquared
        require(!MathUtils.isNearZero(lenSq)) { "Cannot invert a zero-length quaternion"}
        return Quaternion(w / lenSq, -x / lenSq, -y / lenSq, -z / lenSq)
    }

    /**
     * Multiply two quaternions (Hamilton product).
     * q1 * q2 means "apply q2 first, then q1".
     */
    operator fun times(other: Quaternion): Quaternion = Quaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w
    )

    /**
     * Rotate a Vector3 by this quaternion.
     * Assumes this quaternion is normalized.
     */
    fun rotateVector(v: Vector3): Vector3 {
        val qv = Quaternion(0.0, v.x, v.y, v.z)
        val conjugated = this * qv * conjugate()
        return Vector3(conjugated.x, conjugated.y, conjugated.z)
    }

    /**
     * Spherical linear interpolation between two quaternions.
     * Takes the shortest path.
     */
    fun slerp(other: Quaternion, t: Double): Quaternion {
        val clampedT = MathUtils.clamp(t, 0.0, 1.0)

        if (clampedT <= 0.0) return this
        if (clampedT >= 1.0) return other

        var o = other
        var cosOmega = w * o.w + x * o.x + y * o.y + z * o.z

        // Take shortest path
        if (cosOmega < 0.0) {
            o = Quaternion(-o.w, -o.x, -o.y, -o.z)
            cosOmega = -cosOmega
        }

        if (cosOmega > 1.0 - MathUtils.EPSILON) {
            // Too close, use linear interpolation
            val result = Quaternion(
                w + clampedT * (o.w - w),
                x + clampedT * (o.x - x),
                y + clampedT * (o.y - y),
                z + clampedT * (o.z - z)
            )
            return result.normalize()
        }

        val omega = kotlin.math.acos(cosOmega)
        val sinOmega = kotlin.math.sin(omega)
        val a = kotlin.math.sin((1.0 - clampedT) * omega) / sinOmega
        val b = kotlin.math.sin(clampedT * omega) / sinOmega

        return Quaternion(
            a * w + b * o.w,
            a * x + b * o.x,
            a * y + b * o.y,
            a * z + b * o.z
        )
    }

    /**
     * Get the axis of rotation.
     * @throws IllegalArgumentException if this is the identity quaternion.
     */
    fun getAxis(): Vector3 {
        val vecLen = kotlin.math.sqrt(x * x + y * y + z * z)
        require(!MathUtils.isNearZero(vecLen)) { "Cannot get axis from identity quaternion"}
        return Vector3(x / vecLen, y / vecLen, z / vecLen)
    }

    /**
     * Get the angle of rotation in radians.
     */
    fun getAngle(): Double {
        val vecLen = kotlin.math.sqrt(x * x + y * y + z * z)
        if (MathUtils.isNearZero(vecLen)) {
            return 0.0
        }
        return 2.0 * kotlin.math.atan2(vecLen, w)
    }

    /**
     * Convert to a rotation matrix.
     */
    fun toMatrix3(): Matrix3 = Matrix3.fromQuaternion(this)

    /**
     * Check if this quaternion is approximately a unit quaternion.
     */
    fun isUnit(): Boolean = MathUtils.approximatelyEqual(lengthSquared, 1.0)

    override fun toString(): String = "Quaternion(w=$w, x=$x, y=$y, z=$z)"
}
