package io.vakuum.math

/**
 * 3x3 matrix for rotation and scale operations.
 *
 * Stored in row-major order internally as a flat array of 9 doubles.
 * Index layout:
 * ```
 * [0] [1] [2]
 * [3] [4] [5]
 * [6] [7] [8]
 * ```
 */
class Matrix3 {
    private val data: DoubleArray

    constructor(vararg elements: Double) {
        require(elements.size == 9) { "Matrix3 requires exactly 9 elements"}
        data = elements.copyOf()
        for (i in 0 until 9) {
            MathUtils.validateFinite(data[i], "matrix element [$i]")
        }
    }

    companion object {
        /** Identity matrix. */
        val IDENTITY = Matrix3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        )

        /** Zero matrix. */
        val ZERO = Matrix3(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        )

        /**
         * Create a matrix from 9 elements in row-major order.
         */
        fun fromRowMajor(
            m00: Double, m01: Double, m02: Double,
            m10: Double, m11: Double, m12: Double,
            m20: Double, m21: Double, m22: Double
        ): Matrix3 = Matrix3(
            m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22
        )

        /**
         * Create a rotation matrix from a quaternion.
         */
        fun fromQuaternion(q: Quaternion): Matrix3 {
            val x = q.x
            val y = q.y
            val z = q.z
            val x2 = x + x
            val y2 = y + y
            val z2 = z + z
            val xx = x * x2
            val xy = x * y2
            val xz = x * z2
            val yy = y * y2
            val yz = y * z2
            val zz = z * z2
            val wx = q.w * x2
            val wy = q.w * y2
            val wz = q.w * z2

            return Matrix3(
                1.0 - (yy + zz), xy - wz, xz + wy,
                xy + wz, 1.0 - (xx + zz), yz - wx,
                xz - wy, yz + wx, 1.0 - (xx + yy)
            )
        }

        fun rotationX(angle: Double): Matrix3 {
            val c = kotlin.math.cos(angle)
            val s = kotlin.math.sin(angle)
            return Matrix3(
                1.0, 0.0, 0.0,
                0.0, c, -s,
                0.0, s, c
            )
        }

        fun rotationY(angle: Double): Matrix3 {
            val c = kotlin.math.cos(angle)
            val s = kotlin.math.sin(angle)
            return Matrix3(
                c, 0.0, s,
                0.0, 1.0, 0.0,
                -s, 0.0, c
            )
        }

        fun rotationZ(angle: Double): Matrix3 {
            val c = kotlin.math.cos(angle)
            val s = kotlin.math.sin(angle)
            return Matrix3(
                c, -s, 0.0,
                s, c, 0.0,
                0.0, 0.0, 1.0
            )
        }

        fun scaling(x: Double, y: Double, z: Double): Matrix3 {
            return Matrix3(
                x, 0.0, 0.0,
                0.0, y, 0.0,
                0.0, 0.0, z
            )
        }

        fun scaling(s: Double): Matrix3 = scaling(s, s, s)
    }

    fun get(r: Int, c: Int): Double {
        require(r in 0..2 && c in 0..2) { "Row and column must be 0-2"}
        return data[r * 3 + c]
    }

    fun getColumn(c: Int): Vector3 {
        require(c in 0..2) { "Column must be 0-2"}
        return Vector3(data[0 * 3 + c], data[1 * 3 + c], data[2 * 3 + c])
    }

    fun getRow(r: Int): Vector3 {
        require(r in 0..2) { "Row must be 0-2"}
        return Vector3(data[r * 3], data[r * 3 + 1], data[r * 3 + 2])
    }

    operator fun times(other: Matrix3): Matrix3 {
        val result = DoubleArray(9)
        for (r in 0..2) {
            for (c in 0..2) {
                var sum = 0.0
                for (k in 0..2) {
                    sum += data[r * 3 + k] * other.data[k * 3 + c]
                }
                result[r * 3 + c] = sum
            }
        }
        return Matrix3(*result)
    }

    fun multiplyVector(v: Vector3): Vector3 {
        return Vector3(
            data[0] * v.x + data[1] * v.y + data[2] * v.z,
            data[3] * v.x + data[4] * v.y + data[5] * v.z,
            data[6] * v.x + data[7] * v.y + data[8] * v.z
        )
    }

    fun transpose(): Matrix3 {
        return Matrix3(
            data[0], data[3], data[6],
            data[1], data[4], data[7],
            data[2], data[5], data[8]
        )
    }

    val determinant: Double
        get() {
            return data[0] * (data[4] * data[8] - data[5] * data[7])
                   - data[1] * (data[3] * data[8] - data[5] * data[6])
                   + data[2] * (data[3] * data[7] - data[4] * data[6])
        }

    fun inverse(): Matrix3 {
        val det = determinant
        require(!MathUtils.isNearZero(det)) { "Cannot invert a singular matrix (determinant is zero)"}

        val invDet = 1.0 / det
        val m00 = data[0]
        val m01 = data[1]
        val m02 = data[2]
        val m10 = data[3]
        val m11 = data[4]
        val m12 = data[5]
        val m20 = data[6]
        val m21 = data[7]
        val m22 = data[8]

        // Cofactors: C_ij = (-1)^(i+j) * minor_ij
        val c00 = +(m11 * m22 - m12 * m21)
        val c01 = -(m10 * m22 - m12 * m20)
        val c02 = +(m10 * m21 - m11 * m20)
        val c10 = -(m01 * m22 - m02 * m21)
        val c11 = +(m00 * m22 - m02 * m20)
        val c12 = -(m00 * m21 - m01 * m20)
        val c20 = +(m01 * m12 - m02 * m11)
        val c21 = -(m00 * m12 - m02 * m10)
        val c22 = +(m00 * m11 - m01 * m10)

        // Adjugate = transpose of cofactor matrix: A_ij = C_ji
        return Matrix3(
            c00 * invDet, c10 * invDet, c20 * invDet,
            c01 * invDet, c11 * invDet, c21 * invDet,
            c02 * invDet, c12 * invDet, c22 * invDet
        )
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Matrix3) return false
        for (i in 0 until 9) {
            if (!MathUtils.approximatelyEqual(data[i], other.data[i])) return false
        }
        return true
    }

    override fun hashCode(): Int = data.contentHashCode()

    override fun toString(): String {
        val sb = StringBuilder()
        sb.append("Matrix3[\n")
        for (r in 0..2) {
            sb.append("  [")
            for (c in 0..2) {
                sb.append(String.format("%.4f", get(r, c)))
                if (c < 2) sb.append(", ")
            }
            sb.append("]\n")
        }
        sb.append("]")
        return sb.toString()
    }
}
