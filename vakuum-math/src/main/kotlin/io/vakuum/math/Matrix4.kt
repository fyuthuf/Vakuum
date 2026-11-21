package io.vakuum.math

/**
 * 4x4 matrix for affine transformations (translation, rotation, scale).
 *
 * Stored in row-major order as a flat array of 16 doubles.
 */
class Matrix4 {
    private val data: DoubleArray

    constructor(vararg elements: Double) {
        require(elements.size == 16) { "Matrix4 requires exactly 16 elements"}
        data = elements.copyOf()
        for (i in 0 until 16) {
            MathUtils.validateFinite(data[i], "matrix element [$i]")
        }
    }

    companion object {
        val IDENTITY = Matrix4(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        )

        val ZERO = Matrix4(
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        )

        fun fromRowMajor(
            m00: Double, m01: Double, m02: Double, m03: Double,
            m10: Double, m11: Double, m12: Double, m13: Double,
            m20: Double, m21: Double, m22: Double, m23: Double,
            m30: Double, m31: Double, m32: Double, m33: Double
        ): Matrix4 = Matrix4(
            m00, m01, m02, m03,
            m10, m11, m12, m13,
            m20, m21, m22, m23,
            m30, m31, m32, m33
        )

        fun fromTranslationRotationScale(
            translation: Vector3,
            rotation: Quaternion,
            scale: Vector3
        ): Matrix4 {
            val rotMat = Matrix3.fromQuaternion(rotation)
            val s = Matrix3.scaling(scale.x, scale.y, scale.z)
            val rs = rotMat * s

            return Matrix4(
                rs.get(0, 0), rs.get(0, 1), rs.get(0, 2), translation.x,
                rs.get(1, 0), rs.get(1, 1), rs.get(1, 2), translation.y,
                rs.get(2, 0), rs.get(2, 1), rs.get(2, 2), translation.z,
                0.0, 0.0, 0.0, 1.0
            )
        }

        fun translation(x: Double, y: Double, z: Double): Matrix4 = Matrix4(
            1.0, 0.0, 0.0, x,
            0.0, 1.0, 0.0, y,
            0.0, 0.0, 1.0, z,
            0.0, 0.0, 0.0, 1.0
        )

        fun translation(v: Vector3): Matrix4 = translation(v.x, v.y, v.z)

        fun rotation(q: Quaternion): Matrix4 {
            val m = Matrix3.fromQuaternion(q)
            return Matrix4(
                m.get(0, 0), m.get(0, 1), m.get(0, 2), 0.0,
                m.get(1, 0), m.get(1, 1), m.get(1, 2), 0.0,
                m.get(2, 0), m.get(2, 1), m.get(2, 2), 0.0,
                0.0, 0.0, 0.0, 1.0
            )
        }

        fun scaling(x: Double, y: Double, z: Double): Matrix4 = Matrix4(
            x, 0.0, 0.0, 0.0,
            0.0, y, 0.0, 0.0,
            0.0, 0.0, z, 0.0,
            0.0, 0.0, 0.0, 1.0
        )

        fun scaling(s: Double): Matrix4 = scaling(s, s, s)
    }

    fun get(r: Int, c: Int): Double {
        require(r in 0..3 && c in 0..3) { "Row and column must be 0-3"}
        return data[r * 4 + c]
    }

    operator fun times(other: Matrix4): Matrix4 {
        val result = DoubleArray(16)
        for (r in 0..3) {
            for (c in 0..3) {
                var sum = 0.0
                for (k in 0..3) {
                    sum += data[r * 4 + k] * other.data[k * 4 + c]
                }
                result[r * 4 + c] = sum
            }
        }
        return Matrix4(*result)
    }

    fun multiplyPoint(v: Vector3): Vector3 {
        return Vector3(
            data[0] * v.x + data[1] * v.y + data[2] * v.z + data[3],
            data[4] * v.x + data[5] * v.y + data[6] * v.z + data[7],
            data[8] * v.x + data[9] * v.y + data[10] * v.z + data[11]
        )
    }

    fun multiplyDirection(v: Vector3): Vector3 {
        return Vector3(
            data[0] * v.x + data[1] * v.y + data[2] * v.z,
            data[4] * v.x + data[5] * v.y + data[6] * v.z,
            data[8] * v.x + data[9] * v.y + data[10] * v.z
        )
    }

    fun transpose(): Matrix4 {
        return Matrix4(
            data[0], data[4], data[8], data[12],
            data[1], data[5], data[9], data[13],
            data[2], data[6], data[10], data[14],
            data[3], data[7], data[11], data[15]
        )
    }

    val determinant: Double
        get() {
            val a = data[0]
            val b = data[4]
            val c = data[8]
            val d = data[12]
            val e = data[1]
            val f = data[5]
            val g = data[9]
            val h = data[13]
            val ii = data[2]
            val j = data[6]
            val k = data[10]
            val l = data[14]
            val mm = data[3]
            val n = data[7]
            val o = data[11]
            val p = data[15]

            return a * (f * (k * p - l * o) - g * (j * p - l * n) + h * (j * o - k * n))
                   - b * (e * (k * p - l * o) - g * (ii * p - l * mm) + h * (ii * o - k * mm))
                   + c * (e * (j * p - l * n) - f * (ii * p - l * mm) + h * (ii * n - j * mm))
                   - d * (e * (j * o - k * n) - f * (ii * o - k * mm) + g * (ii * n - j * mm))
        }

    fun inverse(): Matrix4 {
        val det = determinant
        require(!MathUtils.isNearZero(det)) { "Cannot invert a singular matrix (determinant is zero)"}

        val m00 = data[0]
        val m01 = data[1]
        val m02 = data[2]
        val m03 = data[3]
        val m10 = data[4]
        val m11 = data[5]
        val m12 = data[6]
        val m13 = data[7]
        val m20 = data[8]
        val m21 = data[9]
        val m22 = data[10]
        val m23 = data[11]
        val m30 = data[12]
        val m31 = data[13]
        val m32 = data[14]
        val m33 = data[15]

        val a00 = m11*m22*m33 - m11*m23*m32 - m12*m21*m33 + m12*m23*m31 + m13*m21*m32 - m13*m22*m31
        val a01 = -m01*m22*m33 + m01*m23*m32 + m02*m21*m33 - m02*m23*m31 - m03*m21*m32 + m03*m22*m31
        val a02 = m01*m12*m33 - m01*m13*m32 - m02*m11*m33 + m02*m13*m31 + m03*m11*m32 - m03*m12*m31
        val a03 = -m01*m12*m23 + m01*m13*m22 + m02*m11*m23 - m02*m13*m21 - m03*m11*m22 + m03*m12*m21
        val a10 = -m10*m22*m33 + m10*m23*m32 + m12*m20*m33 - m12*m23*m30 - m13*m20*m32 + m13*m22*m30
        val a11 = m00*m22*m33 - m00*m23*m32 - m02*m20*m33 + m02*m23*m30 + m03*m20*m32 - m03*m22*m30
        val a12 = -m00*m12*m33 + m00*m13*m32 + m02*m10*m33 - m02*m13*m30 - m03*m10*m32 + m03*m12*m30
        val a13 = m00*m12*m23 - m00*m13*m22 - m02*m10*m23 + m02*m13*m20 + m03*m10*m22 - m03*m12*m20
        val a20 = m10*m21*m33 - m10*m23*m31 - m11*m20*m33 + m11*m23*m30 + m13*m20*m31 - m13*m21*m30
        val a21 = -m00*m21*m33 + m00*m23*m31 + m01*m20*m33 - m01*m23*m30 - m03*m20*m31 + m03*m21*m30
        val a22 = m00*m11*m33 - m00*m13*m31 - m01*m10*m33 + m01*m13*m30 + m03*m10*m31 - m03*m11*m30
        val a23 = -m00*m11*m23 + m00*m13*m21 + m01*m10*m23 - m01*m13*m20 - m03*m10*m21 + m03*m11*m20
        val a30 = -m10*m21*m32 + m10*m22*m31 + m11*m20*m32 - m11*m22*m30 - m12*m20*m31 + m12*m21*m30
        val a31 = m00*m21*m32 - m00*m22*m31 - m01*m20*m32 + m01*m22*m30 + m02*m20*m31 - m02*m21*m30
        val a32 = -m00*m11*m32 + m00*m12*m31 + m01*m10*m32 - m01*m12*m30 - m02*m10*m31 + m02*m11*m30
        val a33 = m00*m11*m22 - m00*m12*m21 - m01*m10*m22 + m01*m12*m20 + m02*m10*m21 - m02*m11*m20

        val invDet = 1.0 / det

        return Matrix4(
            a00 * invDet, a01 * invDet, a02 * invDet, a03 * invDet,
            a10 * invDet, a11 * invDet, a12 * invDet, a13 * invDet,
            a20 * invDet, a21 * invDet, a22 * invDet, a23 * invDet,
            a30 * invDet, a31 * invDet, a32 * invDet, a33 * invDet
        )
    }

    fun getTranslation(): Vector3 = Vector3(data[3], data[7], data[11])

    fun getRotationMatrix(): Matrix3 {
        return Matrix3(
            data[0], data[1], data[2],
            data[4], data[5], data[6],
            data[8], data[9], data[10]
        )
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Matrix4) return false
        for (i in 0 until 16) {
            if (!MathUtils.approximatelyEqual(data[i], other.data[i])) return false
        }
        return true
    }

    override fun hashCode(): Int = data.contentHashCode()

    override fun toString(): String {
        val sb = StringBuilder()
        sb.append("Matrix4[\n")
        for (r in 0..3) {
            sb.append("  [")
            for (c in 0..3) {
                sb.append(String.format("%.4f", get(r, c)))
                if (c < 3) sb.append(", ")
            }
            sb.append("]\n")
        }
        sb.append("]")
        return sb.toString()
    }
}
