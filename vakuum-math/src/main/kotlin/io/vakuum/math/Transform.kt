package io.vakuum.math

/**
 * Transform combining position and rotation.
 *
 * Position is a Vector3, rotation is a unit Quaternion.
 */
data class Transform(val position: Vector3, val rotation: Quaternion) {

    init {
        require(rotation.isUnit()) { "Rotation quaternion must be a unit quaternion"}
    }

    companion object {
        val IDENTITY = Transform(Vector3.ZERO, Quaternion.IDENTITY)
    }

    /**
     * Transform a point from local space to world space.
     */
    fun transformPoint(localPoint: Vector3): Vector3 {
        return rotation.rotateVector(localPoint) + position
    }

    /**
     * Transform a direction from local space to world space (no translation).
     */
    fun transformDirection(localDirection: Vector3): Vector3 {
        return rotation.rotateVector(localDirection)
    }

    /**
     * Transform a point from world space to local space.
     */
    fun inverseTransformPoint(worldPoint: Vector3): Vector3 {
        val relative = worldPoint - position
        return rotation.conjugate().rotateVector(relative)
    }

    /**
     * Compose this transform with another.
     * The result transforms by [other] first, then by this.
     */
    operator fun times(other: Transform): Transform {
        return Transform(
            transformPoint(other.position),
            rotation * other.rotation
        )
    }

    /**
     * Compute the inverse transform.
     */
    fun inverse(): Transform {
        val invRotation = rotation.conjugate()
        return Transform(
            -invRotation.rotateVector(position),
            invRotation
        )
    }

    /**
     * Create a Matrix4 from this transform.
     */
    fun toMatrix4(): Matrix4 {
        return Matrix4.fromTranslationRotationScale(position, rotation, Vector3.ONE)
    }

    override fun toString(): String = "Transform(position=$position, rotation=$rotation)"
}
