package io.vakuum.collision

import io.vakuum.dynamics.RigidBody
import io.vakuum.math.Quaternion
import io.vakuum.math.Vector3

/**
 * Narrowphase collision detection using SAT (Separating Axis Theorem).
 * Supports box-box and box-plane collision tests.
 */
class Narrowphase {

    /**
     * Test collision between two bodies with box shapes.
     * Returns a ContactManifold if they overlap, null otherwise.
     */
    fun testBoxBox(bodyA: RigidBody, bodyB: RigidBody, shapeA: BoxShape, shapeB: BoxShape): ContactManifold? {
        // Get world-space oriented bounding boxes
        val axesA = getLocalAxes(bodyA.transform.rotation)
        val axesB = getLocalAxes(bodyB.transform.rotation)

        val extentsA = shapeA.halfExtents
        val extentsB = shapeB.halfExtents

        val translation = bodyB.transform.position - bodyA.transform.position

        // Test separating axes: 3 face normals from A, 3 from B, and 9 cross products
        val testAxes = mutableListOf<Vector3>()
        testAxes.addAll(axesA)
        testAxes.addAll(axesB)

        // Cross products of axes
        for (a in axesA) {
            for (b in axesB) {
                val cross = a.cross(b)
                if (!cross.isNearZero()) {
                    testAxes.add(cross.normalize())
                }
            }
        }

        var minPenetration = Double.MAX_VALUE
        var bestNormal = Vector3.ZERO

        for (axis in testAxes) {
            val (radiusA, radiusB) = projectOBB(axis, axesA, extentsA, axesB, extentsB)
            val projection = axis.dot(translation)
            val overlap = radiusA + radiusB - kotlin.math.abs(projection)

            if (overlap <= 0.0) {
                // Separating axis found — no collision
                return null
            }

            if (overlap < minPenetration) {
                minPenetration = overlap
                bestNormal = axis
            }
        }

        // Ensure normal points from A to B
        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal

        // Calculate contact point
        val contactPoint = bodyA.transform.position +
                (normal * (projectOntoAxis(translation, normal) - minPenetration / 2.0))

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration
        )
    }

    /**
     * Test collision between a box and a plane.
     * Returns a ContactManifold if the box penetrates the plane.
     */
    fun testBoxPlane(body: RigidBody, shape: BoxShape, planeNormal: Vector3, planeDistance: Double): ContactManifold? {
        val center = body.transform.position
        // getLocalAxes now returns world-space axes directly
        val worldAxes = getLocalAxes(body.transform.rotation)
        val extents = shape.halfExtents

        // Calculate radius of the box projected onto the plane normal
        val halfExtents = listOf(extents.x, extents.y, extents.z)
        var radius = 0.0
        for (i in worldAxes.indices) {
            radius += kotlin.math.abs(worldAxes[i].dot(planeNormal)) * halfExtents[i]
        }

        // Distance from box center to plane along the normal
        // planeNormal · center gives how far the center is along the normal
        // planeDistance is where the plane is along the normal
        // For ground plane: normal=(0,1,0), distance=0, so distToPlane = center.y
        val distToPlane = planeNormal.dot(center)

        // The box extends `radius` units toward the plane (in the -normal direction)
        // So the closest point of the box to the plane is at distToPlane - radius
        // Penetration occurs when this closest point is below the plane
        // i.e., distToPlane - radius < planeDistance
        // penetration = planeDistance - (distToPlane - radius) = radius - (distToPlane - planeDistance)
        val penetration = radius - (distToPlane - planeDistance)

        if (penetration <= 0.0) {
            // No penetration — box is above the plane
            return null
        }

        // Find the contact point (lowest point on box toward the plane)
        var contactPoint = center
        for (i in worldAxes.indices) {
            val proj = worldAxes[i].dot(planeNormal)
            if (proj > 0) {
                // This axis points in the same direction as the normal — go opposite
                contactPoint -= worldAxes[i] * halfExtents[i]
            } else {
                // This axis points opposite to the normal — go along it
                contactPoint += worldAxes[i] * halfExtents[i]
            }
        }

        // Project contact point onto plane
        contactPoint = contactPoint - planeNormal * (planeNormal.dot(contactPoint) - planeDistance)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = planeNormal,
            penetrationDepth = penetration
        )
    }

    private fun getLocalAxes(rotation: Quaternion): List<Vector3> {
        // Transform local axes to world space using the body's rotation
        return listOf(
            rotation.rotateVector(Vector3.UNIT_X),
            rotation.rotateVector(Vector3.UNIT_Y),
            rotation.rotateVector(Vector3.UNIT_Z)
        )
    }

    private fun projectOBB(
        axis: Vector3,
        axesA: List<Vector3>,
        extentsA: Vector3,
        axesB: List<Vector3>,
        extentsB: Vector3
    ): Pair<Double, Double> {
        var radiusA = 0.0
        var radiusB = 0.0

        val eA = listOf(extentsA.x, extentsA.y, extentsA.z)
        val eB = listOf(extentsB.x, extentsB.y, extentsB.z)

        for (i in 0..2) {
            radiusA += kotlin.math.abs(axis.dot(axesA[i])) * eA[i]
            radiusB += kotlin.math.abs(axis.dot(axesB[i])) * eB[i]
        }

        return Pair(radiusA, radiusB)
    }

    private fun projectOntoAxis(vector: Vector3, axis: Vector3): Double {
        return vector.dot(axis)
    }
}
