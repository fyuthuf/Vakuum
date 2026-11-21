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

        // Ensure normal points in the push direction for bodyA (from bodyB toward bodyA).
        // translation = B - A. If bestNormal.dot(translation) < 0, bestNormal points opposite
        // to translation (from B toward A), which is the correct push direction for bodyA.
        // Otherwise, negate it.
        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal

        // Calculate contact point
        val contactPoint = bodyA.transform.position +
                (normal * (projectOntoAxis(translation, normal) - minPenetration / 2.0))

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration,
            bodyAId = bodyA.id,
            bodyBId = bodyB.id
        )
    }

    /**
     * Test collision between two boxes given their transforms directly.
     * This method does not require RigidBody objects, making it suitable for CCD
     * where positions are computed without mutating body state.
     *
     * @param posA Position of box A.
     * @param rotA Rotation of box A.
     * @param posB Position of box B.
     * @param rotB Rotation of box B.
     * @param shapeA Shape of box A.
     * @param shapeB Shape of box B.
     * @return ContactManifold if they overlap, null otherwise.
     */
    fun testBoxBoxAt(
        posA: Vector3,
        rotA: Quaternion,
        posB: Vector3,
        rotB: Quaternion,
        shapeA: BoxShape,
        shapeB: BoxShape
    ): ContactManifold? {
        val axesA = getLocalAxes(rotA)
        val axesB = getLocalAxes(rotB)

        val extentsA = shapeA.halfExtents
        val extentsB = shapeB.halfExtents

        val translation = posB - posA

        val testAxes = mutableListOf<Vector3>()
        testAxes.addAll(axesA)
        testAxes.addAll(axesB)

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
                return null
            }

            if (overlap < minPenetration) {
                minPenetration = overlap
                bestNormal = axis
            }
        }

        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal

        val contactPoint = posA +
                (normal * (projectOntoAxis(translation, normal) - minPenetration / 2.0))

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration,
            bodyAId = -1,
            bodyBId = -1
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
            penetrationDepth = penetration,
            bodyAId = body.id,
            bodyBId = -1 // Plane has no body ID
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

    // ============================================================
    // Sphere collision detection
    // ============================================================

    /**
     * Test collision between two spheres.
     */
    fun testSphereSphere(
        bodyA: RigidBody, bodyB: RigidBody,
        shapeA: SphereShape, shapeB: SphereShape
    ): ContactManifold? {
        val centerA = bodyA.transform.position + bodyA.transform.rotation.rotateVector(shapeA.center)
        val centerB = bodyB.transform.position + bodyB.transform.rotation.rotateVector(shapeB.center)
        return testSphereSphereAt(centerA, centerB, shapeA.radius, shapeB.radius, bodyA.id, bodyB.id)
    }

    /**
     * Test collision between two spheres given their world-space centers directly.
     */
    fun testSphereSphereAt(
        centerA: Vector3, centerB: Vector3,
        radiusA: Double, radiusB: Double,
        bodyAId: Int = -1, bodyBId: Int = -1
    ): ContactManifold? {
        val diff = centerB - centerA
        val dist = diff.length
        val minDist = radiusA + radiusB

        if (dist > minDist || dist < 1e-10) {
            return null
        }

        val normal = diff / dist // Points from A to B; push direction for A is from B toward A
        val contactPoint = centerA + normal * radiusA

        return ContactManifold(
            contactPoint = contactPoint,
            normal = -normal, // Push direction for bodyA: from B toward A
            penetrationDepth = minDist - dist,
            bodyAId = bodyAId,
            bodyBId = bodyBId
        )
    }

    /**
     * Test collision between a sphere and a plane.
     * The plane is defined by normal and distance (n · x = d).
     */
    fun testSpherePlane(
        body: RigidBody, shape: SphereShape,
        planeNormal: Vector3, planeDistance: Double
    ): ContactManifold? {
        val worldCenter = body.transform.position + body.transform.rotation.rotateVector(shape.center)
        return testSpherePlaneAt(worldCenter, shape.radius, planeNormal, planeDistance, body.id)
    }

    /**
     * Test collision between a sphere and a plane given world-space center.
     */
    fun testSpherePlaneAt(
        center: Vector3, radius: Double,
        planeNormal: Vector3, planeDistance: Double,
        bodyAId: Int = -1
    ): ContactManifold? {
        // Distance from sphere center to plane along the normal
        val distToPlane = planeNormal.dot(center) - planeDistance

        // If the sphere is above the plane (positive distance), no collision
        if (distToPlane > radius) return null

        val penetration = radius - distToPlane
        if (penetration <= 0.0) return null

        // Contact point is the projection of the sphere center onto the plane, offset by radius
        val contactPoint = center - planeNormal * (distToPlane + radius)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = planeNormal, // Push direction: along the plane normal
            penetrationDepth = penetration,
            bodyAId = bodyAId,
            bodyBId = -1
        )
    }

    /**
     * Test collision between a sphere and a box using SAT.
     */
    fun testSphereBox(
        body: RigidBody, sphereShape: SphereShape,
        boxBody: RigidBody, boxShape: BoxShape
    ): ContactManifold? {
        val sphereCenter = body.transform.position + body.transform.rotation.rotateVector(sphereShape.center)
        val boxAxes = getLocalAxes(boxBody.transform.rotation)
        val boxExtents = boxShape.halfExtents
        val boxCenter = boxBody.transform.position

        // Find the closest point on the OBB to the sphere center
        val toSphere = sphereCenter - boxCenter
        var closestPoint = boxCenter
        var radius = 0.0

        for (i in 0..2) {
            val proj = toSphere.dot(boxAxes[i])
            val clamped = proj.coerceIn(-boxExtents.getComponent(i), boxExtents.getComponent(i))
            closestPoint += boxAxes[i] * clamped
            radius += kotlin.math.abs(boxExtents.getComponent(i) - clamped)
        }

        // Add sphere radius to the effective radius
        val diff = sphereCenter - closestPoint
        val dist = diff.length

        if (dist > sphereShape.radius || dist < 1e-10) {
            // Check if sphere center is inside the box
            if (dist < 1e-10) {
                // Sphere center is inside the box — find the closest face
                var minPenetration = Double.MAX_VALUE
                var normal = Vector3.UNIT_Y
                for (i in 0..2) {
                    val proj = toSphere.dot(boxAxes[i])
                    val penetration = boxExtents.getComponent(i) - kotlin.math.abs(proj)
                    if (penetration < minPenetration) {
                        minPenetration = penetration
                        normal = if (proj > 0) -boxAxes[i] else boxAxes[i]
                    }
                }
                return ContactManifold(
                    contactPoint = sphereCenter,
                    normal = normal,
                    penetrationDepth = minPenetration + sphereShape.radius,
                    bodyAId = body.id,
                    bodyBId = boxBody.id
                )
            }
            return null
        }

        val normal = (closestPoint - sphereCenter) / dist // Points from sphere to box
        val contactPoint = sphereCenter + normal * sphereShape.radius

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal, // Push direction for sphere: away from box
            penetrationDepth = sphereShape.radius - dist,
            bodyAId = body.id,
            bodyBId = boxBody.id
        )
    }

    // ============================================================
    // Capsule collision detection
    // ============================================================

    /**
     * Transform capsule endpoints to world space.
     */
    private fun getWorldCapsuleEndpoints(body: RigidBody, shape: CapsuleShape): Pair<Vector3, Vector3> {
        val p1 = body.transform.position + body.transform.rotation.rotateVector(shape.p1)
        val p2 = body.transform.position + body.transform.rotation.rotateVector(shape.p2)
        return Pair(p1, p2)
    }

    /**
     * Test collision between a capsule and a plane.
     */
    fun testCapsulePlane(
        body: RigidBody, shape: CapsuleShape,
        planeNormal: Vector3, planeDistance: Double
    ): ContactManifold? {
        val (p1, p2) = getWorldCapsuleEndpoints(body, shape)
        return testCapsulePlaneAt(p1, p2, shape.radius, planeNormal, planeDistance, body.id)
    }

    /**
     * Test collision between a capsule and a plane given world-space endpoints.
     */
    fun testCapsulePlaneAt(
        p1: Vector3, p2: Vector3, radius: Double,
        planeNormal: Vector3, planeDistance: Double,
        bodyAId: Int = -1
    ): ContactManifold? {
        // Find the lowest point of the capsule along the plane normal
        val dist1 = planeNormal.dot(p1) - planeDistance
        val dist2 = planeNormal.dot(p2) - planeDistance

        // The lowest point is the one with the smaller distance
        val (lowestDist, lowestPoint) = if (dist1 < dist2) {
            Pair(dist1, p1)
        } else {
            Pair(dist2, p2)
        }

        // If the lowest point + radius is above the plane, no collision
        if (lowestDist > radius) return null

        val penetration = radius - lowestDist
        if (penetration <= 0.0) return null

        // Contact point is the projection of the lowest endpoint onto the plane
        val contactPoint = lowestPoint - planeNormal * (lowestDist + radius)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = planeNormal,
            penetrationDepth = penetration,
            bodyAId = bodyAId,
            bodyBId = -1
        )
    }

    /**
     * Test collision between a capsule and a sphere.
     */
    fun testCapsuleSphere(
        capsuleBody: RigidBody, capsuleShape: CapsuleShape,
        sphereBody: RigidBody, sphereShape: SphereShape
    ): ContactManifold? {
        val (p1, p2) = getWorldCapsuleEndpoints(capsuleBody, capsuleShape)
        val sphereCenter = sphereBody.transform.position + sphereBody.transform.rotation.rotateVector(sphereShape.center)
        return testCapsuleSphereAt(p1, p2, capsuleShape.radius, sphereCenter, sphereShape.radius,
            capsuleBody.id, sphereBody.id)
    }

    /**
     * Test collision between a capsule and a sphere given world-space data.
     */
    fun testCapsuleSphereAt(
        p1: Vector3, p2: Vector3, capsuleRadius: Double,
        sphereCenter: Vector3, sphereRadius: Double,
        bodyAId: Int = -1, bodyBId: Int = -1
    ): ContactManifold? {
        // Find the closest point on the capsule segment to the sphere center
        val segment = p2 - p1
        val toSphere = sphereCenter - p1
        val t = toSphere.dot(segment) / segment.lengthSquared
        val clampedT = t.coerceIn(0.0, 1.0)
        val closestPoint = p1 + segment * clampedT

        val diff = sphereCenter - closestPoint
        val dist = diff.length
        val minDist = capsuleRadius + sphereRadius

        if (dist > minDist || dist < 1e-10) return null

        val normal = diff / dist // Points from capsule to sphere
        val contactPoint = closestPoint + normal * capsuleRadius

        return ContactManifold(
            contactPoint = contactPoint,
            normal = -normal, // Push direction for capsule: from sphere toward capsule
            penetrationDepth = minDist - dist,
            bodyAId = bodyAId,
            bodyBId = bodyBId
        )
    }

    /**
     * Test collision between a sphere and a box given world-space center directly.
     * Suitable for CCD binary search without mutating body transforms.
     */
    fun testSphereBoxAt(
        sphereCenter: Vector3, radius: Double,
        boxPos: Vector3, boxRot: Quaternion,
        boxShape: BoxShape
    ): ContactManifold? {
        val boxAxes = getLocalAxes(boxRot)
        val boxExtents = boxShape.halfExtents

        // Find the closest point on the OBB to the sphere center
        val toSphere = sphereCenter - boxPos
        var closestPoint = boxPos
        for (i in 0..2) {
            val proj = toSphere.dot(boxAxes[i])
            val clamped = proj.coerceIn(-boxExtents.getComponent(i), boxExtents.getComponent(i))
            closestPoint += boxAxes[i] * clamped
        }

        val diff = sphereCenter - closestPoint
        val dist = diff.length

        if (dist > radius || dist < 1e-10) {
            // Check if sphere center is inside the box
            if (dist < 1e-10) {
                var minPenetration = Double.MAX_VALUE
                var normal = Vector3.UNIT_Y
                for (i in 0..2) {
                    val proj = toSphere.dot(boxAxes[i])
                    val penetration = boxExtents.getComponent(i) - kotlin.math.abs(proj)
                    if (penetration < minPenetration) {
                        minPenetration = penetration
                        normal = if (proj > 0) -boxAxes[i] else boxAxes[i]
                    }
                }
                return ContactManifold(
                    contactPoint = sphereCenter,
                    normal = normal,
                    penetrationDepth = minPenetration + radius,
                    bodyAId = -1,
                    bodyBId = -1
                )
            }
            return null
        }

        val normal = (closestPoint - sphereCenter) / dist
        val contactPoint = sphereCenter + normal * radius

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = radius - dist,
            bodyAId = -1,
            bodyBId = -1
        )
    }

    /**
     * Test collision between a capsule and a box given world-space endpoints directly.
     * Suitable for CCD binary search without mutating body transforms.
     */
    fun testCapsuleBoxAt(
        p1: Vector3, p2: Vector3, radius: Double,
        boxPos: Vector3, boxRot: Quaternion,
        boxShape: BoxShape
    ): ContactManifold? {
        val boxAxes = getLocalAxes(boxRot)
        val boxExtents = boxShape.halfExtents

        fun closestPointOnOBB(point: Vector3): Vector3 {
            val toPoint = point - boxPos
            var closest = boxPos
            for (i in 0..2) {
                val proj = toPoint.dot(boxAxes[i])
                val clamped = proj.coerceIn(-boxExtents.getComponent(i), boxExtents.getComponent(i))
                closest += boxAxes[i] * clamped
            }
            return closest
        }

        val closestP1 = closestPointOnOBB(p1)
        val closestP2 = closestPointOnOBB(p2)

        val distP1 = (p1 - closestP1).length
        val distP2 = (p2 - closestP2).length

        if (distP1 <= radius && distP1 >= 1e-10) {
            val normal = (closestP1 - p1) / distP1
            return ContactManifold(
                contactPoint = p1 + normal * radius,
                normal = normal,
                penetrationDepth = radius - distP1,
                bodyAId = -1,
                bodyBId = -1
            )
        }

        if (distP2 <= radius && distP2 >= 1e-10) {
            val normal = (closestP2 - p2) / distP2
            return ContactManifold(
                contactPoint = p2 + normal * radius,
                normal = normal,
                penetrationDepth = radius - distP2,
                bodyAId = -1,
                bodyBId = -1
            )
        }

        // Sample points along the segment
        val steps = 8
        for (i in 0..steps) {
            val t = i / steps.toDouble()
            val point = p1 + (p2 - p1) * t
            val closest = closestPointOnOBB(point)
            val dist = (point - closest).length
            if (dist <= radius && dist >= 1e-10) {
                val normal = (closest - point) / dist
                return ContactManifold(
                    contactPoint = point + normal * radius,
                    normal = normal,
                    penetrationDepth = radius - dist,
                    bodyAId = -1,
                    bodyBId = -1
                )
            }
        }

        return null
    }

    /**
     * Test collision between a capsule and a box.
     * Uses GJK-like approach: find closest point on box to capsule segment.
     */
    fun testCapsuleBox(
        capsuleBody: RigidBody, capsuleShape: CapsuleShape,
        boxBody: RigidBody, boxShape: BoxShape
    ): ContactManifold? {
        val (p1, p2) = getWorldCapsuleEndpoints(capsuleBody, capsuleShape)
        val boxAxes = getLocalAxes(boxBody.transform.rotation)
        val boxExtents = boxShape.halfExtents
        val boxCenter = boxBody.transform.position

        // Find closest point on the OBB to the capsule segment
        // First, find closest point on OBB to p1 and p2
        fun closestPointOnOBB(point: Vector3): Vector3 {
            val toPoint = point - boxCenter
            var closest = boxCenter
            for (i in 0..2) {
                val proj = toPoint.dot(boxAxes[i])
                val clamped = proj.coerceIn(-boxExtents.getComponent(i), boxExtents.getComponent(i))
                closest += boxAxes[i] * clamped
            }
            return closest
        }

        val closestP1 = closestPointOnOBB(p1)
        val closestP2 = closestPointOnOBB(p2)

        // Check if the segment intersects the box
        val distP1 = (p1 - closestP1).length
        val distP2 = (p2 - closestP2).length

        val effectiveRadius = capsuleShape.radius

        if (distP1 <= effectiveRadius && distP1 >= 1e-10) {
            val normal = (closestP1 - p1) / distP1
            return ContactManifold(
                contactPoint = p1 + normal * effectiveRadius,
                normal = normal,
                penetrationDepth = effectiveRadius - distP1,
                bodyAId = capsuleBody.id,
                bodyBId = boxBody.id
            )
        }

        if (distP2 <= effectiveRadius && distP2 >= 1e-10) {
            val normal = (closestP2 - p2) / distP2
            return ContactManifold(
                contactPoint = p2 + normal * effectiveRadius,
                normal = normal,
                penetrationDepth = effectiveRadius - distP2,
                bodyAId = capsuleBody.id,
                bodyBId = boxBody.id
            )
        }

        // Check if the capsule segment passes through the box
        // by sampling points along the segment
        val steps = 8
        for (i in 0..steps) {
            val t = i / steps.toDouble()
            val point = p1 + (p2 - p1) * t
            val closest = closestPointOnOBB(point)
            val dist = (point - closest).length
            if (dist <= effectiveRadius && dist >= 1e-10) {
                val normal = (closest - point) / dist
                return ContactManifold(
                    contactPoint = point + normal * effectiveRadius,
                    normal = normal,
                    penetrationDepth = effectiveRadius - dist,
                    bodyAId = capsuleBody.id,
                    bodyBId = boxBody.id
                )
            }
        }

        return null
    }

    // ============================================================
    // Convex hull collision detection
    // ============================================================

    /**
     * Transform convex hull vertices to world space.
     */
    private fun getWorldVertices(body: RigidBody, shape: ConvexHullShape): List<Vector3> {
        return shape.vertices.map { body.transform.position + body.transform.rotation.rotateVector(it) }
    }

    /**
     * Test collision between a convex hull and a plane using GJK-like approach.
     */
    fun testConvexPlane(
        body: RigidBody, shape: ConvexHullShape,
        planeNormal: Vector3, planeDistance: Double
    ): ContactManifold? {
        val vertices = getWorldVertices(body, shape)

        // Find the vertex with the maximum penetration into the plane
        var maxPenetration = 0.0
        var contactVertex = Vector3.ZERO

        for (v in vertices) {
            val dist = planeNormal.dot(v) - planeDistance
            // If dist < 0, the vertex is below the plane (penetrating)
            val penetration = -dist
            if (penetration > maxPenetration) {
                maxPenetration = penetration
                contactVertex = v
            }
        }

        if (maxPenetration <= 0.0) return null

        // Project the contact vertex onto the plane
        val contactPoint = contactVertex - planeNormal * (-maxPenetration)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = planeNormal,
            penetrationDepth = maxPenetration,
            bodyAId = body.id,
            bodyBId = -1
        )
    }

    /**
     * Test collision between a convex hull and a box using SAT.
     */
    fun testConvexBox(
        convexBody: RigidBody, convexShape: ConvexHullShape,
        boxBody: RigidBody, boxShape: BoxShape
    ): ContactManifold? {
        val convexVerts = getWorldVertices(convexBody, convexShape)
        val boxAxes = getLocalAxes(boxBody.transform.rotation)
        val boxExtents = boxShape.halfExtents
        val boxCenter = boxBody.transform.position

        // Collect all separating axes:
        // 1. Box face normals (3)
        // 2. Convex face normals (from vertex triplets)
        // 3. Cross products of box edges and convex edges

        val testAxes = mutableListOf<Vector3>()

        // Box face normals
        testAxes.addAll(boxAxes)

        // Convex face normals (from all unique vertex triplets)
        val convexNormals = mutableSetOf<Vector3>()
        for (i in 0 until convexVerts.size) {
            for (j in i + 1 until convexVerts.size) {
                for (k in j + 1 until convexVerts.size) {
                    val n = (convexVerts[j] - convexVerts[i]).cross(convexVerts[k] - convexVerts[i])
                    if (!n.isNearZero()) {
                        val normalized = n.normalize()
                        // Avoid duplicate normals
                        if (convexNormals.none { kotlin.math.abs(it.dot(normalized)) > 0.9999 }) {
                            convexNormals.add(normalized)
                            testAxes.add(normalized)
                        }
                    }
                }
            }
        }

        // Cross products of box edges and convex edges
        val convexEdges = mutableListOf<Vector3>()
        for (i in 0 until convexVerts.size) {
            for (j in i + 1 until convexVerts.size) {
                val edge = convexVerts[j] - convexVerts[i]
                if (!edge.isNearZero()) {
                    convexEdges.add(edge.normalize())
                }
            }
        }

        for (boxAxis in boxAxes) {
            for (convexEdge in convexEdges) {
                val cross = boxAxis.cross(convexEdge)
                if (!cross.isNearZero()) {
                    testAxes.add(cross.normalize())
                }
            }
        }

        // Test each axis
        var minPenetration = Double.MAX_VALUE
        var bestNormal = Vector3.ZERO

        for (axis in testAxes) {
            val (minA, maxA) = projectConvex(axis, convexVerts)
            val (minB, maxB) = projectOBBOnAxis(axis, boxAxes, boxExtents, boxCenter)

            val overlap = kotlin.math.min(maxA, maxB) - kotlin.math.max(minA, minB)
            if (overlap <= 0.0) return null // Separating axis found

            if (overlap < minPenetration) {
                minPenetration = overlap
                bestNormal = axis
            }
        }

        if (minPenetration == Double.MAX_VALUE) return null

        // Orient the normal
        val translation = boxCenter - convexBody.transform.position
        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal

        val contactPoint = convexBody.transform.position + normal * (minPenetration / 2.0)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration,
            bodyAId = convexBody.id,
            bodyBId = boxBody.id
        )
    }

    /**
     * Test collision between a convex hull and a box given transforms directly.
     * This method does not require RigidBody objects, making it suitable for CCD.
     */
    fun testConvexBoxAt(
        convexPos: Vector3,
        convexRot: Quaternion,
        boxPos: Vector3,
        boxRot: Quaternion,
        convexShape: ConvexHullShape,
        boxShape: BoxShape
    ): ContactManifold? {
        val convexVerts = convexShape.vertices.map { convexPos + convexRot.rotateVector(it) }
        val boxAxes = getLocalAxes(boxRot)
        val boxExtents = boxShape.halfExtents

        val testAxes = mutableListOf<Vector3>()
        testAxes.addAll(boxAxes)

        val convexNormals = mutableSetOf<Vector3>()
        for (i in 0 until convexVerts.size) {
            for (j in i + 1 until convexVerts.size) {
                for (k in j + 1 until convexVerts.size) {
                    val n = (convexVerts[j] - convexVerts[i]).cross(convexVerts[k] - convexVerts[i])
                    if (!n.isNearZero()) {
                        val normalized = n.normalize()
                        if (convexNormals.none { kotlin.math.abs(it.dot(normalized)) > 0.9999 }) {
                            convexNormals.add(normalized)
                            testAxes.add(normalized)
                        }
                    }
                }
            }
        }

        val convexEdges = mutableListOf<Vector3>()
        for (i in 0 until convexVerts.size) {
            for (j in i + 1 until convexVerts.size) {
                val edge = convexVerts[j] - convexVerts[i]
                if (!edge.isNearZero()) convexEdges.add(edge.normalize())
            }
        }

        for (boxAxis in boxAxes) {
            for (convexEdge in convexEdges) {
                val cross = boxAxis.cross(convexEdge)
                if (!cross.isNearZero()) testAxes.add(cross.normalize())
            }
        }

        var minPenetration = Double.MAX_VALUE
        var bestNormal = Vector3.ZERO

        for (axis in testAxes) {
            val (minA, maxA) = projectConvex(axis, convexVerts)
            val (minB, maxB) = projectOBBOnAxis(axis, boxAxes, boxExtents, boxPos)
            val overlap = kotlin.math.min(maxA, maxB) - kotlin.math.max(minA, minB)
            if (overlap <= 0.0) return null
            if (overlap < minPenetration) {
                minPenetration = overlap
                bestNormal = axis
            }
        }

        if (minPenetration == Double.MAX_VALUE) return null

        val translation = boxPos - convexPos
        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal
        val contactPoint = convexPos + normal * (minPenetration / 2.0)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration,
            bodyAId = -1,
            bodyBId = -1
        )
    }

    /**
     * Test collision between two convex hulls using SAT.
     */
    fun testConvexConvex(
        bodyA: RigidBody, shapeA: ConvexHullShape,
        bodyB: RigidBody, shapeB: ConvexHullShape
    ): ContactManifold? {
        val vertsA = getWorldVertices(bodyA, shapeA)
        val vertsB = getWorldVertices(bodyB, shapeB)

        // Collect all separating axes
        val testAxes = mutableListOf<Vector3>()

        // Face normals from both hulls
        fun collectFaceNormals(verts: List<Vector3>): List<Vector3> {
            val normals = mutableListOf<Vector3>()
            val seen = mutableSetOf<Vector3>()
            for (i in 0 until verts.size) {
                for (j in i + 1 until verts.size) {
                    for (k in j + 1 until verts.size) {
                        val n = (verts[j] - verts[i]).cross(verts[k] - verts[i])
                        if (!n.isNearZero()) {
                            val normalized = n.normalize()
                            if (seen.none { kotlin.math.abs(it.dot(normalized)) > 0.9999 }) {
                                seen.add(normalized)
                                normals.add(normalized)
                            }
                        }
                    }
                }
            }
            return normals
        }

        testAxes.addAll(collectFaceNormals(vertsA))
        testAxes.addAll(collectFaceNormals(vertsB))

        // Cross products of edges
        val edgesA = mutableListOf<Vector3>()
        val edgesB = mutableListOf<Vector3>()
        for (i in 0 until vertsA.size) {
            for (j in i + 1 until vertsA.size) {
                val e = vertsA[j] - vertsA[i]
                if (!e.isNearZero()) edgesA.add(e.normalize())
            }
        }
        for (i in 0 until vertsB.size) {
            for (j in i + 1 until vertsB.size) {
                val e = vertsB[j] - vertsB[i]
                if (!e.isNearZero()) edgesB.add(e.normalize())
            }
        }

        for (ea in edgesA) {
            for (eb in edgesB) {
                val cross = ea.cross(eb)
                if (!cross.isNearZero()) {
                    testAxes.add(cross.normalize())
                }
            }
        }

        // Test each axis
        var minPenetration = Double.MAX_VALUE
        var bestNormal = Vector3.ZERO

        for (axis in testAxes) {
            val (minA, maxA) = projectConvex(axis, vertsA)
            val (minB, maxB) = projectConvex(axis, vertsB)

            val overlap = kotlin.math.min(maxA, maxB) - kotlin.math.max(minA, minB)
            if (overlap <= 0.0) return null

            if (overlap < minPenetration) {
                minPenetration = overlap
                bestNormal = axis
            }
        }

        if (minPenetration == Double.MAX_VALUE) return null

        val translation = bodyB.transform.position - bodyA.transform.position
        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal

        val contactPoint = bodyA.transform.position + normal * (minPenetration / 2.0)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration,
            bodyAId = bodyA.id,
            bodyBId = bodyB.id
        )
    }

    /**
     * Test collision between two convex hulls given transforms directly.
     * Suitable for CCD binary search without mutating body transforms.
     */
    fun testConvexConvexAt(
        posA: Vector3, rotA: Quaternion,
        posB: Vector3, rotB: Quaternion,
        shapeA: ConvexHullShape, shapeB: ConvexHullShape
    ): ContactManifold? {
        val vertsA = shapeA.vertices.map { posA + rotA.rotateVector(it) }
        val vertsB = shapeB.vertices.map { posB + rotB.rotateVector(it) }

        val testAxes = mutableListOf<Vector3>()

        fun collectFaceNormals(verts: List<Vector3>): List<Vector3> {
            val normals = mutableListOf<Vector3>()
            val seen = mutableSetOf<Vector3>()
            for (i in 0 until verts.size) {
                for (j in i + 1 until verts.size) {
                    for (k in j + 1 until verts.size) {
                        val n = (verts[j] - verts[i]).cross(verts[k] - verts[i])
                        if (!n.isNearZero()) {
                            val normalized = n.normalize()
                            if (seen.none { kotlin.math.abs(it.dot(normalized)) > 0.9999 }) {
                                seen.add(normalized)
                                normals.add(normalized)
                            }
                        }
                    }
                }
            }
            return normals
        }

        testAxes.addAll(collectFaceNormals(vertsA))
        testAxes.addAll(collectFaceNormals(vertsB))

        val edgesA = mutableListOf<Vector3>()
        val edgesB = mutableListOf<Vector3>()
        for (i in 0 until vertsA.size) {
            for (j in i + 1 until vertsA.size) {
                val e = vertsA[j] - vertsA[i]
                if (!e.isNearZero()) edgesA.add(e.normalize())
            }
        }
        for (i in 0 until vertsB.size) {
            for (j in i + 1 until vertsB.size) {
                val e = vertsB[j] - vertsB[i]
                if (!e.isNearZero()) edgesB.add(e.normalize())
            }
        }

        for (ea in edgesA) {
            for (eb in edgesB) {
                val cross = ea.cross(eb)
                if (!cross.isNearZero()) {
                    testAxes.add(cross.normalize())
                }
            }
        }

        var minPenetration = Double.MAX_VALUE
        var bestNormal = Vector3.ZERO

        for (axis in testAxes) {
            val (minA, maxA) = projectConvex(axis, vertsA)
            val (minB, maxB) = projectConvex(axis, vertsB)
            val overlap = kotlin.math.min(maxA, maxB) - kotlin.math.max(minA, minB)
            if (overlap <= 0.0) return null
            if (overlap < minPenetration) {
                minPenetration = overlap
                bestNormal = axis
            }
        }

        if (minPenetration == Double.MAX_VALUE) return null

        val translation = posB - posA
        val direction = bestNormal.dot(translation)
        val normal = if (direction < 0) bestNormal else -bestNormal
        val contactPoint = posA + normal * (minPenetration / 2.0)

        return ContactManifold(
            contactPoint = contactPoint,
            normal = normal,
            penetrationDepth = minPenetration,
            bodyAId = -1,
            bodyBId = -1
        )
    }

    private fun projectConvex(axis: Vector3, vertices: List<Vector3>): Pair<Double, Double> {
        var min = Double.MAX_VALUE
        var max = -Double.MAX_VALUE
        for (v in vertices) {
            val proj = axis.dot(v)
            if (proj < min) min = proj
            if (proj > max) max = proj
        }
        return Pair(min, max)
    }

    private fun projectOBBOnAxis(
        axis: Vector3,
        axes: List<Vector3>,
        extents: Vector3,
        center: Vector3
    ): Pair<Double, Double> {
        val radius = projectOBBRadius(axis, axes, extents)
        val proj = axis.dot(center)
        return Pair(proj - radius, proj + radius)
    }

    private fun projectOBBRadius(
        axis: Vector3,
        axes: List<Vector3>,
        extents: Vector3
    ): Double {
        var radius = 0.0
        val e = listOf(extents.x, extents.y, extents.z)
        for (i in 0..2) {
            radius += kotlin.math.abs(axis.dot(axes[i])) * e[i]
        }
        return radius
    }

    private fun Vector3.getComponent(index: Int): Double =
        when (index) { 0 -> x; 1 -> y; 2 -> z; else -> 0.0 }
}
