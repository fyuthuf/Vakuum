package io.vakuum.collision

import io.vakuum.dynamics.RigidBody
import io.vakuum.math.AABB
import io.vakuum.math.Vector3

/**
 * Continuous Collision Detection (CCD) subsystem.
 *
 * Provides swept collision detection and Time of Impact (TOI) calculation
 * to prevent fast-moving rigid bodies from tunneling through collision geometry
 * between discrete simulation steps.
 *
 * Uses a sweep-based approach with conservative advancement:
 * 1. Compute the swept AABB (volume traced by a body during its motion).
 * 2. Test the swept volume against potential collision geometry.
 * 3. If overlap is detected, calculate the precise TOI using binary search.
 * 4. Advance the body to TOI, resolve the collision, then continue.
 *
 * Currently supports:
 * - Box vs static plane (ground)
 * - Box vs static box
 *
 * Does NOT support:
 * - Dynamic vs dynamic CCD (both bodies moving)
 * - Rotating body CCD
 * - Sphere, Capsule, ConvexHull shapes
 */
object Ccd {

    /**
     * Result of a Time of Impact calculation.
     */
    data class ToiResult(
        /**
         * Fraction of the timestep at which impact occurs (0.0 to 1.0).
         * 0.0 = impact at start of timestep, 1.0 = impact at end.
         */
        val toi: Double,
        /** Contact normal at the point of impact. */
        val normal: Vector3,
        /** Contact point at the point of impact. */
        val contactPoint: Vector3,
        /** Body that is moving (the CCD body). */
        val movingBodyId: Int,
        /** Body or obstacle that was hit. -1 for static plane. */
        val hitBodyId: Int
    )

    /**
     * Calculate the swept AABB for a body moving from its current position
     * with the given linear velocity over the given timestep.
     *
     * The swept AABB is the axis-aligned bounding box that covers the entire
     * volume traced by the body's motion during the timestep.
     */
    fun sweptAABB(body: RigidBody, shape: BoxShape, dt: Double): AABB {
        val center = body.transform.position
        val extents = shape.halfExtents
        val velocity = body.linearVelocity

        // Start and end positions of the body center
        val startCenter = center
        val endCenter = center + velocity * dt

        // AABB of the box at start position
        val startAABB = AABB.fromCenterExtents(startCenter, extents)

        // AABB of the box at end position
        val endAABB = AABB.fromCenterExtents(endCenter, extents)

        // Union of start and end AABBs gives the swept volume
        return startAABB.union(endAABB)
    }

    /**
     * Check if a body should use CCD based on its velocity.
     *
     * @param body The body to check.
     * @param velocityThreshold Minimum velocity magnitude for CCD activation.
     * @return true if the body is moving fast enough to potentially tunnel.
     */
    fun shouldUseCcd(body: RigidBody, velocityThreshold: Double): Boolean {
        return body.linearVelocity.lengthSquared > velocityThreshold * velocityThreshold
    }

    /**
     * Calculate the Time of Impact (TOI) between a moving box and a static ground plane.
     *
     * The ground plane is defined by normal=(0,1,0) and distance=0 (y=0 plane).
     *
     * IMPORTANT: If the body is already penetrating the plane, this method returns null.
     * The caller (SimulationPipeline) should handle existing penetration via discrete
     * collision detection, not CCD. Returning TOI=0 would cause infinite substep loops.
     *
     * @param body The moving body (must have a BoxShape).
     * @param shape The box shape of the moving body.
     * @param planeNormal The ground plane normal (typically Vector3.UNIT_Y).
     * @param planeDistance The ground plane distance from origin (typically 0.0).
     * @param dt The timestep duration.
     * @return ToiResult if the body will hit the plane within the timestep, null otherwise.
     *         Returns null if already penetrating — use discrete collision instead.
     */
    fun toiBoxPlane(
        body: RigidBody,
        shape: BoxShape,
        planeNormal: Vector3,
        planeDistance: Double,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val center = body.transform.position
        val velocity = body.linearVelocity

        // If the body is not moving toward the plane, no TOI
        val velAlongNormal = velocity.dot(planeNormal)
        if (velAlongNormal >= 0.0) return null

        // Calculate the lowest point of the box along the plane normal
        // For a box, the closest point to the plane is center - radius_along_normal
        val worldAxes = getWorldAxes(body.transform.rotation)
        val halfExtents = listOf(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z)
        var radius = 0.0
        for (i in worldAxes.indices) {
            radius += kotlin.math.abs(worldAxes[i].dot(planeNormal)) * halfExtents[i]
        }

        // Distance from the lowest point of the box to the plane
        val distToPlane = planeNormal.dot(center)
        val lowestPointDist = distToPlane - radius

        // If already penetrating, return null — let discrete collision handle it.
        // Returning TOI=0 would cause infinite substep loops in SimulationPipeline.
        if (lowestPointDist <= planeDistance) {
            return null
        }

        // Time to impact: distance / speed
        val distanceToTravel = lowestPointDist - planeDistance
        val toi = distanceToTravel / (-velAlongNormal)

        // Validate TOI is within the timestep and not too close to 0
        if (toi < 0.0 || toi > dt) return null
        val toiFraction = toi / dt
        if (toiFraction < 1e-8) return null // Avoid near-zero TOI that causes substep loops

        return ToiResult(
            toi = toiFraction,
            normal = planeNormal,
            contactPoint = contactPointAtToi(body, shape, planeNormal, planeDistance, toi),
            movingBodyId = body.id,
            hitBodyId = -1
        )
    }

    /**
     * Calculate the Time of Impact (TOI) between a moving box and a static box.
     *
     * Uses swept AABB overlap test followed by binary search for precise TOI.
     *
     * IMPORTANT: This method does NOT mutate the movingBody's transform.
     * It uses a local position variable for the binary search to avoid side effects.
     *
     * If the bodies are already colliding, returns null — let discrete collision handle it.
     *
     * @param movingBody The moving body.
     * @param movingShape The box shape of the moving body.
     * @param staticBody The static body.
     * @param staticShape The box shape of the static body.
     * @param dt The timestep duration.
     * @return ToiResult if the bodies will collide within the timestep, null otherwise.
     */
    fun toiBoxBox(
        movingBody: RigidBody,
        movingShape: BoxShape,
        staticBody: RigidBody,
        staticShape: BoxShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = movingBody.linearVelocity
        if (velocity.lengthSquared < 1e-10) return null

        // Quick check: swept AABB overlap
        val sweptAabb = sweptAABB(movingBody, movingShape, dt)
        val staticAabb = AABB.fromCenterExtents(staticBody.transform.position, staticShape.halfExtents)

        if (!sweptAabb.intersects(staticAabb)) return null

        // Binary search for precise TOI
        // Use a local position variable instead of mutating the body's transform
        val narrowphase = Narrowphase()
        val startPos = movingBody.transform.position
        var lo = 0.0
        var hi = dt
        val maxIterations = 20
        val tolerance = 1e-6

        // Check if already colliding at t=0 — return null, let discrete collision handle it
        val contactAtStart = narrowphase.testBoxBoxAt(startPos, movingBody.transform.rotation,
            staticBody.transform.position, staticBody.transform.rotation, movingShape, staticShape)
        if (contactAtStart != null) {
            return null
        }

        for (i in 0 until maxIterations) {
            val mid = (lo + hi) * 0.5

            // Test collision at mid position without mutating the body
            val midPos = startPos + velocity * mid
            val contact = narrowphase.testBoxBoxAt(midPos, movingBody.transform.rotation,
                staticBody.transform.position, staticBody.transform.rotation, movingShape, staticShape)
            if (contact != null) {
                hi = mid // Collision found — try earlier time
            } else {
                lo = mid // No collision — try later time
            }

            if (hi - lo < tolerance) break
        }

        // Validate TOI
        if (hi <= 0.0 || hi > dt) return null
        val toiFraction = hi / dt
        if (toiFraction < 1e-8) return null // Avoid near-zero TOI

        // Calculate final contact at TOI position
        val toiPosition = startPos + velocity * hi
        val finalContact = narrowphase.testBoxBoxAt(toiPosition, movingBody.transform.rotation,
            staticBody.transform.position, staticBody.transform.rotation, movingShape, staticShape)

        if (finalContact == null) return null

        return ToiResult(
            toi = toiFraction,
            normal = finalContact.normal,
            contactPoint = finalContact.contactPoint,
            movingBodyId = movingBody.id,
            hitBodyId = staticBody.id
        )
    }

    /**
     * Advance a body to the given TOI fraction.
     *
     * @param body The body to advance.
     * @param toiFraction Fraction of the timestep (0.0 to 1.0).
     * @param dt Full timestep duration.
     */
    fun advanceBodyToToi(body: RigidBody, toiFraction: Double, dt: Double) {
        val advanceDt = toiFraction * dt
        val newPosition = body.transform.position + body.linearVelocity * advanceDt
        body.transform = io.vakuum.math.Transform(
            position = newPosition,
            rotation = body.transform.rotation
        )
    }

    private fun getWorldAxes(rotation: io.vakuum.math.Quaternion): List<Vector3> {
        return listOf(
            rotation.rotateVector(Vector3.UNIT_X),
            rotation.rotateVector(Vector3.UNIT_Y),
            rotation.rotateVector(Vector3.UNIT_Z)
        )
    }

    private fun contactPointBoxPlane(
        body: RigidBody,
        shape: BoxShape,
        planeNormal: Vector3,
        planeDistance: Double
    ): Vector3 {
        val center = body.transform.position
        val worldAxes = getWorldAxes(body.transform.rotation)
        val halfExtents = listOf(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z)

        var contactPoint = center
        for (i in worldAxes.indices) {
            val proj = worldAxes[i].dot(planeNormal)
            if (proj > 0) {
                contactPoint -= worldAxes[i] * halfExtents[i]
            } else {
                contactPoint += worldAxes[i] * halfExtents[i]
            }
        }
        // Project onto plane
        return contactPoint - planeNormal * (planeNormal.dot(contactPoint) - planeDistance)
    }

    private fun contactPointAtToi(
        body: RigidBody,
        shape: BoxShape,
        planeNormal: Vector3,
        planeDistance: Double,
        toi: Double
    ): Vector3 {
        // Position at TOI
        val positionAtToi = body.transform.position + body.linearVelocity * toi
        val worldAxes = getWorldAxes(body.transform.rotation)
        val halfExtents = listOf(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z)

        var contactPoint = positionAtToi
        for (i in worldAxes.indices) {
            val proj = worldAxes[i].dot(planeNormal)
            if (proj > 0) {
                contactPoint -= worldAxes[i] * halfExtents[i]
            } else {
                contactPoint += worldAxes[i] * halfExtents[i]
            }
        }
        return contactPoint - planeNormal * (planeNormal.dot(contactPoint) - planeDistance)
    }

    // ============================================================
    // Sphere CCD
    // ============================================================

    /**
     * Calculate TOI between a moving sphere and a static plane.
     * Analytical solution.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiSpherePlane(
        body: RigidBody,
        shape: SphereShape,
        planeNormal: Vector3,
        planeDistance: Double,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val worldCenter = body.transform.position + body.transform.rotation.rotateVector(shape.center)
        val velocity = body.linearVelocity

        // If the body is not moving toward the plane, no TOI
        val velAlongNormal = velocity.dot(planeNormal)
        if (velAlongNormal >= 0.0) return null

        // Distance from sphere center to plane
        val distToPlane = planeNormal.dot(worldCenter) - planeDistance

        // The sphere extends `radius` units toward the plane
        val lowestPointDist = distToPlane - shape.radius

        // If already penetrating, return null
        if (lowestPointDist <= 0.0) return null

        // Time to impact: distance / speed
        val distanceToTravel = lowestPointDist
        val toi = distanceToTravel / (-velAlongNormal)

        if (toi < 0.0 || toi > dt) return null
        val toiFraction = toi / dt
        if (toiFraction < 1e-8) return null

        // Contact point at TOI
        val centerAtToi = worldCenter + velocity * toi
        val contactPoint = centerAtToi - planeNormal * (planeNormal.dot(centerAtToi) - planeDistance + shape.radius)

        return ToiResult(
            toi = toiFraction,
            normal = planeNormal,
            contactPoint = contactPoint,
            movingBodyId = body.id,
            hitBodyId = -1
        )
    }

    /**
     * Calculate TOI between two moving spheres.
     * Analytical solution using relative velocity.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiSphereSphere(
        bodyA: RigidBody, shapeA: SphereShape,
        bodyB: RigidBody, shapeB: SphereShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val centerA = bodyA.transform.position + bodyA.transform.rotation.rotateVector(shapeA.center)
        val centerB = bodyB.transform.position + bodyB.transform.rotation.rotateVector(shapeB.center)

        val diff = centerB - centerA
        val dist = diff.length
        val minDist = shapeA.radius + shapeB.radius

        // If already penetrating, return null
        if (dist <= minDist) return null

        // Relative velocity
        val relVel = bodyA.linearVelocity - bodyB.linearVelocity
        if (relVel.lengthSquared < 1e-10) return null

        // Solve: |diff + relVel * t| = minDist
        // (diff + relVel*t) · (diff + relVel*t) = minDist²
        // a*t² + b*t + c = 0
        val a = relVel.lengthSquared
        val b = 2.0 * diff.dot(relVel)
        val c = diff.lengthSquared - minDist * minDist

        val discriminant = b * b - 4.0 * a * c
        if (discriminant < 0.0) return null // No intersection

        val sqrtD = kotlin.math.sqrt(discriminant)
        val t1 = (-b - sqrtD) / (2.0 * a)
        val t2 = (-b + sqrtD) / (2.0 * a)

        // Use the earliest positive TOI within the timestep
        val toi = if (t1 > 0.0) t1 else if (t2 > 0.0) t2 else return null

        if (toi > dt) return null
        val toiFraction = toi / dt
        if (toiFraction < 1e-8) return null

        // Contact normal and point at TOI
        val centerAAtToi = centerA + bodyA.linearVelocity * toi
        val centerBAtToi = centerB + bodyB.linearVelocity * toi
        val normalDir = centerAAtToi - centerBAtToi
        val normal = normalDir / normalDir.length
        val contactPoint = centerAAtToi - normal * shapeA.radius

        return ToiResult(
            toi = toiFraction,
            normal = normal, // Push direction for bodyA: away from bodyB
            contactPoint = contactPoint,
            movingBodyId = bodyA.id,
            hitBodyId = bodyB.id
        )
    }

    // ============================================================
    // Capsule CCD
    // ============================================================

    /**
     * Calculate TOI between a moving capsule and a static plane.
     * Analytical solution.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiCapsulePlane(
        body: RigidBody,
        shape: CapsuleShape,
        planeNormal: Vector3,
        planeDistance: Double,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val (p1, p2) = getWorldCapsuleEndpoints(body, shape)
        val velocity = body.linearVelocity

        // If the body is not moving toward the plane, no TOI
        val velAlongNormal = velocity.dot(planeNormal)
        if (velAlongNormal >= 0.0) return null

        // Find the lowest point of the capsule along the plane normal
        val dist1 = planeNormal.dot(p1) - planeDistance
        val dist2 = planeNormal.dot(p2) - planeDistance
        val (lowestDist, lowestPoint) = if (dist1 < dist2) Pair(dist1, p1) else Pair(dist2, p2)

        // If already penetrating, return null
        if (lowestDist <= shape.radius) return null

        // Time to impact
        val distanceToTravel = lowestDist - shape.radius
        val toi = distanceToTravel / (-velAlongNormal)

        if (toi < 0.0 || toi > dt) return null
        val toiFraction = toi / dt
        if (toiFraction < 1e-8) return null

        // Contact point at TOI
        val lowestPointAtToi = lowestPoint + velocity * toi
        val contactPoint = lowestPointAtToi - planeNormal * (planeNormal.dot(lowestPointAtToi) - planeDistance + shape.radius)

        return ToiResult(
            toi = toiFraction,
            normal = planeNormal,
            contactPoint = contactPoint,
            movingBodyId = body.id,
            hitBodyId = -1
        )
    }

    /**
     * Calculate TOI between a moving capsule and a static sphere.
     * Uses binary search on the capsule-sphere narrowphase test.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiCapsuleSphere(
        capsuleBody: RigidBody, capsuleShape: CapsuleShape,
        sphereBody: RigidBody, sphereShape: SphereShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = capsuleBody.linearVelocity
        if (velocity.lengthSquared < 1e-10) return null

        val narrowphase = Narrowphase()

        // Check if already colliding
        val contactAtStart = narrowphase.testCapsuleSphere(capsuleBody, capsuleShape, sphereBody, sphereShape)
        if (contactAtStart != null) return null

        // Binary search for precise TOI
        val (startP1, startP2) = getWorldCapsuleEndpoints(capsuleBody, capsuleShape)
        var lo = 0.0
        var hi = dt
        val maxIterations = 20
        val tolerance = 1e-6

        for (i in 0 until maxIterations) {
            val mid = (lo + hi) * 0.5
            val p1AtMid = startP1 + velocity * mid
            val p2AtMid = startP2 + velocity * mid
            val sphereCenter = sphereBody.transform.position + sphereBody.transform.rotation.rotateVector(sphereShape.center)

            val contact = narrowphase.testCapsuleSphereAt(
                p1AtMid, p2AtMid, capsuleShape.radius,
                sphereCenter, sphereShape.radius
            )
            if (contact != null) {
                hi = mid
            } else {
                lo = mid
            }
            if (hi - lo < tolerance) break
        }

        if (hi <= 0.0 || hi > dt) return null
        val toiFraction = hi / dt
        if (toiFraction < 1e-8) return null

        // Calculate final contact at TOI
        val p1AtToi = startP1 + velocity * hi
        val p2AtToi = startP2 + velocity * hi
        val sphereCenter = sphereBody.transform.position + sphereBody.transform.rotation.rotateVector(sphereShape.center)
        val finalContact = narrowphase.testCapsuleSphereAt(
            p1AtToi, p2AtToi, capsuleShape.radius,
            sphereCenter, sphereShape.radius
        )
        if (finalContact == null) return null

        return ToiResult(
            toi = toiFraction,
            normal = finalContact.normal,
            contactPoint = finalContact.contactPoint,
            movingBodyId = capsuleBody.id,
            hitBodyId = sphereBody.id
        )
    }

    // ============================================================
    // Convex Hull CCD
    // ============================================================

    /**
     * Calculate TOI between a moving convex hull and a static plane.
     * Analytical solution.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiConvexPlane(
        body: RigidBody,
        shape: ConvexHullShape,
        planeNormal: Vector3,
        planeDistance: Double,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = body.linearVelocity

        // If the body is not moving toward the plane, no TOI
        val velAlongNormal = velocity.dot(planeNormal)
        if (velAlongNormal >= 0.0) return null

        // Find the lowest vertex of the convex hull along the plane normal
        val worldVerts = shape.vertices.map {
            body.transform.position + body.transform.rotation.rotateVector(it)
        }

        var minDist = Double.MAX_VALUE
        var lowestVertex = Vector3.ZERO
        for (v in worldVerts) {
            val dist = planeNormal.dot(v) - planeDistance
            if (dist < minDist) {
                minDist = dist
                lowestVertex = v
            }
        }

        // If already penetrating, return null
        if (minDist <= 0.0) return null

        // Time to impact
        val toi = minDist / (-velAlongNormal)

        if (toi < 0.0 || toi > dt) return null
        val toiFraction = toi / dt
        if (toiFraction < 1e-8) return null

        // Contact point at TOI
        val contactPoint = lowestVertex + velocity * toi - planeNormal * (planeNormal.dot(lowestVertex + velocity * toi) - planeDistance)

        return ToiResult(
            toi = toiFraction,
            normal = planeNormal,
            contactPoint = contactPoint,
            movingBodyId = body.id,
            hitBodyId = -1
        )
    }

    /**
     * Calculate TOI between a moving sphere and a static box.
     * Uses binary search on the sphere-box narrowphase test.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiSphereBox(
        sphereBody: RigidBody, sphereShape: SphereShape,
        boxBody: RigidBody, boxShape: BoxShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = sphereBody.linearVelocity
        if (velocity.lengthSquared < 1e-10) return null

        // Quick check: swept AABB overlap
        val startCenter = sphereBody.transform.position + sphereBody.transform.rotation.rotateVector(sphereShape.center)
        val endCenter = startCenter + velocity * dt
        val sweptAabb = AABB.fromCenterExtents(
            (startCenter + endCenter) * 0.5,
            Vector3(sphereShape.radius, sphereShape.radius, sphereShape.radius) +
                (endCenter - startCenter).let { Vector3(kotlin.math.abs(it.x), kotlin.math.abs(it.y), kotlin.math.abs(it.z)) } * 0.5
        )
        val staticAabb = AABB.fromCenterExtents(boxBody.transform.position, boxShape.halfExtents)

        if (!sweptAabb.intersects(staticAabb)) return null

        // Binary search for precise TOI
        val narrowphase = Narrowphase()
        val startPos = sphereBody.transform.position
        var lo = 0.0
        var hi = dt
        val maxIterations = 20
        val tolerance = 1e-6

        // Check if already colliding at t=0
        val contactAtStart = narrowphase.testSphereBoxAt(
            startPos + sphereBody.transform.rotation.rotateVector(sphereShape.center), sphereShape.radius,
            boxBody.transform.position, boxBody.transform.rotation, boxShape
        )
        if (contactAtStart != null) return null

        for (i in 0 until maxIterations) {
            val mid = (lo + hi) * 0.5
            val midCenter = startPos + velocity * mid + sphereBody.transform.rotation.rotateVector(sphereShape.center)
            val contact = narrowphase.testSphereBoxAt(
                midCenter, sphereShape.radius,
                boxBody.transform.position, boxBody.transform.rotation, boxShape
            )
            if (contact != null) {
                hi = mid
            } else {
                lo = mid
            }
            if (hi - lo < tolerance) break
        }

        if (hi <= 0.0 || hi > dt) return null
        val toiFraction = hi / dt
        if (toiFraction < 1e-8) return null

        // Calculate final contact at TOI
        val toiCenter = startPos + velocity * hi + sphereBody.transform.rotation.rotateVector(sphereShape.center)
        val finalContact = narrowphase.testSphereBoxAt(
            toiCenter, sphereShape.radius,
            boxBody.transform.position, boxBody.transform.rotation, boxShape
        )
        if (finalContact == null) return null

        return ToiResult(
            toi = toiFraction,
            normal = finalContact.normal,
            contactPoint = finalContact.contactPoint,
            movingBodyId = sphereBody.id,
            hitBodyId = boxBody.id
        )
    }

    /**
     * Calculate TOI between a moving capsule and a static box.
     * Uses binary search on the capsule-box narrowphase test.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiCapsuleBox(
        capsuleBody: RigidBody, capsuleShape: CapsuleShape,
        boxBody: RigidBody, boxShape: BoxShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = capsuleBody.linearVelocity
        if (velocity.lengthSquared < 1e-10) return null

        // Quick check: swept AABB overlap using bounding sphere
        val bs = capsuleShape.boundingSphere
        val startCenter = capsuleBody.transform.position + capsuleBody.transform.rotation.rotateVector(bs.center)
        val endCenter = startCenter + velocity * dt
        val sweptAabb = AABB.fromCenterExtents(
            (startCenter + endCenter) * 0.5,
            Vector3(bs.radius, bs.radius, bs.radius) +
                (endCenter - startCenter).let { Vector3(kotlin.math.abs(it.x), kotlin.math.abs(it.y), kotlin.math.abs(it.z)) } * 0.5
        )
        val staticAabb = AABB.fromCenterExtents(boxBody.transform.position, boxShape.halfExtents)

        if (!sweptAabb.intersects(staticAabb)) return null

        // Binary search for precise TOI
        val narrowphase = Narrowphase()
        val (startP1, startP2) = getWorldCapsuleEndpoints(capsuleBody, capsuleShape)
        var lo = 0.0
        var hi = dt
        val maxIterations = 20
        val tolerance = 1e-6

        // Check if already colliding at t=0
        val contactAtStart = narrowphase.testCapsuleBoxAt(
            startP1, startP2, capsuleShape.radius,
            boxBody.transform.position, boxBody.transform.rotation, boxShape
        )
        if (contactAtStart != null) return null

        for (i in 0 until maxIterations) {
            val mid = (lo + hi) * 0.5
            val p1AtMid = startP1 + velocity * mid
            val p2AtMid = startP2 + velocity * mid
            val contact = narrowphase.testCapsuleBoxAt(
                p1AtMid, p2AtMid, capsuleShape.radius,
                boxBody.transform.position, boxBody.transform.rotation, boxShape
            )
            if (contact != null) {
                hi = mid
            } else {
                lo = mid
            }
            if (hi - lo < tolerance) break
        }

        if (hi <= 0.0 || hi > dt) return null
        val toiFraction = hi / dt
        if (toiFraction < 1e-8) return null

        // Calculate final contact at TOI
        val p1AtToi = startP1 + velocity * hi
        val p2AtToi = startP2 + velocity * hi
        val finalContact = narrowphase.testCapsuleBoxAt(
            p1AtToi, p2AtToi, capsuleShape.radius,
            boxBody.transform.position, boxBody.transform.rotation, boxShape
        )
        if (finalContact == null) return null

        return ToiResult(
            toi = toiFraction,
            normal = finalContact.normal,
            contactPoint = finalContact.contactPoint,
            movingBodyId = capsuleBody.id,
            hitBodyId = boxBody.id
        )
    }

    /**
     * Calculate TOI between a moving convex hull and a static convex hull.
     * Uses binary search on the convex-convex narrowphase test.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiConvexConvex(
        convexBodyA: RigidBody, convexShapeA: ConvexHullShape,
        convexBodyB: RigidBody, convexShapeB: ConvexHullShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = convexBodyA.linearVelocity
        if (velocity.lengthSquared < 1e-10) return null

        // Quick check: swept AABB overlap using bounding sphere
        val bs = convexShapeA.boundingSphere
        val startCenter = convexBodyA.transform.position + convexBodyA.transform.rotation.rotateVector(bs.center)
        val endCenter = startCenter + velocity * dt
        val sweptAabb = AABB.fromCenterExtents(
            (startCenter + endCenter) * 0.5,
            Vector3(bs.radius, bs.radius, bs.radius) +
                (endCenter - startCenter).let { Vector3(kotlin.math.abs(it.x), kotlin.math.abs(it.y), kotlin.math.abs(it.z)) } * 0.5
        )
        val bsB = convexShapeB.boundingSphere
        val staticAabb = AABB.fromCenterExtents(
            convexBodyB.transform.position + convexBodyB.transform.rotation.rotateVector(bsB.center),
            Vector3(bsB.radius, bsB.radius, bsB.radius)
        )

        if (!sweptAabb.intersects(staticAabb)) return null

        // Binary search for precise TOI
        val narrowphase = Narrowphase()
        val startPos = convexBodyA.transform.position
        var lo = 0.0
        var hi = dt
        val maxIterations = 20
        val tolerance = 1e-6

        // Check if already colliding at t=0
        val contactAtStart = narrowphase.testConvexConvexAt(
            startPos, convexBodyA.transform.rotation,
            convexBodyB.transform.position, convexBodyB.transform.rotation,
            convexShapeA, convexShapeB
        )
        if (contactAtStart != null) return null

        for (i in 0 until maxIterations) {
            val mid = (lo + hi) * 0.5
            val midPos = startPos + velocity * mid
            val contact = narrowphase.testConvexConvexAt(
                midPos, convexBodyA.transform.rotation,
                convexBodyB.transform.position, convexBodyB.transform.rotation,
                convexShapeA, convexShapeB
            )
            if (contact != null) {
                hi = mid
            } else {
                lo = mid
            }
            if (hi - lo < tolerance) break
        }

        if (hi <= 0.0 || hi > dt) return null
        val toiFraction = hi / dt
        if (toiFraction < 1e-8) return null

        // Calculate final contact at TOI
        val toiPos = startPos + velocity * hi
        val finalContact = narrowphase.testConvexConvexAt(
            toiPos, convexBodyA.transform.rotation,
            convexBodyB.transform.position, convexBodyB.transform.rotation,
            convexShapeA, convexShapeB
        )
        if (finalContact == null) return null

        return ToiResult(
            toi = toiFraction,
            normal = finalContact.normal,
            contactPoint = finalContact.contactPoint,
            movingBodyId = convexBodyA.id,
            hitBodyId = convexBodyB.id
        )
    }

    /**
     * Calculate TOI between a moving convex hull and a static box.
     * Uses binary search on the convex-box narrowphase test.
     *
     * Returns null if already penetrating — use discrete collision instead.
     */
    fun toiConvexBox(
        convexBody: RigidBody, convexShape: ConvexHullShape,
        boxBody: RigidBody, boxShape: BoxShape,
        dt: Double
    ): ToiResult? {
        if (dt <= 0.0) return null

        val velocity = convexBody.linearVelocity
        if (velocity.lengthSquared < 1e-10) return null

        // Quick check: swept AABB overlap
        val sweptAabb = sweptAABBForConvex(convexBody, convexShape, dt)
        val staticAabb = AABB.fromCenterExtents(boxBody.transform.position, boxShape.halfExtents)

        if (!sweptAabb.intersects(staticAabb)) return null

        // Binary search for precise TOI
        val narrowphase = Narrowphase()
        val startPos = convexBody.transform.position
        var lo = 0.0
        var hi = dt
        val maxIterations = 20
        val tolerance = 1e-6

        // Check if already colliding at t=0
        val contactAtStart = narrowphase.testConvexBoxAt(
            startPos, convexBody.transform.rotation,
            boxBody.transform.position, boxBody.transform.rotation,
            convexShape, boxShape
        )
        if (contactAtStart != null) return null

        for (i in 0 until maxIterations) {
            val mid = (lo + hi) * 0.5
            val midPos = startPos + velocity * mid
            val contact = narrowphase.testConvexBoxAt(
                midPos, convexBody.transform.rotation,
                boxBody.transform.position, boxBody.transform.rotation,
                convexShape, boxShape
            )
            if (contact != null) {
                hi = mid
            } else {
                lo = mid
            }
            if (hi - lo < tolerance) break
        }

        if (hi <= 0.0 || hi > dt) return null
        val toiFraction = hi / dt
        if (toiFraction < 1e-8) return null

        // Calculate final contact at TOI
        val toiPos = startPos + velocity * hi
        val finalContact = narrowphase.testConvexBoxAt(
            toiPos, convexBody.transform.rotation,
            boxBody.transform.position, boxBody.transform.rotation,
            convexShape, boxShape
        )
        if (finalContact == null) return null

        return ToiResult(
            toi = toiFraction,
            normal = finalContact.normal,
            contactPoint = finalContact.contactPoint,
            movingBodyId = convexBody.id,
            hitBodyId = boxBody.id
        )
    }

    private fun sweptAABBForConvex(body: RigidBody, shape: ConvexHullShape, dt: Double): AABB {
        val bs = shape.boundingSphere
        val startCenter = body.transform.position + body.transform.rotation.rotateVector(bs.center)
        val endCenter = startCenter + body.linearVelocity * dt
        val startAABB = AABB.fromCenterExtents(startCenter, Vector3(bs.radius, bs.radius, bs.radius))
        val endAABB = AABB.fromCenterExtents(endCenter, Vector3(bs.radius, bs.radius, bs.radius))
        return startAABB.union(endAABB)
    }

    private fun getWorldCapsuleEndpoints(body: RigidBody, shape: CapsuleShape): Pair<Vector3, Vector3> {
        val p1 = body.transform.position + body.transform.rotation.rotateVector(shape.p1)
        val p2 = body.transform.position + body.transform.rotation.rotateVector(shape.p2)
        return Pair(p1, p2)
    }
}
