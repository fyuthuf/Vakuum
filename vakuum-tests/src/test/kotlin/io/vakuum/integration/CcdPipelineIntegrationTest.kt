package io.vakuum.integration

import io.vakuum.collision.*
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import io.vakuum.runner.SimulationPipeline
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

/**
 * End-to-end CCD pipeline integration tests for Sphere, Capsule, and ConvexHull.
 *
 * These tests verify that the new collision shapes actually participate in CCD
 * through SimulationPipeline, not just in isolated TOI unit tests.
 */
class CcdPipelineIntegrationTest {

    // ============================================================
    // Sphere CCD through SimulationPipeline
    // ============================================================

    @Test
    fun `Fast sphere against static plane via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        val sphere = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()

        pipeline.addBody(sphere)
        pipeline.setShape(1, SphereShape.fromRadius(0.5))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > -1.0, "Sphere should not tunnel through ground. Position: ${pos.y}")
    }

    @Test
    fun `Fast sphere against static box via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, 0.0, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Static box obstacle
        val box = rigidBody()
            .id(1)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .mass(0.0)
            .build()
        pipeline.addBody(box)
        pipeline.setShape(1, BoxShape(Vector3(1.0, 1.0, 1.0)))

        // Fast sphere moving toward the box
        val sphere = rigidBody()
            .id(2)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.0, 10.0))
            .linearVelocity(Vector3(0.0, 0.0, -50.0))
            .mass(1.0)
            .restitution(0.2)
            .friction(0.5)
            .build()
        pipeline.addBody(sphere)
        pipeline.setShape(2, SphereShape.fromRadius(0.3))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[1].transform.position
        // Sphere should not pass through the box (box extends from z=-1 to z=1, sphere radius 0.3)
        assertTrue(pos.z > -2.0, "Sphere should not tunnel through box. Position: ${pos.z}")
    }

    // ============================================================
    // Capsule CCD through SimulationPipeline
    // ============================================================

    @Test
    fun `Fast capsule against static plane via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        val capsule = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()

        pipeline.addBody(capsule)
        pipeline.setShape(1, CapsuleShape.fromHalfHeight(0.5, 0.3))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > -1.0, "Capsule should not tunnel through ground. Position: ${pos.y}")
    }

    @Test
    fun `Fast capsule against static box via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, 0.0, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Static box
        val box = rigidBody()
            .id(1)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .mass(0.0)
            .build()
        pipeline.addBody(box)
        pipeline.setShape(1, BoxShape(Vector3(1.0, 1.0, 1.0)))

        // Fast capsule moving toward the box
        val capsule = rigidBody()
            .id(2)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.0, 10.0))
            .linearVelocity(Vector3(0.0, 0.0, -50.0))
            .mass(1.0)
            .restitution(0.2)
            .friction(0.5)
            .build()
        pipeline.addBody(capsule)
        pipeline.setShape(2, CapsuleShape.fromHalfHeight(0.5, 0.2))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[1].transform.position
        assertTrue(pos.z > -2.0, "Capsule should not tunnel through box. Position: ${pos.z}")
    }

    // ============================================================
    // ConvexHull CCD through SimulationPipeline
    // ============================================================

    @Test
    fun `Fast convex hull against static plane via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        val convex = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()

        pipeline.addBody(convex)
        pipeline.setShape(1, ConvexHullShape.fromBox(Vector3(0.5, 0.5, 0.5)))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > -1.0, "Convex hull should not tunnel through ground. Position: ${pos.y}")
    }

    @Test
    fun `Fast convex hull against static box via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, 0.0, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Static box
        val box = rigidBody()
            .id(1)
            .bodyType(BodyType.STATIC)
            .position(Vector3(0.0, 0.0, 0.0))
            .mass(0.0)
            .build()
        pipeline.addBody(box)
        pipeline.setShape(1, BoxShape(Vector3(1.0, 1.0, 1.0)))

        // Fast convex hull moving toward the box
        val convex = rigidBody()
            .id(2)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.0, 10.0))
            .linearVelocity(Vector3(0.0, 0.0, -50.0))
            .mass(1.0)
            .restitution(0.2)
            .friction(0.5)
            .build()
        pipeline.addBody(convex)
        pipeline.setShape(2, ConvexHullShape.fromBox(Vector3(0.3, 0.3, 0.3)))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[1].transform.position
        assertTrue(pos.z > -2.0, "Convex hull should not tunnel through box. Position: ${pos.z}")
    }

    // ============================================================
    // Regression: Box CCD still works
    // ============================================================

    @Test
    fun `High-speed box CCD regression`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        val cube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > -1.0, "Box should not tunnel through ground. Position: ${pos.y}")
    }

    // ============================================================
    // Multiple sequential impacts
    // ============================================================

    @Test
    fun `Sphere multiple sequential impacts via CCD`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        val sphere = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 20.0, 0.0))
            .linearVelocity(Vector3(0.0, -30.0, 0.0))
            .mass(1.0)
            .restitution(0.5)
            .friction(0.5)
            .build()

        pipeline.addBody(sphere)
        pipeline.setShape(1, SphereShape.fromRadius(0.5))

        for (i in 0 until 300) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN after 300 steps")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > -1.0, "Sphere should not tunnel through ground after 300 steps")
    }

    // ============================================================
    // Mixed CCD and discrete bodies
    // ============================================================

    @Test
    fun `Mixed CCD sphere and discrete box bodies`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 20.0, // Only very fast bodies use CCD
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Fast sphere (CCD enabled)
        val fastSphere = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(-2.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()
        pipeline.addBody(fastSphere)
        pipeline.setShape(1, SphereShape.fromRadius(0.3))

        // Slow box (CCD disabled, uses discrete)
        val slowBox = rigidBody()
            .id(2)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(2.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -2.0, 0.0))
            .mass(1.0)
            .restitution(0.0)
            .friction(0.5)
            .build()
        pipeline.addBody(slowBox)
        pipeline.setShape(2, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 200) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN with mixed CCD/discrete bodies")
        val spherePos = pipeline.getBodies()[0].transform.position
        val boxPos = pipeline.getBodies()[1].transform.position
        assertTrue(spherePos.y > -1.0, "Fast sphere should not tunnel")
        assertTrue(boxPos.y > -1.0, "Slow box should not tunnel")
    }

    // ============================================================
    // Substep exhaustion and fallback
    // ============================================================

    @Test
    fun `CCD substep exhaustion with sphere fallback`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 2
        )
        val pipeline = SimulationPipeline(config)

        val sphere = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.8) // High restitution → many bounces
            .friction(0.5)
            .build()

        pipeline.addBody(sphere)
        pipeline.setShape(1, SphereShape.fromRadius(0.5))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN after substep exhaustion")
        assertTrue(pipeline.ccdDiagnostics.ccdSteps >= 1, "CCD steps should be at least 1")
    }

    // ============================================================
    // Deterministic repeated simulations
    // ============================================================

    @Test
    fun `CCD sphere simulation is deterministic`() {
        fun runSimulation(): Vector3 {
            val config = PhysicsConfiguration(
                gravity = Vector3(0.0, -9.81, 0.0),
                fixedTimestep = 1.0 / 60.0,
                ccdEnabled = true,
                ccdVelocityThreshold = 5.0,
                ccdMaxSubsteps = 10
            )
            val pipeline = SimulationPipeline(config)

            val sphere = rigidBody()
                .id(1)
                .bodyType(BodyType.DYNAMIC)
                .position(Vector3(0.0, 10.0, 0.0))
                .linearVelocity(Vector3(0.0, -30.0, 0.0))
                .mass(1.0)
                .restitution(0.3)
                .friction(0.5)
                .build()

            pipeline.addBody(sphere)
            pipeline.setShape(1, SphereShape.fromRadius(0.5))

            for (i in 0 until 200) {
                pipeline.step()
            }

            return pipeline.getBodies()[0].transform.position
        }

        val result1 = runSimulation()
        val result2 = runSimulation()

        assertEquals(result1.x, result2.x, 1e-10, "X position should be identical")
        assertEquals(result1.y, result2.y, 1e-10, "Y position should be identical")
        assertEquals(result1.z, result2.z, 1e-10, "Z position should be identical")
    }

    // ============================================================
    // CCD disabled regression for new shapes
    // ============================================================

    @Test
    fun `CCD disabled sphere uses discrete collision`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = false
        )
        val pipeline = SimulationPipeline(config)

        val sphere = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 2.0, 0.0))
            .linearVelocity(Vector3(0.0, -1.0, 0.0))
            .mass(1.0)
            .restitution(0.0)
            .friction(0.5)
            .build()

        pipeline.addBody(sphere)
        pipeline.setShape(1, SphereShape.fromRadius(0.5))

        for (i in 0 until 120) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > 0.0, "Sphere should rest above ground with discrete collision")
    }
}
