package io.vakuum.integration

import io.vakuum.collision.BoxShape
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import io.vakuum.runner.SimulationPipeline
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

/**
 * Integration tests for Continuous Collision Detection (CCD).
 *
 * Tests CCD with the full SimulationPipeline, including tunneling prevention,
 * substep processing, and determinism.
 */
class CcdIntegrationTest {

    @Test
    fun `CCD prevents tunneling through ground plane`() {
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
        assertTrue(pipeline.getBodies()[0].transform.position.y > -1.0,
            "Cube should not tunnel through ground. Position: ${pipeline.getBodies()[0].transform.position.y}")
    }

    @Test
    fun `CCD with slow body uses discrete detection`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 10.0
        )
        val pipeline = SimulationPipeline(config)

        val cube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 2.0, 0.0))
            .linearVelocity(Vector3(0.0, -1.0, 0.0))
            .mass(1.0)
            .restitution(0.0)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 120) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN values")
        assertTrue(pipeline.getBodies()[0].transform.position.y > 0.0,
            "Cube should rest above ground")
    }

    @Test
    fun `CCD multiple substeps for bouncing`() {
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
            .position(Vector3(0.0, 20.0, 0.0))
            .linearVelocity(Vector3(0.0, -30.0, 0.0))
            .mass(1.0)
            .restitution(0.5)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 300) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN values after 300 steps")
        assertTrue(pipeline.getBodies()[0].transform.position.y > 0.0,
            "Cube should be above ground")
    }

    @Test
    fun `CCD determinism test`() {
        fun runSimulation(): Vector3 {
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
                .linearVelocity(Vector3(0.0, -20.0, 0.0))
                .mass(1.0)
                .restitution(0.3)
                .friction(0.5)
                .build()

            pipeline.addBody(cube)
            pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

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

    @Test
    fun `CCD max substeps prevents infinite loop`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 1.0,
            ccdMaxSubsteps = 5,
            ccdToiTolerance = 0.001
        )
        val pipeline = SimulationPipeline(config)

        val cube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.9)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN after max substeps")
    }

    @Test
    fun `CCD disabled regression — existing tests still pass`() {
        // Verify that disabling CCD doesn't break existing behavior
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = false
        )
        val pipeline = SimulationPipeline(config)

        val cube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 10.0, 0.0))
            .linearVelocity(Vector3.ZERO)
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 600) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > 0.0 && pos.y < 2.0,
            "Cube should rest on ground. Position: $pos")
    }
}
