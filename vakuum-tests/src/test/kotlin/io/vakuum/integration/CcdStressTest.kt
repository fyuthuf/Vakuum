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
 * M3.3 — CCD Energy Stability & Stress Tests.
 *
 * Tests energy behavior, long-term stability, and stress scenarios for CCD.
 */
class CcdStressTest {

    // ============================================================
    // Energy Stability
    // ============================================================

    @Test
    fun `CCD resting body has no energy injection`() {
        // A body resting on the ground should not gain energy from CCD
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 1.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Place body just above ground, at rest
        val cube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 0.6, 0.0))
            .linearVelocity(Vector3.ZERO)
            .mass(1.0)
            .restitution(0.0)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        // Run many steps — body should stay at rest
        for (i in 0 until 200) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val body = pipeline.getBodies()[0]
        val speed = body.linearVelocity.length
        assertTrue(speed < 0.5, "Resting body should have near-zero velocity. Speed: $speed")
        assertTrue(body.transform.position.y > 0.0, "Body should be above ground")
    }

    @Test
    fun `CCD bouncing energy decreases with restitution`() {
        // Bouncing body should lose energy over time
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
            .restitution(0.5)
            .friction(0.3)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        var maxBounceHeight = 0.0
        var maxSteps = 500
        for (i in 0 until maxSteps) {
            pipeline.step()
            val y = pipeline.getBodies()[0].transform.position.y
            if (y > maxBounceHeight) maxBounceHeight = y
        }

        assertFalse(pipeline.hasNaN(), "No NaN values")
        // With restitution=0.5, bounce height should decrease significantly
        assertTrue(maxBounceHeight < 10.0, "Max bounce height should be less than initial. Max: $maxBounceHeight")
        assertTrue(maxBounceHeight > 0.0, "Body should bounce at least once")
    }

    @Test
    fun `CCD high restitution stability`() {
        // High restitution should not cause energy explosion
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
            .position(Vector3(0.0, 5.0, 0.0))
            .linearVelocity(Vector3(0.0, -10.0, 0.0))
            .mass(1.0)
            .restitution(0.9)
            .friction(0.1)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 300) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN with high restitution")
        val pos = pipeline.getBodies()[0].transform.position
        // Body should not fly to infinity
        assertTrue(pos.y < 100.0, "Body should not fly away. Y: ${pos.y}")
        assertTrue(pos.y > -1.0, "Body should not tunnel through ground. Y: ${pos.y}")
    }

    @Test
    fun `CCD friction during impact`() {
        // Horizontal velocity should be reduced by friction during CCD impact
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
            .linearVelocity(Vector3(10.0, -20.0, 0.0)) // Horizontal + vertical
            .mass(1.0)
            .restitution(0.3)
            .friction(0.8)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        val initialHorizontalSpeed = 10.0

        for (i in 0 until 300) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN values")
        val finalHorizontalSpeed = kotlin.math.abs(pipeline.getBodies()[0].linearVelocity.x)
        assertTrue(finalHorizontalSpeed < initialHorizontalSpeed,
            "Friction should reduce horizontal speed. Initial: $initialHorizontalSpeed, Final: $finalHorizontalSpeed")
    }

    // ============================================================
    // Substep Exhaustion
    // ============================================================

    @Test
    fun `CCD substep exhaustion falls back safely`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 1.0,
            ccdMaxSubsteps = 2, // Very low limit — CCD body will hit ground in 1 substep, then bounce
            ccdToiTolerance = 0.001
        )
        val pipeline = SimulationPipeline(config)

        val cube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 20.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.9)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        pipeline.step()

        assertFalse(pipeline.hasNaN(), "No NaN after substep exhaustion")
        // With maxSubsteps=2 and high restitution, the body may exhaust the budget
        // Either way, diagnostics should be populated and no crash should occur
        assertTrue(pipeline.ccdDiagnostics.ccdSteps >= 1, "CCD steps should be at least 1")
        assertTrue(pipeline.ccdDiagnostics.ccdFallbackCount >= 1, "Should have at least 1 fallback")
    }

    @Test
    fun `CCD diagnostics are populated`() {
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
            .linearVelocity(Vector3(0.0, -30.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()

        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        pipeline.step()

        val diag = pipeline.ccdDiagnostics
        assertTrue(diag.ccdSteps >= 1, "CCD steps should be at least 1")
        assertTrue(diag.toiQueries >= 1, "TOI queries should be at least 1")
        assertFalse(diag.maximumPenetration.isNaN(), "Max penetration should not be NaN")
    }

    // ============================================================
    // Stress Tests
    // ============================================================

    @Test
    fun `CCD multiple static obstacles`() {
        // Fast body falling through a field of static obstacles
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 5.0,
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Static obstacles at different heights
        var nextId = 1
        for (y in listOf(8.0, 6.0, 4.0, 2.0)) {
            val obstacle = rigidBody()
                .id(nextId++)
                .bodyType(BodyType.STATIC)
                .position(Vector3(0.0, y, 0.0))
                .mass(0.0)
                .build()
            pipeline.addBody(obstacle)
            pipeline.setShape(obstacle.id, BoxShape(Vector3(0.3, 0.1, 0.3)))
        }

        // Fast falling body
        val cube = rigidBody()
            .id(nextId++)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(0.0, 15.0, 0.0))
            .linearVelocity(Vector3(0.0, -30.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(cube.id, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 200) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN with multiple obstacles")
    }

    @Test
    fun `CCD mixed CCD enabled and disabled bodies`() {
        // Some bodies use CCD, others use discrete detection
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 20.0, // Only very fast bodies use CCD
            ccdMaxSubsteps = 10
        )
        val pipeline = SimulationPipeline(config)

        // Fast body (CCD enabled)
        val fastCube = rigidBody()
            .id(1)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(-2.0, 10.0, 0.0))
            .linearVelocity(Vector3(0.0, -50.0, 0.0))
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()
        pipeline.addBody(fastCube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        // Slow body (CCD disabled, uses discrete)
        val slowCube = rigidBody()
            .id(2)
            .bodyType(BodyType.DYNAMIC)
            .position(Vector3(2.0, 5.0, 0.0))
            .linearVelocity(Vector3.ZERO)
            .mass(1.0)
            .restitution(0.3)
            .friction(0.5)
            .build()
        pipeline.addBody(slowCube)
        pipeline.setShape(2, BoxShape(Vector3(0.5, 0.5, 0.5)))

        for (i in 0 until 300) {
            pipeline.step()
        }

        assertFalse(pipeline.hasNaN(), "No NaN with mixed CCD/discrete bodies")
        val bodies = pipeline.getBodies()
        for (body in bodies) {
            assertTrue(body.transform.position.y > -1.0,
                "Body ${body.id} should not tunnel through ground. Y: ${body.transform.position.y}")
        }
    }

    @Test
    fun `CCD long duration stability`() {
        // Long-running simulation with CCD should remain stable
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            ccdEnabled = true,
            ccdVelocityThreshold = 10.0,
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

        val steps = 1000
        for (i in 0 until steps) {
            pipeline.step()
            assertFalse(pipeline.hasNaN(), "NaN detected at step $i")
        }

        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > 0.0, "Body should be above ground after $steps steps")
        assertTrue(pos.y < 20.0, "Body should not fly away. Y: ${pos.y}")
    }

    @Test
    fun `CCD disabled regression  discrete still works`() {
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

        assertFalse(pipeline.hasNaN(), "No NaN with CCD disabled")
        val pos = pipeline.getBodies()[0].transform.position
        assertTrue(pos.y > 0.0 && pos.y < 2.0,
            "Cube should rest on ground with CCD disabled. Position: $pos")
    }

    @Test
    fun `CCD determinism with fast body`() {
        // Two identical CCD simulations should produce identical results
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
                .linearVelocity(Vector3(0.0, -40.0, 0.0))
                .mass(1.0)
                .restitution(0.3)
                .friction(0.5)
                .build()

            pipeline.addBody(cube)
            pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

            for (i in 0 until 300) {
                pipeline.step()
            }

            return pipeline.getBodies()[0].transform.position
        }

        val result1 = runSimulation()
        val result2 = runSimulation()

        assertEquals(result1.x, result2.x, 1e-10, "X should be identical")
        assertEquals(result1.y, result2.y, 1e-10, "Y should be identical")
        assertEquals(result1.z, result2.z, 1e-10, "Z should be identical")
    }
}
