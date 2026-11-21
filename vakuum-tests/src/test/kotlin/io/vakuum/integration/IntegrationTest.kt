package io.vakuum.integration

import io.vakuum.collision.BoxShape
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.core.PhysicsWorldSettings
import io.vakuum.core.PhysicsWorldState
import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Vector3
import io.vakuum.runner.SimulationPipeline
import io.vakuum.runner.SimulationExporter
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.DisplayName

/**
 * Integration tests for the full physics pipeline.
 * Tests the complete simulation: dynamics + collision + solver.
 */
@DisplayName("Integration Tests")
class IntegrationTest {

    private val epsilon = 1e-4

    private fun createConfig(): PhysicsConfiguration {
        return PhysicsConfiguration(
            gravity = Vector3(0.0, -9.81, 0.0),
            fixedTimestep = 1.0 / 60.0,
            maxSolverIterations = 10,
            linearDamping = 0.0,
            angularDamping = 0.0
        )
    }

    @Test
    @DisplayName("Single cube falls and rests on ground")
    fun `single cube falls and rests on ground`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        // Ground
        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

        // Falling cube
        val cube = rigidBody()
            .id(1)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .restitution(0.3)
            .friction(0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        // Run 600 steps (10 seconds)
        repeat(600) {
            pipeline.step()
            if (pipeline.hasNaN()) {
                fail("NaN detected during simulation")
            }
        }

        val finalCube = pipeline.getBodies().find { it.id == 1 }
        assertNotNull(finalCube)

        // Cube should be near the ground (y ≈ 0.5 + 0.1 = 0.6, top of ground)
        assertTrue(finalCube!!.transform.position.y > 0.0, "Cube should be above ground")
        assertTrue(finalCube.transform.position.y < 2.0, "Cube should be near ground, not flying")

        // Velocity should be small (resting)
        val speed = finalCube.linearVelocity.length
        assertTrue(speed < 1.0, "Cube should be nearly at rest, speed: $speed")
    }

    @Test
    @DisplayName("Multiple cubes fall without NaN")
    fun `multiple cubes fall without NaN`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        // Ground
        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

        // 10 cubes
        for (i in 0 until 10) {
            val x = (i % 5 - 2) * 1.5
            val z = (i / 5 - 1) * 1.5
            val y = 5.0 + i * 2.0
            val mass = 0.5 + i * 0.1

            val cube = rigidBody()
                .id(i + 1)
                .position(x, y, z)
                .mass(mass)
                .halfExtents(0.4, 0.4, 0.4)
                .restitution(0.2)
                .friction(0.5)
                .build()
            pipeline.addBody(cube)
            pipeline.setShape(i + 1, BoxShape(Vector3(0.4, 0.4, 0.4)))
        }

        repeat(600) {
            pipeline.step()
            if (pipeline.hasNaN()) {
                fail("NaN detected with multiple cubes")
            }
        }

        // All cubes should be above ground
        for (body in pipeline.getBodies()) {
            if (body.id == 0) continue // skip ground
            assertTrue(body.transform.position.y > -1.0, "Cube ${body.id} should be above ground")
        }
    }

    @Test
    @DisplayName("Cube stack remains stable")
    fun `cube stack remains stable`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        // Ground
        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

        // 4x2 stack
        var id = 1
        for (row in 0 until 2) {
            for (col in 0 until 4) {
                val x = (col - 1.5) * 1.05
                val y = 0.5 + row * 1.05
                val z = 0.0

                val cube = rigidBody()
                    .id(id)
                    .position(x, y, z)
                    .mass(1.0)
                    .halfExtents(0.5, 0.5, 0.5)
                    .restitution(0.0)
                    .friction(0.8)
                    .build()
                pipeline.addBody(cube)
                pipeline.setShape(id, BoxShape(Vector3(0.5, 0.5, 0.5)))
                id++
            }
        }

        repeat(1200) {
            pipeline.step()
            if (pipeline.hasNaN()) {
                fail("NaN detected in stack simulation")
            }
        }

        // Check that cubes are still roughly in stack positions
        for (body in pipeline.getBodies()) {
            if (body.id == 0) continue
            // Cubes should not have fallen far from their initial positions
            assertTrue(body.transform.position.y > -1.0, "Cube ${body.id} fell through ground")
            assertTrue(body.transform.position.y < 5.0, "Cube ${body.id} flew away")
        }
    }

    @Test
    @DisplayName("Stress test: 100 cubes without NaN")
    fun `stress test 100 cubes`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        // Ground
        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(50.0, 0.1, 50.0)))

        // 100 cubes
        for (i in 0 until 100) {
            val x = (i % 10 - 5) * 1.2
            val z = (i / 10 - 5) * 1.2
            val y = 5.0 + (i % 20) * 2.0

            val cube = rigidBody()
                .id(i + 1)
                .position(x, y, z)
                .mass(0.5 + (i % 5) * 0.2)
                .halfExtents(0.4, 0.4, 0.4)
                .restitution(0.1)
                .friction(0.5)
                .build()
            pipeline.addBody(cube)
            pipeline.setShape(i + 1, BoxShape(Vector3(0.4, 0.4, 0.4)))
        }

        val startTime = System.nanoTime()
        repeat(600) {
            pipeline.step()
            if (pipeline.hasNaN()) {
                fail("NaN detected in stress test")
            }
        }
        val elapsed = (System.nanoTime() - startTime) / 1_000_000.0

        // Performance check: should complete in reasonable time
        assertTrue(elapsed < 30000, "Stress test took too long: ${elapsed}ms")
    }

    @Test
    @DisplayName("Reproducibility: two identical runs produce same results")
    fun `reproducibility test`() {
        val config = createConfig()

        // Run 1
        val pipeline1 = SimulationPipeline(config)
        setupReproTest(pipeline1)
        val results1 = mutableListOf<Vector3>()
        repeat(300) {
            pipeline1.step()
            val cube = pipeline1.getBodies().find { it.id == 1 }
            results1.add(cube?.transform?.position ?: Vector3.ZERO)
        }

        // Run 2
        val pipeline2 = SimulationPipeline(config)
        setupReproTest(pipeline2)
        val results2 = mutableListOf<Vector3>()
        repeat(300) {
            pipeline2.step()
            val cube = pipeline2.getBodies().find { it.id == 1 }
            results2.add(cube?.transform?.position ?: Vector3.ZERO)
        }

        // Compare
        for (i in results1.indices) {
            assertEquals(results1[i].x, results2[i].x, 1e-10, "X differs at frame $i")
            assertEquals(results1[i].y, results2[i].y, 1e-10, "Y differs at frame $i")
            assertEquals(results1[i].z, results2[i].z, 1e-10, "Z differs at frame $i")
        }
    }

    private fun setupReproTest(pipeline: SimulationPipeline) {
        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

        val cube = rigidBody()
            .id(1)
            .position(0.0, 10.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .restitution(0.3)
            .friction(0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
    }

    @Test
    @DisplayName("Free fall matches analytical solution")
    fun `free fall matches analytical solution`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        // Single cube, no ground
        val cube = rigidBody()
            .id(1)
            .position(0.0, 100.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        // Simulate 60 steps (1 second)
        repeat(60) {
            pipeline.step()
        }

        val finalCube = pipeline.getBodies().find { it.id == 1 }
        assertNotNull(finalCube)

        // v = g * t = -9.81 * 1.0
        assertEquals(-9.81, finalCube!!.linearVelocity.y, 0.01, "Velocity after 1s free fall")

        // y = y0 + 0.5 * g * t^2 = 100 + 0.5 * (-9.81) * 1 = 95.095
        val expectedY = 100.0 + 0.5 * (-9.81) * 1.0 * 1.0
        assertEquals(expectedY, finalCube.transform.position.y, 0.1, "Position after 1s free fall")
    }

    @Test
    @DisplayName("Static body remains immovable")
    fun `static body remains immovable`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

        val cube = rigidBody()
            .id(1)
            .position(0.0, 5.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .restitution(0.5)
            .friction(0.5)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        repeat(600) {
            pipeline.step()
        }

        val finalGround = pipeline.getBodies().find { it.id == 0 }
        assertNotNull(finalGround)
        assertEquals(Vector3.ZERO, finalGround!!.transform.position, "Ground should not move")
        assertEquals(Vector3.ZERO, finalGround.linearVelocity, "Ground velocity should be zero")
    }

    @Test
    @DisplayName("Long running simulation stability")
    fun `long running simulation stability`() {
        val config = createConfig()
        val pipeline = SimulationPipeline(config)

        val ground = rigidBody()
            .id(0)
            .bodyType(BodyType.STATIC)
            .position(0.0, 0.0, 0.0)
            .mass(0.0)
            .build()
        pipeline.addBody(ground)
        pipeline.setShape(0, BoxShape(Vector3(10.0, 0.1, 10.0)))

        val cube = rigidBody()
            .id(1)
            .position(0.0, 5.0, 0.0)
            .mass(1.0)
            .halfExtents(0.5, 0.5, 0.5)
            .restitution(0.0)
            .friction(0.8)
            .build()
        pipeline.addBody(cube)
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))

        // Run 1000 steps (~16.7 seconds)
        repeat(1000) {
            pipeline.step()
            if (pipeline.hasNaN()) {
                fail("NaN detected in long-running simulation")
            }
        }

        val finalCube = pipeline.getBodies().find { it.id == 1 }
        assertNotNull(finalCube)

        // Cube should be resting on ground
        assertTrue(finalCube!!.transform.position.y > 0.0, "Cube should be above ground")
        val speed = finalCube.linearVelocity.length
        assertTrue(speed < 0.5, "Cube should be nearly at rest after 1000 steps, speed: $speed")
    }
}
