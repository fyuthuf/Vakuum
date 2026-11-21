package io.vakuum.integration

import io.vakuum.runner.SimulationPipeline
import io.vakuum.runner.SimulationExporter
import io.vakuum.dynamics.RigidBody
import io.vakuum.dynamics.rigidBody
import io.vakuum.collision.BoxShape
import io.vakuum.core.PhysicsConfiguration
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

/**
 * Physics validation test that analyzes the simulation data directly.
 * This test verifies the physics behavior without requiring Blender.
 */
class PhysicsValidationTest {
    
    
    @Test
    fun `verify free fall acceleration`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        // Create ground
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        // Create falling cube
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        // Run a few steps and check acceleration
        val initialY = cube.transform.position.y
        pipeline.step()
        val after1Step = cube.transform.position.y
        
        pipeline.step()
        val after2Steps = cube.transform.position.y
        
        // Check that cube is falling
        assertTrue(after1Step < initialY, "Cube should fall after 1 step")
        assertTrue(after2Steps < after1Step, "Cube should continue falling after 2 steps")
        
        // Check acceleration is approximately constant
        val distance1 = initialY - after1Step
        val distance2 = after1Step - after2Steps
        assertTrue(distance2 > distance1, "Cube should accelerate during free fall")
    }
    
    @Test
    fun `verify cube does not pass through ground`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        // Run simulation for 600 steps
        for (i in 1..600) {
            pipeline.step()
        }
        
        // Cube should not pass through ground
        val finalY = cube.transform.position.y
        assertTrue(finalY >= 0.4, "Cube should not pass through ground, final y=$finalY")
    }
    
    @Test
    fun `verify cube bounces after collision`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .restitution(0.3)
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        var maxBounceY = 0.0
        var collided = false
        
        // Run simulation and track bounces
        for (i in 1..600) {
            pipeline.step()
            
            // Check if cube bounced (velocity changed from negative to positive)
            if (cube.linearVelocity.y > 0 && !collided) {
                collided = true
            }
            
            if (collided && cube.transform.position.y > maxBounceY) {
                maxBounceY = cube.transform.position.y
            }
        }
        
        assertTrue(collided, "Cube should collide with ground")
        assertTrue(maxBounceY > 0.5, "Cube should bounce after collision, max bounce y=$maxBounceY")
    }
    
    @Test
    fun `verify cube comes to rest`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        // Run simulation
        for (i in 1..600) {
            pipeline.step()
        }
        
        // Check that cube is at rest
        val finalVelocity = Math.sqrt(
            cube.linearVelocity.x * cube.linearVelocity.x +
            cube.linearVelocity.y * cube.linearVelocity.y +
            cube.linearVelocity.z * cube.linearVelocity.z
        )
        val finalY = cube.transform.position.y
        
        assertTrue(finalVelocity < 0.1, "Cube should come to rest, final velocity=$finalVelocity")
        assertTrue(finalY > 0.4 && finalY < 0.7, "Cube should rest on ground, final y=$finalY")
    }
    
    @Test
    fun `verify no NaN or Infinity values`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        // Run simulation for 600 steps
        for (i in 1..600) {
            pipeline.step()
            
            // Check for NaN/Infinity
            assertFalse(cube.transform.position.x.isNaN(), "Position X should not be NaN at step $i")
            assertFalse(cube.transform.position.y.isNaN(), "Position Y should not be NaN at step $i")
            assertFalse(cube.transform.position.z.isNaN(), "Position Z should not be NaN at step $i")
            assertFalse(cube.linearVelocity.x.isNaN(), "Velocity X should not be NaN at step $i")
            assertFalse(cube.linearVelocity.y.isNaN(), "Velocity Y should not be NaN at step $i")
            assertFalse(cube.linearVelocity.z.isNaN(), "Velocity Z should not be NaN at step $i")
            
            assertFalse(cube.transform.position.x.isInfinite(), "Position X should not be Infinite at step $i")
            assertFalse(cube.transform.position.y.isInfinite(), "Position Y should not be Infinite at step $i")
            assertFalse(cube.transform.position.z.isInfinite(), "Position Z should not be Infinite at step $i")
        }
    }
    
    @Test
    fun `verify ground remains static`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        // Run simulation
        for (i in 1..600) {
            pipeline.step()
        }
        
        // Ground should not move
        assertEquals(0.0, ground.transform.position.x, 1e-10, "Ground X should not move")
        assertEquals(0.0, ground.transform.position.y, 1e-10, "Ground Y should not move")
        assertEquals(0.0, ground.transform.position.z, 1e-10, "Ground Z should not move")
    }
    
    @Test
    fun `verify multiple cubes fall without NaN`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.addBody(ground)
        
        // Create 10 cubes at different positions
        val cubes = mutableListOf<RigidBody>()
        for (i in 0..9) {
            val cube = rigidBody()
                .id(i + 1)
                .position(Vector3(i * 2.0 - 9.0, 10.0 + (i % 3) * 2.0, 0.0))
                .mass(1.0 + i * 0.1)
                .halfExtents(Vector3(0.5, 0.5, 0.5))
                .build()
            
            pipeline.setShape(i + 1, BoxShape(Vector3(0.5, 0.5, 0.5)))
            pipeline.addBody(cube)
            cubes.add(cube)
        }
        
        // Run simulation
        for (step in 1..600) {
            pipeline.step()
            
            // Check all cubes
            for ((idx, cube) in cubes.withIndex()) {
                assertFalse(cube.transform.position.y.isNaN(), "Cube $idx position Y should not be NaN at step $step")
                assertFalse(cube.linearVelocity.y.isNaN(), "Cube $idx velocity Y should not be NaN at step $step")
            }
        }
    }
    
    @Test
    fun `verify simulation is deterministic`() {
        // Run the same simulation twice and compare results
        fun runSimulation(): List<Pair<Double, Double>> {
            val config = PhysicsConfiguration()
            val pipeline = SimulationPipeline(config)
            
            val ground = rigidBody()
                .id(0)
                .bodyType(io.vakuum.dynamics.BodyType.STATIC)
                .position(Vector3.ZERO)
                .mass(0.0)
                .build()
            
            val cube = rigidBody()
                .id(1)
                .position(Vector3(0.0, 10.0, 0.0))
                .mass(1.0)
                .halfExtents(Vector3(0.5, 0.5, 0.5))
                .build()
            
            pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
            pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
            
            pipeline.addBody(ground)
            pipeline.addBody(cube)
            
            val results = mutableListOf<Pair<Double, Double>>()
            
            for (i in 1..600) {
                pipeline.step()
                results.add(Pair(cube.transform.position.y, cube.linearVelocity.y))
            }
            
            return results
        }
        
        val run1 = runSimulation()
        val run2 = runSimulation()
        
        assertEquals(run1.size, run2.size, "Both runs should have same number of frames")
        
        for (i in run1.indices) {
            assertEquals(run1[i].first, run2[i].first, 1e-10, "Position Y should match at frame $i")
            assertEquals(run1[i].second, run2[i].second, 1e-10, "Velocity Y should match at frame $i")
        }
    }
    
    @Test
    fun `verify JSON export is valid`() {
        val config = PhysicsConfiguration()
        val pipeline = SimulationPipeline(config)
        val exporter = SimulationExporter(config)
        
        val ground = rigidBody()
            .id(0)
            .bodyType(io.vakuum.dynamics.BodyType.STATIC)
            .position(Vector3.ZERO)
            .mass(0.0)
            .build()
        
        val cube = rigidBody()
            .id(1)
            .position(Vector3(0.0, 10.0, 0.0))
            .mass(1.0)
            .halfExtents(Vector3(0.5, 0.5, 0.5))
            .build()
        
        pipeline.setShape(0, BoxShape(Vector3(5.0, 0.1, 5.0)))
        pipeline.setShape(1, BoxShape(Vector3(0.5, 0.5, 0.5)))
        
        pipeline.addBody(ground)
        pipeline.addBody(cube)
        
        // Run simulation with export
        for (i in 1..100) {
            pipeline.step()
            exporter.record(i, i * config.fixedTimestep, pipeline.getBodies())
        }
        
        val json = exporter.toJson()
        assertNotNull(json)
        assertTrue(json.contains("\"metadata\""))
        assertTrue(json.contains("\"frames\""))
        assertTrue(json.contains("\"position\""))
        assertTrue(json.contains("\"orientation\""))
    }
}
