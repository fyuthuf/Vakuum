package io.vakuum.core

import io.vakuum.math.Vector3
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.*

class PhysicsConfigurationTest {

    @Test
    fun `default configuration is valid`() {
        val config = PhysicsConfiguration.DEFAULT
        assertEquals(0.0, config.gravity.x, 1e-15)
        assertEquals(-9.81, config.gravity.y, 1e-15)
        assertEquals(0.0, config.gravity.z, 1e-15)
        assertEquals(1.0 / 60.0, config.fixedTimestep, 1e-15)
        assertEquals(10, config.maxSolverIterations)
    }

    @Test
    fun `custom configuration`() {
        val config = PhysicsConfiguration(
            gravity = Vector3(0.0, -1.62, 0.0),
            fixedTimestep = 1.0 / 120.0,
            maxSolverIterations = 20
        )
        assertEquals(-1.62, config.gravity.y, 1e-15)
        assertEquals(1.0 / 120.0, config.fixedTimestep, 1e-15)
        assertEquals(20, config.maxSolverIterations)
    }

    @Test
    fun `negative timestep throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(fixedTimestep = -1.0)
        }
    }

    @Test
    fun `zero timestep throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(fixedTimestep = 0.0)
        }
    }

    @Test
    fun `infinite timestep throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(fixedTimestep = Double.POSITIVE_INFINITY)
        }
    }

    @Test
    fun `NaN gravity throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(gravity = Vector3(Double.NaN, 0.0, 0.0))
        }
    }

    @Test
    fun `negative solver iterations throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(maxSolverIterations = -1)
        }
    }

    @Test
    fun `zero solver iterations throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(maxSolverIterations = 0)
        }
    }

    @Test
    fun `damping out of range throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(linearDamping = -0.1)
        }
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(linearDamping = 1.1)
        }
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(angularDamping = -0.1)
        }
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(angularDamping = 1.1)
        }
    }

    @Test
    fun `negative sleep threshold throws`() {
        assertThrows<IllegalArgumentException> {
            PhysicsConfiguration(sleepThreshold = -1.0)
        }
    }
}

class PhysicsWorldTest {

    @Test
    fun `initial state is IDLE`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertEquals(PhysicsWorldState.IDLE, world.currentState)
    }

    @Test
    fun `initialize transitions to RUNNING`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        assertEquals(PhysicsWorldState.RUNNING, world.currentState)
    }

    @Test
    fun `double initialize throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        assertThrows<IllegalArgumentException> { world.initialize() }
    }

    @Test
    fun `step before initialize throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertThrows<IllegalArgumentException> { world.step() }
    }

    @Test
    fun `single step returns correct delta time`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        val step = world.step()
        assertEquals(1.0 / 60.0, step.deltaTime, 1e-15)
        assertEquals(0, step.stepIndex)
    }

    @Test
    fun `multiple steps increment index`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        val step1 = world.step()
        val step2 = world.step()
        assertEquals(0, step1.stepIndex)
        assertEquals(1, step2.stepIndex)
    }

    @Test
    fun `total steps count`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.step()
        world.step()
        world.step()
        assertEquals(3, world.totalSteps)
    }

    @Test
    fun `step with elapsed time`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        val dt = 1.0 / 60.0
        val steps = world.step(dt * 3.0)
        assertEquals(3, steps.size)
    }

    @Test
    fun `step with fractional elapsed time`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        val dt = 1.0 / 60.0
        val steps = world.step(dt * 1.5)
        assertEquals(1, steps.size)
    }

    @Test
    fun `step with zero elapsed time`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        val steps = world.step(0.0)
        assertEquals(0, steps.size)
    }

    @Test
    fun `negative elapsed time throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        assertThrows<IllegalArgumentException> { world.step(-1.0) }
    }

    @Test
    fun `pause from running`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.pause()
        assertEquals(PhysicsWorldState.PAUSED, world.currentState)
    }

    @Test
    fun `pause from idle throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertThrows<IllegalArgumentException> { world.pause() }
    }

    @Test
    fun `step while paused throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.pause()
        assertThrows<IllegalArgumentException> { world.step() }
    }

    @Test
    fun `resume from paused`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.pause()
        world.resume()
        assertEquals(PhysicsWorldState.RUNNING, world.currentState)
    }

    @Test
    fun `resume from running throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        assertThrows<IllegalArgumentException> { world.resume() }
    }

    @Test
    fun `shutdown from running`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.shutdown()
        assertEquals(PhysicsWorldState.STOPPED, world.currentState)
    }

    @Test
    fun `shutdown from paused`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.pause()
        world.shutdown()
        assertEquals(PhysicsWorldState.STOPPED, world.currentState)
    }

    @Test
    fun `shutdown from idle throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertThrows<IllegalArgumentException> { world.shutdown() }
    }

    @Test
    fun `step after shutdown throws`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        world.shutdown()
        assertThrows<IllegalArgumentException> { world.step() }
    }

    @Test
    fun `full lifecycle`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertEquals(PhysicsWorldState.IDLE, world.currentState)
        world.initialize()
        assertEquals(PhysicsWorldState.RUNNING, world.currentState)
        world.step()
        assertEquals(1, world.totalSteps)
        world.pause()
        assertEquals(PhysicsWorldState.PAUSED, world.currentState)
        world.resume()
        assertEquals(PhysicsWorldState.RUNNING, world.currentState)
        world.step()
        assertEquals(2, world.totalSteps)
        world.shutdown()
        assertEquals(PhysicsWorldState.STOPPED, world.currentState)
    }

    @Test
    fun `isInitialized returns correct values`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertFalse(world.isInitialized())
        world.initialize()
        assertTrue(world.isInitialized())
        world.pause()
        assertTrue(world.isInitialized())
        world.shutdown()
        assertFalse(world.isInitialized())
    }

    @Test
    fun `isRunning returns correct values`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertFalse(world.isRunning())
        world.initialize()
        assertTrue(world.isRunning())
        world.pause()
        assertFalse(world.isRunning())
    }

    @Test
    fun `isPaused returns correct values`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertFalse(world.isPaused())
        world.initialize()
        assertFalse(world.isPaused())
        world.pause()
        assertTrue(world.isPaused())
    }

    @Test
    fun `isStopped returns correct values`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        assertFalse(world.isStopped())
        world.initialize()
        assertFalse(world.isStopped())
        world.shutdown()
        assertTrue(world.isStopped())
    }

    @Test
    fun `custom world name`() {
        val settings = PhysicsWorldSettings(worldName = "TestWorld")
        val world = PhysicsWorld(settings)
        assertEquals("TestWorld", world.name)
    }

    @Test
    fun `custom gravity`() {
        val config = PhysicsConfiguration(gravity = Vector3(0.0, -1.62, 0.0))
        val settings = PhysicsWorldSettings(configuration = config)
        val world = PhysicsWorld(settings)
        assertEquals(-1.62, world.gravity.y, 1e-15)
    }

    @Test
    fun `custom timestep`() {
        val config = PhysicsConfiguration(fixedTimestep = 1.0 / 120.0)
        val settings = PhysicsWorldSettings(configuration = config)
        val world = PhysicsWorld(settings)
        world.initialize()
        val step = world.step()
        assertEquals(1.0 / 120.0, step.deltaTime, 1e-15)
    }

    @Test
    fun `solver iterations in step result`() {
        val config = PhysicsConfiguration(maxSolverIterations = 15)
        val settings = PhysicsWorldSettings(configuration = config)
        val world = PhysicsWorld(settings)
        world.initialize()
        val step = world.step()
        assertEquals(15, step.solverIterations)
    }

    @Test
    fun `spiral of death prevention`() {
        val world = PhysicsWorld(PhysicsWorldSettings())
        world.initialize()
        // Very large elapsed time should not cause infinite steps
        val steps = world.step(1000.0)
        assertTrue(steps.size <= 6)
    }
}
