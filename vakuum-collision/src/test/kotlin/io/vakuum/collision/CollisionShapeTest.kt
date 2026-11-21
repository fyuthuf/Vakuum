package io.vakuum.collision

import io.vakuum.dynamics.BodyType
import io.vakuum.dynamics.rigidBody
import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class CollisionShapeTest {

    @Test
    fun `box shape with positive extents is valid`() {
        val shape = BoxShape(Vector3(0.5, 0.5, 0.5))
        assertEquals(Vector3(0.5, 0.5, 0.5), shape.halfExtents)
    }

    @Test
    fun `box shape with zero extent throws`() {
        var thrown = false
        try {
            BoxShape(Vector3(0.0, 0.5, 0.5))
        } catch (e: IllegalArgumentException) {
            thrown = true
        }
        assertTrue(thrown, "BoxShape with zero extent should throw")
    }

    @Test
    fun `box shape with negative extent throws`() {
        var thrown = false
        try {
            BoxShape(Vector3(-0.5, 0.5, 0.5))
        } catch (e: IllegalArgumentException) {
            thrown = true
        }
        assertTrue(thrown, "BoxShape with negative extent should throw")
    }

    @Test
    fun `plane shape with unit normal is valid`() {
        val shape = PlaneShape(Vector3.UNIT_Y, 0.0)
        assertEquals(Vector3.UNIT_Y, shape.normal)
        assertEquals(0.0, shape.distance)
    }

    @Test
    fun `plane shape with zero normal throws`() {
        var thrown = false
        try {
            PlaneShape(Vector3.ZERO, 0.0)
        } catch (e: IllegalArgumentException) {
            thrown = true
        }
        assertTrue(thrown, "PlaneShape with zero normal should throw")
    }
}
