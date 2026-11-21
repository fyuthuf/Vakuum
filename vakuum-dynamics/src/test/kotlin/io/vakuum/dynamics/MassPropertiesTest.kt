package io.vakuum.dynamics

import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

class MassPropertiesTest {

    private val epsilon = 1e-9

    @Test
    fun `static mass properties have zero mass`() {
        val props = MassProperties.static()
        assertEquals(0.0, props.mass, epsilon)
        assertEquals(0.0, props.inverseMass, epsilon)
        assertTrue(props.isMassless)
    }

    @Test
    fun `static mass properties center of mass is zero`() {
        val props = MassProperties.static()
        assertEquals(Vector3.ZERO, props.centerOfMass)
    }

    @Test
    fun `box mass properties calculate correct inverse mass`() {
        val props = MassProperties.box(Vector3(0.5, 0.5, 0.5), mass = 10.0)
        assertEquals(10.0, props.mass, epsilon)
        assertEquals(0.1, props.inverseMass, epsilon)
        assertFalse(props.isMassless)
    }

    @Test
    fun `box mass properties with zero mass are static`() {
        val props = MassProperties.box(Vector3(0.5, 0.5, 0.5), mass = 0.0)
        assertEquals(0.0, props.mass, epsilon)
        assertEquals(0.0, props.inverseMass, epsilon)
        assertTrue(props.isMassless)
    }

    @Test
    fun `box inertia tensor is diagonal for uniform box`() {
        val halfExtents = Vector3(1.0, 1.0, 1.0)
        val mass = 12.0
        val props = MassProperties.box(halfExtents, mass)

        // Ixx = (1/6) * m * (hy^2 + hz^2) = (1/6) * 12 * (1 + 1) = 4
        // Iyy = (1/6) * m * (hx^2 + hz^2) = (1/6) * 12 * (1 + 1) = 4
        // Izz = (1/6) * m * (hx^2 + hy^2) = (1/6) * 12 * (1 + 1) = 4
        val inertia = props.inertiaTensor
        assertEquals(4.0, inertia.get(0, 0), epsilon, "Ixx")
        assertEquals(4.0, inertia.get(1, 1), epsilon, "Iyy")
        assertEquals(4.0, inertia.get(2, 2), epsilon, "Izz")
        // Off-diagonal should be zero
        assertEquals(0.0, inertia.get(0, 1), epsilon, "Ixy")
        assertEquals(0.0, inertia.get(0, 2), epsilon, "Ixz")
        assertEquals(0.0, inertia.get(1, 2), epsilon, "Iyz")
    }

    @Test
    fun `box inverse inertia tensor is correct`() {
        val halfExtents = Vector3(1.0, 1.0, 1.0)
        val mass = 12.0
        val props = MassProperties.box(halfExtents, mass)

        // I = 4, so invI = 0.25
        val invInertia = props.inverseInertiaTensor
        assertEquals(0.25, invInertia.get(0, 0), epsilon, "invIxx")
        assertEquals(0.25, invInertia.get(1, 1), epsilon, "invIyy")
        assertEquals(0.25, invInertia.get(2, 2), epsilon, "invIzz")
    }

    @Test
    fun `box with different dimensions has correct inertia`() {
        // 2x1x0.5 box, mass = 6
        val halfExtents = Vector3(1.0, 0.5, 0.25)
        val mass = 6.0
        val props = MassProperties.box(halfExtents, mass)

        // Ixx = (1/6) * 6 * (0.5^2 + 0.25^2) = 0.25 + 0.0625 = 0.3125
        // Iyy = (1/6) * 6 * (1^2 + 0.25^2) = 1 + 0.0625 = 1.0625
        // Izz = (1/6) * 6 * (1^2 + 0.5^2) = 1 + 0.25 = 1.25
        val inertia = props.inertiaTensor
        assertEquals(0.3125, inertia.get(0, 0), epsilon, "Ixx")
        assertEquals(1.0625, inertia.get(1, 1), epsilon, "Iyy")
        assertEquals(1.25, inertia.get(2, 2), epsilon, "Izz")
    }

    @Test
    fun `negative mass throws exception`() {
        assertThrows<IllegalArgumentException> {
            MassProperties.box(Vector3(0.5, 0.5, 0.5), mass = -1.0)
        }
    }

    @Test
    fun `zero half extents throw exception`() {
        assertThrows<IllegalArgumentException> {
            MassProperties.box(Vector3(0.0, 0.5, 0.5), mass = 1.0)
        }
    }

    @Test
    fun `negative inverse mass throws exception`() {
        assertThrows<IllegalArgumentException> {
            MassProperties(
                mass = 1.0,
                inverseMass = -1.0,
                inertiaTensor = io.vakuum.math.Matrix3.IDENTITY,
                inverseInertiaTensor = io.vakuum.math.Matrix3.IDENTITY
            )
        }
    }

    @Test
    fun `NaN center of mass throws exception`() {
        assertThrows<IllegalArgumentException> {
            MassProperties(
                mass = 1.0,
                inverseMass = 1.0,
                inertiaTensor = io.vakuum.math.Matrix3.IDENTITY,
                inverseInertiaTensor = io.vakuum.math.Matrix3.IDENTITY,
                centerOfMass = Vector3(Double.NaN, 0.0, 0.0)
            )
        }
    }

    @Test
    fun `box with positive mass uses provided mass`() {
        val props = MassProperties.box(Vector3(0.5, 0.5, 0.5), mass = 5.0)
        assertEquals(5.0, props.mass, epsilon)
        assertEquals(0.2, props.inverseMass, epsilon)
    }
}
