package io.vakuum.dynamics

import io.vakuum.math.Quaternion
import io.vakuum.math.Transform
import io.vakuum.math.Vector3
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class BodyTypeTest {

    @Test
    fun `static body type exists`() {
        assertEquals(BodyType.STATIC, BodyType.valueOf("STATIC"))
    }

    @Test
    fun `dynamic body type exists`() {
        assertEquals(BodyType.DYNAMIC, BodyType.valueOf("DYNAMIC"))
    }

    @Test
    fun `kinematic body type exists`() {
        assertEquals(BodyType.KINEMATIC, BodyType.valueOf("KINEMATIC"))
    }

    @Test
    fun `body type has three values`() {
        assertEquals(3, BodyType.values().size)
    }
}
