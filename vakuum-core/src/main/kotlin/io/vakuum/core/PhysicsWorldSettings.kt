package io.vakuum.core

/**
 * World-specific settings that extend the base configuration.
 */
data class PhysicsWorldSettings(
    /** Base physics configuration. */
    val configuration: PhysicsConfiguration = PhysicsConfiguration.DEFAULT,

    /** Name of the physics world (for debugging). */
    val worldName: String = "DefaultWorld",
)
